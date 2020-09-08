/*
 * ami.c
 *
 * USART1: прием данных от ПЛИС (ами).
 * Запускаем DMA rx, включаем IDLE interrupt (no hal).
 * При срабатывании IDLE стартуем rx timeout (USART продолжает находиться в состоянии приема).
 * Если таймаут вышел - останавливаем USART1, отключаем таймер таймаута, вызываем ami_rx_timeout_callback().
 * ami_rx_timeout_callback: если приняли 4 и больше байт - ставим флаг ami_rx_buffer_updated, обрабатываем в main, иначе снова запускаем прием.
 *
 * USART2 \ USART6
 * ami_tx(): стартуем ami_start_tx_timeout (1 sec +), стартуем ШИМ для ПЛИС. Далее запускаем USART6 + DMA (в случае high speed) или USART2 без DMA.
 * Если таймаут вышел, вызываем ami_tx_timeout_callback(): отключаем таймер таймаута, отключаем ШИМ, инициализируем USART2\6, заново запускаем прием.
 * Иначе попадаем в HAL_UART_TxCpltCallback.
 *
 */
#include "main.h"
#include "ami.h"
#include "memory.h"

extern UART_HandleTypeDef huart1;			// USART1 - прием данных из ПЛИС
extern UART_HandleTypeDef huart2;			// USART2 - передача данных в ПЛИС
extern DMA_HandleTypeDef hdma_usart1_rx;
extern TIM_HandleTypeDef htim1;				// ШИМ для ПЛИС 625 kHz
extern TIM_HandleTypeDef htim2;				// ami rx timeout timer (150 us)
//extern TIM_HandleTypeDef htim6;				// ami rx high speed timeout timer
extern TIM_HandleTypeDef htim10;			// ami tx timeout timer (1 sec +)
//extern NAND_HandleTypeDef hnand1;

bool ami_speed_HS = false;
bool ami_rx_buffer_updated = false;			// приняли новые данные из ami

uint32_t ami_byte_counter = 0;				// счетчик принятых байт
uint8_t ami_Buffer_rx[AMI_RXBUF_LEN];		// буфер приема данных ami
uint8_t ami_Buffer_tx[AMI_TXBUF_LEN];		// буфер отправки данных ami

static ami_status global_ami_status = AMI_READY;

inline void ami_start_tx_timeout(uint16_t tx_length);
inline void ami_stop_tx_timeout();
inline void ami_start_rx_timeout();
inline void ami_tx_pwm_start();
inline void ami_tx_pwm_stop();
inline void ami_reset_rx_timeout();

/*********************************/
// Functions
/*********************************/
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void UART_FixBaudRate(UART_HandleTypeDef *huart, uint32_t speed);

/**
 * @brief Запускает передачу данных по ami
 * @param Buf - указатель на отправляемые данные
 * @param Len - длина передаваемых данных, максимально AMI_TXBUF_LEN байт
 */

void ami_tx(uint8_t* Buf, uint16_t Len)
{
	if(Len > AMI_TXBUF_LEN)
		return;

	ami_rx_suspend();
	set_ami_status(AMI_TX_BUSY);

	memcpy(ami_Buffer_tx, Buf, Len);
//	memcpy(ami_Buffer_tx + 1, Buf, Len);
//
//	ami_Buffer_tx[0] = 0x4d;//for properly work with adapters
//	Len++;
	ami_byte_counter = 0;

	ami_tx_pwm_start();

	HAL_GPIO_WritePin(AMI_ENABLE_GPIO_Port, AMI_ENABLE_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AMI_ENABLE_N_GPIO_Port, AMI_ENABLE_N_Pin, GPIO_PIN_RESET);

	for(uint32_t i = 0; i < Len; i++)
	{
		ami_Buffer_tx[i] = __RBIT((uint32_t)ami_Buffer_tx[i]) >> 24;
		ami_Buffer_tx[i] = 0xff - ami_Buffer_tx[i];
	}

	ami_start_tx_timeout(Len);

	HAL_UART_Transmit_DMA(&huart2, ami_Buffer_tx, Len);
}

/*
 * @brief Включает IDLE прерывание для USART (ami rx), no HAL.
 */
void uart_rx_IDLE_IE(UART_HandleTypeDef huart)
{
	huart.Instance->CR1 |= USART_CR1_IDLEIE;
}

/*
 * @brief Останавливает ami прием (деинициализирует USART1 и DMA).
 */
void ami_rx_suspend()
{
	HAL_UART_Abort(&AMI_RX_UART);
	CLEAR_BIT(AMI_RX_UART.Instance->CR1, USART_CR1_RE);
}

/*
 * @brief Запускает ami передачу, предварительно переинициализировав USART.
 */
void ami_rx_restart()
{
	ami_byte_counter = 0;
	SET_BIT(AMI_RX_UART.Instance->CR1, USART_CR1_RE);
	HAL_UART_Receive_DMA(&huart1, ami_Buffer_rx, AMI_RXBUF_LEN);
}

void set_ami_status(ami_status status)
{
	global_ami_status = status;
}

ami_status get_ami_status()
{
	return global_ami_status;
}

inline void timer_start_IT(TIM_HandleTypeDef *htim)
{
	__HAL_TIM_SET_COUNTER(htim, 0);
	CLEAR_BIT((htim)->Instance->SR, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(htim);
}

// AMI TX timeout
/*
 * @brief Запускает таймаут передачи ami
 *        Время таймаута = 1000 мс + время передачи посылки tx_length байт
 * @param tx_length - длина передаваемой посылки, байт
 */
inline void ami_start_tx_timeout(uint16_t tx_length)
{
	// 8 bits + 1 start + 2 stop = 11 bits
	// 625000 ami speed (lowest)
	// tx_length max = AMI_TXBUF_LEN

	switch(htim10.Init.Prescaler)
	{
		case (19200 - 1):
		case (18000 - 1):
			__HAL_TIM_SET_AUTORELOAD(&htim10, 10000 + ((uint32_t)tx_length * 11 * 10) / 625);
			break;

		case (60000 - 1):
		case (45000 - 1):
			__HAL_TIM_SET_AUTORELOAD(&htim10, 1000 + (tx_length * 11)/625);		// 1000 + 144.1792
			break;

		default:
			// Protect from wrong configuration
			_Error_Handler(__FILE__, __LINE__);
			break;
	}

	timer_start_IT(&htim10);
}

inline void ami_stop_tx_timeout()
{
	HAL_TIM_Base_Stop(&htim10);
}

// AMI RX timeout
inline void ami_start_rx_timeout()
{
	timer_start_IT(&htim2);
}

inline void ami_reset_rx_timeout()
{
	__HAL_TIM_SET_COUNTER(&htim2, 0);
}

void ami_stop_rx_timeout()
{
	HAL_TIM_Base_Stop(&htim2);
	CLEAR_BIT((&htim2)->Instance->SR, TIM_FLAG_UPDATE);
}

/*
 * @brief Включает ШИМ для ПЛИС (необходим для генерации USART CTS, выдерживает
 *        время между байтами для поддержки старых адаптеров со скоростью 460800).
 *        Включать на время ami передачи.
 */
inline void ami_tx_pwm_start()
{
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

/*
 * @brief Останавливает ШИМ для ПЛИС (необходим для генерации USART CTS, выдерживает
 *        время между байтами для поддержки старых адаптеров со скоростью 460800).
 */
inline void ami_tx_pwm_stop()
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
}

/*
 * @brief Callback функция таймаута ami приема
 */
void ami_rx_timeout_callback()
{
	if(ami_byte_counter >= FRAME_LENGTH_MIN)
	{
		ami_rx_suspend();
		set_ami_status(AMI_READY);

		uint32_t byte_number = ami_byte_counter;

		for(uint16_t a = 0; a < byte_number; a++)
		{
			ami_Buffer_rx[a] = __RBIT((uint32_t)ami_Buffer_rx[a]) >> 24;
			ami_Buffer_rx[a] = 0xff - ami_Buffer_rx[a];
		}

		// set flag indicates ami RX buffer update
		ami_rx_buffer_updated = true;
	}
	else
	{
		ami_rx_restart();
	}
}

/*
 * @brief Callback функция таймаута ami передачи.
 */
void ami_tx_timeout_callback()
{
	ami_stop_tx_timeout();
	ami_tx_pwm_stop();
	HAL_GPIO_WritePin(AMI_ENABLE_GPIO_Port, AMI_ENABLE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(AMI_ENABLE_N_GPIO_Port, AMI_ENABLE_N_Pin, GPIO_PIN_SET);

	//Tx UART reinit
	HAL_UART_Init(&huart2);

	set_ami_status(AMI_READY);
	ami_rx_restart();
}

/*
 * @brief  Ждет готовности ami (окончания приема\передачи).
 * @param  tout_ms - время ожидания в мс
 * @retval true если дождались готовности ami, иначе false.
 */
bool ami_ready_wait(uint16_t tout_ms)
{
	if(!tout_ms)
		return false;

	do
	{
		if(AMI_READY == get_ami_status())
			return true;
		else
			HAL_Delay(1);
	} while(--tout_ms);

	return false;
}

bool comm_channel_ready_wait(data_source g_comm_channel)
{
	bool r = false;

	switch(g_comm_channel)
	{
		case ami:
			r = ami_ready_wait(AMI_WAIT_MS);
			break;

		default:
			break;
	}

	return r;
}

/*********************************/
// Interrupts
/*********************************/

// (No HAL) user handler for USART IDLE interrupt
void UART1_User_IRQHandler(void)
{
//	ami_reset_rx_timeout();
	if(USART1->SR & USART_SR_IDLE)
	{
		volatile uint32_t tmp = 0;
		(void)tmp;
		tmp = USART1->SR;		// clear IDLE interrupt flag
		tmp = USART1->DR;
		ami_stop_rx_timeout();
		ami_reset_rx_timeout();
		ami_start_rx_timeout();
	}
}

//AMI tx callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
//	ami_reset_rx_timeout_hs();

	if(huart->Instance == USART2)
	{
		ami_stop_tx_timeout();

		for(int i = 0; i < 4000; i++)
			__asm("NOP");

		ami_tx_pwm_stop();
		set_ami_status(AMI_READY);

		HAL_GPIO_WritePin(AMI_ENABLE_GPIO_Port, AMI_ENABLE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AMI_ENABLE_N_GPIO_Port, AMI_ENABLE_N_Pin, GPIO_PIN_SET);
		ami_rx_restart();
	}
}

//AMI error callback
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
//	ami_reset_rx_timeout_hs();

	if(huart->Instance == USART1)			// прием
	{
		ami_reset_rx_timeout();
		ami_stop_rx_timeout();

		HAL_NVIC_DisableIRQ(DMA2_Stream2_IRQn);
		HAL_NVIC_DisableIRQ(USART1_IRQn);
		HAL_UART_DeInit(&huart1);
		//__HAL_RCC_USART1_FORCE_RESET();
		//__HAL_RCC_USART1_RELEASE_RESET();
		MX_USART1_UART_Init();
		UART_FixBaudRate(&huart1, huart1.Init.BaudRate);
		HAL_NVIC_EnableIRQ(USART1_IRQn);
		HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

		ami_rx_restart();
	}
	else if(huart->Instance == USART2)		// передача
	{
		ami_stop_tx_timeout();
		ami_tx_pwm_stop();

		// reset & reinit usart
		HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
		HAL_NVIC_DisableIRQ(USART2_IRQn);
		HAL_UART_DeInit(&huart2);
		__HAL_RCC_USART2_FORCE_RESET();
		__HAL_RCC_USART2_RELEASE_RESET();
		MX_USART2_UART_Init();
		UART_FixBaudRate(&huart2, huart2.Init.BaudRate);
		HAL_NVIC_EnableIRQ(USART2_IRQn);
		HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

		HAL_GPIO_WritePin(AMI_ENABLE_GPIO_Port, AMI_ENABLE_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(AMI_ENABLE_N_GPIO_Port, AMI_ENABLE_N_Pin, GPIO_PIN_SET);
		ami_rx_restart();
	}
}

// line eqiv/ami rx callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		set_ami_status(AMI_RX_BUSY);
		ami_reset_rx_timeout();
		ami_stop_rx_timeout();
		ami_byte_counter = AMI_RXBUF_LEN - hdma_usart1_rx.Instance->NDTR;
		ami_rx_suspend();
		ami_rx_timeout_callback();
	}
}
