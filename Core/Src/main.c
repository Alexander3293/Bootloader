
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "main_defines.h"
#include "mt29fxg08_nand.h"
#include "memory.h"
//#include "memory_sh.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CRC_HandleTypeDef hcrc;
IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;	///<ami_tx_pwm
TIM_HandleTypeDef htim2;	///<ami_rx_timeout
TIM_HandleTypeDef htim10;	///<ami_tx_timeout
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart6_tx;
NAND_HandleTypeDef hnand1;
DMA_HandleTypeDef hdma_memtomem_dma2_stream0;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define FLASH_FIRMWARE_ADDR			0x800C000
#define FLASH_FWLENGTH_ADDR			(FLASH_FIRMWARE_ADDR - 4)

typedef void (*pFunction)(void);
pFunction Jump_To_Application;
uint32_t JumpAddress;

data_source g_comm_channel;
bool bootloader_timeout_en = false;//true;
bool firmware_write = false;
bool erase_flag = false;
bool reboot_flag = false;

//CSpare spare;

extern uint8_t ami_Buffer_rx[AMI_RXBUF_LEN];		// буфер приема данных ami
extern bool ami_rx_buffer_updated;
extern uint32_t ami_byte_counter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_IWDG_Init(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init_45(void);
static void MX_TIM2_Init_45(void);
static void MX_TIM10_Init_45(void);
static void SPI4_Init_45(void);
static void MX_FMC_Init(void);

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART6_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// TIM2 - ami rx таймаут 150 мкс с момента приема последнего байта от ПЛИС (USART)
// TIM10 - ami tx timeout timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Таймаут AMI RX (USART1)
	if(htim->Instance == TIM2)
	{
		ami_rx_suspend();
		ami_stop_rx_timeout();
		ami_byte_counter = AMI_RXBUF_LEN - hdma_usart1_rx.Instance->NDTR;
		g_comm_channel = ami;
		ami_rx_timeout_callback();
	}

	// Таймаут AMI передачи (USART) AMI TX WDT
	if(htim->Instance == TIM10)
	{
		ami_tx_timeout_callback();
	}
}

/*
 * @brief Calculates timer period according to timer's PLLCLK
 * @return period, 32 bits for tim2 and tim5, 16 bits for the rest
 */
uint32_t period_calc(TIM_HandleTypeDef *htim)
{
	uint32_t apb1clk_tim, apb2clk_tim;
//	uint32_t period_16;		// period value for 16 bit timer (0-65535)
	uint32_t period_32 = 0;		// period value for 32 bit timer

	// Get PCLK1 (APB1), get PCLK2 (APB2) frequency
	apb1clk_tim = HAL_RCC_GetPCLK1Freq();
	apb2clk_tim = HAL_RCC_GetPCLK2Freq();

	// Get prescalers for APB1, APB2
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	uint32_t FLatency = 0;
	HAL_RCC_GetClockConfig(&RCC_ClkInitStruct, &FLatency);

	// If prescaler > 1, than timer clock multiplies X2
	if(RCC_ClkInitStruct.APB1CLKDivider != RCC_HCLK_DIV1) { apb1clk_tim *= 2; }
	if(RCC_ClkInitStruct.APB2CLKDivider != RCC_HCLK_DIV1) { apb2clk_tim *= 2; }

	if(htim->Instance == TIM2)
	{
		// APB1, 32 bits timer
		if(htim2.Init.Prescaler != 0)
		{
			// Protection, period is not calculated for other prescalers, add if necessary
			_Error_Handler(__FILE__, __LINE__);
		}

		// 150 us period
		// 0.00015 sec period = apb1clk_tim / x
		// x = 1/0.00015 = 6666.6666666666666666666666666667
		period_32 = apb1clk_tim / 6667;
	}

	// Correct period to period - 1 for proper timer period value (see datasheet)
	if(period_32)
		--period_32;

	return period_32;
}


void UART_FixBaudRate(UART_HandleTypeDef *huart, uint32_t speed)
{
	uint32_t mantissa, fraction;
	uint32_t apbclk;
	if((huart->Instance == USART1) || (huart->Instance == USART6))
		apbclk = HAL_RCC_GetPCLK2Freq();
	else
		apbclk = HAL_RCC_GetPCLK1Freq();

	switch(apbclk)
	{/*BaudRate = Fclk / 8*(2-OVER8)*USARTDIV*/
		case 96000000:
			// ONEBIT = 1 to get max tolerance
			huart->Instance->CR3 |= USART_CR3_ONEBIT;

			if(speed == 12000000)
			{
				// OVER8 = 1 to get max speed
				huart->Instance->CR1 |= USART_CR1_OVER8;
				mantissa = 1;
				fraction = 0;
			}
			else if(speed == 3000000)
			{
				// OVER8 = 1 to get max speed
				huart->Instance->CR1 |= USART_CR1_OVER8;
				mantissa = 4;
				fraction = 0;
			}
			else if(speed == 6000000)
			{
				// OVER8 = 1 to get max speed
				huart->Instance->CR1 |= USART_CR1_OVER8;
				mantissa = 2;
				fraction = 0;
			}
			else //if(speed == 625000)
			{
				// OVER8 = 0 to get max accurate			// USARTDIV = 9.625; BaudRate = Fclk / (8*(2-OVER8)*USARTDIV) = (96 000 000 / (16 * x)) = 623376.62337662337662337662337662
				huart->Instance->CR1 &= ~USART_CR1_OVER8;
				mantissa = 9;
				fraction = 10;
			}
			huart->Instance->BRR = (mantissa << 4) | fraction;
			break;

		case 84000000:
			// ONEBIT = 1 to get max tolerance
			huart->Instance->CR3 |= USART_CR3_ONEBIT;
			// OVER8 = 1 to get max speed
			huart->Instance->CR1 |= USART_CR1_OVER8;
			if(speed == 6000000)
			{
				mantissa = 1;
				fraction = 6;
			}

			if(speed == 3000000)
			{
				mantissa = 3;
				fraction = 4;
			}
			else //if(speed == 625000)
			{
				// OVER8 = 0 to get max accurate
				huart->Instance->CR1 &= ~USART_CR1_OVER8;
				mantissa = 8;
				fraction = 6;
			}
			huart->Instance->BRR = (mantissa << 4) | fraction;
			break;

		case 45000000:
			// ONEBIT = 1 to get max tolerance
			huart->Instance->CR3 |= USART_CR3_ONEBIT;
			// OVER8 = 1 to get max speed
			huart->Instance->CR1 |= USART_CR1_OVER8;

			if(speed == 3000000)
			{
				//30000000 = 45000000 / (8 * USARTDIV)
				//USARTDIV = 1.875
				//Mantissa = 1
				//Fraction = 0.875*8 = 7
				mantissa = 1;
				fraction = 7;
			}
			else			// if(speed == 625000)
			{
				// OVER8 = 0 to get max accurate
				huart->Instance->CR1 &= ~USART_CR1_OVER8;
				//625000 = 45000000 / (16 * USARTDIV)
				//USARTDIV = 4.5
				//Mantissa = 4
				//Fraction = 0.5 * 16 = 8
				mantissa = 4;
				fraction = 8;
			}
			huart->Instance->BRR = (mantissa << 4) | fraction;
			break;

		case 42000000:
			// ONEBIT = 1 to get max tolerance
			huart->Instance->CR3 |= USART_CR3_ONEBIT;
			// OVER8 = 1 to get max speed
			huart->Instance->CR1 |= USART_CR1_OVER8;
			if(speed == 3000000)
			{
				mantissa = 1;
				fraction = 6;
			}
			else			// if(speed == 625000)
			{
				huart->Instance->CR1 &= ~USART_CR1_OVER8;
				mantissa = 4;
				fraction = 3;
			}
			huart->Instance->BRR = (mantissa << 4) | fraction;
			break;

		case 30000000:
			// ONEBIT = 1 to get max tolerance
			huart->Instance->CR3 |= USART_CR3_ONEBIT;
			// OVER8 = 0 to get max accurate
			huart->Instance->CR1 &= ~USART_CR1_OVER8;

			mantissa = 3;
			fraction = 0;

			huart->Instance->BRR = (mantissa << 4) | fraction;
			break;

		default:
			// add necessary frequency if needed
			break;
	}
}

void NVIC_Priorities_Fix()
{
	/* SysTick highest priority for HAL */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	HAL_NVIC_SetPriority(FMC_IRQn, 1, 0);			// NAND memory  (must be higher than NAND write process function)
	HAL_NVIC_SetPriority(SPI4_IRQn, 2, 0);			// m95m01_eeprom (must be higher than NAND write process function)

	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);	// ami tx USART2
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);	// ami rx USART1

	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);	// nand (must be higher than NAND write process function)
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 1, 0);	// nand (must be higher than NAND write process function)

	HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 6, 0); // tim10 ami tx timeout timer
	HAL_NVIC_SetPriority(TIM2_IRQn, 4, 0);			// ami rx таймаут 150 мкс с момента приема последнего байта от ПЛИС (USART)

	HAL_NVIC_SetPriority(USART1_IRQn, 9, 0);		// USART1: прием данных от ПЛИС (ами)
	HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);		// USART2 передача данных по ами (по прерываниям, low speed)
}

void JumpToApplication()
{
	HAL_DeInit();

	HAL_IWDG_Refresh(&hiwdg);
	JumpAddress = *(uint32_t*) (FLASH_FIRMWARE_ADDR	 + 4);
	Jump_To_Application = (pFunction) JumpAddress;
	__set_MSP(*(uint32_t *)FLASH_FIRMWARE_ADDR);	// Initialize user application's Stack Pointer
	Jump_To_Application();
}

void HAL_SYSTICK_Callback(void)
{
	HAL_IWDG_Refresh(&hiwdg);
}

uint16_t crc_calc16 = 0;

bool firmware_flash()
{
	bool res = false;
	uint8_t firmware_buffer[NAND_PAGE_SIZE_BYTES];

	// Read firmware length (from the first nand page)
	if(!nand_read_page(firmware_buffer, NAND_DATA_ADDRESS_BYTES))
		return false;

	// Длина прошивки (без 4х байт crc32)
	uint32_t fw_length = firmware_buffer[0]
						| firmware_buffer[1] << 8
						| firmware_buffer[2] << 16
						| firmware_buffer[3] << 24;


	if(!fw_length || (fw_length > (2048 - 32)*1024))
		return false;

	// Check firmware crc (in nand)
//	uint16_t crc_calc16 = 0;
	uint16_t fw_crc16 = 0;
	uint32_t buffer_length = 0;
	uint32_t fw_addr = 0;

	init_accumulate_crc16();

	for(; fw_addr < fw_length; )
	{
		res = nand_read_page(firmware_buffer, NAND_DATA_ADDRESS_BYTES + (fw_addr/COMPAT_PAGE_SIZE) * NAND_PAGE_SIZE_BYTES);
		if(!res)
		{
			crc_calc16 = 0;		///debug
			break;
		}

		fw_addr += COMPAT_PAGE_SIZE;
		buffer_length = fw_addr > fw_length ? (fw_length % COMPAT_PAGE_SIZE) : COMPAT_PAGE_SIZE;

		crc_calc16 = calc_accumulate_crc16(firmware_buffer, buffer_length);
	}

	// Read firmware crc16 from nand
	uint64_t crc16_addr = (fw_length / COMPAT_PAGE_SIZE) * NAND_PAGE_SIZE_BYTES  + fw_length % COMPAT_PAGE_SIZE;
	uint16_t bytes_readed = 0;
	res = nand_read_data_byte(crc16_addr, (uint8_t*)&fw_crc16, sizeof(fw_crc16), &bytes_readed);

	if(crc_calc16 != fw_crc16)
		return false;


	// Write firmware to stm32 flash
	flash_stm32_erase(fw_length, FLASH_FWLENGTH_ADDR);

	for(fw_addr = 0; fw_addr < fw_length; fw_addr += COMPAT_PAGE_SIZE)
	{
		nand_read_page(firmware_buffer, NAND_DATA_ADDRESS_BYTES + (fw_addr/COMPAT_PAGE_SIZE) * NAND_PAGE_SIZE_BYTES);
		buffer_length = (fw_addr + COMPAT_PAGE_SIZE) > fw_length ? (fw_length % COMPAT_PAGE_SIZE) : COMPAT_PAGE_SIZE;
		flash_stm32_write((uint32_t*)&firmware_buffer, buffer_length, FLASH_FWLENGTH_ADDR + fw_addr);
	}

	// Check firmware crc in stm32 flash (for test purposes)
	init_accumulate_crc16();

	uint32_t flash_data;
	for(uint32_t fw_addr = 0; fw_addr < fw_length; fw_addr += 4)
	{
		flash_stm32_read((uint32_t*)&flash_data, 4, FLASH_FWLENGTH_ADDR + fw_addr);
		crc_calc16 = calc_accumulate_crc16((uint8_t*)&flash_data, 4);
	}

	if(crc_calc16 != fw_crc16)
		return false;

	return true;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART1_UART_Init();
//	MX_USART6_UART_Init();
	MX_TIM1_Init_45();
	MX_TIM2_Init_45();
	MX_TIM10_Init_45();
	SPI4_Init_45();
	MX_USART2_UART_Init();
	MX_FMC_Init();
	MX_CRC_Init();
	NVIC_Priorities_Fix();

	// EEPROM hold disable
	m95m01_eeprom_hold(false);

	// NAND init: reset & read block table
	nand_select_target(0);
	HAL_NAND_Reset(&hnand1);
	nand_block_table_read();
	eeprom_block_table_read();

	// Check bootflag (eeprom)
	uint8_t bootflag = read_bootflag_memory();
	if(bootflag)
	{
		firmware_write = true;
		set_tool_status(busy_status);
	}
	else
		set_tool_status(idle_status);

	// AMI start receive
	HAL_UART_Receive_DMA(&huart1, ami_Buffer_rx, AMI_RXBUF_LEN);
	uart_rx_IDLE_IE(huart1);	//for ami receiving end event

	// Main cycle
	uint32_t timeout_tickstart = HAL_GetTick();
	uint16_t crc16;
	uint8_t address;
	bool res = false;

	while(1)
	{
		if(erase_flag)
		{
			erase_flag = false;
			m95m01_eeprom_erase(EEP_CYCLOGRAM_ADDRESS, EEP_CYCLOGRAM_SIZE/EEP_PAGE_SIZE);
			erase_memory();
			set_tool_status(ready_status);
		}

		if(firmware_write)
		{
			firmware_write = false;

	    	// Set status 'busy'
			set_tool_status(busy_status);

			// Set bootflag in eeprom
	    	write_bootflag_memory(1);

			// Flash the firmware (into stm32 flash memory)
			res = firmware_flash();

			// Reset bootflag (in eeprom)
			bootflag = false;
			write_bootflag_memory(bootflag);

			// Reset bootloader timeout (if it is enabled)
			if(bootloader_timeout_en)
				timeout_tickstart = HAL_GetTick();

			// Set status 'upload success\fail'
			if(res)
				set_tool_status(firmware_upload_success_status);
			else
				set_tool_status(firmware_upload_fail_status);
		}

		if(reboot_flag)
			HAL_NVIC_SystemReset();

		// if bootloader timeout elapsed, jump to firmware
		if(bootloader_timeout_en)
		{
			if((HAL_GetTick() - timeout_tickstart) > BOOTLOADER_TIMEOUT_MS)
				JumpToApplication();
		}

		// ami received
		if(ami_rx_buffer_updated)
		{
			address = ami_Buffer_rx[0];

			if(address == OWN_BOOTLOADER_ADDRESS)
			{
				crc16 = ami_Buffer_rx[ami_byte_counter - 2] | ((uint16_t)ami_Buffer_rx[ami_byte_counter - 1] << 8);
				if(calc_crc16(ami_Buffer_rx + 1, ami_byte_counter-1-sizeof(crc16)) == crc16)
					post_tx_data(ami_Buffer_rx, ami_byte_counter, ami, message_box, 0);
			}

			//Reset AMI buffer flag
			ami_rx_buffer_updated = false;

			if(AMI_READY == get_ami_status())
				ami_rx_restart();
		}
	}


}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 90;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CRC init function */
static void MX_CRC_Init(void)
{
	hcrc.Instance = CRC;
	if(HAL_CRC_Init(&hcrc) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* IWDG init function */
void MX_IWDG_Init(void)
{
	// IWDG clock source = 32 kHz
	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_8;	// 32/8 = 4 kHz, count every 250 us
	hiwdg.Init.Reload = 4000;					// 4000 * 250 us = 1 second
	if(HAL_IWDG_Init(&hiwdg) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);
}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PE2   ------> SPI4_SCK
 PE5   ------> SPI4_MISO
 PE6   ------> SPI4_MOSI

 PI3   ------> SPI2_MOSI
 PI2   ------> SPI2_MISO
 PH6   ------> S_TIM12_CH1
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOI_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOG_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(NAND_MUX_GPIO_Port, NAND_MUX_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(EEPROM_HOLD_GPIO_Port, EEPROM_HOLD_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOG, AMI_ENABLE_N_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(AMI_ENABLE_GPIO_Port, AMI_ENABLE_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : PI7 PI6 PI5 PI9
	 PI4 PI1 PI10 PI11
	 PI0 */
	GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_9 | GPIO_PIN_4 | GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pins : PI3 PI2 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_2;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
	HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

	/*Configure GPIO pins : PE2 PE5 PE6 */
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_5 | GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pins : AMI_ENABLE_N_Pin AMI_ENABLE_Pin MODEM_READY_Pin */
	GPIO_InitStruct.Pin = AMI_ENABLE_N_Pin | AMI_ENABLE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : PH6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pin : EEPROM_HOLD_Pin */
	GPIO_InitStruct.Pin = EEPROM_HOLD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(EEPROM_HOLD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SPI4_CS_Pin */
	GPIO_InitStruct.Pin = SPI4_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI4_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NAND_MUX_Pin */
	GPIO_InitStruct.Pin = NAND_MUX_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(NAND_MUX_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */
static void SPI4_Init_45()
{
	// SPI4 m95m01_eeprom spi
	// APB2 45 MHz
	// SPI4 5.625 Mhz

	/* Reset peripheral */
	RCC->APB2RSTR = RCC_APB2RSTR_SPI4RST;
	RCC->APB2RSTR &= ~RCC_APB2RSTR_SPI4RST;

	/* Peripheral clock enable */
	__HAL_RCC_SPI4_CLK_ENABLE()
	;

	// Master
	SPI4->CR1 = SPI_CR1_SSM			// Software Slave management (NSS)
				| SPI_CR1_MSTR		// Master
//				| SPI_CR1_BR_2;		// fPCLK/32 (2.8125 Mhz or 2.8125 Mbit/s)
//				| SPI_CR1_BR_0;		// fPCLK/4 (7.5 Mhz)
				| SPI_CR1_BR_1;		// fPCLK/8 (5.625 Mhz)
//				| SPI_CR1_BR_0		// fPCLK/16 (5.625 Mhz or 5.625 Mbit/s)
//				| SPI_CR1_BR_1;

	SPI4->CR1 |= SPI_CR1_SSI;		// NSS = 1 (CS = 1 - device is not selected)

	// Enable SPI4
	SPI4->CR1 |= SPI_CR1_SPE;
	// Wait for SPI4 bus free
	while(!(SPI4->SR & SPI_SR_TXE))
		;
	while(SPI4->SR & SPI_SR_BSY)
		;

	/* SPI4 interrupt Init */
	HAL_NVIC_SetPriority(SPI4_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(SPI4_IRQn);
}

static void MX_TIM1_Init_45(void)
{
	// ШИМ для ПЛИС 625 kHz (ami)
	// RCC_APB2ENR_TIM1EN
	// APB2 45 MHz, X2 = 90 MHz timer clock
	// period 1.6 us

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 2 - 1;							// 45 MHz
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 72 - 1;								// 1.6 us (625 kHz)
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;		// no division
	htim1.Init.RepetitionCounter = 0;

	if(HAL_TIM_Base_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if(HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	if(HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 36;									// 1/2 period
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if(HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if(HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_TIM_MspPostInit(&htim1);
}

/* TIM2 init function */
static void MX_TIM2_Init_45(void)
{
	// ami rx timeout timer (150 us)

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = period_calc(&htim2);
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
	if(HAL_TIM_Base_Init(&htim2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if(HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if(HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* TIM10 init function */
static void MX_TIM10_Init_45(void)
{
	// ami tx timeout timer (1 sec +)
	// APB2 45 MHz x1 = 45 MHz

	htim10.Instance = TIM10;
	htim10.Init.Prescaler = 45000 - 1;		// 45 000 000 / 45 000 = 1 000 Hz
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 1000 - 1;			// 1 second default
	htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	if(HAL_TIM_Base_Init(&htim10) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 625000;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if(HAL_UART_Init(&huart1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* USART2 init function */
void MX_USART2_UART_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 625000;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_2;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_CTS;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if(HAL_HalfDuplex_Init(&huart2) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* USART6 init function */
void MX_USART6_UART_Init(void)
{
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 625000;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_2;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if(HAL_UART_Init(&huart6) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;
	__HAL_RCC_DMA2_CLK_ENABLE()
	;

	/* Configure DMA request hdma_memtomem_dma2_stream0 on DMA2_Stream0 */
	hdma_memtomem_dma2_stream0.Instance = DMA2_Stream0;
	hdma_memtomem_dma2_stream0.Init.Channel = DMA_CHANNEL_0;
	hdma_memtomem_dma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
	hdma_memtomem_dma2_stream0.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_memtomem_dma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
	hdma_memtomem_dma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_memtomem_dma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_memtomem_dma2_stream0.Init.Mode = DMA_NORMAL;
	hdma_memtomem_dma2_stream0.Init.Priority = DMA_PRIORITY_LOW;
	hdma_memtomem_dma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
	hdma_memtomem_dma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
	hdma_memtomem_dma2_stream0.Init.MemBurst = DMA_MBURST_SINGLE;
	hdma_memtomem_dma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
	if(HAL_DMA_Init(&hdma_memtomem_dma2_stream0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

//	// hdma_nand_write
//	hdma_nand_write.Instance = DMA2_Stream1;
//	hdma_nand_write.Init.Channel = DMA_CHANNEL_0;
//	hdma_nand_write.Init.Direction = DMA_MEMORY_TO_MEMORY;
//	hdma_nand_write.Init.PeriphInc = DMA_PINC_DISABLE;
//	hdma_nand_write.Init.MemInc = DMA_MINC_ENABLE;
//	hdma_nand_write.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//	hdma_nand_write.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//	hdma_nand_write.Init.Mode = DMA_NORMAL;
//	hdma_nand_write.Init.Priority = DMA_PRIORITY_LOW;
//	hdma_nand_write.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
//	hdma_nand_write.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
//	hdma_nand_write.Init.MemBurst = DMA_MBURST_SINGLE;
//	hdma_nand_write.Init.PeriphBurst = DMA_PBURST_SINGLE;
//	if(HAL_DMA_Init(&hdma_nand_write) != HAL_OK)
//	{
//		_Error_Handler(__FILE__, __LINE__);
//	}

	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */		//todo remove unused
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 1, 0);		// SPI3 to incl
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);		// SPI3 to incl
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
	/* DMA2_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
	/* DMA2_Stream7_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
	/* DMA2_Stream4_IRQn interrupt configuration */		// SPI5 to empulse 8s
	HAL_NVIC_SetPriority(DMA2_Stream4_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
	/* DMA2_Stream5_IRQn interrupt configuration */		// SPI5 to empulse 8s
	HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
}

/* FMC initialization function */
static void MX_FMC_Init(void)
{
	FMC_NAND_PCC_TimingTypeDef ComSpaceTiming;
	FMC_NAND_PCC_TimingTypeDef AttSpaceTiming;

	/** Perform the NAND1 memory initialization sequence
	 */
	hnand1.Instance = FMC_NAND_DEVICE;
	/* hnand1.Init */
	hnand1.Init.NandBank = FMC_NAND_BANK2;
	hnand1.Init.Waitfeature = FMC_NAND_PCC_WAIT_FEATURE_ENABLE;
	hnand1.Init.MemoryDataWidth = FMC_NAND_PCC_MEM_BUS_WIDTH_8;
	hnand1.Init.EccComputation = FMC_NAND_ECC_DISABLE;
	hnand1.Init.ECCPageSize = FMC_NAND_ECC_PAGE_SIZE_4096BYTE;
	hnand1.Init.TCLRSetupTime = 1;		//0;
	hnand1.Init.TARSetupTime = 1;		//0;
	/* hnand1.Config */
	hnand1.Config.PageSize = NAND_PAGE_SIZE_BYTES;
	hnand1.Config.SpareAreaSize = NAND_SPARE_AREA_SIZE_BYTES;
	hnand1.Config.BlockSize = NAND_BLOCK_SIZE_PAGES;
	hnand1.Config.BlockNbr = NAND_MEMORY_SIZE_BLOCKS;
	hnand1.Config.PlaneNbr = 2;
	hnand1.Config.PlaneSize = NAND_PLANE_SIZE_BLOCKS;
	hnand1.Config.ExtraCommandEnable = ENABLE;
	/* ComSpaceTiming */
	ComSpaceTiming.SetupTime = 2;		//0;
	ComSpaceTiming.WaitSetupTime = 4;//2;		//1;
	ComSpaceTiming.HoldSetupTime = 2;		//2;
	ComSpaceTiming.HiZSetupTime = 2;//2;		//1;
	/* AttSpaceTiming */
	AttSpaceTiming.SetupTime = 2;//2;		//0;
	AttSpaceTiming.WaitSetupTime = 2;//2;		//1;
	AttSpaceTiming.HoldSetupTime = 2;//2;		//2;
	AttSpaceTiming.HiZSetupTime = 2;//2;		//1;

	if(HAL_NAND_Init(&hnand1, &ComSpaceTiming, &AttSpaceTiming) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(const char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
