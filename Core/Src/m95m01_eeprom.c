/*
 * m95m01_eeprom.c
 *
 * Read & write data in non-blocking mode using spi interrupts
 *
 * // • Memory array
 * // – 1 Mbit (128 Kbytes) of EEPROM = 131072 bytes
 * // – Page size: 256 bytes
 * // • Write
 * // – Byte Write within 5 ms
 * // – Page Write within 5 ms
 * // High-speed clock: 16 MHz
 * // The whole memory can be read with a single READ instruction
 * // Write size max = page size
 */

#include "stm32f4xx.h"
#include "string.h"
#include "m95m01_eeprom.h"

#define SPI_EEPROM_TIMEOUT_MS	100	// таймаут spi-передачи

static uint8_t *SpiTxData;			// указатель на передаваемые данные, spi
static uint8_t *SpiRxData;			// указатель на получаемые данные, spi
static uint32_t SpiSize;			// размер даннных для приемопередачи, spi
static uint32_t SpiByte;			// текущий байт приемопередачи, spi

static uint8_t tx_buf[256+4];		// буфер для передачи данных в m95m01 (spi)
static uint8_t rx_buf[256+4];		// буфер для передачи данных из m95m01 (spi)
static uint16_t tx_size;			// размер передаваемых данных
static uint32_t rx_size;			// размер принимаемых данных

volatile status_type status_reg;	// регистр STATUS m95m01

enum {
	IDLE,
	WRITE_STATUS_REGISTER, WRITE_STATUS_REGISTER_2,
	READ_STATUS_REGISTER,
	WRITE_DATA, WRITE_DATA_2,
	READ_DATA,
} spi_type = IDLE;					// состояние spi-передачи

void NOHAL_SPI_TxRxCpltCallback();

//
// @brief Запускает приемопередачу по SPI. Не использует HAL-драйвера.
// @param *pTxData - указатель на передаваемые данные
// @param *pRxData - указатель для получаемых данных
// @param Size - размер данных для приемопередачи
// @ret none
//
void NOHAL_SPI_TransmitReceive_IT(uint8_t *pTxData, uint8_t *pRxData, uint32_t Size)
{
	if(!Size)
		return;

	SpiTxData = pTxData;
	SpiRxData = pRxData;
	SpiSize = Size;
	SpiByte = 0;

	// Clear RXNE
	__IO uint16_t tmp = SPI4->DR;
	UNUSED(tmp);

	// Enable RXNE interrupt
	SPI4->CR2 |= SPI_CR2_RXNEIE;

	// Transmit first byte
	SPI4->DR = SpiTxData[0];
}

//
// @brief Обработчик прерывания для SPI4. Вызывается из SPI4_IRQHandler().
//        Не использует HAL-драйвера.
// @param none
// @ret none
//
void NOHAL_SPI4_IRQHandler()
{
	if(SPI4->SR & SPI_SR_RXNE)
	{
		if(spi_type == READ_DATA)		// если читаем данные из памяти, то начинаем складывать данные начиная с 4-ого байта
		{
			uint8_t tmp = SPI4->DR;
			if(SpiByte > 3)
				SpiRxData[SpiByte - 4] = tmp;
		}
		else							// иначе все принятые данные складываются в SpiRxData
			SpiRxData[SpiByte] = SPI4->DR;

		if(++SpiByte < SpiSize)
			SPI4->DR = SpiTxData[SpiByte];
		else
		{
			SPI4->CR2 &= ~SPI_CR2_RXNEIE;
			NOHAL_SPI_TxRxCpltCallback();
		}
	}
}

//
// @brief Записать STATUS регистр. Не блокирующая функция.
// Перед следующим обращением к EEPROM необходимо дождаться окончания spi-передачи m95m01_wait_spi().
// @ret none
//
void m95m01_eeprom_write_status_reg()
{
	// WREN, WRSR, status register 1 byte
	m95m01_eeprom_wait_ready(10);

	tx_buf[0] = WREN_INSTR;
	spi_type = WRITE_STATUS_REGISTER;

	m95m01_eeprom_cs(1);
	NOHAL_SPI_TransmitReceive_IT(tx_buf, rx_buf, 1);
}

//
// @brief Прочитать STATUS регистр. Не блокирующая функция.
// Перед следующим обращением к EEPROM необходимо дождаться окончания spi-передачи m95m01_wait_spi().
// @ret none
//
void m95m01_eeprom_read_status_reg()
{
	// RDSR, status register 1 byte
	tx_buf[0] = RDSR_INSTR;
	tx_buf[1] = 0;
	spi_type = READ_STATUS_REGISTER;

	m95m01_eeprom_cs(1);
	NOHAL_SPI_TransmitReceive_IT(tx_buf, rx_buf, 2);
}

//
// @brief Подождать окончания spi-передачи.
// @ret результат ожидания: true - дождались окончания передачи или false - вышел таймаут SPI_EEPROM_TIMEOUT_MS.
//
bool m95m01_eeprom_wait_spi()
{
	uint16_t timeout_ms = SPI_EEPROM_TIMEOUT_MS;

	while(spi_type != IDLE && timeout_ms)
		{
		--timeout_ms;
		HAL_Delay(1);
		}

	return (spi_type == IDLE);
}

//
// @brief Подождать окончания записи EEPROM (см. WIP-бит в STATUS регистре).
// @param timeout_ms - время ожидания в мс.
// @ret результат ожидания: true - дождались окончания записи или false - вышел таймаут.
//
bool m95m01_eeprom_wait_ready(uint16_t timeout_ms)
{
	m95m01_eeprom_wait_spi();

	for(;;)
	{
		m95m01_eeprom_read_status_reg();
		m95m01_eeprom_wait_spi();

		if(!status_reg.bits.wip || !timeout_ms)
			break;

		--timeout_ms;
		HAL_Delay(1);
	}

	return (!status_reg.bits.wip);
}

//
// @brief Записать данные по указанному адресу указанной длины (не более 256 байт в пределах страницы).
//        Не блокирующая функция.
// @param *data - указатель на данные для записи
// @param addr - адрес, с которого начнется запись
// @param size - длина данных в байтах, не более 256 байт в пределах страницы
// @ret none
//
void m95m01_eeprom_write_page(uint8_t *data, uint32_t addr, uint16_t size)
{
	// WREN, WRITE
	// One page (256 bytes) can be written with a single WRITE instruction
	m95m01_eeprom_wait_ready(10);

	tx_buf[0] = WREN_INSTR;
	tx_buf[1] = addr >> 16;
	tx_buf[2] = addr >> 8;
	tx_buf[3] = addr & 0xFF;
	tx_size = size > 256 ? 256 : size;
	memmove((uint8_t*)&tx_buf[4], data, tx_size);

	spi_type = WRITE_DATA;

	m95m01_eeprom_cs(1);
	NOHAL_SPI_TransmitReceive_IT(tx_buf, rx_buf, 1);
}

// @brief Записать данные по указанному адресу (любому адресу) указанной длины (любой длины).
//        Функция пишет данные постранично, автоматически следит за размерами страниц.
// @param *data - указатель на данные для записи
// @param addr - адрес, с которого начнется запись
// @param size - длина данных в байтах
// @param wait - дождаться окончания записи данных (true\false)
// @ret none
//
void m95m01_eeprom_write(uint8_t *data, uint32_t addr, uint32_t size, bool wait)
{
	// Вычисляем границу первой записываемой страницы
	uint32_t n_byte = 0;
	uint32_t size_limit = 0;										// граница записи для текущей страницы, в байтах
	uint32_t n_page = addr / EEP_PAGE_SIZE;							// страница, в которую будет идти запись
	size_limit = EEP_PAGE_SIZE - (addr - n_page * EEP_PAGE_SIZE);

	// Пишем первую страницу до границы
	if(size <= size_limit)
	{
		m95m01_eeprom_write_page(data, addr, size);

		if(wait)
			m95m01_eeprom_wait_ready(10);
		return;
	}
	else
	{
		m95m01_eeprom_write_page(data, addr, size_limit);
		n_byte += size_limit;
		size -= size_limit;
	}

	// Пишем оставшиеся страницы
	while(size)
	{
		if(size <= EEP_PAGE_SIZE)
		{
			m95m01_eeprom_write_page(data + n_byte, addr + n_byte, size);

			if(wait)
				m95m01_eeprom_wait_ready(10);
			return;
		}
		else
		{
			m95m01_eeprom_write_page(data + n_byte, addr + n_byte, EEP_PAGE_SIZE);
			n_byte += EEP_PAGE_SIZE;
			size -= EEP_PAGE_SIZE;
		}
	}

	if(wait)
		m95m01_eeprom_wait_ready(10);
}

//
// @brief Прочитать данные по указанному адресу (любому адресу) указанной длины (любой длины).
// @param *data - указатель на буфер, куда будут скопированы данные после чтения
// @param addr - адрес, с которого начнется чтение
// @param size - длина данных в байтах
// @param wait - дождаться окончания чтения данных (true\false)
// @ret none
//
void m95m01_eeprom_read(uint8_t *data, uint32_t addr, uint32_t size, bool wait)
{
	// READ, 24 bit address, Data
	// The whole memory can be read with a single READ instruction.
	m95m01_eeprom_wait_ready(10);

	tx_buf[0] = READ_INSTR;
	tx_buf[1] = addr >> 16;
	tx_buf[2] = addr >> 8;
	tx_buf[3] = addr & 0xFF;
	rx_size = size;
	spi_type = READ_DATA;

	m95m01_eeprom_cs(1);
	NOHAL_SPI_TransmitReceive_IT(tx_buf, data, 4 + rx_size);

	if(wait)
		m95m01_eeprom_wait_spi();
}

//
// @brief Стереть заданное число страниц. Блокирующая функция.
// @param addr - адрес, с которого начнется стирание
// @param n_pages - число 256-байтных страниц
// @ret результат операции
//
bool m95m01_eeprom_erase(uint32_t addr, uint32_t n_pages)
{
	if(!n_pages)
		return false;

	m95m01_eeprom_wait_ready(10);

	uint32_t address = addr;
	memset(tx_buf, 0xFF, sizeof(tx_buf));

	for(uint32_t i = 0; i < n_pages; ++i)
		{
		// WREN, WRITE
		// One page (256 bytes) can be written with a single WRITE instruction
		tx_buf[0] = WREN_INSTR;
		tx_buf[1] = address >> 16;
		tx_buf[2] = address >> 8;
		tx_buf[3] = address & 0xFF;
		tx_size = 256;

		spi_type = WRITE_DATA;

		m95m01_eeprom_cs(1);
		NOHAL_SPI_TransmitReceive_IT(tx_buf, rx_buf, 1);

		if(!m95m01_eeprom_wait_ready(10))
			return false;

		address += 256;
		}

	return true;
}

//
// @brief Управляет пином CS для m95m01
// @param select_device - true = select device (set cs to low level), false = deselect device (set cs to high level)
// @ret none
//
void m95m01_eeprom_cs(bool select_device)
{
	if(select_device)
		HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET);
}

//
// @brief Управляет пином HOLD для m95m01.
//        HOLD-режим активен, когда HOLD pin = 0.
// @param hold - true = (reset pin to low level), false = (set pin to high level)
// @ret none
//
void m95m01_eeprom_hold(bool hold)
{
	if(hold)
		HAL_GPIO_WritePin(EEPROM_HOLD_GPIO_Port, EEPROM_HOLD_Pin, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(EEPROM_HOLD_GPIO_Port, EEPROM_HOLD_Pin, GPIO_PIN_SET);
}

//
// @brief Вызывается после окончания приемопередачи по SPI.
//        Не использует HAL-драйвера.
// @param none
// @ret none
//
void NOHAL_SPI_TxRxCpltCallback()
{
	m95m01_eeprom_cs(0);

	switch(spi_type)
		{
		case WRITE_STATUS_REGISTER:
			tx_buf[0] = WRSR_INSTR;
			tx_buf[1] = status_reg.byte;

			spi_type = WRITE_STATUS_REGISTER_2;

			m95m01_eeprom_cs(1);
			NOHAL_SPI_TransmitReceive_IT(tx_buf, rx_buf, 2);
			break;

		case WRITE_STATUS_REGISTER_2:
			spi_type = IDLE;
			break;

		case READ_STATUS_REGISTER:
			status_reg.byte = rx_buf[1];
			spi_type = IDLE;
			break;

		case WRITE_DATA:
			tx_buf[0] = WRITE_INSTR;

			spi_type = WRITE_DATA_2;

			m95m01_eeprom_cs(1);
			NOHAL_SPI_TransmitReceive_IT(tx_buf, rx_buf, 4+tx_size);
			break;

		case WRITE_DATA_2:
			spi_type = IDLE;
			break;

		case READ_DATA:
			spi_type = IDLE;
			break;

		default:
			spi_type = IDLE;
			break;
		}
}
