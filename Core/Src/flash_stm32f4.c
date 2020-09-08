/*
 * flash_stm32f4.c
 */

#include "flash_stm32f4.h"
#include "stm32f4xx_hal.h"

static uint32_t GetSector(uint32_t Address);
static uint32_t GetSectorSize(uint32_t Sector);

static uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
static uint32_t SectorError = 0;
static uint32_t EndAddr;
static FLASH_EraseInitTypeDef EraseInitStruct;

/**
 * @brief  Записывает данные во флэш память (по 4 байта за раз).
 * @param  Data64: указатель а записываемые данные
 * @param  SizeBytes: размер записываем данных в байтах
 * @param  AddrStart: адрес, по которому начнется запись данных
 * @retval none
 */
void flash_stm32_write(const uint32_t *Data32, const uint32_t SizeBytes, const uint32_t AddrStart)
{
	// Unlock the Flash to enable the flash control register access
	HAL_FLASH_Unlock();

	// Erase the user Flash area

//	/* Get the 1st sector to erase */
//	FirstSector = GetSector(AddrStart);
//
//	/* Get the number of sector to erase from 1st sector*/
//	NbOfSectors = GetSector(AddrStart + SizeBytes) - FirstSector + 1;
//
//	/* Fill EraseInit structure*/
//	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
//	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
//	EraseInitStruct.Sector = FirstSector;
//	EraseInitStruct.NbSectors = NbOfSectors;
//
//	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
//	{
//		/*
//		 FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
//		 */
//		HAL_FLASH_Lock();
//		return;
//	}
//
//	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
//	 you have to make sure that these data are rewritten before they are accessed during code
//	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
//	 DCRST and ICRST bits in the FLASH_CR register. */
//	__HAL_FLASH_DATA_CACHE_DISABLE();
//	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
//
//	__HAL_FLASH_DATA_CACHE_RESET();
//	__HAL_FLASH_INSTRUCTION_CACHE_RESET();
//
//	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
//	__HAL_FLASH_DATA_CACHE_ENABLE();

	/* Program the user Flash area word by word */
	EndAddr = AddrStart + GetSectorSize(AddrStart) - 1;
	Address = AddrStart;

	while(Address < (SizeBytes + AddrStart))
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, Data32[(Address - AddrStart) >> 2]) == HAL_OK)
			Address = Address + 4;
		else
			break;

		if(Address >= EndAddr)
			break;
	}

	// Lock the Flash to disable the flash control register access (recommended
	// to protect the FLASH memory against possible unwanted operation)
	HAL_FLASH_Lock();
}

/**
 * @brief  Стирает флэш память по секторам.
 * @param  SizeBytes: размер стираемых данных в байтах (если размер больше одного сектора, то сотрется и следующий сектор)
 * @param  AddrStart: адрес начала или внутри сектора, который будет стираться
 * @retval none
 */
void flash_stm32_erase(const uint32_t SizeBytes, const uint32_t AddrStart)
{
	// Unlock the Flash to enable the flash control register access
	HAL_FLASH_Unlock();

	// Erase the user Flash area

	/* Get the 1st sector to erase */
	FirstSector = GetSector(AddrStart);

	/* Get the number of sector to erase from 1st sector*/
	NbOfSectors = GetSector(AddrStart + SizeBytes) - FirstSector + 1;

	/* Fill EraseInit structure*/
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	EraseInitStruct.Sector = FirstSector;
	EraseInitStruct.NbSectors = NbOfSectors;

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	{
		/*
		 FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
		 */
		HAL_FLASH_Lock();
		return;
	}

	/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	 you have to make sure that these data are rewritten before they are accessed during code
	 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	 DCRST and ICRST bits in the FLASH_CR register. */
	__HAL_FLASH_DATA_CACHE_DISABLE();
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	__HAL_FLASH_DATA_CACHE_RESET();
	__HAL_FLASH_INSTRUCTION_CACHE_RESET();

	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	__HAL_FLASH_DATA_CACHE_ENABLE();

	// Lock the Flash to disable the flash control register access (recommended
	// to protect the FLASH memory against possible unwanted operation)
	HAL_FLASH_Lock();
}

/**
 * @brief  Записывает данные во флэш память по байтам.
 *         Может записывать предварительно не стирая память.
 * @param  Data8: указатель а записываемые данные
 * @param  SizeBytes: размер записываем данных в байтах
 * @param  AddrStart: адрес, по которому начнется запись данных
 * @param  EraseBeforeWrite: true = стереть память перед записью
 * @retval none
 */
void flash_stm32_write_byte(const uint8_t *Data8, const uint32_t SizeBytes, const uint32_t AddrStart, bool EraseBeforeWrite)
{
	// Unlock the Flash to enable the flash control register access
	HAL_FLASH_Unlock();

	// Erase the user Flash area

	if(EraseBeforeWrite)
	{
		/* Get the 1st sector to erase */
		FirstSector = GetSector(AddrStart);

		/* Get the number of sector to erase from 1st sector*/
		NbOfSectors = GetSector(AddrStart + SizeBytes) - FirstSector + 1;

		/* Fill EraseInit structure*/
		EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
		EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
		EraseInitStruct.Sector = FirstSector;
		EraseInitStruct.NbSectors = NbOfSectors;

		if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
		{
			/*
			 FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
			 */
			HAL_FLASH_Lock();
			return;
		}

		/* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
		 you have to make sure that these data are rewritten before they are accessed during code
		 execution. If this cannot be done safely, it is recommended to flush the caches by setting the
		 DCRST and ICRST bits in the FLASH_CR register. */
		__HAL_FLASH_DATA_CACHE_DISABLE();
		__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

		__HAL_FLASH_DATA_CACHE_RESET();
		__HAL_FLASH_INSTRUCTION_CACHE_RESET();

		__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
		__HAL_FLASH_DATA_CACHE_ENABLE();
	}

	/* Program the user Flash area word by word */
	EndAddr = AddrStart + GetSectorSize(AddrStart) - 1;
	Address = AddrStart;

	while(Address < (SizeBytes + AddrStart))
	{
		if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, Address, Data8[Address - AddrStart]) == HAL_OK)
			++Address;
		else
			break;

		if(Address >= EndAddr)
			break;
	}

	// Lock the Flash to disable the flash control register access (recommended
	// to protect the FLASH memory against possible unwanted operation)
	HAL_FLASH_Lock();
}

/**
 * @brief  Читает данные из флэш памяти (по 4 байта за раз)
 * @param  Data32: указатель на область в памяти, куда будут считаны данные
 * @param  SizeBytes: размер читаемых данных в байтах
 * @param  AddrStart: адрес, по которому начнется чтение данных
 * @retval none
 */
void flash_stm32_read(uint32_t *Data32, const uint32_t SizeBytes, const uint32_t AddrStart)
{
	uint32_t Address = AddrStart;

	while(Address < (SizeBytes + AddrStart))
	{
		Data32[(Address - AddrStart) >> 2] = *(__IO uint32_t *)Address;
		Address += 4;
	}
}

/**
 * @brief  Читает данные из флэш памяти (по байту за раз)
 * @param  Data8: указатель на область в памяти, куда будут считаны данные
 * @param  SizeBytes: размер читаемых данных в байтах
 * @param  AddrStart: адрес, по которому начнется чтение данных
 * @retval none
 */
void flash_stm32_read_byte(uint8_t *Data8, const uint32_t SizeBytes, const uint32_t AddrStart)
{
	uint32_t Address = AddrStart;

	while(Address < (SizeBytes + AddrStart))
	{
		Data8[Address - AddrStart] = *(__IO uint8_t *)Address;
		++Address;
	}
}

/**
 * @brief  Gets the sector of a given address
 * @param  None
 * @retval The sector of a given address
 */
static uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;

	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
		sector = FLASH_SECTOR_0;
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
		sector = FLASH_SECTOR_1;
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
		sector = FLASH_SECTOR_2;
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
		sector = FLASH_SECTOR_3;
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
		sector = FLASH_SECTOR_4;
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
		sector = FLASH_SECTOR_5;
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
		sector = FLASH_SECTOR_6;
	}
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
		sector = FLASH_SECTOR_7;
	}
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
		sector = FLASH_SECTOR_8;
	}
	else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
		sector = FLASH_SECTOR_9;
	}
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
		sector = FLASH_SECTOR_10;
	}
	else if((Address < ADDR_FLASH_SECTOR_12) && (Address >= ADDR_FLASH_SECTOR_11))
	{
		sector = FLASH_SECTOR_11;
	}
	else if((Address < ADDR_FLASH_SECTOR_13) && (Address >= ADDR_FLASH_SECTOR_12))
	{
		sector = FLASH_SECTOR_12;
	}
	else if((Address < ADDR_FLASH_SECTOR_14) && (Address >= ADDR_FLASH_SECTOR_13))
	{
		sector = FLASH_SECTOR_13;
	}
	else if((Address < ADDR_FLASH_SECTOR_15) && (Address >= ADDR_FLASH_SECTOR_14))
	{
		sector = FLASH_SECTOR_14;
	}
	else if((Address < ADDR_FLASH_SECTOR_16) && (Address >= ADDR_FLASH_SECTOR_15))
	{
		sector = FLASH_SECTOR_15;
	}
	else if((Address < ADDR_FLASH_SECTOR_17) && (Address >= ADDR_FLASH_SECTOR_16))
	{
		sector = FLASH_SECTOR_16;
	}
	else if((Address < ADDR_FLASH_SECTOR_18) && (Address >= ADDR_FLASH_SECTOR_17))
	{
		sector = FLASH_SECTOR_17;
	}
	else if((Address < ADDR_FLASH_SECTOR_19) && (Address >= ADDR_FLASH_SECTOR_18))
	{
		sector = FLASH_SECTOR_18;
	}
	else if((Address < ADDR_FLASH_SECTOR_20) && (Address >= ADDR_FLASH_SECTOR_19))
	{
		sector = FLASH_SECTOR_19;
	}
	else if((Address < ADDR_FLASH_SECTOR_21) && (Address >= ADDR_FLASH_SECTOR_20))
	{
		sector = FLASH_SECTOR_20;
	}
	else if((Address < ADDR_FLASH_SECTOR_22) && (Address >= ADDR_FLASH_SECTOR_21))
	{
		sector = FLASH_SECTOR_21;
	}
	else if((Address < ADDR_FLASH_SECTOR_23) && (Address >= ADDR_FLASH_SECTOR_22))
	{
		sector = FLASH_SECTOR_22;
	}
	else/*(Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_23))*/
	{
		sector = FLASH_SECTOR_23;
	}

	return sector;
}

/**
 * @brief  Gets sector Size
 * @param  None
 * @retval The size of a given sector
 */
static uint32_t GetSectorSize(uint32_t Sector)
{
	uint32_t sectorsize = 0x00;
	if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3) || (Sector == FLASH_SECTOR_12)
			|| (Sector == FLASH_SECTOR_13) || (Sector == FLASH_SECTOR_14) || (Sector == FLASH_SECTOR_15))
	{
		sectorsize = 16 * 1024;
	}
	else if((Sector == FLASH_SECTOR_4) || (Sector == FLASH_SECTOR_16))
	{
		sectorsize = 64 * 1024;
	}
	else
	{
		sectorsize = 128 * 1024;
	}
	return sectorsize;
}
