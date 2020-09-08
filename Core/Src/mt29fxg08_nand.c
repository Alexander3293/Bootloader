/*
 * mt29f2g08_nand.c
 */

#include <mt29fxg08_nand.h>
#include "memory.h"
#include "string.h"
#include "stdbool.h"

#include "CRC16.h"
//#include "test_func.h"
//#include "debug_logger.h"

/*
 * Распределение NAND-памяти:
 * BLOCK 0 - блок под block_table
 * Таблица из NAND_MEMORY_SIZE_BLOCKS байт - блоков: блок валиден - 1, не валиден - 0.
 *
 * 	// todo Для некоторых микросхем надо использовать R\B2 и CE2 для второй области, переключаем плис
 * 	HAL_GPIO_WritePin(NAND_MUX_GPIO_Port, NAND_MUX_Pin, (NAND_Address.Block > 4095) ? GPIO_PIN_SET : GPIO_PIN_RESET);
 *
 * 	//todo Возможно для некоторых микросхем размер block_table будет более одной страницы, в этом случае
 * 	нужно сохранять block_table в несколько страниц
 */

extern NAND_HandleTypeDef hnand1;
extern CRC_HandleTypeDef hcrc;
extern DMA_HandleTypeDef hdma_memtomem_dma2_stream0;

extern BlockTable_t BlockTable_NAND;		// таблица блоков, хранящаяся в NAND. Обновляется только при стирании памяти.

uint8_t ss;

uint8_t page_temp[NAND_PAGE_SIZE_BYTES];
static bool nand_is_full = false;

CSpare spare;	// spare page area
PAGE_BUFF NAND_PAGE_R_BUF = {0}; //buffer for nand page

__weak void memory_full_callback()
{
}

/* @brief Переключает 'Target' (die\LUNs) для NAND памяти.
 *        (Переключает R/B и CE пины для NAND).
 *        NAND_MUX_pin в CPLD переключает RB1 -> RB2 и CE1 -> CE2
 */
void nand_select_target(uint8_t target)
{
	if(target)
		{
		// R\B2, CE2
		HAL_GPIO_WritePin(NAND_MUX_GPIO_Port, NAND_MUX_Pin, GPIO_PIN_SET);
		}
	else
		{
		// R\B1, CE1
		HAL_GPIO_WritePin(NAND_MUX_GPIO_Port, NAND_MUX_Pin, GPIO_PIN_RESET);
		}
}

static bool check_address_nand_full(uint64_t address)
{
	if(address >= NAND_MEMORY_SIZE_BYTES)
		{
		memory_full_callback();
		nand_is_full = true;
		}
	else
		nand_is_full = false;

	return nand_is_full;
}

HAL_StatusTypeDef HAL_NAND_Read_Page_8b_fast(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumPageToRead)
{
//	  __IO uint32_t index  = 0U;
	  uint32_t tickstart = 0U;
	  uint32_t deviceaddress = 0U, size = 0U, numPagesRead = 0U, nandaddress = 0U;

	  /* Process Locked */
	  __HAL_LOCK(hnand);

	  /* Check the NAND controller state */
	  if(hnand->State == HAL_NAND_STATE_BUSY)
	  {
	     return HAL_BUSY;
	  }

	  /* Identify the device address */
	  if(hnand->Init.NandBank == FMC_NAND_BANK2)
	  {
	    deviceaddress = NAND_DEVICE1;
	  }
	  else
	  {
	    deviceaddress = NAND_DEVICE2;
	  }

	  /* Update the NAND controller state */
	  hnand->State = HAL_NAND_STATE_BUSY;

	  /* NAND raw address calculation */
	  nandaddress = ARRAY_ADDRESS(pAddress, hnand);

	  /* Page(s) read loop */
	  while((NumPageToRead != 0U) && (nandaddress < ((hnand->Config.BlockSize) * (hnand->Config.BlockNbr))))
	  {
	    /* update the buffer size */
	    size = (hnand->Config.PageSize) + ((hnand->Config.PageSize) * numPagesRead);

	    /* Send read page command sequence */
	    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_A;

	    /* Cards with page size <= 512 bytes */
	    if((hnand->Config.PageSize) <= 512U)
	    {
	      if (((hnand->Config.BlockSize)*(hnand->Config.BlockNbr)) <= 65535U)
	      {
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = 0x00;
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
	      }
	      else /* ((hnand->Config.BlockSize)*(hnand->Config.BlockNbr)) > 65535 */
	      {
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = 0x00;
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
	      }
	    }
	    else /* (hnand->Config.PageSize) > 512 */
	    {
	      if (((hnand->Config.BlockSize)*(hnand->Config.BlockNbr)) <= 65535U)
	      {
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = 0x00;
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = 0x00;
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
	      }
	      else /* ((hnand->Config.BlockSize)*(hnand->Config.BlockNbr)) > 65535 */
	      {
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = 0x00;
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = 0x00;
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
	        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
	      }
	    }

	    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA))  = NAND_CMD_AREA_TRUE1;

	    /* Check if an extra command is needed for reading pages  */
	    if(hnand->Config.ExtraCommandEnable == ENABLE)
	    {
	      /* Get tick */
	      tickstart = HAL_GetTick();

	      /* Read status until NAND is ready */
	      while(HAL_NAND_Read_Status(hnand) != NAND_READY)
	      {
	        if((HAL_GetTick() - tickstart ) > NAND_WRITE_TIMEOUT)
	        {
	          return HAL_TIMEOUT;
	        }
	      }

	      /* Go back to read mode */
	      *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = ((uint8_t)0x00);
	      __DSB();
	    }

	    /* Get Data into Buffer */
	//    for(; index < size; index++)
	//    {
	//      *(uint8_t *)pBuffer++ = *(uint8_t *)deviceaddress;
	//    }
	    //HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port,DEBUG_PIN_Pin, 1);
	    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)deviceaddress, (uint32_t)pBuffer, size);
		HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0,HAL_DMA_FULL_TRANSFER, 1000000);
		//HAL_GPIO_WritePin(DEBUG_PIN_GPIO_Port,DEBUG_PIN_Pin, 0);
	    //HAL_DMA_Start_IT(&hdma_memtomem_dma2_stream0, (uint8_t *)deviceaddress, (uint8_t*)pBuffer, size);
	    /* Increment read pages number */
	    numPagesRead++;

	    /* Decrement pages to read */
	    NumPageToRead--;

	    /* Increment the NAND address */
	    nandaddress = (uint32_t)(nandaddress + 1U);
	  }

	  /* Update the NAND controller state */
	  hnand->State = HAL_NAND_STATE_READY;

	  /* Process unlocked */
	  __HAL_UNLOCK(hnand);

	  return HAL_OK;
	}

HAL_StatusTypeDef HAL_NAND_Read_SpareArea_8b_fast(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumSpareAreaToRead)
{
//  __IO uint32_t index = 0U;
  uint32_t tickstart = 0U;
  uint32_t deviceaddress = 0U, size = 0U, numSpareAreaRead = 0U, nandaddress = 0U, columnaddress = 0U;

  /* Process Locked */
  __HAL_LOCK(hnand);

  /* Check the NAND controller state */
  if(hnand->State == HAL_NAND_STATE_BUSY)
  {
     return HAL_BUSY;
  }

  /* Identify the device address */
  if(hnand->Init.NandBank == FMC_NAND_BANK2)
  {
    deviceaddress = NAND_DEVICE1;
  }
  else
  {
    deviceaddress = NAND_DEVICE2;
  }

  /* Update the NAND controller state */
  hnand->State = HAL_NAND_STATE_BUSY;

  /* NAND raw address calculation */
  nandaddress = ARRAY_ADDRESS(pAddress, hnand);

  /* Column in page address */
  columnaddress = COLUMN_ADDRESS(hnand);

  /* Spare area(s) read loop */
  while((NumSpareAreaToRead != 0U) && (nandaddress < ((hnand->Config.BlockSize) * (hnand->Config.BlockNbr))))
  {
    /* update the buffer size */
    size = (hnand->Config.SpareAreaSize) + ((hnand->Config.SpareAreaSize) * numSpareAreaRead);

    /* Cards with page size <= 512 bytes */
    if((hnand->Config.PageSize) <= 512U)
    {
      /* Send read spare area command sequence */
      *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_C;

      if (((hnand->Config.BlockSize)*(hnand->Config.BlockNbr)) <= 65535U)
      {
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = 0x00;
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
      }
      else /* ((hnand->Config.BlockSize)*(hnand->Config.BlockNbr)) > 65535 */
      {
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = 0x00;
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
      }
    }
    else /* (hnand->Config.PageSize) > 512 */
    {
      /* Send read spare area command sequence */
      *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_A;

      if (((hnand->Config.BlockSize)*(hnand->Config.BlockNbr)) <= 65535U)
      {
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_1ST_CYCLE(columnaddress);
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_2ND_CYCLE(columnaddress);
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
      }
      else /* ((hnand->Config.BlockSize)*(hnand->Config.BlockNbr)) > 65535 */
      {
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_1ST_CYCLE(columnaddress);
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = COLUMN_2ND_CYCLE(columnaddress);
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_1ST_CYCLE(nandaddress);
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_2ND_CYCLE(nandaddress);
        *(__IO uint8_t *)((uint32_t)(deviceaddress | ADDR_AREA)) = ADDR_3RD_CYCLE(nandaddress);
      }
    }

    *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = NAND_CMD_AREA_TRUE1;

    if(hnand->Config.ExtraCommandEnable == ENABLE)
    {
      /* Get tick */
      tickstart = HAL_GetTick();

      /* Read status until NAND is ready */
      while(HAL_NAND_Read_Status(hnand) != NAND_READY)
      {
        if((HAL_GetTick() - tickstart ) > NAND_WRITE_TIMEOUT)
        {
          return HAL_TIMEOUT;
        }
      }

      /* Go back to read mode */
      *(__IO uint8_t *)((uint32_t)(deviceaddress | CMD_AREA)) = ((uint8_t)0x00);
    }

    /* Get Data into Buffer */
//    for(; index < size; index++)
//    {
//      *(uint8_t *)pBuffer++ = *(uint8_t *)deviceaddress;
//    }

    HAL_DMA_Start(&hdma_memtomem_dma2_stream0, (uint32_t)deviceaddress, (uint32_t)pBuffer, size);

	HAL_DMA_PollForTransfer(&hdma_memtomem_dma2_stream0,HAL_DMA_FULL_TRANSFER, 1000000);

    /* Increment read spare areas number */
    numSpareAreaRead++;

    /* Decrement spare areas to read */
    NumSpareAreaToRead--;

    /* Increment the NAND address */
    nandaddress = (uint32_t)(nandaddress + 1U);
  }

  /* Update the NAND controller state */
  hnand->State = HAL_NAND_STATE_READY;

  /* Process unlocked */
  __HAL_UNLOCK(hnand);

  return HAL_OK;
}

/**
  * @brief  Переводит логические адрес в физический для nand
  * @param  Address
  * @param  pNandAddress
  */
void NAND_GetAddress(uint64_t Address, NAND_AddressTypeDef *pNandAddress)
{
	// Защита от выхода за физические границы nand-памяти
	if(Address >= NAND_MEMORY_SIZE_BYTES)
		Address = NAND_MEMORY_SIZE_BYTES - NAND_PAGE_SIZE_BYTES;

	pNandAddress->Page  = (Address % NAND_BLOCK_SIZE_BYTES)
					/ NAND_PAGE_SIZE_BYTES;
	pNandAddress->Block = (Address % (NAND_PLANE_SIZE_BLOCKS * NAND_BLOCK_SIZE_BYTES))
					/ NAND_BLOCK_SIZE_BYTES;
	pNandAddress->Plane = Address
					/ (NAND_PLANE_SIZE_BLOCKS * NAND_BLOCK_SIZE_BYTES);
}

/* @brief  Корректирует адрес пропуская битые блоки.
 * @param  [in] address - адрес в байтах
 * @param  [out] *pNAND_Address - указатель на структуру адреса, который будет скорректирован
 * @retval none
 * */
static void nand_correct_address(const uint64_t address, NAND_AddressTypeDef *pNAND_Address)
{
	uint64_t corrected_address = address;
	// Корректируем номер блока пропуская битые блоки
	for(uint16_t i = NAND_DATA_ADDRESS_BLOCKS; i <= (pNAND_Address->Plane * NAND_PLANE_SIZE_BLOCKS + pNAND_Address->Block); ++i)
		{
		if(!BlockTable_NAND.block_table[i])
			{
			corrected_address += NAND_BLOCK_SIZE_BYTES;
			if(check_address_nand_full(corrected_address))
				return;
			NAND_GetAddress(corrected_address, pNAND_Address);
			}
		}
}

/* @brief  Смещает адрес на начало следующего блока nand памяти.
 * @param  [in] address - адрес в байтах
 * @retval адрес начала следующего блока, в байтах
 * */
static uint64_t nand_skip_address_to_next_block(uint64_t address)
{
	NAND_AddressTypeDef NAND_Address;

	// Смещаем адрес на начало следующего блока
	address += NAND_BLOCK_SIZE_BYTES;

	if(check_address_nand_full(address))
		return address;

	NAND_GetAddress(address, &NAND_Address);
	NAND_Address.Page = 0;

	address = (NAND_Address.Plane * NAND_PLANE_SIZE_BLOCKS + NAND_Address.Block) * NAND_BLOCK_SIZE_BYTES;
	return address;
}

eccdiff_t crc32_check_ecc()
{
	uint8_t ecc_val[3];
	eccdiff_t status;

	// Считаем ecc для crc32 из spare-области
	//memset(temp, 0, 512);//~80us
	memset(page_temp, 0x00, 512);
	memcpy(page_temp, (uint8_t*)&spare.crc32, sizeof(spare.crc32));

	make_ecc((uint8_t*)&ecc_val, page_temp);//~380us

	// Проверяем\исправляем crc32 из spare-области
	status = ecc_check(page_temp, spare.crc32_ecc, ecc_val);

	if(status == ECC_CORRECTABLE_ERROR)
		memcpy((uint8_t*)&spare.crc32, page_temp, sizeof(spare.crc32));

	return status;
}

// Write read erase nand memory

/*
 * @brief  Пишет страницу nand памяти. Автоматически пропускает битые блоки, используя таблицу блоков.
 *         В случае неудачи пробует повторить запись в первую страницу следующего блока nand памяти.
 * @param  [in] *data - указатель на данные для записи
 * @param  [in] address - адрес страницы для записи в nand память, в байтах (без учета битых блоков).
 * @retval [out] address - адрес, по которому была записана страница с данными, в байтах (без учета битых блоков).
 *
 * @note   Адреса [in] и [out] могут отличаться друг от друга, если записать страницу в nand память с первого раза не удалось.
 * */
uint64_t nand_write_page(uint8_t *data, uint64_t address)
{
	HAL_StatusTypeDef status;
	NAND_AddressTypeDef NAND_Address;

	// Считаем ecc для данных в странице
	for(uint8_t i = 0; i < NAND_PAGE_SIZE_BYTES/512; ++i)
		make_ecc((uint8_t*)&spare.page_ecc[i], (uint8_t*)&data[i*512]);

	// Считаем crc для данных в странице
	spare.crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, NAND_PAGE_SIZE_BYTES/4);

	// Считаем ecc для crc32
	memset(page_temp, 0x00, 512);
	memcpy(page_temp, (uint8_t*)&spare.crc32, sizeof(spare.crc32));
	make_ecc(spare.crc32_ecc, page_temp);

	// Пишем страницу
	status = HAL_BUSY;

	while(status != HAL_OK)
		{
		if(check_address_nand_full(address))
			return CODE_NAND_FULL;

		// Вычисляем адрес для записи страницы
		NAND_GetAddress(address, &NAND_Address);

		// Корректируем адрес пропуская битые блоки
		nand_correct_address(address, &NAND_Address);
		if(nand_is_full)
			return address;

		// Пишем данные
		status = HAL_NAND_Write_Page_8b(&hnand1, &NAND_Address, data, 1);

		if(status != HAL_OK)
			{
			// в случае ошибки записи: маркируем блок битым в BlockTable_EEP
			eeprom_block_table_mark_block(NAND_Address.Block + NAND_PLANE_SIZE_BLOCKS * NAND_Address.Plane, false);
			// обновляем BlockTable в EEPROM
			eeprom_block_table_write();
			// пишем страницу с начала следующего блока
			address = nand_skip_address_to_next_block(address);
			if(nand_is_full)
				return address;

			continue;
			}

		// Пишем spare
		status = HAL_NAND_Write_SpareArea_8b(&hnand1, &NAND_Address, (uint8_t*)&spare, 1);
		if(status != HAL_OK)
			{
			// в случае ошибки записи: маркируем блок битым в BlockTable_EEP
			eeprom_block_table_mark_block(NAND_Address.Block + NAND_PLANE_SIZE_BLOCKS * NAND_Address.Plane, false);
			// обновляем BlockTable в EEPROM
			eeprom_block_table_write();
			// пишем страницу с начала следующего блока
			address = nand_skip_address_to_next_block(address);
			if(nand_is_full)
				return address;

			continue;
			}
		}

	return address;
}
//
//uint64_t nand_write_ff_page(uint64_t address)
//{
//	HAL_StatusTypeDef status;
//	NAND_AddressTypeDef NAND_Address;
//
//	memset(page_temp, 0xff, sizeof(page_temp));
//
//	// Пишем страницу
//	status = HAL_BUSY;
//
//	while(status != HAL_OK)
//		{
//		if(check_address_nand_full(address))
//			return address;
//
//		// Вычисляем адрес для записи страницы
//		NAND_GetAddress(address, &NAND_Address);
//
//		// Корректируем адрес пропуская битые блоки
//		nand_correct_address(address, &NAND_Address);
//		if(nand_is_full)
//			return address;
//
//		// Пишем данные
//		status = HAL_NAND_Write_Page_8b(&hnand1, &NAND_Address, page_temp, 1);
//
//		if(status != HAL_OK)
//			{
//			// в случае ошибки записи: маркируем блок битым в BlockTable_EEP
//			eeprom_block_table_mark_block(NAND_Address.Block + NAND_PLANE_SIZE_BLOCKS * NAND_Address.Plane, false);
//			// обновляем BlockTable в EEPROM
//			eeprom_block_table_write();
//			// пишем страницу с начала следующего блока
//			address = nand_skip_address_to_next_block(address);
//			if(nand_is_full)
//				return address;
//
//			continue;
//			}
//
//		// Пишем spare
//		status = HAL_NAND_Write_SpareArea_8b(&hnand1, &NAND_Address, page_temp, 1);
//		if(status != HAL_OK)
//			{
//			// в случае ошибки записи: маркируем блок битым в BlockTable_EEP
//			eeprom_block_table_mark_block(NAND_Address.Block + NAND_PLANE_SIZE_BLOCKS * NAND_Address.Plane, false);
//			// обновляем BlockTable в EEPROM
//			eeprom_block_table_write();
//			// пишем страницу с начала следующего блока
//			address = nand_skip_address_to_next_block(address);
//			if(nand_is_full)
//				return address;
//
//			continue;
//			}
//		}
//
//	return address;
//}

HAL_StatusTypeDef nand_write_page_dummy(uint8_t *data, uint64_t address)
{
	HAL_StatusTypeDef status;
	NAND_AddressTypeDef NAND_Address;

	NAND_GetAddress(address, &NAND_Address);
	// Пишем страницу
	status = HAL_NAND_Write_Page_8b(&hnand1, &NAND_Address, data, 1);
	if(status != HAL_OK)
		{}
	return status;
}

HAL_StatusTypeDef nand_write_spare_dummy(uint8_t *data, uint64_t address)
{
	HAL_StatusTypeDef status;
	NAND_AddressTypeDef NAND_Address;
    uint64_t addr = address;
	NAND_GetAddress(addr, &NAND_Address);
	// Пишем страницу
	status = HAL_NAND_Write_SpareArea_8b(&hnand1, &NAND_Address, data, 1);
	return status;
}


bool nand_read_page_bufferezied(uint8_t *data, uint64_t address)
{
	bool res = false;
	if(address == NAND_PAGE_R_BUF.address)
	{
		res = true;
	}
	else
	{
		res = nand_read_page(&NAND_PAGE_R_BUF.data[0], address);
	}

	if(res)
		memcpy(data, &NAND_PAGE_R_BUF.data[0], NAND_PAGE_SIZE_BYTES);

	return res;
}

bool nand_read_page(uint8_t *data, uint64_t address)
{
	HAL_StatusTypeDef status;
	NAND_AddressTypeDef NAND_Address;
	uint8_t ecc_val[3];

	NAND_GetAddress(address, &NAND_Address);

	// Корректируем номер блока пропуская битые блоки
	uint64_t corrected_address = address;
	for(uint16_t i = NAND_DATA_ADDRESS_BLOCKS; i <= (NAND_Address.Plane * NAND_PLANE_SIZE_BLOCKS + NAND_Address.Block); ++i)
	{
		if(!BlockTable_NAND.block_table[i])
		{
			corrected_address += NAND_BLOCK_SIZE_BYTES;
			NAND_GetAddress(corrected_address, &NAND_Address);
		}
	}

	// Читаем страницу
	status = HAL_NAND_Read_Page_8b_fast(&hnand1, &NAND_Address, data, 1);//~2.4ms
	if(status != HAL_OK)
		{}
	// Читаем spare area
	status = HAL_NAND_Read_SpareArea_8b_fast(&hnand1, &NAND_Address, (uint8_t*)&spare, 1);//~180us
	if(status != HAL_OK)
		{}


	// Проверяем crc для данных
	uint32_t crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, NAND_PAGE_SIZE_BYTES/4);//~1.6ms
	if(crc32 == spare.crc32)
	{
		return true;
	}

	// Ошибка при проверке crc

	// Проверяем валидность spare.crc32 и восстанавливаем в случае необходимости
	eccdiff_t s = crc32_check_ecc(); //~460us

	// Если исправили CRC32
	if(s == ECC_CORRECTABLE_ERROR)
	{
		// Снова проверяем crc для данных, если она в порядке - выходим
		crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, NAND_PAGE_SIZE_BYTES / 4);
		if(crc32 == spare.crc32)
		{
			return true;
		}
	}

	// Попытаемся использовать коды ecc для восстановления данных
	for(uint8_t i = 0; i < NAND_PAGE_SIZE_BYTES/512; ++i)
	{
		make_ecc(ecc_val, (uint8_t*)&data[i*512]);
		eccdiff_t stat = ecc_check((uint8_t*)&data[i*512], (uint8_t*)&spare.page_ecc[i], ecc_val);

		if(stat == ECC_UNCORRECTABLE_ERROR)
		{
			// Считаем страницу битой
			memset(data, 0xff, NAND_PAGE_SIZE_BYTES);
			return false;
		}
	}

	// Снова проверяем CRC, если она в порядке - выходим
	crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)data, NAND_PAGE_SIZE_BYTES/4);
	if(crc32 != spare.crc32)
	{
		// Считаем страницу битой
		memset(data, 0xff, NAND_PAGE_SIZE_BYTES);
		return false;
	}

	return true;
}

HAL_StatusTypeDef nand_read_page_dummy(uint8_t *data, uint64_t address)
{
	HAL_StatusTypeDef status;
	NAND_AddressTypeDef NAND_Address;
	NAND_GetAddress(address, &NAND_Address);

	status = HAL_NAND_Read_Page_8b_fast(&hnand1, &NAND_Address, data, 1);
	if(status != HAL_OK)
		{}
	return status;
}


HAL_StatusTypeDef nand_read_spare_dummy(uint8_t *data, uint64_t address)
{
	HAL_StatusTypeDef status;
	NAND_AddressTypeDef NAND_Address;
	NAND_GetAddress(address, &NAND_Address);

	status = HAL_NAND_Read_SpareArea_8b_fast(&hnand1, &NAND_Address, data, 1);
	if(status != HAL_OK)
		{}
	return status;
}

void nand_erase_block(uint64_t address)
{
	NAND_AddressTypeDef NAND_Address;
	NAND_GetAddress(address, &NAND_Address);

	// Корректируем номер блока пропуская битые блоки
	uint64_t corrected_address = address;
	for(uint16_t i = NAND_DATA_ADDRESS_BLOCKS; i <= (NAND_Address.Plane * NAND_PLANE_SIZE_BLOCKS + NAND_Address.Block); ++i)
	{
		if(!BlockTable_NAND.block_table[i])
		{
			corrected_address += NAND_BLOCK_SIZE_BYTES;
			NAND_GetAddress(corrected_address, &NAND_Address);
		}
	}

	nand_erase_block_raw(corrected_address);
}

void nand_erase_block_raw(uint64_t address)
{
	HAL_StatusTypeDef status;
	NAND_AddressTypeDef NAND_Address;
	NAND_GetAddress(address, &NAND_Address);

	if(NAND_Address.Block >= NAND_MEMORY_SIZE_BLOCKS)
		return;

	status = HAL_NAND_Erase_Block(&hnand1, &NAND_Address);
	if(status != HAL_OK)
		nand_block_table_mark_block(NAND_Address.Block, false);
}
