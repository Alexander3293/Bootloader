/*
 * nand_block_table.c
 */

#include "string.h"
#include "memory.h"
#include "nand_ecc.h"

extern CSpare spare;

extern NAND_HandleTypeDef hnand1;
extern CRC_HandleTypeDef hcrc;

//extern uint8_t page_temp[NAND_PAGE_SIZE_BYTES];
static uint8_t temp[NAND_PAGE_SIZE_BYTES];

BlockTable_t BlockTable_NAND;		// ������� ������, ���������� � NAND. ����������� ������ ��� �������� ������.
BlockTable_t BlockTable_EEP;		// ������� ������, ���������� � EEPROM. ����������� ��� ��������� ������ ��������.

static void nand_block_table_write();

// NAND Block table

/*
* @brief �������������� BlockTable_NAND ��������� ���������� (��� ����� �������)
*/
void nand_block_table_init()
{
	memset((uint8_t*)&BlockTable_NAND.block_table, 0x01, sizeof(BlockTable_NAND.block_table));

	// ��������� � ������ NAND
	BlockTable_NAND.save_to_memory = true;
}

/*
 * @brief ���������� ����� ����� � BlockTable_NAND
 */
void nand_block_table_mark_block(uint16_t n_block, bool valid)
{
	if(BlockTable_NAND.block_table[n_block] != valid)
	{
		BlockTable_NAND.block_table[n_block] = valid;
		BlockTable_NAND.save_to_memory = true;
	}
}
/*
* @brief ��������� BlockTable_NAND � ������ NAND ��� �������������
*/
void nand_update_block_table()
{
	if(BlockTable_NAND.save_to_memory)
	{
		// 0 block erase for block table
//		nand_erase_block_raw(NAND_BLOCKTABLE_ADDRESS);

		nand_block_table_write();
		BlockTable_NAND.save_to_memory = false;
	}
}

/*
* @brief ������ BlockTable_NAND �� ������ NAND. � ������ ������� �������� nand_block_table_init()
*/
void nand_block_table_read()
{
	NAND_AddressTypeDef NAND_Address;
	HAL_StatusTypeDef status;
	uint8_t ecc_val[3];

	NAND_GetAddress(0, &NAND_Address);

	// ������ �������� � block_table
	status = HAL_NAND_Read_Page_8b(&hnand1, &NAND_Address, (uint8_t*)&temp, 1);

	if(status != HAL_OK)
		{
		// �� ������� ������� block_table, �������� ��� ����� ��������� (1), �������
		nand_block_table_init();
		return;
		}

	// ������ spare area
	status = HAL_NAND_Read_SpareArea_8b(&hnand1, &NAND_Address, (uint8_t*)&spare, 1);
	if(status != HAL_OK)
		{
		// �� ������� ������� spare area, �������� ��� ����� ��������� (1), �������
		nand_block_table_init();
		return;
		}

	// �������� � block_table
	memcpy((uint8_t*)&BlockTable_NAND.block_table, temp, sizeof(BlockTable_NAND.block_table));

	// ��������� crc ��� block_table
	uint32_t crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)&temp, sizeof(temp)/4);
	if(crc32 == spare.crc32)
		return;

	// ������ ��� �������� crc

	// ��������� ���������� spare.crc32 � ��������������� � ������ �������������
	eccdiff_t s = crc32_check_ecc();

	// ���� ��������� CRC32
	if(s == ECC_CORRECTABLE_ERROR)
		{
		// ����� ��������� crc ��� block_table, ���� ��� � ������� - �������
		crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)&temp, sizeof(temp)/4);
		if(crc32 == spare.crc32)
			return;
		}

	// ���������� ������������ ���� ecc ��� �������������� ������
	for(uint8_t i = 0; i < NAND_PAGE_SIZE_BYTES/512; ++i)
		{
		make_ecc(ecc_val, (uint8_t*)&temp[i*512]);
		eccdiff_t stat = ecc_check((uint8_t*)&temp[i*512], (uint8_t*)&spare.page_ecc[i], ecc_val);

		if(stat == ECC_UNCORRECTABLE_ERROR)
			{
			nand_block_table_init();
			return;
			}
		}

	// ����� ��������� CRC, ���� ��� � ������� - �������
	crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)&temp, sizeof(temp)/4);
	if(crc32 != spare.crc32)
		{
		nand_block_table_init();
		return;
		}
}

/*
* @brief ���������� BlockTable_NAND � ������ NAND.
*/
static void nand_block_table_write()
{
	HAL_StatusTypeDef status;
	NAND_AddressTypeDef NAND_Address;

	// ������� ������ ����
	NAND_GetAddress(0, &NAND_Address);

	status = HAL_NAND_Erase_Block(&hnand1, &NAND_Address);
	if(status != HAL_OK)
		return;

	NAND_GetAddress(0, &NAND_Address);

	memcpy(temp, (uint8_t*)&BlockTable_NAND.block_table, sizeof(BlockTable_NAND.block_table));

	// ����� block_table
	status = HAL_NAND_Write_Page_8b(&hnand1, &NAND_Address, (uint8_t*)&temp, 1);
	if(status != HAL_OK)
		return;

	// ������� ecc ��� block_table
	for(uint8_t j = 0; j < NAND_PAGE_SIZE_BYTES/512; ++j)
		make_ecc((uint8_t*)&spare.page_ecc[j], (uint8_t*)&temp[j*512]);

	// ������� crc ��� block_table
	spare.crc32 = HAL_CRC_Calculate(&hcrc, (uint32_t*)&temp, sizeof(temp)/4);

	// ������� ecc ��� crc32
	memset(temp, 0, 512);
	memcpy(temp, (uint8_t*)&spare.crc32, sizeof(spare.crc32));
	make_ecc(spare.crc32_ecc, temp);

	// ����� spare
	status = HAL_NAND_Write_SpareArea_8b(&hnand1, &NAND_Address, (uint8_t*)&spare, 1);
	if(status != HAL_OK)
		return;
}

// EEPROM Block table

void eeprom_block_table_init()
{
	memset((uint8_t*)&BlockTable_EEP.block_table, 0x01, sizeof(BlockTable_EEP.block_table));

	// set up all update flags
	memset(&BlockTable_EEP.updated_part[0], 1, sizeof(BlockTable_EEP.updated_part));

	// ��������� � ������
	BlockTable_EEP.save_to_memory = true;
}

/*
 * @brief ���������� ����� ����� � BlockTable_EEP
 */
void eeprom_block_table_mark_block(uint16_t n_block, bool valid)
{
	if(BlockTable_EEP.block_table[n_block] != valid)
	{
		BlockTable_EEP.block_table[n_block] = valid;
		BlockTable_EEP.updated_part[n_block / EEP_PAGE_SIZE] = 1;///< ���������� ���� ���������� ��� �������� EEP

		BlockTable_EEP.save_to_memory = true;
	}
}

void eeprom_block_table_write()
{
	if(!BlockTable_EEP.save_to_memory)
		return;

	BlockTable_EEP.save_to_memory = false;

	// update table
	for(uint8_t i = 0; i < EEP_BLOCKTABLE_SIZE_PAGES; i++)
	{
		if(BlockTable_EEP.updated_part[i] == 1)
			m95m01_eeprom_write((uint8_t*)&BlockTable_EEP.block_table[0] + i * EEP_PAGE_SIZE, EEP_BLOCKTABLE_ADDRESS + i * EEP_PAGE_SIZE, 256, false);
	}

	// update crc16
	BlockTable_EEP.CRC16 = calc_crc16((uint8_t*)&BlockTable_EEP.block_table[0], sizeof(BlockTable_EEP.block_table));
	m95m01_eeprom_write((uint8_t*)&BlockTable_EEP.CRC16, EEP_BLOCKTABLE_ADDRESS + sizeof(BlockTable_EEP.block_table), 2, false);
	memset((uint8_t*)&BlockTable_EEP.updated_part[0], 0, EEP_BLOCKTABLE_SIZE_PAGES);
}

void eeprom_block_table_read()
{
	m95m01_eeprom_read((uint8_t*)&BlockTable_EEP.block_table[0], EEP_BLOCKTABLE_ADDRESS, sizeof(BlockTable_EEP.block_table) + sizeof(BlockTable_EEP.CRC16), true);

	if(BlockTable_EEP.CRC16 != calc_crc16((uint8_t*)&BlockTable_EEP.block_table[0], sizeof(BlockTable_EEP.block_table)))
		eeprom_block_table_init();
}
