#include <mt29fxg08_nand.h>
#include "memory.h"
#include "string.h"
//#include "blackbox.h"
//#include "xyc_sensors.h"
//#include "status.h"

//uint32_t data_address = 0;
//uint32_t data_page = 0;
//tool_time last_data_time;
uint8_t nand_buf[NAND_PAGE_SIZE_BYTES];

extern BlockTable_t BlockTable_NAND;	// таблица блоков, хранящаяся в NAND. Обновляется только при стирании памяти.
extern BlockTable_t BlockTable_EEP;		// таблица блоков, хранящаяся в EEPROM. Обновляется при неудачной записи страницы.
//
//void set_data_page(uint32_t page)
//{
//	data_address = 0;
//	data_page = page;
//}
//
//void data_page_increment()
//{
//	++data_page;
//}
//
//void data_page_write()
//{
//	uint64_t address = NAND_DATA_ADDRESS_BYTES + data_page * NAND_PAGE_SIZE_BYTES;
//	uint64_t address_written = nand_write_page(nand_buf, address);
//
//	// если страницу записали по тому адресу, который указывали, то выходим
//	if(address == address_written)
//		return;
//
//	// иначе страница была записана по другому адресу
//	if(address_written < NAND_DATA_ADDRESS_BYTES)
//		data_page = 0;
//	else
//		data_page = (address_written - NAND_DATA_ADDRESS_BYTES) / NAND_PAGE_SIZE_BYTES;
//}

//void data_page_write_ff()
//{
//	uint64_t address = NAND_DATA_ADDRESS_BYTES + data_page * NAND_PAGE_SIZE_BYTES;
//	uint64_t address_written = nand_write_ff_page(address);
//
//	// если страницу записали по тому адресу, который указывали, то выходим
//	if(address == address_written)
//		return;
//
//	// иначе страница была записана по другому адресу
//	if(address_written < NAND_DATA_ADDRESS_BYTES)
//		data_page = 0;
//	else
//		data_page = (address_written - NAND_DATA_ADDRESS_BYTES) / NAND_PAGE_SIZE_BYTES;
//}

//void write_data_packet(uint8_t *data, uint16_t len)
//{
//	if((COMPAT_PAGE_SIZE - sizeof(tool_time)) < (len + data_address))
//		{
//		memmove((uint8_t*)&nand_buf[COMPAT_PAGE_SIZE - sizeof(tool_time)], (uint8_t*)(&last_data_time), sizeof(tool_time));
//
//		// пишем страницу
//		data_page_write();
//
//		// сохраняем последний номер блока в blackbox (eeprom)
//		//uint16_t block = nand_calc_block_raw_from_address(NAND_DATA_ADDRESS_BYTES + data_page * NAND_PAGE_SIZE_BYTES);
//		uint16_t block = (NAND_DATA_ADDRESS_BYTES + data_page * NAND_PAGE_SIZE_BYTES) / NAND_BLOCK_SIZE_BYTES;
//		if(blackbox.NAND_LastBlock != block)
//			{
//			blackbox.NAND_LastBlock = block;
//			blackbox_save();
//			}
//
//		data_page_increment();
//		data_address = 0;
//		}
//
//////todo remove - test nand crash
////static bool crash_nand = true;			//todo
////if((data_page == 160) && crash_nand)
////	{
////	crash_nand = false;
////	data_page = 0;
////	}
//
//	if(data_address == 0)
//		{
//		last_data_time = get_current_time();
//		last_data_time.split_seconds++;
//
//		memset(nand_buf, 0xff, sizeof(nand_buf));
//		memmove((uint8_t*)&nand_buf[data_address], &last_data_time, sizeof(tool_time));
//		data_address += sizeof(tool_time);
//
//		uint32_t measure = get_measure_number() - 1;
//		uint8_t measure_array[4];
//		measure_array[0] = measure;
//		measure_array[1] = measure >> 8;
//		measure_array[2] = measure >> 16;
//		measure_array[3] = measure >> 24;
//		memmove((uint8_t*)&nand_buf[data_address], measure_array, 4);
//		data_address += 4;
//
//		uint16_t vbat = measure_vbat();
//		uint8_t vbat_array[2];
//		vbat_array[0] = vbat;
//		vbat_array[1] = vbat >> 8;
//		memmove((uint8_t*)&nand_buf[data_address], vbat_array, 2);
//		data_address += 2;
//		}
//
//	memmove((uint8_t*)&nand_buf[data_address], data, len);
//	data_address += len;
//	last_data_time = get_current_time();
//	last_data_time.split_seconds++;
//}

void erase_memory()
{
	// erase all memory blocks, except 0 block
	for(uint32_t i = 0; i < NAND_DATA_SIZE_BLOCKS; ++i)
		nand_erase_block_raw(NAND_DATA_ADDRESS_BYTES + i * NAND_BLOCK_SIZE_BYTES);

	// merge block tables
	bool merged_block;

	for(uint16_t j = 0; j < sizeof(BlockTable_NAND.block_table); j++)
	{
		merged_block = BlockTable_NAND.block_table[j] && BlockTable_EEP.block_table[j];
		nand_block_table_mark_block(j, merged_block);
		eeprom_block_table_mark_block(j, merged_block);
	}

	// update block tables in memory
	nand_update_block_table();
	eeprom_block_table_write();
}

/*
 * \brief Чтение данных из NAND начиная с указанного адреса
 * \param byte_addr - адрес данных для чтения, в байтах
 * \param p_data - указатель на область памяти, куда будут считаны данные
 * \param len - длина читаемых данных в байтах
 */
bool nand_read_data_byte(uint64_t byte_addr, uint8_t *p_data, uint16_t len, uint16_t *bytes_readed)
{
	bool result = true;

	uint64_t page_address = 0;		// адрес (номер) читаемой страницы
	uint16_t page_offset = 0;		// отступ с начала страницы
	uint16_t read_len = 0;			// текущая читаемая длина
	uint16_t n_byte = 0;			// текущее число прочитанных байт

	while(len)
	{
		page_address = byte_addr / NAND_PAGE_SIZE_BYTES;
		page_offset = byte_addr % NAND_PAGE_SIZE_BYTES;

		// Читаем данные из NAND постранично

		//nand_read_page((uint8_t*)&tmp, (uint64_t)(NAND_DATA_ADDRESS_BYTES + page_address * NAND_PAGE_SIZE_BYTES));
		nand_read_page((uint8_t*)&nand_buf, (uint64_t)(NAND_DATA_ADDRESS_BYTES + page_address * NAND_PAGE_SIZE_BYTES));
		read_len = len > (NAND_PAGE_SIZE_BYTES - page_offset) ? (NAND_PAGE_SIZE_BYTES - page_offset) : len;

		memcpy((uint8_t*)&p_data[n_byte], &nand_buf[page_offset], read_len);

		n_byte += read_len;
		byte_addr += read_len;
		len -= read_len;
	}

	if(bytes_readed != NULL)
		*bytes_readed = n_byte;
	return result;
}

