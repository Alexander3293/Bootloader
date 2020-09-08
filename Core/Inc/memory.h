/*!
\file memory.h
\brief Файл все, что связано с памятью
\date 01.04.2016 9:17:00 Created
\author Gleb Maslennikov
*/

#ifndef MEMORY_H_
#define  MEMORY_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "mt29fxg08_nand.h"
#include "flash_stm32f4.h"
#include "memory_eeprom.h"
//#include "blackbox.h"
//#include "memory_cyclogram.h"
//#include "memory_scenario.h"
//#include "memory_flash_stm32.h"
#include "memory_status.h"
//#include "memory_calibration.h"
//#include "memory_ewl_cyclogram.h"
#include "CRC16.h"
//#include "CRC8.h"
//#include "memcpy_fast.h"


#define PAGE_DATA_SIZE		(NAND_PAGE_SIZE_BYTES - sizeof(NandPageHeader_t))
#define NAND_BUF_TRSHLD		(PAGE_DATA_SIZE-1)

#define COMPAT_PAGE_SIZE	256UL

#define EEP_BLOCKTABLE_SIZE_PAGES	((NAND_MEMORY_SIZE_BLOCKS/EEP_PAGE_SIZE) + 1)


typedef struct {
	bool		save_to_memory;
	uint8_t		updated_part[EEP_BLOCKTABLE_SIZE_PAGES];	// for writing to EEPROM only specific page
	uint8_t		block_table[NAND_MEMORY_SIZE_BLOCKS];		// size must be less or equal to nand page size
	uint16_t	CRC16;
} __packed BlockTable_t;

///*
// * \brief Шапка страницы nand
// */
//typedef struct
//{
//	uint8_t status;
//	uint8_t param;
//	uint16_t data_shift;		// адрес начала нового пакета в данной странице
//} __packed NandPageHeader_t;
//
///*
// * \brief Структура страницы nand
// */
//typedef struct
//{
//	NandPageHeader_t page_header;
//	uint8_t data[PAGE_DATA_SIZE];
//} __packed PagePack;
//
//
///*
// *\ brief Структура временного буфера записи в NAND
// */
//typedef struct
//{
//	PagePack page_buf[2];
//	uint16_t nand_buf_idx;
//	bool buf_sel;
//} __packed NandTmpBuf_t;
//

//extern BlockTable_t BlockTable;
//extern BlockTable_t BlockTable_EEP;

//extern uint8_t nand_buf_usb[2][NAND_PAGE_SIZE_BYTES + NAND_SPARE_AREA_SIZE_BYTES];
//extern uint8_t nand_buf2_usb[NAND_PAGE_SIZE_BYTES + NAND_SPARE_AREA_SIZE_BYTES + 32];

//extern uint32_t last_packet;
//extern uint32_t next_wr_page;
//
//extern bool nand_write_flag;
//
//extern uint8_t* pbuf1;
//extern uint8_t* pbuf2;

//uint32_t get_first_free_data_page();

//uint32_t get_last_position();

//void set_data_page(uint32_t page);

//void write_data_packet(uint8_t *data, uint16_t len);

//void close_data_packet();

//ami_time get_last_data_time();

void erase_memory();
//uint32_t memory_get_erase_time();
//
//void update_block_table_eeprom();
//void read_block_table_eeprom();
//
//uint32_t get_measure_number_from_page(uint32_t page);
//bool write_tmp_buf(uint8_t* pData, uint16_t len);
//
//
//bool write_tmp_buf_NAND();
//void nand_tmp_buf_init();
//void nand_write_process();
//bool flush_tmp_buf_NAND();
//
//void init_temp_page(uint8_t* p_buf, uint16_t dlen, uint32_t page);
//HAL_StatusTypeDef nand_page_tst(uint32_t page);
//HAL_StatusTypeDef nand_page_spare_tst(uint32_t page);
//HAL_StatusTypeDef nand_spare_tst(uint32_t page);
//bool nand_read_data(uint32_t page_addr, uint32_t buf_idx, uint8_t* p_data, uint16_t len);
//bool nand_read_data_byte(uint64_t byte_addr, uint8_t* p_data, uint16_t len);
//
//uint16_t serial_number_read_flash();

bool nand_read_data_byte(uint64_t byte_addr, uint8_t *p_data, uint16_t len, uint16_t *bytes_readed);

#ifdef __cplusplus
}
#endif

#endif /* MEMORY_H_ */
