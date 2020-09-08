/*
 * nand_driver.h
 *
 *  Created on: 22 рту. 2017 у.
 *      Author: alexey.chibryakov
 */

#ifndef MT29FXG08_NAND_H_
#define MT29FXG08_NAND_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "nand_block_table.h"
#include "nand_ecc.h"

// Physical defines
#define MT29F8G08ABABA
//#define MT29F32G08ABAAA

#ifdef MT29F32G08ABAAA
#define NAND_PAGE_SIZE_BYTES			8192ULL	// bytes in one page
#define NAND_BLOCK_SIZE_PAGES			128ULL	// pages per block
#define NAND_PLANE_SIZE_BLOCKS			2048ULL	// blocks per plane
#define NAND_SPARE_AREA_SIZE_BYTES		448ULL	// bytes in page spare area

#define NAND_MEMORY_SIZE_BLOCKS			4096ULL
#define NAND_MEMORY_SIZE_PAGES			(NAND_MEMORY_SIZE_BLOCKS * NAND_BLOCK_SIZE_PAGES)
#define NAND_MEMORY_SIZE_BYTES			(NAND_MEMORY_SIZE_PAGES * NAND_PAGE_SIZE_BYTES)
#endif

#ifdef MT29F8G08ABABA
#define NAND_PAGE_SIZE_BYTES			4096UL	// bytes in one page
#define NAND_BLOCK_SIZE_PAGES			128UL	// pages per block
#define NAND_PLANE_SIZE_BLOCKS			1024L	// blocks per plane
#define NAND_SPARE_AREA_SIZE_BYTES		224UL	// bytes in page spare area

#define NAND_MEMORY_SIZE_BLOCKS			2048UL
#define NAND_MEMORY_SIZE_PAGES			(NAND_MEMORY_SIZE_BLOCKS * NAND_BLOCK_SIZE_PAGES)
#define NAND_MEMORY_SIZE_BYTES			(NAND_MEMORY_SIZE_PAGES * NAND_PAGE_SIZE_BYTES)
#endif

#define NAND_BLOCK_SIZE_BYTES			(NAND_BLOCK_SIZE_PAGES * NAND_PAGE_SIZE_BYTES)

// User areas defines
// Block table - 2 blocks
#define NAND_BLOCKTABLE_ADDRESS			0
#define NAND_BLOCKTABLE_SIZE_BYTES		(1UL * NAND_BLOCK_SIZE_PAGES * NAND_PAGE_SIZE_BYTES)
// Data
#define NAND_DATA_ADDRESS_BYTES			NAND_BLOCKTABLE_SIZE_BYTES
#define NAND_DATA_ADDRESS_BLOCKS		(NAND_BLOCKTABLE_SIZE_BYTES / NAND_PAGE_SIZE_BYTES / NAND_BLOCK_SIZE_PAGES)

#define NAND_DATA_SIZE_BYTES			(NAND_MEMORY_SIZE_BYTES - NAND_DATA_ADDRESS_BYTES)
#define NAND_DATA_SIZE_PAGES			(NAND_DATA_SIZE_BYTES / NAND_PAGE_SIZE_BYTES)
#define NAND_DATA_SIZE_BLOCKS			(NAND_DATA_SIZE_PAGES / NAND_BLOCK_SIZE_PAGES)

// Write\read function return codes
#define CODE_NAND_FULL					0xFFFFFFFFFFFFFFFF

// Structures
typedef struct
	{
	uint8_t		page_ecc[NAND_PAGE_SIZE_BYTES/512][3];
	uint32_t	crc32;
	uint8_t		crc32_ecc[3];
	uint8_t		reserved[NAND_SPARE_AREA_SIZE_BYTES - (NAND_PAGE_SIZE_BYTES/512*3*sizeof(uint8_t)) - (sizeof(uint32_t)) - (3*sizeof(uint8_t))];
	} __packed CSpare;

typedef struct
{
	uint64_t address;
	uint8_t data[NAND_PAGE_SIZE_BYTES];
} __packed PAGE_BUFF;
// Functions
void nand_select_target(uint8_t target);
uint64_t nand_write_page(uint8_t *data, uint64_t address);
uint64_t nand_write_ff_page(uint64_t address);
bool nand_read_page(uint8_t *data, uint64_t address);
void nand_erase_block(uint64_t address);
void nand_erase_block_raw(uint64_t address);

eccdiff_t crc32_check_ecc();

HAL_StatusTypeDef nand_read_page_dummy(uint8_t *data, uint64_t address);
HAL_StatusTypeDef nand_write_page_dummy(uint8_t *data, uint64_t address);
HAL_StatusTypeDef nand_write_spare_dummy(uint8_t *data, uint64_t address);
HAL_StatusTypeDef nand_read_spare_dummy(uint8_t *data, uint64_t address);

void NAND_GetAddress(uint64_t Address, NAND_AddressTypeDef *pNandAddress);
HAL_StatusTypeDef HAL_NAND_Read_Page_8b_fast(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumPageToRead);
HAL_StatusTypeDef HAL_NAND_Read_SpareArea_8b_fast(NAND_HandleTypeDef *hnand, NAND_AddressTypeDef *pAddress, uint8_t *pBuffer, uint32_t NumSpareAreaToRead);

#ifdef __cplusplus
}
#endif

#endif /* MT29FXG08_NAND_H_ */
