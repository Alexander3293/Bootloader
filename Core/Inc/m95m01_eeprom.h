/*
 * m95m01_eeprom.h
 *
 *  Created on: 29 рту. 2017 у.
 *      Author: alexey.chibryakov
 */

#ifndef M95M01_EEPROM_H_
#define M95M01_EEPROM_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdbool.h"

// Instruction set
#define EEP_PAGE_SIZE				256UL

#define WREN_INSTR	0x06	// Write Enable
#define WRDI_INSTR	0x04	// Write Disable
#define RDSR_INSTR	0x05	// Read Status Register
#define WRSR_INSTR	0x01	// Write Status Register
#define READ_INSTR	0x03	// Read from Memory Array
#define WRITE_INSTR	0x02	// Write to Memory Array

// Instructions available only for the M95M01-D device
#define RDID_INSTR	0x83	// Read Identification Page
#define WRID_INSTR	0x82	// Write Identification Page
#define RDLS_INSTR	0x83	// Reads the Identification Page lock status
#define LID_INSTR	0x82	// Locks the Identification page in read-only mode

// Status register
typedef union
	{
	struct {
		uint8_t wip : 1;		// Write in progress (read only)
		uint8_t wel : 1;		// Write Enable Latch (read only)
		uint8_t bp0 : 1;
		uint8_t bp1 : 1;		// Block Protect
		uint8_t reserved : 3;
		uint8_t srwd : 1;		// Status Register Write Disable
		} bits;

	uint8_t byte;
	} __packed status_type;

bool m95m01_eeprom_wait_spi();
bool m95m01_eeprom_wait_ready(uint16_t timeout_ms);

void m95m01_eeprom_write_page(uint8_t *data, uint32_t addr, uint16_t size);
void m95m01_eeprom_write(uint8_t *data, uint32_t addr, uint32_t size, bool wait);
void m95m01_eeprom_read(uint8_t *data, uint32_t addr, uint32_t size, bool wait);
bool m95m01_eeprom_erase(uint32_t addr, uint32_t n_pages);

void m95m01_eeprom_write_status_reg();
void m95m01_eeprom_read_status_reg();

void m95m01_eeprom_cs(bool select_device);
void m95m01_eeprom_hold(bool hold);

#ifdef __cplusplus
}
#endif

#endif /* M95M01_EEPROM_H_ */
