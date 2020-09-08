/*
 * memory_status.c - Functions related to 'STATUS' eeprom area
 */

#include "memory.h"
#include "string.h"

static CBootflagSaved bootflag_saved;

void write_bootflag_memory(uint8_t bootflag)
{
	bootflag_saved.flag = bootflag;
	bootflag_saved.reserved = 0xAA;
	bootflag_saved.crc16 = calc_crc16((uint8_t*)&bootflag_saved, sizeof(bootflag_saved) - 2);

	m95m01_eeprom_write((uint8_t*)&bootflag_saved, EEP_BOOTFLAG_ADDRESS, sizeof(bootflag_saved), true);
}

uint8_t read_bootflag_memory()
{
	memset((uint8_t*)&bootflag_saved, 0, sizeof(bootflag_saved));

	m95m01_eeprom_read((uint8_t*)&bootflag_saved, EEP_BOOTFLAG_ADDRESS, sizeof(bootflag_saved), true);

	if(bootflag_saved.crc16 != calc_crc16((uint8_t*)&bootflag_saved, sizeof(bootflag_saved) - 2))
		return 0;

	return bootflag_saved.flag;
}

void erase_bootflag_memory()
{
	m95m01_eeprom_erase(EEP_BOOTFLAG_ADDRESS, EEP_BOOTFLAG_SIZE/EEP_PAGE_SIZE);
}
