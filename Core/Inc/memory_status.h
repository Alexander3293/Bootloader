/*
 * memory_status.h
 */

#ifndef INC_MEMORY_STATUS_H_
#define INC_MEMORY_STATUS_H_

// Структура status для хранения в энергонезависимой памяти
typedef struct
{
	uint8_t flag;
	uint8_t reserved;
	uint16_t crc16;
} __attribute__((__packed__)) CBootflagSaved;

void write_bootflag_memory(uint8_t bootflag);
uint8_t read_bootflag_memory();
void erase_bootflag_memory();

#endif /* INC_MEMORY_STATUS_H_ */
