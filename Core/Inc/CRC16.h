#ifndef CRC16_C_H_
#define CRC16_C_H_

#include "stdint.h"
#include "stdio.h"
#include "stdbool.h"

uint16_t calc_crc16(const uint8_t *array, uint16_t len);

bool check_crc16(const uint8_t *array, uint16_t len, uint16_t crc);

uint16_t add_crc16(uint8_t *array, uint16_t len);

void init_accumulate_crc16();

uint16_t calc_accumulate_crc16(const uint8_t *array, uint16_t len);

#endif /* CRC16_C_H_ */
