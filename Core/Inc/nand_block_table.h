/*
 * nand_block_table.h
 */

#ifndef NAND_BLOCK_TABLE_H_
#define NAND_BLOCK_TABLE_H_

void nand_block_table_init();
void nand_block_table_mark_block(uint16_t n_block, bool valid);
void nand_update_block_table();
void nand_block_table_read();

void eeprom_block_table_write();
void eeprom_block_table_init();
void eeprom_block_table_mark_block(uint16_t n_block, bool valid);
void eeprom_block_table_update();
void eeprom_block_table_read();

#endif /* NAND_BLOCK_TABLE_H_ */
