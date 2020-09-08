//#include <mt29fxg08_nand.h>
#include "ami_protocol.h"
#include "CRC16.h"
#include "memory.h"

const uint32_t firmware_ver = 1;
const uint32_t date_ver = 190410;

static uint8_t message_box_buffer[8192+12];
static uint8_t command;
static uint8_t message_id;
static uint8_t tool_status_reg;
static message_header message;
static uint16_t tx_len;
static uint32_t pbuf;
static uint32_t last_data_address;
static int64_t last_data_address_prev;

extern bool bootloader_timeout_en;
extern bool firmware_write;
extern bool erase_flag;
extern bool reboot_flag;

void message_box_mod(uint8_t* Buf, uint16_t len, uint8_t master_address, uint8_t own_address, data_source source)
{
	if(len > sizeof(message_box_buffer))
		len = sizeof(message_box_buffer);

	memmove(message_box_buffer, Buf, len);

	command = message_box_buffer[2];
	message_id = message_box_buffer[3];
	//device_id = message_box_buffer[7];

    message.command = command;
    message.own_address = own_address;
    message.master_address = master_address;
    message.message_id = message_id;
    message.data_len = message.packet_adr = 0;

//	packet_number = message_box_buffer[8]
//			| (uint32_t) message_box_buffer[9] << 8
//			| (uint32_t) message_box_buffer[10] << 16
//			| (uint32_t) message_box_buffer[11] << 24;

	switch(command)
		{
		case SCAN:
			// Команда для проверки наличия прибора
			// Возвращаем пустой ответ (длина данных ответа = 0)
			bootloader_timeout_en = false;
			break;

	    case STATUS:
	    	message_box_buffer[11] = tool_status_reg;
	        message.data_len = 1;
	        break;

	    case GET_INFO:
			// Получить версии
			pbuf = TX_HEADER_LENGTH + DATA_ADDRESS_LENGTH;
			message_box_buffer[pbuf++] = firmware_ver & 0xFF;
			message_box_buffer[pbuf++] = (firmware_ver >> 8) & 0xFF;
			message_box_buffer[pbuf++] = (firmware_ver >> 16) & 0xFF;
			message_box_buffer[pbuf++] = (firmware_ver >> 24) & 0xFF;
			message_box_buffer[pbuf++] = date_ver & 0xFF;
			message_box_buffer[pbuf++] = (date_ver >> 8) & 0xFF;
			message_box_buffer[pbuf++] = (date_ver >> 16) & 0xFF;
			message_box_buffer[pbuf++] = (date_ver >> 24) & 0xFF;
			message.data_len = 8;
	        break;

	    case CABLE_ERASE:
	    	// Стереть память
			last_data_address_prev = -1;
			erase_flag = true;
	        break;

	    case FIRMWARE_UPLOAD:
	    	// Записать прошивку 256 байт в NAND
	        last_data_address = message_box_buffer[7]
								| (uint32_t)message_box_buffer[8] << 8
								| (uint32_t)message_box_buffer[9] << 16
								| (uint32_t)message_box_buffer[10] << 24;

	        // Если в эту область NAND уже писали ранее, не записываем
	        if((int64_t)last_data_address > last_data_address_prev)
	        {
	        	memset(message_box_buffer+11+COMPAT_PAGE_SIZE, 0x00, NAND_PAGE_SIZE_BYTES - COMPAT_PAGE_SIZE);
	        	nand_write_page(message_box_buffer+11, NAND_DATA_ADDRESS_BYTES + last_data_address/COMPAT_PAGE_SIZE * NAND_PAGE_SIZE_BYTES);
	        	last_data_address_prev = last_data_address;
	        }

	        message.packet_adr = last_data_address;
	        break;

	    case SET_BOOTFLAG:
	    	// Поставить bootflag, загрузить прошивки в память мк
	    	firmware_write = true;
	    	break;

	    case REBOOT_MCU:
	    	reboot_flag = true;
	    	break;

	    default:
	    	// Команда неопознана
	    	return;
	}

	tx_len = message_header_to_array(&message);

	//todo nonblocking delay (only if needed)
	for(uint16_t i = 0; i < 350; ++i)		// ~ 30 us delay
		__asm("nop;");

	post_tx_data(message_box_buffer, tx_len, message_box, ami, 0);
}

uint16_t message_header_to_array(const message_header* message)
{
	message_box_buffer[0] = message->master_address;
	message_box_buffer[1] = message->own_address;
	message_box_buffer[2] = message->command;
	message_box_buffer[3] = message->message_id;
	message_box_buffer[4] = 0x01;

	uint16_t message_len = 0;

	message_len = TX_HEADER_LENGTH + DATA_ADDRESS_LENGTH + message->data_len;
	message_box_buffer[5] = DATA_ADDRESS_LENGTH + message->data_len;
	message_box_buffer[6] = (DATA_ADDRESS_LENGTH + message->data_len) >> 8;
	message_box_buffer[7] = message->packet_adr;
	message_box_buffer[8] = message->packet_adr >> 8;
	message_box_buffer[9] = message->packet_adr >> 16;
	message_box_buffer[10] = message->packet_adr >> 24;

	uint16_t crc = calc_crc16(message_box_buffer + 1, message_len - 1);

	message_box_buffer[message_len] = crc;
	message_box_buffer[message_len + 1] = crc >> 8;
	message_len += CRC_LENGTH;

	return message_len;
}

void set_tool_status(tool_status status)
{
    tool_status_reg = status;
}

tool_status get_tool_status()
{
    return tool_status_reg;
}
