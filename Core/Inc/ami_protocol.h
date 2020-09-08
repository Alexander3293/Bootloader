/*!
\file ami_protocol.h
\brief Файл содержащий протокол обмена по шине АМИ
\date 01.04.2016 9:17:00 Created
\author Gleb Maslennikov
*/

#ifndef AMI_PROTOCOL_H_
#define AMI_PROTOCOL_H_

//#include <tool_time.h>
#include "main_defines.h"

#define OWN_BOOTLOADER_ADDRESS	0xB1
#define MASTER_ADDRESS			0x4D
#define BROADCAST_ADDR			0xA5

#define RX_HEADER_LENGTH	6
#define TX_HEADER_LENGTH	7
#define DATA_ADDRESS_LENGTH	4
#define CRC_LENGTH			2
#define DATA_LENGTH			256
#define FRAME_LENGTH  		(TX_HEADER_LENGTH + DATA_ADDRESS_LENGTH + DATA_LENGTH + CRC_LENGTH)
#define FRAME_LENGTH_MIN	(TX_HEADER_LENGTH + DATA_ADDRESS_LENGTH + CRC_LENGTH)

#define GOOD_QUICK_ANSWER	0x1D
#define BAD_QUICK_ANSWER	0x0D

#define SCAN    			0x0A    ///	сканирование;
//#define SET_TIME	        0x1A	//	установка времени
//#define GET_TIME	        0x2A  	//  запрос времени &  текущих данных
//#define SET_INFO	        0x3A	//	принять
#define GET_INFO			0x4A	//	запрос
//#define SET_CYCLOGRAM       0x5A	//	принять циклограмму
//#define GET_CYCLOGRAM       0x6A	//	запрос циклограммы
//#define GET_DATA			0x7A	//	запрос данных
//#define GET_SIZE			0x9A	//	запрос кол-ва данных в памяти
//#define GET_POSITION		0x8A	//	запрос текущей позиции
//#define STOP				0x0C	//	старт
//#define START				0x1C	//	стоп
#define STATUS				0x5C	//	статус прибора
/////////////команды синхронизации
//#define STATUSS				0x7D	//	статус параметров синхронизации
//#define SYNCRO				0x2D	// Синхронизация цикла приборов
//#define SETDELAY			0x4E	// Установка задержки запуска измерения от синхронизации
//#define CLRMASTER			0x3C	// Отмена режима мастера синхронизации
//#define SETMASTER			0x4C	// Запуск мастера синхронизации
//#define BLACKOUT			0x1B	// Сообщения о блекаутах
//#define VERSIONS			0x2B	// Информация о версиях
//#define SET_CALIBRATION		0x3B	// Установить калибровочную таблицу
//#define GET_CALIBRATION		0x4B	// Получить калибровочную таблицу
////////////////// cable comands
//#define CABLE_START			0xC6
//#define CABLE_STOP			0xC7
//#define CABLE_SET			0xC8
//#define GET_SWITCH			0xC9
//#define CABLE_DATA			0xCA
#define CABLE_ERASE			0xCB

//#define SET_DATA			0xD0
//#define SET_CUSTOM_PARAM	0xD9
//#define GET_CUSTOM_PARAM	0xDA

//#define INFO_ADDRESS		0x00

//#define TO_HIGHT_SPEED		0x3C
//#define TO_LOW_SPEED		0x8C
#define FIRMWARE_UPLOAD		0XE0
#define SET_BOOTFLAG		0xEA
#define REBOOT_MCU			0xED

typedef struct
{
	uint32_t packet_adr;
	uint8_t command;
	uint8_t message_id;
	uint8_t master_address;
	uint8_t own_address;
	uint16_t data_len;
} __packed message_header;

typedef enum {
	ready_status = 1,
//	work_status = 2,
	idle_status = 8,
//	ewl_work_status = 12,
//	paused_status = 14
	busy_status = 16,
	firmware_upload_success_status = 17,
	firmware_upload_fail_status = 18,
}tool_status;

uint8_t mod_status_reg;

//void ami_rx_callback(uint8_t* Buf, uint16_t Len , uint8_t master_address, uint8_t own_address);

//void ami_tx_callback();

void message_box_mod(uint8_t* Buf, uint16_t len, uint8_t master_address, uint8_t own_address, data_source source);
void message_box_process();

uint16_t message_header_to_array(const message_header* message);

//void answer_messege(uint8_t* Buf, uint16_t len);

void set_tool_status(tool_status status);
tool_status get_tool_status();

#endif /* AMI_PROTOCOL_H_ */
