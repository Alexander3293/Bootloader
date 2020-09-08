/*
 * post_module.h
 *
 *  Created on: 19 февр. 2016 г.
 *      Author: Gleb
 */

#ifndef POST_MODULE_H_
#define POST_MODULE_H_

typedef enum {
//	line 		= 1,
	ami 		= 2,
//	usb			= 3,
//	mid_script	= 4,
	message_box = 5,
//	spi_device = 6,
	no_source  = 7,
}data_source ;

#ifdef __cplusplus
 extern "C" {
#endif

#include "main_defines.h"
#include "main.h"

//#define PM_MID_DATA_BUFFER_SIZE 256

#define MASTER_ADDR		0x4D


void post_tx_data(uint8_t* data, uint16_t len, data_source source, data_source destination, uint8_t parameter);

void post_suspend(uint8_t dev_id);

void post_resume(uint8_t dev_id);

#ifdef __cplusplus
}
#endif

#endif /* POST_MODULE_H_ */
