/*
 * post_module.c
 *
 *  Created on: 21 июля 2016 г.
 *      Author: Gleb
 */
#include "post_module.h"

bool global_suspend = false;

uint8_t destination_address;
uint8_t suspend_dev_id = 0;
uint8_t suspend_data[1024];
uint16_t suspend_len;

data_source suspend_source;
data_source suspend_destination;

void post_suspend(uint8_t dev_id)
{
	if(global_suspend == false)
	{
		suspend_dev_id = dev_id;
		global_suspend = true;
	}
}

void post_resume(uint8_t dev_id)
{
	if(global_suspend == true)
	{
		suspend_dev_id = 0;
		global_suspend = false;

		if(suspend_len > 0)
			suspend_len = 0;
	}
}

void post_tx_data(uint8_t* data, uint16_t len, data_source source, data_source destination, uint8_t parameter)
{
	uint8_t *p_data = (uint8_t*)data;

	if(len > AMI_RXBUF_LEN)
		return;

	destination_address = data[0];

	if(global_suspend == true) //Have global suspend here
	{
		switch(destination)
		{
			case ami:
				if(data[0] == MASTER_ADDR)
					ami_tx(p_data, len);
				break;

			case message_box:
				message_box_mod(data, len, MASTER_ADDR, OWN_BOOTLOADER_ADDRESS, source);
				break;

			default:
				break;
		}
	}

	else //Have no global suspend here
	{
		switch(destination)
		{
			case ami:
				ami_tx(p_data, len);
				break;

			case message_box:
				message_box_mod(data, len, MASTER_ADDR, OWN_BOOTLOADER_ADDRESS, source);
				break;

			default:
				break;
		}
	}
}
