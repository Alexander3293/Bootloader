/*
 * ami.h
 *
 *  Created on: 15 марта 2016 г.
 *      Author: Gleb
 */

#ifndef AMI_H_
#define AMI_H_

#include "main_defines.h"
#include "post_module.h"

#define AMI_WAIT_MS (100)

#define AMI_WDT_TIM (htim6)

#define AMI_TX_LS_UART (huart2)
#define AMI_TX_HS_UART (huart6)
#define AMI_RX_UART    (huart1)

#define AMI_LS_BAUDRATE (625000)
#define AMI_HS_BAUDRATE (12000000)

typedef enum {
	AMI_TX_BUSY = 1,
	AMI_TX_FINISH = 2,
	AMI_RX_BUSY = 3,
	AMI_RX_FINISH = 4,
	AMI_READY = 5,
	AMI_TX_BUSY_WAIT_NEXT_BYTE = 6,
}ami_status;

void set_ami_status(ami_status status);
ami_status get_ami_status();

void uart_rx_IDLE_IE(UART_HandleTypeDef huart);
void ami_tx(uint8_t* Buf, uint16_t Len);
void set_ami_tx_pwm_clock(uint32_t speed);
void ami_rx_timeout_callback();
void ami_tx_timeout_callback();
bool ami_ready_wait(uint16_t tout_ms);
bool comm_channel_ready_wait(data_source g_comm_channel);

void ami_rx_suspend();
void ami_rx_restart();
void ami_stop_rx_timeout();
void timer_start_IT(TIM_HandleTypeDef *htim);

bool ami_set_high_speed();
bool ami_set_low_speed();

#endif /* AMI_H_ */
