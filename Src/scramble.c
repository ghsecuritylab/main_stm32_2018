/*
 * scramble.c
 *
 *  Created on: 2018/03/16
 *      Author: spiralray
 */


/* Includes ------------------------------------------------------------------*/
#include "scramble.h"
#include "cmsis_os.h"
#include "lwip.h"
#include <string.h>

#include "main.h"
#include "uart.h"

#define __GPIO_SET(_PORT, _PIN)         ((_PORT)->BSRR = (_PIN))
#define __GPIO_RST(_PORT, _PIN)         ((_PORT)->BSRR = ((uint32_t)_PIN << 16U))

#define DRIBBLE_ENABLE()	__GPIO_RST(DRI_STBY_GPIO_Port, DRI_STBY_Pin)
#define DRIBBLE_DISABLE()	__GPIO_SET(DRI_STBY_GPIO_Port, DRI_STBY_Pin)

#define RS485_DE_ENABLE()	__GPIO_SET(RS485_DE_GPIO_Port, RS485_DE_Pin)
#define RS485_DE_DISABLE()	__GPIO_RST(RS485_DE_GPIO_Port, RS485_DE_Pin)

extern TIM_HandleTypeDef htim1;

extern inline uint32_t RBIT(uint32_t din);

volatile uint8_t udp_updated = 0x00;
volatile scramble_udp_msg udp_msg = {0};

void scramble_init(){
	UART_Init();
	printf("\r\n=====Scramble MainBoard=====\r\n");
	RS485_DE_ENABLE();
	osDelay(1);

	set_dribble(0);
	DRIBBLE_ENABLE();

	Scramble_LWIP_Init();

	udp_server_init();
}

void scramble_main(){
	for(;;)
	{
		scramble_udp_msg tmp_msg;
		uint8_t flag = 0x00;

		__disable_irq();
		if(udp_updated){
			memcpy( &tmp_msg, (void *)&udp_msg, SIZE_UDP_MSG);
			flag = 1;
			udp_updated = 0;
		}
		__enable_irq();

		if(flag){
			/*
			const float w[4][3] = {
					{-0.5, 0.86603,     WHEEL_DISTANCE},
					{-0.70711,-0.70711, WHEEL_DISTANCE},
					{ 0.70711,-0.70711, WHEEL_DISTANCE},
					{ 0.5, 0.86603,     WHEEL_DISTANCE}
			};
			 */

			const float w[4][3] = {
					{-0.5, 0.6,     WHEEL_DISTANCE},
					{-0.70711,-0.70711, WHEEL_DISTANCE},
					{ 0.70711,-0.70711, WHEEL_DISTANCE},
					{ 0.5, 0.6,     WHEEL_DISTANCE}
			};

			const float v[3] = {tmp_msg.vx, tmp_msg.vy, tmp_msg.omega};
			float motor_rpm[4]={0};

			for(int m=0;m<4;m++){
				for(int i=0;i<3;i++){
					motor_rpm[m] += w[m][i] * v[i];
				}
			}

			for(int i=0;i<4;i++){
				motor_rpm[i] *= GEAR_RATIO / (WHEEL_DIAMETER * M_PI) * 60;
				set_motor(i, motor_rpm[i]);
			}
			//printf("%6d %6d %6d %6d\r\n", (int)motor_rpm[0], (int)motor_rpm[1], (int)motor_rpm[2], (int)motor_rpm[3]);

			const uint8_t kick_flag = (tmp_msg.dribble_kick[0]>>4) & 0x01;
			const uint8_t is_chip = (tmp_msg.dribble_kick[0]>>3) & 0x01;
			const uint8_t kick_power = tmp_msg.dribble_kick[1] & 0x0f;

			//if the ball is held
			if( kick_flag && HAL_GPIO_ReadPin(BALL_GPIO_Port, BALL_Pin) == GPIO_PIN_SET ){
				//if the kicker is charged
				if( HAL_GPIO_ReadPin(KICK_READY_GPIO_Port, KICK_READY_Pin) == GPIO_PIN_SET ){
					set_kick(kick_flag, is_chip, kick_power);
				}
			}

			const uint8_t dribble_power = (tmp_msg.dribble_kick[1] >> 4) & 0x0f;
			set_dribble(dribble_power*7);
		}
		osDelay(1);
	}
}

inline uint32_t RBIT(uint32_t din){
	uint32_t dout;
	__asm__("RBIT %[Rd], %[Rs1]" : [Rd] "=r" (dout) : [Rs1] "r" (din));
	return(dout);
}

int get_rotary_switch(){
	return RBIT(GPIOD->IDR >> 12)>>28;
}


void set_dribble(uint32_t power){
	if(power > 100){
		power = 100;
	}
	else if(power < 0){
		power = 0;
	}
	TIM1->CCR1 = power * 8;
}

void rs485_send_msg(uint8_t id, uint8_t addr, uint8_t len, uint8_t *data){
	uint8_t parity = 0x00;

	RS485_SendByte(0xFA);
	RS485_SendByte(0xAF);

	RS485_SendByte(id);
	RS485_SendByte(addr);
	RS485_SendByte(len);

	parity^= id;
	parity^= addr;
	parity^= len;

	for(int i=0; i<len; i++){
		RS485_SendByte(data[i]);
		parity ^= data[i];
	}
	RS485_SendByte(parity);
}

void set_motor(uint8_t id, int32_t rpm){
	rs485_send_msg(0x10+id, 0x00, 4, (uint8_t *)&rpm);
}

void set_kick(uint8_t is_enabled, uint8_t is_chip, uint8_t power){
	uint8_t addr = 0x01;
	if(is_chip)
		addr = 0x02;

	if(power > 11)
		power = 11;

	if(is_enabled>1)
		is_enabled=1;

	uint8_t data[2] = {is_enabled, power};
	rs485_send_msg(0x20, addr, 2, data);
}
