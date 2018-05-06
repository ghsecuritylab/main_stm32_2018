/*
 * scramble.h
 *
 *  Created on: 2018/03/16
 *      Author: spiralray
 */

#ifndef SCRAMBLE_H_
#define SCRAMBLE_H_

#include "stm32f4xx_hal.h"
#include "uart.h"

#define _USE_MATH_DEFINES
#include <math.h>

typedef struct{
	uint8_t team_id;
	uint8_t robot_id;
	uint8_t dribble_kick[2];
	float vx; // [m/s]
	float vy; // [m/s]
	float omega; // [rad/s]
	uint8_t parity[2];
} scramble_udp_msg;

#define SIZE_UDP_MSG 18

#define WHEEL_DIAMETER 0.052
#define WHEEL_DISTANCE 0.08		//distance from the center of the robot to each wheel
#define GEAR_RATIO 3.0f

extern volatile uint8_t udp_updated;
extern volatile scramble_udp_msg udp_msg;

void scramble_init();

int get_rotary_switch();
void set_dribble(uint32_t power);

void set_motor(uint8_t id, int32_t rpm);
void set_kick(uint8_t is_enabled, uint8_t is_chip, uint8_t power);

#endif /* SCRAMBLE_H_ */
