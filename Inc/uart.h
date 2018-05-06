/*
 * uart.h
 *
 *  Created on: 2018/03/16
 *      Author: spiralray
 */

#ifndef UART_H_
#define UART_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

#define UART_BUFFER_SIZE       256  /* must be power of two */

typedef struct
{
    uint16_t head;
    uint16_t tail;
    uint16_t pending_size;
    uint8_t data[UART_BUFFER_SIZE] __attribute__ ((aligned (8)));
} UART_Buffer;


//General function
void UART_Init();
int16_t UART_SendByte(UART_HandleTypeDef *huart, UART_Buffer *buf, uint8_t data);

//High Level function.
int putch(uint8_t data);
uint8_t getch(void);


//RS485
int RS485_SendByte(uint8_t data);
int16_t RS485_ReceiveByte(void);

#endif /* UART_H_ */
