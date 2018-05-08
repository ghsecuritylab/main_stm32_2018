/*
 * uart.c
 *
 *  Created on: 2018/03/16
 *      Author: spiralray
 */

// http://www.eevblog.com/forum/microcontrollers/'best'-way-to-load-uart-data-to-ring-buffer-with-stm32hal/

#include "uart.h"
#include "cmsis_os.h"

/* The UART receiver DMA must be setup as CIRCULAR. */

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

static UART_Buffer xbee_tx_buf;
static uint8_t xbee_rx_buf[UART_BUFFER_SIZE] __attribute__ ((aligned (8)));
static UART_HandleTypeDef *huart_xbee;
static uint32_t xbee_rx_index;


static UART_Buffer rs485_tx_buf;
static uint8_t rs485_rx_buf[UART_BUFFER_SIZE] __attribute__ ((aligned (8)));
static UART_HandleTypeDef *huart_rs485;
static uint32_t rs485_rx_index;

#define DMA_WRITE_PTR(hu) ( (UART_BUFFER_SIZE - hu->hdmarx->Instance->NDTR) & (UART_BUFFER_SIZE - 1) )

void UART_Init() {
	huart_xbee = &huart3;
	HAL_UART_Receive_DMA(huart_xbee, (uint8_t *)xbee_rx_buf, UART_BUFFER_SIZE);
	xbee_rx_index = 0;
	xbee_tx_buf.head = 0;
	xbee_tx_buf.tail = 0;

	huart_rs485 = &huart2;
	HAL_UART_Receive_DMA(huart_rs485, (uint8_t *)rs485_rx_buf, UART_BUFFER_SIZE);
	rs485_rx_index = 0;
	rs485_tx_buf.head = 0;
	rs485_tx_buf.tail = 0;
}

static bool msgrx_circ_buf_is_empty(UART_HandleTypeDef *huart, uint32_t *rx_index) {
	if( *rx_index == DMA_WRITE_PTR(huart) ) {
		return true;
	}
	return false;
}

static uint8_t msgrx_circ_buf_get(UART_HandleTypeDef *huart, uint8_t *rx_buf, uint32_t *rx_index) {
	uint8_t c = 0;
	if( *rx_index != DMA_WRITE_PTR(huart) ) {
		c = rx_buf[(*rx_index)++];
		*rx_index &= (UART_BUFFER_SIZE - 1);
	}
	return c;
}

int16_t UART_GetBufferSize(UART_Buffer *buf)
{
	if (buf->tail == buf->head)
	{
		return 0;
	}
	else if ((buf->tail) < (buf->head))
	{
		return (UART_BUFFER_SIZE - buf->head) + buf->tail;
	}
	return buf->tail - buf->head;
}

int16_t UART_WriteBuffer(UART_Buffer *buf, uint8_t data)
{
	while (UART_GetBufferSize(buf) >= (UART_BUFFER_SIZE - 2))
	{
		osDelay(1);
	}
	__disable_irq();
	buf->data[buf->tail] = data;
	buf->tail = (buf->tail + 1) % UART_BUFFER_SIZE;
	__enable_irq();
	return 1;
}

int16_t UART_SendByte(UART_HandleTypeDef *huart, UART_Buffer *buf, uint8_t data)
{
	int16_t ret, len;
	ret = UART_WriteBuffer(buf, data);


	__disable_irq();
	if (huart->gState == HAL_UART_STATE_READY)
	{
		// Transmit size
		len = UART_GetBufferSize(buf);
		if (buf->head + len > UART_BUFFER_SIZE)
		{
			len = UART_BUFFER_SIZE - buf->head;
		}
		// Transmit
		if (HAL_UART_Transmit_DMA(huart, &(buf->data[buf->head]), len) == HAL_OK)
		{
			buf->pending_size = len;
		}
	}
	__enable_irq();
	return ret;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	UART_Buffer *tx_buffer;
	uint16_t len = 0;

	if (huart->Instance == huart_xbee->Instance)
	{
		tx_buffer = &xbee_tx_buf;
	}
	else if (huart->Instance == huart_rs485->Instance)
	{
		tx_buffer = &rs485_tx_buf;
	}
	else
	{
		return;
	}

	// Transmitted Size
	len = tx_buffer->pending_size;

	// head
	tx_buffer->head = (tx_buffer->head + len) % UART_BUFFER_SIZE;

	// Transmit size
	len = UART_GetBufferSize(tx_buffer);
	if (len == 0)
	{
		// complete
		return;
	}
	if (tx_buffer->head + len > UART_BUFFER_SIZE)
	{
		len = UART_BUFFER_SIZE - tx_buffer->head;
	}

	if (HAL_UART_Transmit_DMA(huart, &(tx_buffer->data[tx_buffer->head]), len) == HAL_OK)
	{
		tx_buffer->pending_size = len;
	}
}

/**************************************************************************/
/*!
    High Level function.
 */
/**************************************************************************/
/* Send 1 character */
int putch(uint8_t data)
{
	return UART_SendByte(huart_xbee, &xbee_tx_buf, data);
}

/* Receive 1 character */
uint8_t getch(void)
{
	if ( !msgrx_circ_buf_is_empty(huart_xbee, &xbee_rx_index) )
		return msgrx_circ_buf_get(huart_xbee, xbee_rx_buf, &xbee_rx_index);
	else
		return false;
}

int __io_putchar(int ch){
	return putch(ch);
}

int __io_getchar(void){
	while( msgrx_circ_buf_is_empty(huart_xbee, &xbee_rx_index) ){
		osDelay(1);
	}
	return msgrx_circ_buf_get(huart_xbee, xbee_rx_buf, &xbee_rx_index);
}


/**************************************************************************/
/*!
    RS485
 */
/**************************************************************************/
/* Send 1 byte */
int RS485_SendByte(uint8_t data)
{
	return UART_SendByte(huart_rs485, &rs485_tx_buf, data);
}

/* Receive 1 byte */
int16_t RS485_ReceiveByte(void){
	if ( !msgrx_circ_buf_is_empty(huart_rs485, &rs485_rx_index) )
		return msgrx_circ_buf_get(huart_rs485, rs485_rx_buf, &rs485_rx_index);
	else
		return -1;
}
