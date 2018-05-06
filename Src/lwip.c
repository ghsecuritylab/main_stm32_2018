/**
 ******************************************************************************
  * File Name          : LWIP.c
  * Description        : This file provides initialization code for LWIP
  *                      middleWare.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
  
/* Includes ------------------------------------------------------------------*/
#include "lwip.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#if defined ( __CC_ARM )  /* MDK ARM Compiler */
#include "lwip/sio.h"
#endif /* MDK ARM Compiler */

/* USER CODE BEGIN 0 */
#include "cmsis_os.h"

#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/udp.h"

#include "string.h"

#include "scramble.h"
/* USER CODE END 0 */
/* Private function prototypes -----------------------------------------------*/
/* ETH Variables initialization ----------------------------------------------*/
void _Error_Handler(char * file, int line);

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/* Variables Initialization */
struct netif gnetif;
ip4_addr_t ipaddr;
ip4_addr_t netmask;
ip4_addr_t gw;
uint8_t IP_ADDRESS[4];
uint8_t NETMASK_ADDRESS[4];
uint8_t GATEWAY_ADDRESS[4];

/* USER CODE BEGIN 2 */
osThreadId udpThreadHandle;

static void udpecho_raw_recv(void *arg, struct udp_pcb *pcb, struct pbuf *p,  const ip_addr_t *addr, u16_t port)
{
	LWIP_UNUSED_ARG(arg);
	if(p == NULL)
		return;

	if( p->len == SIZE_UDP_MSG ){
		uint8_t parity=0x00;
		for(int i=1; i<SIZE_UDP_MSG-2; i++){
			parity ^= ((char *)p->payload)[i];
		}
		if( parity == ((char *)p->payload)[SIZE_UDP_MSG-2] && ((char *)p->payload)[SIZE_UDP_MSG-2] ^ 0xff == ((char *)p->payload)[SIZE_UDP_MSG-1]){
			__disable_irq();
			memcpy(&udp_msg, p->payload, SIZE_UDP_MSG);
			udp_updated = 1;
			__enable_irq();
			//printf("%6d %6d %6d\r\n", (int)(udp_msg.vx*1000), (int)(udp_msg.vy*1000), (int)(udp_msg.omega*1000));
		}
	}

#if 0
	for(int i=0; i<p->len; i++){
		//putch(((char *)p->payload)[i]);
		printf("%02x ", ((char *)p->payload)[i]);
	}
	printf("\r\n");
#endif

	//udp_sendto(pcb, p, addr, port);
	pbuf_free(p);
}
/*-----------------------------------------------------------------------------------*/
static void udp_thread(void const * argument)
{
	struct udp_pcb *pcb;
	err_t err;

	LWIP_UNUSED_ARG(argument);
	printf("starting %s() ...", __func__);

	pcb = udp_new();
	if( pcb == NULL ){
		printf("udp_new() returned NULL");
	}

	err = udp_bind(pcb, IP_ADDR_ANY, 12345);
	if( err != ERR_OK ){
		printf("udp_bind() returned %d", err);
	}

	/* no need to loop forever */
	udp_recv(pcb , udpecho_raw_recv, pcb);

	for(;;){
		osDelay(1000);
	}
}
/*-----------------------------------------------------------------------------------*/
void udp_server_init(void)
{
	osThreadDef(udpThread, udp_thread, osPriorityNormal, 0, 1024);
	udpThreadHandle = osThreadCreate(osThread(udpThread), NULL);
}

void Scramble_LWIP_Init(void)
{

	// TODO: Change the IP address

	/* IP addresses initialization */
	IP_ADDRESS[0] = 192;
	IP_ADDRESS[1] = 168;
	IP_ADDRESS[2] = 15;
	IP_ADDRESS[3] = 10 + get_rotary_switch();
	NETMASK_ADDRESS[0] = 255;
	NETMASK_ADDRESS[1] = 255;
	NETMASK_ADDRESS[2] = 255;
	NETMASK_ADDRESS[3] = 0;
	GATEWAY_ADDRESS[0] = 192;
	GATEWAY_ADDRESS[1] = 168;
	GATEWAY_ADDRESS[2] = 15;
	GATEWAY_ADDRESS[3] = 1;

	printf("IP: %d.%d.%d.%d",IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);

	/* Initilialize the LwIP stack with RTOS */
	tcpip_init( NULL, NULL );

	/* IP addresses initialization without DHCP (IPv4) */
	IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
	IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
	IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

	for(;;){
		/* add the network interface (IPv4/IPv6) with RTOS */
		netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);
		if( netif_is_link_up(&gnetif) ){
			break;
		}
		netif_remove(&gnetif);
		osDelay(100);
	}

	/* Registers the default network interface */
	netif_set_default(&gnetif);


	netif_set_up(&gnetif);
	printf("netif_set_up");

	/* USER CODE BEGIN 3 */
	/* USER CODE END 3 */
}

/* USER CODE END 2 */

/**
  * LwIP initialization function
  */
void MX_LWIP_Init(void)
{
  /* IP addresses initialization */
  IP_ADDRESS[0] = 192;
  IP_ADDRESS[1] = 168;
  IP_ADDRESS[2] = 11;
  IP_ADDRESS[3] = 60;
  NETMASK_ADDRESS[0] = 255;
  NETMASK_ADDRESS[1] = 255;
  NETMASK_ADDRESS[2] = 255;
  NETMASK_ADDRESS[3] = 0;
  GATEWAY_ADDRESS[0] = 192;
  GATEWAY_ADDRESS[1] = 168;
  GATEWAY_ADDRESS[2] = 11;
  GATEWAY_ADDRESS[3] = 1;
  
  /* Initilialize the LwIP stack with RTOS */
  tcpip_init( NULL, NULL );

  /* IP addresses initialization without DHCP (IPv4) */
  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);

  /* add the network interface (IPv4/IPv6) with RTOS */
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &tcpip_input);

  /* Registers the default network interface */
  netif_set_default(&gnetif);

  if (netif_is_link_up(&gnetif))
  {
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }

/* USER CODE BEGIN 3 */
/* USER CODE END 3 */
}

#ifdef USE_OBSOLETE_USER_CODE_SECTION_4
/* Kept to help code migration. (See new 4_1, 4_2... sections) */
/* Avoid to use this user section which will become obsolete. */
/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
#endif

#if defined ( __CC_ARM )  /* MDK ARM Compiler */
/**
 * Opens a serial device for communication.
 *
 * @param devnum device number
 * @return handle to serial device if successful, NULL otherwise
 */
sio_fd_t sio_open(u8_t devnum)
{
  sio_fd_t sd;

/* USER CODE BEGIN 7 */
	sd = 0; // dummy code
/* USER CODE END 7 */
	
  return sd;
}

/**
 * Sends a single character to the serial device.
 *
 * @param c character to send
 * @param fd serial device handle
 *
 * @note This function will block until the character can be sent.
 */
void sio_send(u8_t c, sio_fd_t fd)
{
/* USER CODE BEGIN 8 */
/* USER CODE END 8 */
}

/**
 * Reads from the serial device.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received - may be 0 if aborted by sio_read_abort
 *
 * @note This function will block until data can be received. The blocking
 * can be cancelled by calling sio_read_abort().
 */
u32_t sio_read(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 9 */
	recved_bytes = 0; // dummy code
/* USER CODE END 9 */	
  return recved_bytes;
}

/**
 * Tries to read from the serial device. Same as sio_read but returns
 * immediately if no data is available and never blocks.
 *
 * @param fd serial device handle
 * @param data pointer to data buffer for receiving
 * @param len maximum length (in bytes) of data to receive
 * @return number of bytes actually received
 */
u32_t sio_tryread(sio_fd_t fd, u8_t *data, u32_t len)
{
  u32_t recved_bytes;

/* USER CODE BEGIN 10 */
	recved_bytes = 0; // dummy code
/* USER CODE END 10 */	
  return recved_bytes;
}
#endif /* MDK ARM Compiler */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
