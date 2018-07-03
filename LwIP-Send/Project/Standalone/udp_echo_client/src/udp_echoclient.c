/**
  ******************************************************************************
  * @file    udp_echoclient.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013
  * @brief   UDP echo client
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "led.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port);

u8_t   data[100];
__IO uint32_t message_count = 0;


/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Connect to UDP echo server
  * @param  None
  * @retval None
  */
void udp_echoclient_connect(void)
{
  struct udp_pcb *upcb;
  struct pbuf *p;
  struct ip_addr DestIPaddr;
  err_t err;

  /* Create a new UDP control block  */
  upcb = udp_new();
  
  if (upcb!=NULL)
  {
    /*assign destination IP address */
    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
  
    /* configure destination IP address and port */
    err= udp_connect(upcb, &DestIPaddr, UDP_SERVER_PORT);
    
    if (err == ERR_OK)
    {
      /* Set a receive callback for the upcb */
      udp_recv(upcb, udp_receive_callback, NULL);
      
      sprintf((char*)data, "sending udp client message %d", message_count);
  
      /* allocate pbuf from pool*/
      p = pbuf_alloc(PBUF_TRANSPORT,strlen((char*)data), PBUF_POOL);
      
      if (p != NULL)
      {
        /* copy data to pbuf */
        pbuf_take(p, (char*)data, strlen((char*)data));
          
        /* send udp data */
        udp_send(upcb, p); 
        
        /* free pbuf */
        pbuf_free(p);
      }
      else
      {
        /* free the UDP connection, so we can accept new clients */
        udp_remove(upcb);
        #ifdef SERIAL_DEBUG
        printf("\n\r can not allocate pbuf ");
        #endif
      }
    }
    else
    {
      /* free the UDP connection, so we can accept new clients */
      udp_remove(upcb);
      #ifdef SERIAL_DEBUG
       printf("\n\r can not connect udp pcb");
      #endif
    }
  }
  else
  {
    #ifdef SERIAL_DEBUG
     printf("\n\r can not create udp pcb");
    #endif
  } 
}


/**
  * @brief This function is called when an UDP datagrm has been received on the port UDP_PORT.
  * @param arg user supplied argument (udp_pcb.recv_arg)
  * @param pcb the udp_pcb which received data
  * @param p the packet buffer that was received
  * @param addr the remote IP address from which the packet was received
  * @param port the remote port from which the packet was received
  * @retval None
  */
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, struct ip_addr *addr, u16_t port)
{
	LED2_ON;
  /*increment message count */
  message_count++;
  /* Free receive pbuf */
  pbuf_free(p);
  Delay(300);
  /* free the UDP connection, so we can accept new clients */
  udp_remove(upcb); 
	LED2_ON;
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
