/**
  ******************************************************************************
  * @file    main.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013
  * @brief   Main program body
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
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "lwip/tcp.h"
#include "serial_debug.h"
#include "led.h"
#include "stm32f4xx_usart.h"
#include "udp_echoserver.h"
#include "udp_echoclient.h"
#include	"pbuf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10

/*--------------- LCD Messages ---------------*/
#if defined (STM32F40XX)
#define MESSAGE1   "    STM32F40/41x     "
#elif defined (STM32F427X)
#define MESSAGE1   "     STM32F427x      "
#endif
#define MESSAGE2   "  STM32F-4 Series   "
#define MESSAGE3   " UDP echoclient Demo"
#define MESSAGE4   "                    "

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
u32_t timingdelay;

uint8_t TEXT_TO_SEND2[]={"ALIENTEK Explorer STM32F4 DMA 串口实验"};
u16_t Depacket_len = 0;
u8_t Packet_buff[1800];/*unsigned char*/
u8_t Depacket_buff[1800];
u8_t Depacket_state = 0;
QUEUE Transmit_buffer;
QUEUE Receive_buffer;
PQUEUE Trans = &Transmit_buffer; 
PQUEUE Recv = &Receive_buffer; 
struct pbuf *p;

/* Private function prototypes -----------------------------------------------*/
void LCD_LED_BUTTON_Init(void);

/* Private functions ---------------------------------------------------------*/


void Task_RecvData(void);
void Task_SendData(void);
void CreateQueue(PQUEUE Q,int maxsize);
int Enqueue(PQUEUE Q, u8_t DATA);
int Dequeue(PQUEUE Q, u8_t *val);
int FullQueue(PQUEUE Q);
int EmptyQueue(PQUEUE Q);
void Depacket_Data(u8_t in);


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{	
	  /*!< At this stage the microcontroller clock setting is already configured to 
       168 MHz, this is done through SystemInit() function which is called from
       startup file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f4xx.c file
     */

	long i;
	memset(Packet_buff,0,3600);     //这个地方并不会报错
	memset(Depacket_buff,0,3600);  /*两个指定空间置零了*/
	CreateQueue(Recv, QUEUE_MAX_SIZE); /*开辟了5*1500大小的空间*/
	CreateQueue(Trans, QUEUE_MAX_SIZE);

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

#ifdef SERIAL_DEBUG
  DebugComPort_Init();
#endif

  /* Configure ethernet (GPIOs, clocks, MAC, DMA) */
  ETH_BSP_Config();

  /* Initilaize the LwIP stack */
  LwIP_Init();
	LED_GPIO_Config();
	USART2_Init();
	USART1_Init();
	LED1_ON;
	udp_echoserver_init();
  /* Infinite loop */
  while (1)
  {  
    /* check if any packet received */
    if (ETH_CheckFrameReceived())
    { 
      /* process received ethernet packet */
      LwIP_Pkt_Handle();
    }
    /* handle periodic timers for LwIP */
    //-LwIP_Periodic_Handle(LocalTime);
		Task_RecvData();
		/*udp_echoclient_connect()test*/;
//		Task_SendData();
//			if(i<4000000)
//		{
//			i++;
//		}
//		else
//		{
//			udp_echoclient_connect();
//			i=0;
//		}
  }
}

             /*  new added Tasks  */
void Task_RecvData(void)
{
	//if (ETH_CheckFrameReceived())
   // { 
      /* process received ethernet packet */
     // LwIP_Pkt_Handle();
    //}
	u8_t val;

	while(Dequeue(Recv, &val))
	{
		LED2_ON;
		Depacket_Data(val);
	}
	LED2_OFF;
    /* handle periodic timers for LwIP */
    //LwIP_Periodic_Handle(LocalTime);
}

void Task_SendData(void)
{
	
}
                           /* 串口中断处理函数 */
void USART2_IRQHandler(void) /*USART中断处理函数*/
{
  u16_t temp;
  /* USART in Receiver mode */
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)/*USART_IT_RXNE: the receiving data register not empty*/
  {
		/*
		if(First_Data == 1)
		{
			TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
		}
		*/
		/* Receive Transaction data */
		temp = USART_ReceiveData(USART2); /*return (uint16_t)(USARTx->DR & (uint16_t)0x01FF);*/
		Enqueue(Recv, temp);
		
		USART_ClearITPendingBit(USART2, USART_IT_RXNE);
		/* Disable the Rx buffer not empty interrupt */
		//USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
  }
	
  /* USART in Tramitter mode */
  if (USART_GetITStatus(USART2, USART_IT_TXE) == SET)
  {
		/* Send Transaction data */
		USART_ClearITPendingBit(USART2, USART_IT_TXE);
		/* Disable the Tx buffer empty interrupt */
		USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  }
}

/**
  * @brief  Inserts a delay time.
  * @param  nCount: number of 10ms periods to wait for.
  * @retval None
  */
void Delay(uint32_t nCount)
{
  /* Capture the current local time */
  timingdelay = LocalTime + nCount;  

  /* wait until the desired delay finish */
  while(timingdelay > LocalTime)
  {     
  }
}


/****************************Queue Operation***********************/
void CreateQueue(PQUEUE Q,int maxsize)
{
	Q->front=0;         //初始化参数  
	Q->rear=0;  
	Q->maxsize=maxsize;
}

int FullQueue(PQUEUE Q)
{
	if(Q->front==(Q->rear+1)%Q->maxsize)    //判断循环链表是否满，留一个预留空间不用  
        return true;  
    else  
        return false;  
}

int EmptyQueue(PQUEUE Q)  
{  
    if(Q->front==Q->rear)    //判断是否为空  
        return true;  
    else  
        return false;  
}

int Enqueue(PQUEUE Q, u8_t val)  //进队列
{  
    if(FullQueue(Q))  
        return false;  
    else  
    {  
				Q->pBase[Q->rear]=val;  
				Q->rear=(Q->rear+1)%Q->maxsize;  
        return true;  
    }  
}

int Dequeue(PQUEUE Q, u8_t *val)  //出队列
{  
    if(EmptyQueue(Q))  
    {  
        return false;  
    }  
    else  
    {
				*val=Q->pBase[Q->front]; /*？*/ 
				Q->front=(Q->front+1)%Q->maxsize;  
        return true;  
    }  
}


void Depacket_Data(u8_t in)
{	
	static int counter = 0;
	switch(Depacket_state){
		case 0:
			if(in == 0xEB)
				Depacket_state = 1;
			else 
				Depacket_state = 0;
			break;
			
		case 1:
			if(in == 0x90)
				Depacket_state = 2;
			else 
				Depacket_state = 0;
			break;
			
		case 2:
			if(in == 0xFd)
				Depacket_state = 3;
			else 
				Depacket_state = 0;
			break;
			
		case 3:
			if(in == 0x0d)
				Depacket_state = 4;
			else 
				Depacket_state = 0;
			break;
			
		case 4:
			Depacket_len = in<<8;
			Depacket_state = 5;
			break;
		
		case 5:
			Depacket_len += in;   //这不就2*len？
			Depacket_state = 6;
			break;
		
		case 6:
			if(Depacket_len > 0)
			{
					Depacket_buff[counter++] = in;
					if(counter >= Depacket_len)
					{
						int i = 0;
						struct pbuf *packet_buff;
//						USART1_output(Depacket_buff,Depacket_len);
//						LED2_OFF;
						//DP83848Send(Depacket_buff,Depacket_len);
						i=sizeof(Depacket_buff);
						packet_buff = pbuf_alloc(PBUF_RAW, counter, PBUF_POOL);// the length of the frame = sizeof(Depacket_buff)
						pbuf_take(packet_buff,Depacket_buff,counter);
						/*
						if (p != NULL)
						{
							u8_t remainder[1800];
							p->payload = (void*) ((u8_t *)p->payload - PBUF_LINK_HLEN);//check
							
							p->len = p->len + PBUF_LINK_HLEN;
							memcpy(remainder, Depacket_buff, sizeof(Depacket_buff));
							while(p->next != NULL)
							{
								int j = 0;
								for(i = 0;i < p->len; i++)
								{
									*((u8_t *)p->payload) = remainder[i];//check
									p->payload = (void *)((u8_t *)p->payload + 1); //check
								}
								memset(remainder, '\0', sizeof(remainder));
								for(i = p->len;i < counter; i++)
								{									
									remainder[j] = Depacket_buff[i];
									j++;
								}
								p = p->next;
							}
						}
						*/
						New_LwIP_Pkt_Handle(packet_buff);
						Depacket_state = 0;
						counter = 0;
						//return Depacket_buff; //here, MAC frame is cotained in the Depacket_buff;
					}

			}
			else{
				Depacket_state = 0;
				counter = 0;
			}
			break;
	}
}

/**
  * @brief  Updates the system local time
  * @param  None
  * @retval None
  */
void Time_Update(void)
{
  LocalTime += SYSTEMTICK_PERIOD_MS;
}

/**
  * @brief  Initializes STM324xG-EVAL's LCD, LEDs and push-buttons resources.
  * @param  None
  * @retval None
  */

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

