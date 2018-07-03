/**
  ******************************************************************************
  * @file    stm324xg_eval.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    11-January-2013
  * @brief   This file provides
  *            - set of firmware functions to manage Leds, push-button and COM ports
  *            - low level initialization functions for SD card (on SDIO) and
  *              serial EEPROM (sEE)
  *          available on STM324xG-EVAL evaluation board(MB786) RevB from
  *          STMicroelectronics.
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
#include "stm324xg_eval.h"
#include "stm32f4xx_sdio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_i2c.h"

/** @addtogroup Utilities
  * @{
  */ 

/** @addtogroup STM32_EVAL
  * @{
  */ 

/** @addtogroup STM324xG_EVAL
  * @{
  */   
    
/** @defgroup STM324xG_EVAL_LOW_LEVEL 
  * @brief This file provides firmware functions to manage Leds, push-buttons, 
  *        COM ports, SD card on SDIO and serial EEPROM (sEE) available on 
  *        STM324xG-EVAL evaluation board from STMicroelectronics.
  * @{
  */ 

/** @defgroup STM324xG_EVAL_LOW_LEVEL_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM324xG_EVAL_LOW_LEVEL_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM324xG_EVAL_LOW_LEVEL_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup STM324xG_EVAL_LOW_LEVEL_Private_Variables
  * @{
  */ 


USART_TypeDef* COM_USART[COMn] = {EVAL_COM1}; 

GPIO_TypeDef* COM_TX_PORT[COMn] = {EVAL_COM1_TX_GPIO_PORT};
 
GPIO_TypeDef* COM_RX_PORT[COMn] = {EVAL_COM1_RX_GPIO_PORT};

const uint32_t COM_USART_CLK[COMn] = {EVAL_COM1_CLK};

const uint32_t COM_TX_PORT_CLK[COMn] = {EVAL_COM1_TX_GPIO_CLK};
 
const uint32_t COM_RX_PORT_CLK[COMn] = {EVAL_COM1_RX_GPIO_CLK};

const uint16_t COM_TX_PIN[COMn] = {EVAL_COM1_TX_PIN};

const uint16_t COM_RX_PIN[COMn] = {EVAL_COM1_RX_PIN};
 
const uint16_t COM_TX_PIN_SOURCE[COMn] = {EVAL_COM1_TX_SOURCE};

const uint16_t COM_RX_PIN_SOURCE[COMn] = {EVAL_COM1_RX_SOURCE};
 
const uint16_t COM_TX_AF[COMn] = {EVAL_COM1_TX_AF};
 
const uint16_t COM_RX_AF[COMn] = {EVAL_COM1_RX_AF};

DMA_InitTypeDef    sEEDMA_InitStructure; 
NVIC_InitTypeDef   NVIC_InitStructure;

/**
  * @}
  */ 


/** @defgroup STM324xG_EVAL_LOW_LEVEL_Private_FunctionPrototypes
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup STM324xG_EVAL_LOW_LEVEL_Private_Functions
  * @{
  */ 

/**
  * @brief  Configures LED GPIO.
  * @param  Led: Specifies the Led to be configured. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4
  * @retval None
  */


/**
  * @brief  Turns selected LED On.
  * @param  Led: Specifies the Led to be set on. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4  
  * @retval None
  */


/**
  * @brief  Turns selected LED Off.
  * @param  Led: Specifies the Led to be set off. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4 
  * @retval None
  */


/**
  * @brief  Toggles the selected LED.
  * @param  Led: Specifies the Led to be toggled. 
  *   This parameter can be one of following parameters:
  *     @arg LED1
  *     @arg LED2
  *     @arg LED3
  *     @arg LED4  
  * @retval None
  * @brief  Configures Button GPIO and EXTI Line.
  * @param  Button: Specifies the Button to be configured.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_WAKEUP: Wakeup Push Button
  *     @arg BUTTON_TAMPER: Tamper Push Button  
  *     @arg BUTTON_KEY: Key Push Button 
  *     @arg BUTTON_RIGHT: Joystick Right Push Button 
  *     @arg BUTTON_LEFT: Joystick Left Push Button 
  *     @arg BUTTON_UP: Joystick Up Push Button 
  *     @arg BUTTON_DOWN: Joystick Down Push Button
  *     @arg BUTTON_SEL: Joystick Sel Push Button
  * @param  Button_Mode: Specifies Button mode.
  *   This parameter can be one of following parameters:   
  *     @arg BUTTON_MODE_GPIO: Button will be used as simple IO 
  *     @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
  *                     generation capability  
  * @retval None
  */

/**
  * @brief  Returns the selected Button state.
  * @param  Button: Specifies the Button to be checked.
  *   This parameter can be one of following parameters:    
  *     @arg BUTTON_WAKEUP: Wakeup Push Button
  *     @arg BUTTON_TAMPER: Tamper Push Button  
  *     @arg BUTTON_KEY: Key Push Button 
  *     @arg BUTTON_RIGHT: Joystick Right Push Button 
  *     @arg BUTTON_LEFT: Joystick Left Push Button 
  *     @arg BUTTON_UP: Joystick Up Push Button 
  *     @arg BUTTON_DOWN: Joystick Down Push Button
  *     @arg BUTTON_SEL: Joystick Sel Push Button    
  * @retval The Button GPIO pin value.
  */


/**
  * @brief  Configures COM port.
  * @param  COM: Specifies the COM port to be configured.
  *   This parameter can be one of following parameters:    
  *     @arg COM1
  *     @arg COM2  
  * @param  USART_InitStruct: pointer to a USART_InitTypeDef structure that
  *   contains the configuration information for the specified USART peripheral.
  * @retval None
  */
void STM_EVAL_COMInit(COM_TypeDef COM, USART_InitTypeDef* USART_InitStruct)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(COM_TX_PORT_CLK[COM] | COM_RX_PORT_CLK[COM], ENABLE);

  if (COM == COM1)
  {
    /* Enable UART clock */
    RCC_APB1PeriphClockCmd(COM_USART_CLK[COM], ENABLE);
  }

  /* Connect PXx to USARTx_Tx*/
  GPIO_PinAFConfig(COM_TX_PORT[COM], COM_TX_PIN_SOURCE[COM], COM_TX_AF[COM]);

  /* Connect PXx to USARTx_Rx*/
  GPIO_PinAFConfig(COM_RX_PORT[COM], COM_RX_PIN_SOURCE[COM], COM_RX_AF[COM]);

  /* Configure USART Tx as alternate function  */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

  GPIO_InitStructure.GPIO_Pin = COM_TX_PIN[COM];
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(COM_TX_PORT[COM], &GPIO_InitStructure);

  /* Configure USART Rx as alternate function  */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Pin = COM_RX_PIN[COM];
  GPIO_Init(COM_RX_PORT[COM], &GPIO_InitStructure);

  /* USART configuration */
  USART_Init(COM_USART[COM], USART_InitStruct);
    
  /* Enable USART */
  USART_Cmd(COM_USART[COM], ENABLE);
}

/**
  * @brief  DeInitializes the SDIO interface.
  * @param  None
  * @retval None
  */
void SD_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /*!< Disable SDIO Clock */
  SDIO_ClockCmd(DISABLE);
  
  /*!< Set Power State to OFF */
  SDIO_SetPowerState(SDIO_PowerState_OFF);

  /*!< DeInitializes the SDIO peripheral */
  SDIO_DeInit();
  
  /* Disable the SDIO APB2 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, DISABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_MCO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_MCO);

  /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure PC.12 pin: CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

/**
  * @brief  Initializes the SD Card and put it into StandBy State (Ready for 
  *         data transfer).
  * @param  None
  * @retval None
  */
void SD_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* GPIOC and GPIOD Periph clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD | SD_DETECT_GPIO_CLK, ENABLE);

  GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SDIO);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_SDIO);

  /* Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /* Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  /* Configure PC.12 pin: CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  
  /*!< Configure SD_SPI_DETECT_PIN pin: SD Card detect pin */
  GPIO_InitStructure.GPIO_Pin = SD_DETECT_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(SD_DETECT_GPIO_PORT, &GPIO_InitStructure);

  /* Enable the SDIO APB2 Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);

  /* Enable the DMA2 Clock */
  RCC_AHB1PeriphClockCmd(SD_SDIO_DMA_CLK, ENABLE);
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Tx request.
  * @param  BufferSRC: pointer to the source buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF | SD_SDIO_DMA_FLAG_DMEIF | SD_SDIO_DMA_FLAG_TEIF | SD_SDIO_DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

  /* DMA2 Stream3  or Stream6 disable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

  /* DMA2 Stream3  or Stream6 Config */
  DMA_DeInit(SD_SDIO_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferSRC;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SD_SDIO_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  /* DMA2 Stream3  or Stream6 enable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
    
}

/**
  * @brief  Configures the DMA2 Channel4 for SDIO Rx request.
  * @param  BufferDST: pointer to the destination buffer
  * @param  BufferSize: buffer size
  * @retval None
  */
void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef SDDMA_InitStructure;

  DMA_ClearFlag(SD_SDIO_DMA_STREAM, SD_SDIO_DMA_FLAG_FEIF | SD_SDIO_DMA_FLAG_DMEIF | SD_SDIO_DMA_FLAG_TEIF | SD_SDIO_DMA_FLAG_HTIF | SD_SDIO_DMA_FLAG_TCIF);

  /* DMA2 Stream3  or Stream6 disable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, DISABLE);

  /* DMA2 Stream3 or Stream6 Config */
  DMA_DeInit(SD_SDIO_DMA_STREAM);

  SDDMA_InitStructure.DMA_Channel = SD_SDIO_DMA_CHANNEL;
  SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferDST;
  SDDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
  SDDMA_InitStructure.DMA_BufferSize = BufferSize;
  SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  SDDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
  SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
  DMA_Init(SD_SDIO_DMA_STREAM, &SDDMA_InitStructure);
  DMA_ITConfig(SD_SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);
  DMA_FlowControllerConfig(SD_SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

  /* DMA2 Stream3 or Stream6 enable */
  DMA_Cmd(SD_SDIO_DMA_STREAM, ENABLE);
}

/**
  * @brief  DeInitializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void sEE_LowLevel_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
   
  /* sEE_I2C Peripheral Disable */
  I2C_Cmd(sEE_I2C, DISABLE);
 
  /* sEE_I2C DeInit */
  I2C_DeInit(sEE_I2C);

  /*!< sEE_I2C Periph clock disable */
  RCC_APB1PeriphClockCmd(sEE_I2C_CLK, DISABLE);
    
  /*!< GPIO configuration */  
  /*!< Configure sEE_I2C pins: SCL */
  GPIO_InitStructure.GPIO_Pin = sEE_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(sEE_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sEE_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = sEE_I2C_SDA_PIN;
  GPIO_Init(sEE_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

  /* Configure and enable I2C DMA TX Stream interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = sEE_I2C_DMA_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = sEE_I2C_DMA_PREPRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = sEE_I2C_DMA_SUBPRIO;
  NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C DMA RX Stream interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = sEE_I2C_DMA_RX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = sEE_I2C_DMA_PREPRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = sEE_I2C_DMA_SUBPRIO;
  NVIC_Init(&NVIC_InitStructure);   
  
  /* Disable and Deinitialize the DMA Streams */
  DMA_Cmd(sEE_I2C_DMA_STREAM_TX, DISABLE);
  DMA_Cmd(sEE_I2C_DMA_STREAM_RX, DISABLE);
  DMA_DeInit(sEE_I2C_DMA_STREAM_TX);
  DMA_DeInit(sEE_I2C_DMA_STREAM_RX);
}

/**
  * @brief  Initializes peripherals used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void sEE_LowLevel_Init(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure; 
   
  /*!< sEE_I2C Periph clock enable */
  RCC_APB1PeriphClockCmd(sEE_I2C_CLK, ENABLE);
  
  /*!< sEE_I2C_SCL_GPIO_CLK and sEE_I2C_SDA_GPIO_CLK Periph clock enable */
  RCC_AHB1PeriphClockCmd(sEE_I2C_SCL_GPIO_CLK | sEE_I2C_SDA_GPIO_CLK, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  
  /* Reset sEE_I2C IP */
  RCC_APB1PeriphResetCmd(sEE_I2C_CLK, ENABLE);
  
  /* Release reset signal of sEE_I2C IP */
  RCC_APB1PeriphResetCmd(sEE_I2C_CLK, DISABLE);
 
  /*!< GPIO configuration */
  /* Connect PXx to I2C_SCL*/
  GPIO_PinAFConfig(sEE_I2C_SCL_GPIO_PORT, sEE_I2C_SCL_SOURCE, sEE_I2C_SCL_AF);
  /* Connect PXx to I2C_SDA*/
  GPIO_PinAFConfig(sEE_I2C_SDA_GPIO_PORT, sEE_I2C_SDA_SOURCE, sEE_I2C_SDA_AF);  
  
  /*!< Configure sEE_I2C pins: SCL */   
  GPIO_InitStructure.GPIO_Pin = sEE_I2C_SCL_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_Init(sEE_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);

  /*!< Configure sEE_I2C pins: SDA */
  GPIO_InitStructure.GPIO_Pin = sEE_I2C_SDA_PIN;
  GPIO_Init(sEE_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);

 
  /* Configure and enable I2C DMA TX Channel interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = sEE_I2C_DMA_TX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = sEE_I2C_DMA_PREPRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = sEE_I2C_DMA_SUBPRIO;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* Configure and enable I2C DMA RX Channel interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = sEE_I2C_DMA_RX_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = sEE_I2C_DMA_PREPRIO;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = sEE_I2C_DMA_SUBPRIO;
  NVIC_Init(&NVIC_InitStructure);  
  
  /*!< I2C DMA TX and RX channels configuration */
  /* Enable the DMA clock */
  RCC_AHB1PeriphClockCmd(sEE_I2C_DMA_CLK, ENABLE);
  
  /* Clear any pending flag on Rx Stream  */
  DMA_ClearFlag(sEE_I2C_DMA_STREAM_TX, sEE_TX_DMA_FLAG_FEIF | sEE_TX_DMA_FLAG_DMEIF | sEE_TX_DMA_FLAG_TEIF | \
                                       sEE_TX_DMA_FLAG_HTIF | sEE_TX_DMA_FLAG_TCIF);
  /* Disable the EE I2C Tx DMA stream */
  DMA_Cmd(sEE_I2C_DMA_STREAM_TX, DISABLE);
  /* Configure the DMA stream for the EE I2C peripheral TX direction */
  DMA_DeInit(sEE_I2C_DMA_STREAM_TX);
  sEEDMA_InitStructure.DMA_Channel = sEE_I2C_DMA_CHANNEL;
  sEEDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)sEE_I2C_DR_Address;
  sEEDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)0;    /* This parameter will be configured durig communication */;
  sEEDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral; /* This parameter will be configured durig communication */
  sEEDMA_InitStructure.DMA_BufferSize = 0xFFFF;              /* This parameter will be configured durig communication */
  sEEDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  sEEDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  sEEDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  sEEDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  sEEDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  sEEDMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  sEEDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
  sEEDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
  sEEDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  sEEDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(sEE_I2C_DMA_STREAM_TX, &sEEDMA_InitStructure);

  /* Clear any pending flag on Rx Stream */
  DMA_ClearFlag(sEE_I2C_DMA_STREAM_RX, sEE_RX_DMA_FLAG_FEIF | sEE_RX_DMA_FLAG_DMEIF | sEE_RX_DMA_FLAG_TEIF | \
                                       sEE_RX_DMA_FLAG_HTIF | sEE_RX_DMA_FLAG_TCIF);
  /* Disable the EE I2C DMA Rx stream */
  DMA_Cmd(sEE_I2C_DMA_STREAM_RX, DISABLE);
  /* Configure the DMA stream for the EE I2C peripheral RX direction */
  DMA_DeInit(sEE_I2C_DMA_STREAM_RX);
  DMA_Init(sEE_I2C_DMA_STREAM_RX, &sEEDMA_InitStructure);
  
  /* Enable the DMA Channels Interrupts */
  DMA_ITConfig(sEE_I2C_DMA_STREAM_TX, DMA_IT_TC, ENABLE);
  DMA_ITConfig(sEE_I2C_DMA_STREAM_RX, DMA_IT_TC, ENABLE);      
}

/**
  * @brief  Initializes DMA channel used by the I2C EEPROM driver.
  * @param  None
  * @retval None
  */
void sEE_LowLevel_DMAConfig(uint32_t pBuffer, uint32_t BufferSize, uint32_t Direction)
{ 
  /* Initialize the DMA with the new parameters */
  if (Direction == sEE_DIRECTION_TX)
  {
    /* Configure the DMA Tx Stream with the buffer address and the buffer size */
    sEEDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pBuffer;
    sEEDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;    
    sEEDMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;  
    DMA_Init(sEE_I2C_DMA_STREAM_TX, &sEEDMA_InitStructure);  
  }
  else
  { 
    /* Configure the DMA Rx Stream with the buffer address and the buffer size */
    sEEDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pBuffer;
    sEEDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    sEEDMA_InitStructure.DMA_BufferSize = (uint32_t)BufferSize;      
    DMA_Init(sEE_I2C_DMA_STREAM_RX, &sEEDMA_InitStructure);    
  }
}

/**
  * @}
  */ 


/**
  * @}
  */ 

/**
  * @}
  */

/**
  * @}
  */    

/**
  * @}
  */ 
    
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
