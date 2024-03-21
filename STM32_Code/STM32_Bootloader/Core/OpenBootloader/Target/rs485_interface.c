/**
  ******************************************************************************
  * @file    rs485_interface.c
  * @author  MCD Application Team
  * @brief   Contains RS485 HW configuration
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"

#include "openbl_core.h"
#include "openbl_usart_cmd.h"

#include "rs485_interface.h"
#include "iwdg_interface.h"
#include "interfaces_conf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void OPENBL_RS485_Init(void);

/* Private functions ---------------------------------------------------------*/

static void OPENBL_RS485_Init(void)
{
  LL_USART_InitTypeDef USART_InitStruct;

  USART_InitStruct.PrescalerValue      = LL_USART_PRESCALER_DIV1;
  USART_InitStruct.BaudRate            = 115200U;
  USART_InitStruct.DataWidth           = LL_USART_DATAWIDTH_9B;
  USART_InitStruct.StopBits            = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity              = LL_USART_PARITY_EVEN;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.TransferDirection   = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.OverSampling        = LL_USART_OVERSAMPLING_16;

  if (IS_USART_AUTOBAUDRATE_DETECTION_INSTANCE(RS485x))
  {
    LL_USART_EnableAutoBaudRate(RS485x);
    LL_USART_SetAutoBaudRateMode(RS485x, LL_USART_AUTOBAUD_DETECT_ON_7F_FRAME);
  }
  else
  {
    LL_USART_DisableAutoBaudRate(RS485x);
    USART_InitStruct.BaudRate = 115200;
  }

  LL_USART_Init(RS485x, &USART_InitStruct);
  LL_USART_Enable(RS485x);
}

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  This function is used to configure RS485 pins and then initialize the used RS485 instance.
 * @retval None.
 */
void OPENBL_RS485_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  /* Enable all resources clocks --------------------------------------------*/
  /* Enable used GPIOx clocks */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /* Enable UART clock */
  __HAL_RCC_UART5_CLK_ENABLE();

  /* USART1 pins configuration -----------------------------------------------*/

  GPIO_InitStruct.Pin       = RS485x_TX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = RS485x_ALTERNATE;
  HAL_GPIO_Init(RS485x_TX_GPIO_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = RS485x_RX_PIN;
  HAL_GPIO_Init(RS485x_RX_GPIO_PORT, &GPIO_InitStruct);

  OPENBL_RS485_Init();
}

/**
 * @brief  This function is used to detect if there is any activity on RS485 protocol.
 * @retval None.
 */
uint8_t OPENBL_RS485_ProtocolDetection(void)
{
  uint8_t detected;

  /* Check if the RS485x is addressed */
  // do not check auto baud rate flags for now
  //if (((RS485x->ISR & LL_USART_ISR_ABRF) != 0) && ((RS485x->ISR & LL_USART_ISR_ABRE) == 0))
  if ((RS485x->ISR & USART_ISR_RXNE_RXFNE))  // check if byte received
  {
    /* Read byte in order to flush the 0x7F synchronization byte */
    OPENBL_RS485_ReadByte();

    /* Acknowledge the host */
    OPENBL_RS485_SendByte(ACK_BYTE);

    detected = 1;
  }
  else
  {
    detected = 0;
  }

  return detected;
}

/**
 * @brief  This function is used to get the command opcode from the host.
 * @retval Returns the command.
 */
uint8_t OPENBL_RS485_GetCommandOpcode(void)
{
  uint8_t command_opc = 0x0;

  /* Get the command opcode */
  command_opc = OPENBL_RS485_ReadByte();

  /* Check the data integrity */
  if ((command_opc ^ OPENBL_RS485_ReadByte()) != 0xFF)
  {
    command_opc = ERROR_COMMAND;
  }
  else
  {
    /* nothing to do */
  }

  return command_opc;
}

/**
  * @brief  This function is used to read one byte from RS485 pipe.
  * @retval Returns the read byte.
  */
uint8_t OPENBL_RS485_ReadByte(void)
{
  while (!LL_USART_IsActiveFlag_RXNE_RXFNE(RS485x))
  {
    OPENBL_IWDG_Refresh();
  }

  return LL_USART_ReceiveData8(RS485x);
}

/**
  * @brief  This function is used to send one byte through RS485 pipe.
  * @param  Byte The byte to be sent.
  * @retval None.
  */
void OPENBL_RS485_SendByte(uint8_t Byte)
{
  LL_USART_TransmitData8(RS485x, (Byte & 0xFF));

  while (!LL_USART_IsActiveFlag_TC(RS485x))
  {
  }
}


