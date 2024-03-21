/**
  ******************************************************************************
  * @file    rs485_interface.h
  * @author  MCD Application Team
  * @brief   Header for rs485_interface.c module
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * modified by Philip Kiehnle
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RS485_INTERFACE_H
#define RS485_INTERFACE_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void OPENBL_RS485_Configuration(void);
uint8_t OPENBL_RS485_ProtocolDetection(void);

uint8_t OPENBL_RS485_GetCommandOpcode(void);
uint8_t OPENBL_RS485_ReadByte(void);
void OPENBL_RS485_SendByte(uint8_t Byte);

#endif /* RS485_INTERFACE_H */


