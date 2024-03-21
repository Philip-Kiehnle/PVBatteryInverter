/**
  ******************************************************************************
  * @file    openbl_rs485_cmd.h
  * @author  MCD Application Team   modified by Philip Kiehnle
  * @brief   Header for openbl_rs485_cmd.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OPENBL_RS485_CMD_H
#define OPENBL_RS485_CMD_H

/* Includes ------------------------------------------------------------------*/
#include "openbl_core.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define OPENBL_RS485_VERSION                 0x31U               /* Open Bootloader RS485 protocol V3.1 */

/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern OPENBL_CommandsTypeDef OPENBL_RS485_Commands;

/* Exported functions ------------------------------------------------------- */
OPENBL_CommandsTypeDef *OPENBL_RS485_GetCommandsList(void);

#endif /* OPENBL_RS485_CMD_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
