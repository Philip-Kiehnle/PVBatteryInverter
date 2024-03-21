/**
  ******************************************************************************
  * @file    interfaces_conf.h
  * @author  MCD Application Team
  * @brief   Contains Interfaces configuration
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INTERFACES_CONF_H
#define INTERFACES_CONF_H

#define MEMORIES_SUPPORTED                7U

/* ------------------------- Definitions for USART -------------------------- */
#define USARTx                            USART3

#define USARTx_TX_PIN                     GPIO_PIN_10
#define USARTx_TX_GPIO_PORT               GPIOC
#define USARTx_RX_PIN                     GPIO_PIN_11
#define USARTx_RX_GPIO_PORT               GPIOC
#define USARTx_ALTERNATE                  GPIO_AF7_USART3

/* ------------------------- Definitions for UART_RS485 -------------------------- */
#define RS485x                            UART5

#define RS485x_TX_PIN                     GPIO_PIN_12
#define RS485x_TX_GPIO_PORT               GPIOC
#define RS485x_RX_PIN                     GPIO_PIN_2
#define RS485x_RX_GPIO_PORT               GPIOD
#define RS485x_ALTERNATE                  GPIO_AF5_UART5


#endif /* INTERFACES_CONF_H */


