
#include "serial.h"

extern UART_HandleTypeDef huart5;

__IO ITStatus UartReady = RESET;

void serial_write(int serial_fd_, const char* data, int size)
{
	HAL_UART_Transmit(&huart5, (uint8_t*)data, size, 10);

	  /*##-3- Start the transmission process #####################################*/
	  /* While the UART in reception process, user can transmit data through
	     "aTxBuffer" buffer */
//	  if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)aTxBuffer, TXBUFFERSIZE) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }
}


int serial_read(int serial_fd_, char* data, int size, int timeout_usec)
{

// V1: polling 1 byte is too slow without UART in FIFO mode

	uint16_t rx_len = 0;

	for (int i= 0; i<size; i++) {

		HAL_StatusTypeDef status = HAL_UART_Receive(&huart5, (uint8_t*)&data[rx_len], 1, timeout_usec/1000);

		if (status == HAL_TIMEOUT) {
			break;
		} else if (status != HAL_OK) {
			Error_Handler();
		}
		rx_len++;
	}

	return rx_len;


  // V2: polling all bytes at once

//	HAL_StatusTypeDef status = HAL_UART_Receive(&huart5, (uint8_t*)&data, size, timeout_usec/1000);
//
//	if (status == HAL_TIMEOUT) {
//		return 0;
//	} else if (status != HAL_OK) {
//		Error_Handler();
//	}
//
//  	return size;


//	// V3: DMA
//  if (HAL_UART_Receive_DMA(&huart5, (uint8_t*)data, size) != HAL_OK)
//  {
//	Error_Handler();
//  }
//
//  /*##-4- Wait for the end of the transfer ###################################*/
//  while (UartReady != SET)
//  {
//  }
//
//  /* Reset transmission flag */
//  UartReady = RESET;
}
