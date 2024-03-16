
#include "stm32g4xx_hal.h"

#include <stdio.h>

#include "serial.h"
#include "electricity_meter.h"
#include <sml/sml_parser.h>


#define METER_READ_DMA 0  // 0 means polling, which is better tested and finds sml start sequence more reliably
#define COMM_ERR_TIMEOUT_SEC 20

#define BYTES_PER_READ 16

static char rx_buf[EL_METER_RX_BUF_TYP_SIZE+2*BYTES_PER_READ];
static int pcc_power;
static enum el_meter_status_t el_meter_status;


#if METER_READ_DMA == 1
__IO ITStatus UartReady = RESET;

//reports end of DMA Rx transfer
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
  UartReady = SET;

}
#endif

// test in linux:
// stty raw -F /dev/ttyUSB1 9600  # without raw, linux adds \r to \n and corrupts data!
// cat testmsg.bin > /dev/ttyUSB1
// cat /dev/ttyUSB0 > /dev/ttyUSB2  # UART relay
// cat /dev/ttyUSB0 | /data/local/nc.openbsd -l 55000  # tx command for UART WiFi bridge
// nc 192.168.2.2 55000 > /dev/ttyUSB1  # rx command for UART WiFi bridge
el_meter_status_t electricity_meter_read(UART_HandleTypeDef* huart)
{
	el_meter_status = EL_METER_CONN_WARN;
	static uint16_t smart_meter_err_cnt = 0;
	static uint32_t smart_meter_no_comm_cnt = 0;

	uint16_t rx_len = 0;

#if METER_READ_DMA == 1
	static bool rx_started = false;

	if (!rx_started) {

	  /*## Put UART peripheral in reception process ###########################*/
	  // half full for FIFO size 8 -> every 4 bytes, interrupt stores them to memory

		// DMA
		if (HAL_UART_Receive_DMA(huart, (uint8_t *)rx_buf, EL_METER_RX_BUF_TYP_SIZE) != HAL_OK)	{
			Error_Handler();
		}

		// Interrupt does not work with FiFo mode
//		if (HAL_UART_Receive_IT(huart, (uint8_t *)rx_buf, EL_METER_RX_BUF_TYP_SIZE) != HAL_OK) {
//			Error_Handler();
//		}

		rx_started = true;
	}

	if ( UartReady == SET) {
		rx_started = false;
	    UartReady = RESET;
#else  // polling mode. Advantage: direct search for start sequence.
	static volatile uint16_t n_msg = 0;  // number of bytes in message / write position in rx buffer
	static uint8_t n_start_bytes = 0;
	static uint8_t n_smlstart_bytes = 0;

	if (n_msg == 0) {
		n_start_bytes = 0;
		n_smlstart_bytes = 0;
	}
	HAL_UARTEx_ReceiveToIdle(huart, (uint8_t *)&rx_buf[n_msg], BYTES_PER_READ, &rx_len, 2);  // 2ms timeout

	if (rx_len == 0) {
		if (smart_meter_no_comm_cnt < COMM_ERR_TIMEOUT_SEC*200) {  // 1sec/2ms = 500 but takes longer -> 200
			smart_meter_no_comm_cnt++;
		} else {
			el_meter_status = EL_METER_CONN_ERR;
		}
	}

	for (uint8_t i = 0; i < rx_len; i++ ) {
			// keep bytes only if start sequence detected
			// 1B 1B 1B 1B         Escape-Sequence
			// 01 01 01 01         Begin of SML file (Version 1)
			if (n_start_bytes < 4) {
				if (rx_buf[n_msg] == 0x1b) {
					n_start_bytes++;
					n_msg++;
				} else {
					n_msg = 0;
				}
			} else {  // n_start_bytes==4 means start bytes found
				if (n_smlstart_bytes < 4) {
					if (rx_buf[n_msg] == 0x01) {
						n_smlstart_bytes++;
						n_msg++;
					} else {
						n_msg = 0;
					}
				} else {  // n_smlstart_bytes==4 means sml start found
					n_msg++;
				}
			}
	}

	if ( n_msg >= EL_METER_RX_BUF_TYP_SIZE ) {
        n_msg = 0;
#endif

		if ( *((uint32_t*) rx_buf) == 0x1b1b1b1b) {
			meterdata_t meterdata = {0};
			if (parse_sml(rx_buf, EL_METER_RX_BUF_TYP_SIZE, &meterdata) == 1) {
				pcc_power = meterdata.power;
				smart_meter_err_cnt = 0;
				smart_meter_no_comm_cnt = 0;
				el_meter_status = EL_METER_OKAY;
			} else {
				if (smart_meter_err_cnt > 20) {  // 20 sec without successful read triggers error
					el_meter_status = EL_METER_CONN_ERR;
				} else {
					smart_meter_err_cnt++;
				}
			}
		}

		// clean rx fifo
		HAL_UARTEx_ReceiveToIdle(huart, (uint8_t *)rx_buf, BYTES_PER_READ, &rx_len, 2);  // 2ms timeout

	}

	return el_meter_status;
}


el_meter_status_t electricity_meter_get_status()
{
	return el_meter_status;
}


int electricity_meter_get_power()
{
	return pcc_power;
}

