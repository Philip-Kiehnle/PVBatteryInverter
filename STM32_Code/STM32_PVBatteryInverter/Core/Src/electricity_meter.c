
#include "stm32g4xx_hal.h"

#include <stdio.h>

#include "serial.h"
#include "electricity_meter.h"
#include <sml/sml_crc16.h>

//#include <byteswap.h>
/* Swap bytes in 16 bit value.  */
#ifndef __bswap_constant_16
#define __bswap_constant_16(x)					\
  ((__uint16_t) ((((x) >> 8) & 0xff) | (((x) & 0xff) << 8)))
#endif

// /* Swap bytes in 32 bit value.  */
#ifndef __bswap_constant_32
#define __bswap_constant_32(x) \
     ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >>  8) |                      \
      (((x) & 0x0000ff00) <<  8) | (((x) & 0x000000ff) << 24))
#endif

#define METER_READ_DMA 0  // 0 means polling, which is better tested and finds sml start sequence more reliably
#define COMM_ERR_TIMEOUT_SEC 20

#define BYTES_PER_READ 16

static char rx_buf[EL_METER_RX_BUF_TYP_SIZE+2*BYTES_PER_READ];
static int pcc_power;


static int parse_sml(const char* data, int size, meterdata_t* meterdata)
{
    int success = 0;
    uint16_t sml_start_idx = 0;
    uint16_t sml_stop_idx = 0;

//    for (int j = 0; j<size; j++) {
//        printf("0x%.2X, ", (uint8_t)data[j]);
//    }
//#define BUFLEN 151
//uint8_t buf[BUFLEN] = {0x76, 0x05, 0x00, 0x0F, 0xB5, 0xCC, 0x62, 0x00, 0x62, 0x00, 0x72, 0x63, 0x07, 0x01, 0x77, 0x01, 0x0B, 0x0A, 0x01, 0x49, 0x53, 0x4B, 0x00, 0x05, 0x02, 0x12, 0x45, 0x07, 0x01, 0x00, 0x62, 0x0A, 0xFF, 0xFF, 0x72, 0x62, 0x01, 0x65, 0x00, 0x05, 0x3A, 0x46, 0x75, 0x77, 0x07, 0x01, 0x00, 0x60, 0x32, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x04, 0x49, 0x53, 0x4B, 0x01, 0x77, 0x07, 0x01, 0x00, 0x60, 0x01, 0x00, 0xFF, 0x01, 0x01, 0x01, 0x01, 0x0B, 0x0A, 0x01, 0x49, 0x53, 0x4B, 0x00, 0x05, 0x02, 0x12, 0x45, 0x01, 0x77, 0x07, 0x01, 0x00, 0x01, 0x08, 0x00, 0xFF, 0x65, 0x00, 0x1C, 0x19, 0x04, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x65, 0x00, 0x01, 0xB2, 0xFC, 0x01, 0x77, 0x07, 0x01, 0x00, 0x02, 0x08, 0x00, 0xFF, 0x01, 0x01, 0x62, 0x1E, 0x52, 0xFF, 0x65, 0x00, 0x10, 0xC9, 0x92, 0x01, 0x77, 0x07, 0x01, 0x00, 0x10, 0x07, 0x00, 0xFF, 0x01, 0x01, 0x62, 0x1B, 0x52, 0x00, 0x53, 0xEF, 0xFB, 0x01, 0x01, 0x01, 0x63, 0x76, 0x2C};

    for (int i = 0; i < size-8; i++) {
        if ( (i > 53) && *((uint16_t*)(data+i)) == 0x0576 ) { // second SML-Message
            sml_start_idx = i;
        }

        if ( *((uint64_t*)(data+i)) == 0x65FF000801000107) {  // 1.8.1 Wirkarbeit Bezug (in 0.1 Wh) //0x07010001 0800FF65 -> bigend

            if ( *(uint8_t*) (data+i+17) == 0x65) {  // 0x65 -> datatype is uint32
                meterdata->e_consumed =  (uint32_t) __bswap_constant_32( *(uint32_t*) (data+i+18) );
            } else {
                return 0;
            }
        }

        if ( *((uint64_t*)(data+i)) == 0x01FF000802000107) {  // 2.8.1 Wirkarbeit Lieferung (in 0.1 Wh) //0x07010002 0800FF01 -> bigend

            if ( *(uint8_t*) (data+i+13) == 0x65) {  // 0x65 -> datatype is uint32
                meterdata->e_produced =  (uint32_t) __bswap_constant_32( *(uint32_t*) (data+i+14) );
            } else {
                return 0;
            }
        }

        if ( *((uint64_t*)(data+i)) == 0x01FF000710000107) {  // 16.7.0 Wirkleistung (Momentanwert in kW) //0x07010010 0700FF01 -> bigend

            if ( *(uint8_t*) (data+i+13) == 0x52) {  // 0x52 -> datatype is int8
                meterdata->power =  *(int8_t*) (data+i+14);
                sml_stop_idx = i+18;
                break;
            } else {  // 0x53 -> datatype is int16
                meterdata->power =  (int16_t) __bswap_constant_16( *(uint16_t*) (data+i+14) );
                sml_stop_idx = i+19;
                break;
            }
        }
    }

    uint16_t crc16_rx;

    if ( *(uint8_t*)&data[sml_stop_idx] == 0x62) {  // 0x62 -> datatype is uint8
        crc16_rx = *(uint8_t*)&data[sml_stop_idx+1];
    } else {  // 0x63 -> datatype is uint16
        crc16_rx = __bswap_constant_16( *(uint16_t*)&data[sml_stop_idx+1] );
    }

    if (sml_stop_idx < sml_start_idx) {  // prevents hard fault in case of comm errors
        return 0;
    }

    uint16_t crc16_calc = sml_crc16_calculate((unsigned char *) &data[sml_start_idx], sml_stop_idx-sml_start_idx);

//    printf("\nCRC16_rx: %.2X\n", crc16_rx);
//    printf("CRC16_sml: %.2X\n", crc16_calc);

//    int k = 0;
//    for (int j = sml_start_idx; j<sml_stop_idx+3; j++) {
//        printf("0x%.2X, ", (uint8_t)data[j]);
//    }

    if (crc16_rx == crc16_calc)
        success=1;

    return success;
}


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
// stty raw -F /dev/ttyUSB1  # without this command, linux adds \r to \n and corrupts data!
// cat testmsg.bin > /dev/ttyUSB1
el_meter_status_t electricity_meter_read(UART_HandleTypeDef* huart)
{
	enum el_meter_status_t el_meter_status = EL_METER_CONN_WARN;
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


int electricity_meter_get_power()
{
	return pcc_power;
}

