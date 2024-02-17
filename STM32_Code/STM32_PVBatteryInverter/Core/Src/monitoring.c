
#include "stm32g4xx_hal.h"

#include "monitoring.h"


bool monitoring_binary_en;
volatile bool monitoring_request;

volatile fast_monitor_vars_t fast_monitor_vars[FAST_MON_FRAMES];
volatile bool fast_mon_vars_trig;
volatile uint16_t frame_nr;


extern void fill_monitor_vars_sys(monitor_vars_t* mon_vars);

// sends monitor_vars via UART
static void send_slow_monitor_vars(UART_HandleTypeDef *huart)
{
	//GPIOC->BSRR = (1<<4);  // set Testpin TP201 PC4

	static monitor_packet_t mon_packet = {.header=0xDEADBEEF, 0};  // static for DMA
	fill_monitor_vars_sys(&mon_packet.monitor_vars);

	// V1 : FIFO 30bytes transmit best case: 2607us
//	HAL_UART_Transmit(&huart3, (uint8_t *)&mon_vars_snapshot, sizeof(mon_vars_snapshot), 10);

	// V2 : FIFO+DMA 30bytes transmit best case: 1.65us  worst case : 17.9us
	if (HAL_UART_Transmit_DMA(huart, (uint8_t *)&mon_packet, sizeof(mon_packet)) != HAL_OK)
	{
		Error_Handler();
	}
	//GPIOC->BRR = (1<<4);  // reset Testpin TP201 PC4
}


static void send_fast_monitor_vars(UART_HandleTypeDef *huart)
{
	if (HAL_UART_Transmit_DMA(huart, (uint8_t *)fast_monitor_vars, FAST_MON_BYTES) != HAL_OK) {
		Error_Handler();
	}
}


void async_monitor_check(UART_HandleTypeDef *huart)
{
	if (monitoring_binary_en && monitoring_request) {
		monitoring_request = false;
		send_slow_monitor_vars(huart);
	}

	if (frame_nr == FAST_MON_FRAMES) {
		send_fast_monitor_vars(huart);
		fast_mon_vars_trig = false;
		frame_nr = 0;
	}
}
