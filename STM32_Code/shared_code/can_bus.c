#include "stm32g4xx_hal.h"

#include <string.h>
#include "can_bus.h"
#include "BatteryManagement/bms_types.h"


extern FDCAN_HandleTypeDef hfdcan2;

extern void Error_Handler(void);


static FDCAN_RxHeaderTypeDef RxHeader;
static FDCAN_FilterTypeDef sFilterConfig;
static uint8_t rx_data[8];
FDCAN_TxHeaderTypeDef TxHeader;

// todo: clear FIFO before next message is received or rewrite and use multiple filters

void can_bus_stop(uint8_t fifo_nr)
{
	// filter 0-7 : used for FIFO0
	if (fifo_nr == 0) {
		for (uint8_t i=0; i<8; i++) {
			sFilterConfig.FilterIndex = i;
			sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
			if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
				Error_Handler();
			}
		}

	// filter 8-15 : used for FIFO1
	} else {
		for (uint8_t i=8; i<16; i++) {
			sFilterConfig.FilterIndex = i;
			sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
			if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
				Error_Handler();
			}
		}
	}
}


// message ID is written to id
// return pointer to data or null in case of error
uint8_t* can_bus_getRxMsg_8byte(uint8_t fifo_nr, uint32_t* id, bool blocking)
{
	const uint32_t fdcan_rx_fifo = (fifo_nr == 0) ? FDCAN_RX_FIFO0 : FDCAN_RX_FIFO1;

	if (blocking) {
		/* Wait for one message received in one FDCAN instance */
		while ( (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, fdcan_rx_fifo) < 1)) {}
	} else {
		if ( HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, fdcan_rx_fifo) <= 0) return NULL;
	}

	/* Retrieve Rx message from RX FIFO */
	if (HAL_FDCAN_GetRxMessage(&hfdcan2, fdcan_rx_fifo, &RxHeader, rx_data) != HAL_OK) {
		Error_Handler();
	}

	if (   (RxHeader.IdType == FDCAN_STANDARD_ID)
		&& (RxHeader.DataLength == FDCAN_DLC_BYTES_8)
	){
		*id = RxHeader.Identifier;
		return rx_data;
	}

	return NULL;
}


bool addTxMsg_8byte(uint32_t id, uint8_t* tx_data)
{
	/* Prepare Tx Header */
	TxHeader.Identifier = 0x40D;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;

	TxHeader.Identifier = id;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &TxHeader, tx_data) != HAL_OK) {
		  /* Transmission request Error */
		  Error_Handler();
		  return false;
	}
	return true;
}


void can_bus_set_filter(uint8_t fifo_nr, uint8_t filter_index, uint32_t filter_type, uint32_t filter_id1, uint32_t filter_id2)
{
	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = filter_index;
	sFilterConfig.FilterConfig = (fifo_nr == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterType = filter_type;
	sFilterConfig.FilterID1 = filter_id1;
	sFilterConfig.FilterID2 = filter_id2;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}


void can_bus_disable_filter(uint8_t filter_index)
{
	sFilterConfig.FilterIndex = filter_index;
	sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
}


// avoid old values, e.g. battery current and keep only x messages
void can_bus_FIFO_drop_msgs(uint8_t fifo_nr, uint8_t nr_msgs_left)
{
	const uint32_t fdcan_rx_fifo = (fifo_nr == 0) ? FDCAN_RX_FIFO0 : FDCAN_RX_FIFO1;

	while ( HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, fdcan_rx_fifo) > nr_msgs_left) {
		/* Retrieve Rx message from RX FIFO */
		if (HAL_FDCAN_GetRxMessage(&hfdcan2, fdcan_rx_fifo, &RxHeader, rx_data) != HAL_OK) {
			Error_Handler();
		}
	}
}
