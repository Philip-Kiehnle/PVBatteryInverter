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

static void getRxMsg_blocking()
{
	/* Wait for one message received in one FDCAN instance */
	while ( (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) < 1)) {}

	/* Retrieve Rx message from RX FIFO0 */
	if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, rx_data) != HAL_OK) {
		Error_Handler();
	}
}


void can_bus_stop_rx()
{
	sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
}

// message ID is written to id
// return pointer to data or null in case of error
uint8_t* getRxMsg_8byte_blocking(uint32_t* id)
{
	getRxMsg_blocking();
	if (   (RxHeader.IdType == FDCAN_STANDARD_ID)
		&& (RxHeader.DataLength == FDCAN_DLC_BYTES_8)
	){
		*id = RxHeader.Identifier;
		return rx_data;
	}

	return NULL;
}


uint8_t* getRxMsg_8byte_singleID_blocking(uint32_t filter_id)
{
	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterID1 = filter_id;
	sFilterConfig.FilterID2 = filter_id;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	uint32_t id;
	getRxMsg_8byte_blocking(&id);

	if (id == filter_id) return rx_data;

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


void can_bus_set_filter(uint32_t filter_type, uint32_t filter_id1, uint32_t filter_id2)
{
	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	sFilterConfig.FilterType = filter_type;
	sFilterConfig.FilterID1 = filter_id1;
	sFilterConfig.FilterID2 = filter_id2;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

