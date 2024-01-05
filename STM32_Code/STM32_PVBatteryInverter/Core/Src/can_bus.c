#include "stm32g4xx_hal.h"

#include <string.h>
#include "can_bus.h"
#include "battery/bms_types.h"

uint16_t vCell_mV[96];
struct CellStack cellStack;

extern FDCAN_HandleTypeDef hfdcan2;

void can_bus_read()
{

	static FDCAN_RxHeaderTypeDef RxHeader;
	static uint8_t RxData[8];

	FDCAN_FilterTypeDef sFilterConfig;

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	//sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE_NO_EIDM;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;

	/*********************************************/
	/* safety relevant data -> high update rate  */
	/* 8 CAN IDs                                 */
	/*********************************************/
	// 1031 CSC01_TEMPERATURES: 8 CSC01
	// 1063 CSC02_TEMPERATURES
	// 1095 CSC03_TEMPERATURES
	// 1127 CSC04_TEMPERATURES
	// 1029 CSC01_CELL_VOLT_MIN_MAX
	// 1061 CSC02_CELL_VOLT_MIN_MAX
	// 1093 CSC03_CELL_VOLT_MIN_MAX
	// 1125 CSC04_CELL_VOLT_MIN_MAX


	/*********************************************/
	/* less relevant data -> low update rate     */
	/* 4*7=28 CAN IDs                            */
	/*********************************************/
	// 1037 CSC01_CELL_VOLT_21_24: 8 CSC01         0b10000001101
	// ...
	// 1032 CSC01_CELL_VOLT_01_04: 8 CSC01         0b10000001000
	// 1027 CSC01_BALANCING_STATE_01_32: 8 CSC01

	// 1069 CSC02_CELL_VOLT_21_24: 8 CSC02         0b10000101101
	// ...
	// 1064 CSC02_CELL_VOLT_01_04: 8 CSC02         0b10000101000
	// 1059 CSC02_BALANCING_STATE_01_32: 8 CSC02

	// 1101 CSC03_CELL_VOLT_21_24: 8 CSC03         0b10001001101
	// ...
	// 1096 CSC03_CELL_VOLT_01_04: 8 CSC03         0b10001001000
	// 1091 CSC03_BALANCING_STATE_01_32: 8 CSC03

	// 1133 CSC04_CELL_VOLT_21_24: 8 CSC04         0b10001101101
	// ...
	// 1128 CSC04_CELL_VOLT_01_04: 8 CSC04         0b10001101000
	// 1123 CSC04_BALANCING_STATE_01_32: 8 CSC04

	// for now, cycle through all IDs

	#define NR_CSCS 4
	#define CELLS_PER_CANID 4
	#define CELLS_PER_CSC 24

#if (NR_CSCS*CELLS_PER_CSC  != 96)
    #warning Software is tested with 96 cells only!
#endif

	memset(vCell_mV, 0, sizeof(vCell_mV));
	uint16_t IDs_csc_vCell_begin[NR_CSCS] = { 1032, 1064, 1096, 1128};

	#define NR_SLOTS (CELLS_PER_CSC/CELLS_PER_CANID)
	for (uint8_t csc=0; csc<NR_CSCS; csc++) {

		uint16_t ID_begin = IDs_csc_vCell_begin[csc];
		uint16_t ID_end = IDs_csc_vCell_begin[csc] + NR_SLOTS-1;

		/* Configure Rx filter */
//		sFilterConfig.FilterID1 = ID_begin;
//		sFilterConfig.FilterID2 = ID_end;
//		if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
//		{
//			Error_Handler();
//		}

		for (uint8_t cnt=0; cnt<CELLS_PER_CANID; cnt++) {

			/* Wait for one message received in one FDCAN instance */
			while ( (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) < 1)) {}

			/* Retrieve Rx message from RX FIFO0 */
			if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
			{
				Error_Handler();
			}

			uint32_t id = RxHeader.Identifier;

			if (   (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8)
				&& (id >= ID_begin) && (id <= ID_end)
			){
				memcpy(&vCell_mV[CELLS_PER_CSC*csc + CELLS_PER_CANID*(id-ID_begin)], RxData, RxHeader.DataLength);
			}

		}
	}


}


