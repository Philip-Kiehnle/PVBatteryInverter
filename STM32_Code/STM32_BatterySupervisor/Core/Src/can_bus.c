#include "stm32g4xx_hal.h"

#include <string.h>
#include "can_bus.h"
#include "BatteryManagement/bms_types.h"

struct CellStack cellStack;

extern FDCAN_HandleTypeDef hfdcan2;

extern void Error_Handler(void);


static FDCAN_RxHeaderTypeDef RxHeader;
static FDCAN_FilterTypeDef sFilterConfig;
static uint8_t RxData[8];

static void getRxMsg_blocking()
{
	/* Wait for one message received in one FDCAN instance */
	while ( (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) < 1)) {}

	/* Retrieve Rx message from RX FIFO0 */
	if (HAL_FDCAN_GetRxMessage(&hfdcan2, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
	{
		Error_Handler();
	}

}

static bool getRxMsg_8byte_blocking(uint16_t id_filt)
{

	/* Configure Rx filter */
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterID1 = id_filt;
	sFilterConfig.FilterID2 = id_filt;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	getRxMsg_blocking();
	uint32_t id = RxHeader.Identifier;

	if (   (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8)
		&& (id == id_filt)
	){
		return true;
	}

	return false;
}


void can_bus_read()
{

	/* Configure Rx filter */
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;


	// todo implement higher update rate
	/*********************************************/
	/* safety relevant data -> high update rate  */
	/*********************************************/
	// 1029 CSC01_CELL_VOLT_MIN_MAX
	// 1061 CSC02_CELL_VOLT_MIN_MAX
	// 1093 CSC03_CELL_VOLT_MIN_MAX
	// 1125 CSC04_CELL_VOLT_MIN_MAX


	/*********************************************/
	/* less relevant data -> low update rate     */
	/*********************************************/
	// 1025 CSC01_ERRORS: 8 CSC01
	// 1027 CSC01_BALANCING_STATE_01_32: 8 CSC01
	// 1031 CSC01_TEMPERATURES: 8 CSC01
	// 1032 CSC01_CELL_VOLT_01_04: 8 CSC01         0b10000001000
	// ...
	// 1037 CSC01_CELL_VOLT_21_24: 8 CSC01         0b10000001101

	// 1057 CSC02_ERRORS: 8 CSC02
	// 1059 CSC02_BALANCING_STATE_01_32: 8 CSC02
	// 1063 CSC02_TEMPERATURES
	// 1064 CSC02_CELL_VOLT_01_04: 8 CSC02         0b10000101000
	// ...
	// 1069 CSC02_CELL_VOLT_21_24: 8 CSC02         0b10000101101

	// 1089 CSC03_ERRORS: 8 CSC03
	// 1091 CSC03_BALANCING_STATE_01_32: 8 CSC03
	// 1095 CSC03_TEMPERATURES
	// 1096 CSC03_CELL_VOLT_01_04: 8 CSC03         0b10001001000
	// ...
	// 1101 CSC03_CELL_VOLT_21_24: 8 CSC03         0b10001001101

	// 1121 CSC04_ERRORS: 8 Vector__XXX
	// 1123 CSC04_BALANCING_STATE_01_32: 8 CSC04
	// 1127 CSC04_TEMPERATURES
	// 1128 CSC04_CELL_VOLT_01_04: 8 CSC04         0b10001101000
	// ...
	// 1133 CSC04_CELL_VOLT_21_24: 8 CSC04         0b10001101101

	#define NR_CSCS 4
	#define CELLS_PER_CANID 4
	#define CELLS_PER_CSC 24
	#define TEMPSENSORS_PER_CSC 14

#if (NR_CSCS*CELLS_PER_CSC  != 96)
    #warning Software is tested with 96 cells only!
#endif

	/*******************/
	/* read CSC errors */
	/*******************/
	uint64_t csc_err[NR_CSCS] = {0};
	uint16_t IDs_csc_err[NR_CSCS] = { 1025, 1057, 1089, 1121};

	for (uint8_t csc=0; csc<NR_CSCS; csc++) {
		if ( getRxMsg_8byte_blocking(IDs_csc_err[csc]) ) {
			memcpy(&csc_err[csc], RxData, 8);
		}
		// else ... todo
	}
	//memcpy(&cellStack.csc_err, csc_err, sizeof(cellStack.csc_err));  // todo

	/*********************************/
	/* read CSC cell balancing state */
	/*********************************/
#if 0
	uint16_t IDs_csc_cellBal[NR_CSCS] = { 1027, 1059, 1091, 1123};

	for (uint8_t csc=0; csc<NR_CSCS; csc++) {
		if ( getRxMsg_8byte_blocking(IDs_csc_cellBal[csc]) ) {
			uint64_t cellbal = 0;
			memcpy(&cellbal, RxData, 8);
			for (uint8_t c=0; c<CELLS_PER_CSC; c++) {
				cellStack.balancingState[csc*CELLS_PER_CSC + c] = ((uint64_t)1<<(2*c)) & cellbal;
//				if (c==3) {
//					cellStack.balancingState[csc*CELLS_PER_CSC + c] = 1;
//				}
			}
		}
	}
#endif

	/**************************/
	/* read cell temperatures */
	/**************************/
	int8_t temperature[NR_CSCS*TEMPSENSORS_PER_CSC] = {-128};
	uint16_t IDs_csc_temperature[NR_CSCS] = { 1031, 1063, 1095, 1127};

	/* Configure Rx filter */
	sFilterConfig.FilterType = FDCAN_FILTER_DUAL;

	for (uint8_t cscgroup=0; cscgroup<2; cscgroup++) {
		uint8_t csc = 2*cscgroup;
		uint8_t csc_next = csc+1;
		sFilterConfig.FilterID1 = IDs_csc_temperature[csc];
		sFilterConfig.FilterID2 = IDs_csc_temperature[csc_next];
		if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
		{
			Error_Handler();
		}

		for (uint8_t cnt=0; cnt<4; cnt++) {

			getRxMsg_blocking();
			uint32_t id = RxHeader.Identifier;

			if ( (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8) )
			{
				uint8_t multiplex = RxData[0];
				if (multiplex == 0 || multiplex == 1) {
					if (id == sFilterConfig.FilterID1) {
						uint8_t idx_cell_begin = csc*TEMPSENSORS_PER_CSC + multiplex*TEMPSENSORS_PER_CSC/2;
						memcpy(&temperature[idx_cell_begin], &RxData[1], 7);
					} else if (id == sFilterConfig.FilterID2) {
						uint8_t idx_cell_begin = csc_next*TEMPSENSORS_PER_CSC + multiplex*TEMPSENSORS_PER_CSC/2;
						memcpy(&temperature[idx_cell_begin], &RxData[1], 7);
					}
				}
			}
		}
	}
	memcpy(&cellStack.temperature, temperature, sizeof(cellStack.temperature));


	/***********************/
	/* read cell voltages  */
	/***********************/
	uint16_t vCell_mV[96] = {0};
	//memset(vCell_mV, 0, sizeof(vCell_mV));  // not necessary

	uint16_t IDs_csc_vCell_begin[NR_CSCS] = { 1032, 1064, 1096, 1128};

	#define NR_SLOTS (CELLS_PER_CSC/CELLS_PER_CANID)

	for (uint8_t csc=0; csc<NR_CSCS; csc++) {

		uint16_t ID_begin = IDs_csc_vCell_begin[csc];
		uint16_t ID_end = IDs_csc_vCell_begin[csc] + NR_SLOTS-1;

		/* Configure Rx filter */
		sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
		sFilterConfig.FilterID1 = ID_begin;
		sFilterConfig.FilterID2 = ID_end;
		if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
		{
			Error_Handler();
		}

		for (uint8_t cnt=0; cnt<NR_SLOTS; cnt++) {

			getRxMsg_blocking();
			uint32_t id = RxHeader.Identifier;

			if (   (RxHeader.IdType == FDCAN_STANDARD_ID) && (RxHeader.DataLength == FDCAN_DLC_BYTES_8)
				&& (id >= ID_begin) && (id <= ID_end)
			){
				uint8_t idx_cell_begin = csc*CELLS_PER_CSC + (id-ID_begin)*CELLS_PER_CANID;
				memcpy(&vCell_mV[idx_cell_begin], RxData, 8);
			}

		}
	}
	memcpy(&cellStack.vCell_mV, vCell_mV, sizeof(cellStack.vCell_mV));

	sFilterConfig.FilterConfig = FDCAN_FILTER_DISABLE;
	if (HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

}


