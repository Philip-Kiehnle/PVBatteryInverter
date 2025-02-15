#ifndef INC_BATTERY_CONFIG_H
#define INC_BATTERY_CONFIG_H

#include "BatteryManagement/STW_mBMS.hpp"


constexpr BatteryParameter_t BATTERY = {
	.CHEMISTRY = CHEM_NMC,
	.CONFIG = BAT_SINGLE,
	.NR_PACKS = 1,
	.NR_MODULES_PER_PACK = 8,  // 4 CSCs x 2 Containers per CSC
	.NR_CELLS_PER_MODULE = 12,
	.PARALLEL_CELLS = 3,
	.R_SINGLE_CELL_mOHM = (29+1),  // internal resistance of single cell and the corresponding part of cell connector
	.V_CELL_NOM_mV              = 3700,
	.V_CELL_MIN_PROTECT_mV      = 3050,  // BMS CSC PCBS UV_THRESHOLD_mV 3000  ~7% SoC
	.V_CELL_MIN_POWER_REDUCE_mV = 3300,
	.V_CELL_MAX_POWER_REDUCE_mV = 3900,
	.V_CELL_MAX_PROTECT_mV      = 4200,
	.I_CHARGE_MAX    = 8.0,  // ADC watchdog 10A
	.I_DISCHARGE_MAX = 8.0,  // ADC watchdog 10A
	.T_CELL_MIN_ERR  = -5,
	.T_CELL_MIN_WARN = -3,
	.T_CELL_MAX_WARN = 48,
	.T_CELL_MAX_ERR  = 50,
	.T_PCB_MAX_WARN = 58,
	.T_PCB_MAX_ERR  = 60,
};

constexpr uint16_t V_CELL_IMBALANCE_INFO_mV = 250;
constexpr uint16_t V_CELL_IMBALANCE_WARN_mV = 270;
//constexpr uint16_t V_CELL_IMBALANCE_ERR_mV = 300;  // BMS value ?

constexpr stateBattery_t DEFAULT_BATTERY_STATE = BMS_OFF__BAT_OFF;
//constexpr stateBattery_t DEFAULT_BATTERY_STATE = BMS_ON__BAT_OFF;  // CAN communication without need for high DC voltage

// Battery voltage tolerance while connected:
// coarse in case where current flow
// fine in case no current flows

//constexpr uint16_t BAT_BUS_VOLTAGE_TOLERANCE_COARSE_100mV = 10*10;  // 10V
//constexpr uint16_t BAT_BUS_VOLTAGE_TOLERANCE_FINE_100mV = 5;  // 0.5V

constexpr uint16_t BAT_BUS_VOLTAGE_TOLERANCE_COARSE_100mV = 15*10;  // works with 35W light bulb as series resistor
constexpr uint16_t BAT_BUS_VOLTAGE_TOLERANCE_FINE_100mV = 6*10;

// NMC chemistry
constexpr uint16_t SoC_mV_lookup_table_100[100] = {
     2605,2720,2799,2867,2930,2977,3015,3049,3081,3110,  // 1%, 2%, ...
     3137,3162,3188,3215,3241,3264,3287,3310,3332,3354,
     3374,3393,3412,3431,3451,3468,3480,3489,3495,3502,
     3512,3523,3535,3547,3560,3573,3586,3599,3611,3622,
     3631,3638,3645,3651,3657,3664,3674,3684,3696,3707,
     3717,3727,3735,3743,3751,3758,3767,3777,3787,3796,
     3804,3812,3819,3827,3835,3843,3852,3861,3871,3880,
     3889,3898,3908,3920,3932,3945,3957,3969,3981,3994,
     4007,4020,4031,4040,4047,4052,4057,4061,4065,4068,
     4071,4073,4075,4078,4081,4086,4097,4113,4134,4180  // ... 99%, 100%
 };


#endif /* INC_BATTERY_CONFIG_H */
