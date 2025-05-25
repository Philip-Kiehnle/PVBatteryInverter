#ifndef INC_CONFIG_BATTERY_H
#define INC_CONFIG_BATTERY_H

#include "BatteryManagement/STW_mBMS.hpp"

// KaRaceIng13 355V 3x6Ah
constexpr BatteryParameter_t BATTERY = {
	.CHEMISTRY = CHEM_NMC,
	.CONFIG = BAT_SINGLE,
	.NR_PACKS = 1,
	.NR_MODULES_PER_PACK = 8,  // 4 CSCs x 2 Containers per CSC
	.NR_CELLS_PER_MODULE = 12,
	.PARALLEL_CELLS = 3,
	.R_SINGLE_CELL_mOHM = (50+1),  // internal resistance of single cell and the corresponding part of cell connector; worst cell
	.CAPACITY_Ah = 12,  // 12 instead of 18 because of aging
	.I_CHARGE_MAX            = 3.0,
	.I_CHARGE_MAX_PROTECT    = 4.0,  // Battery_Supervisor: softlimit 8A, ADC watchdog 15A
	.I_DISCHARGE_MAX         = 6.0,
	.I_DISCHARGE_MAX_PROTECT = 7.0,  // Battery_Supervisor: softlimit 8A, ADC watchdog 15A
	.V_CELL_NOM_mV              = 3800,
	.V_CELL_MIN_PROTECT_mV      = 3500,  // Battery_Supervisor: 3050mV
	.V_CELL_MIN_POWER_REDUCE_mV = 3700,  // early power reduction to protect old cells
	.V_CELL_MAX_POWER_REDUCE_mV = 4100,
	.V_CELL_MAX_PROTECT_mV      = 4150,  // Battery_Supervisor: 4200mV
	.T_CELL_MIN_ERR  = -2,  // Battery_Supervisor: -5°C
	.T_CELL_MIN_WARN = 0,
	.T_CELL_MAX_WARN = 42,
	.T_CELL_MAX_ERR  = 44,  // Battery_Supervisor: 50°C
	.T_PCB_MAX_WARN = 56,
	.T_PCB_MAX_ERR  = 58,  // Battery_Supervisor: 60°C
};

constexpr KalmanParameter_t KALMAN = {
	.ETA_CHG = 0.99,
	.ETA_DISCHG = 1.0,
	//.R0 = ((float)BATTERY.R_SINGLE_CELL_mOHM) / BATTERY.PARALLEL_CELLS /1000,  // internal resistance R0 of parallel cells for simple model
	.R0 = 0.5*((float)BATTERY.R_SINGLE_CELL_mOHM) / BATTERY.PARALLEL_CELLS /1000,  // internal resistance R0 of parallel cells for advanced model
	//.SIGMA_V_CELL = 0.04,  // 40mV standard deviation because of 100Hz ripple of single phase inverter; assuming 2A at 20mOhm -> has 1.2% SoC jumps
	.SIGMA_V_CELL = 0.1,  // 40mV standard deviation because of 100Hz ripple of single phase inverter; assuming 2A at 20mOhm; 60mV extra because of simple model
	//.SIGMA_I_BAT = 0.02,  // 20mA standard deviation of current sensor
	.SIGMA_I_BAT = 0.001,  // 1mA standard deviation of current sensor for smooth SoC estimation
	.SIGMA_R0 = 0.01,    // 10mOhm
// For advanced model:
	.R1 = 0.5*((float)BATTERY.R_SINGLE_CELL_mOHM) / BATTERY.PARALLEL_CELLS /1000,  // internal resistance R1 of parallel cells; paper: 0.72mOhm with 38Ah cell
	.R2 = 0.0,  // paper: 0.69mOhm with 38Ah cell
	.TAU1 = 420,     // paper: 36s
	.TAU2 = 642,    // paper: 642s
	.GAMMA = 17,    // paper: 17
	.M = 0.002,  // maximum polarization voltage (half the difference of charge and discharge OCV curves); paper: M(q) LUT
	.REST_TIME = 400,  // s
	.SIGMA_R1 = 0.008,   // paper: 0.1mOhm with 38Ah cell
	.SIGMA_R2 = 0.009,  // paper: 0.35mOhm with 38Ah cell
	.SIGMA_TAU1 = 200,  // paper: 8s
	.SIGMA_TAU2 = 200,  // paper: 200s
	.SIGMA_GAMMA = 10,  // paper: 10
	.SIGMA_M = 0.005  // paper: 0.2 * M(q)
};


constexpr uint16_t V_CELL_IMBALANCE_INFO_mV = 180;
constexpr uint16_t V_CELL_IMBALANCE_WARN_mV = 200;
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


#endif /* INC_CONFIG_BATTERY_H */
