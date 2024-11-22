#ifndef INC_BATTERY_CONFIG_H
#define INC_BATTERY_CONFIG_H

#include "BatteryManagement/ETI_DualBMS.hpp"


constexpr BatteryParameter_t BATTERY = {
	.CHEMISTRY = CHEM_LFP,
	.CONFIG = BAT_SINGLE,
	.NR_PACKS = 1,
	.NR_MODULES_PER_PACK = 1,
	.NR_CELLS_PER_MODULE = 15,
	.PARALLEL_CELLS = 1,
	.R_SINGLE_CELL_mOHM = (8+1), // internal resistance of single cell and the corresponding part of cell connector
	.V_CELL_NOM_mV              = 3200,
	.V_CELL_MIN_PROTECT_mV      = 2820,  // BMS UV_THRESHOLD_mV 2800  // LiitoKala discharge cutoff 2.5V
	.V_CELL_MIN_POWER_REDUCE_mV = 3000,
	.V_CELL_MAX_POWER_REDUCE_mV = 3480,
	.V_CELL_MAX_PROTECT_mV      = 3500,  // BMS OV_THRESHOLD_mV 3600
	.I_CHARGE_MAX    = 3.0,  // LiitoKala allows 3.0A rated charge
	.I_DISCHARGE_MAX = 7.5,  // BMS limit 8A,  LiitoKala allows 30A continuous, 36A max
	.T_CELL_MIN_ERR  = 1,  // BMS limit 0°C
	.T_CELL_MIN_WARN = 2,
	.T_CELL_MAX_WARN = 43,
	.T_CELL_MAX_ERR  = 44,  // BMS limit 45°C
};

constexpr uint16_t V_CELL_IMBALANCE_INFO_mV = 250;
constexpr uint16_t V_CELL_IMBALANCE_WARN_mV = 270;
//constexpr uint16_t V_CELL_IMBALANCE_ERR_mV = 300;  // BMS value

// LiitoKala 32700 7000mAh LiFePO4 battery: 35A cont. discharge, 55A max
// curve was not available -> using Liitokala 32700 5.5Ah data from
// https://www.kirich.blog/obzory/akkumulyatory/862-liitokala-32700-lifepo4-i-sravnenie-ih-s-analogichnymi-akkumulyatorami-varicore.html
// the curve with 0.5C/3.25A was fitted with Engauge Digitizer
// postprocessing with python included compensation with 3.5A*8mOhm, because our cell has 7Ah
// this is used as the OCV curve for now:
constexpr uint16_t SoC_mV_lookup_table_100[100] = {
	2647,2754,2837,2902,2948,2985,3012,3035,3052,3068,  // 1%, 2%, ...
	3081,3092,3100,3108,3115,3123,3130,3136,3142,3148,
	3152,3156,3160,3164,3168,3172,3175,3177,3179,3182,
	3184,3186,3188,3190,3193,3195,3197,3199,3201,3202,
	3204,3205,3207,3208,3210,3211,3212,3214,3215,3216,
	3217,3218,3219,3220,3221,3223,3224,3225,3227,3228,
	3229,3230,3231,3232,3233,3235,3236,3237,3238,3240,
	3241,3243,3244,3246,3247,3248,3249,3250,3251,3252,
	3254,3256,3258,3259,3260,3261,3262,3264,3265,3266,
	3267,3268,3270,3271,3272,3273,3274,3279,3312,3649  // ... 99%, 100%
};


#endif /* INC_BATTERY_CONFIG_H */
