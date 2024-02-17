
#include <algorithm>

#include "battery.h"
//#include "battery/STW_mBMS.hpp"
#include "battery/ETI_DualBMS.hpp"


#if SYSTEM_HAS_BATTERY == 1
//static STW_mBMS bms(2);  // BMS address
ETI_DualBMS bms(15);  // BMS address
#endif

static volatile bool update_request;
static volatile bool state_change_request;
static volatile bool batteryOnOff;


// LiitoKala 32700 7000mAh LiFePO4 battery: 35A cont. discharge, 55A max
// curve was not available -> using Liitokala 32700 5.5Ah data from
// https://www.kirich.blog/obzory/akkumulyatory/862-liitokala-32700-lifepo4-i-sravnenie-ih-s-analogichnymi-akkumulyatorami-varicore.html
// the curve with 0.5C/3.25A was fitted with Engauge Digitizer
// postprocessing with python included compensation with 3.5A*8mOhm, because our cell has 7Ah
// this is used as the OCV curve for now:
uint16_t SoC_lookup_table[100] = {
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

static void estimateSoC()
{
	int16_t current_mA = bms.dual_bms.battery[0].current_mA;
	// simple 5 second relaxation model
#define I_CHRG_AVG_SEC 5
	static int i_chrg_sum_mA[I_CHRG_AVG_SEC] = {0};
	i_chrg_sum_mA[0] += current_mA - i_chrg_sum_mA[I_CHRG_AVG_SEC-1];
	for (int i=0; i<I_CHRG_AVG_SEC-1; i++){
		i_chrg_sum_mA[I_CHRG_AVG_SEC-1-i] = i_chrg_sum_mA[I_CHRG_AVG_SEC-2-i];
	}
	// voltage increased when charging.
	int i_chrg_avg_mA = i_chrg_sum_mA[0]/I_CHRG_AVG_SEC;
	// 5A for last 5sec, max. 100mV increase
	uint16_t charge_comp_mV = 0;//(100 * std::clamp(i_chrg_avg_mA, 0, 5000)) / 5000;

#define NUM_CELLS 15
	uint16_t avgVcell_mV = bms.batteryStatus.voltage_100mV*100/NUM_CELLS;

#define R_CELL_MILLIOHM (8+1) // internal resistance of cell and cell connector in mOhm
#define PARALLEL_CELLS 1
        avgVcell_mV -= (R_CELL_MILLIOHM * (int32_t)current_mA)/PARALLEL_CELLS/1000;  // current compensation: I>0 means charging
        for (int soc=1; soc<=100; soc++) {
            if (avgVcell_mV >= (SoC_lookup_table[soc-1]+charge_comp_mV)) {
                bms.batteryStatus.soc = soc;  // todo max soc change 1
            }
        }
}


const batteryStatus_t* get_batteryStatus()
{
	 return &bms.batteryStatus;
}


void shutdownBattery()
{
	batteryOnOff = BATTERY_OFF;
	state_change_request = true;
}


void battery_update_request()
{
	update_request = true;
}


void battery_state_request(bool state)
{
	if (state == BATTERY_ON) {
		batteryOnOff = BATTERY_ON;
	} else {
		batteryOnOff = BATTERY_OFF;
	}
	state_change_request = true;
}


void async_battery_communication()
{

#if SYSTEM_HAS_BATTERY == 1
	if (state_change_request) {
		if (batteryOnOff == BATTERY_ON) {
			bms.batteryOn();
		} else {
			bms.batteryOff();
		}
		state_change_request = false;
	}

	if (update_request) {
		update_request = false;
		if (bms.get_summary() == 0) {
			estimateSoC();
		}
	}
#endif
}
