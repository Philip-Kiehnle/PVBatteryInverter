
#include <algorithm>

#include "dc_control.h"
#include "battery.h"

#include "BatteryManagement/ETI_DualBMS.hpp"
extern ETI_DualBMS bms;

// does not compile with BaseBMS
//#include "battery/BaseBMS.hpp"
//#include "battery/ETI_DualBMS.hpp"
//extern BaseBMS bms;

constexpr batteryParameter_t BATTERY = {
	.CHEMISTRY = CHEM_LFP,
	.CONFIG = BAT_SINGLE,
	.NR_STACKS = 1,
	.NR_CELLS = 15,
	.PARALLEL_CELLS = 1,
	.R_PCELL_mOHM = (8+1), // internal resistance of parallel cells and cell connector in mOhm
	.V_CELL_NOM_mV              = 3200,
	.V_CELL_MIN_PROTECT_mV      = 2820,  // BMS UV_THRESHOLD_mV 2800  // LiitoKala discharge cutoff 2.5V
	.V_CELL_MIN_POWER_REDUCE_mV = 2900,
	.V_CELL_MAX_POWER_REDUCE_mV = 3520,
	.V_CELL_MAX_PROTECT_mV      = 3560,  // BMS OV_THRESHOLD_mV 3600
	.I_CHARGE_MAX    = 3.0,  // LiitoKala allows 3.0A rated charge
	.I_DISCHARGE_MAX = 7.5,  // BMS limit 8A,  LiitoKala allows 30A continuous, 36A max
	.T_CELL_MIN_ERR  = 1,  // BMS limit 0°C
	.T_CELL_MIN_WARN = 2,
	.T_CELL_MAX_WARN = 43,
	.T_CELL_MAX_ERR  = 44,  // BMS limit 45°C
};

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


static volatile bool update_request;
static volatile stateBattery_t stateBattery_next = BMS_ON__BAT_OFF;
static stateBattery_t stateBattery = BMS_ON__BAT_OFF;
static uint16_t bat_comm_fail_cnt;
static bool bat_connected;

#if SYSTEM_HAS_BATTERY == 1
const batteryStatus_t* get_batteryStatus()
{
	 return &bms.batteryStatus;
}

const stateBattery_t get_stateBattery()
{
	return stateBattery;
}


const bool battery_maxVcell_OK()
{
	if (bms.batteryStatus.maxVcell_mV < BATTERY.V_CELL_MAX_PROTECT_mV ) {
		return true;
	}
	return false;
}


const bool battery_connected()
{
	return bat_connected;
}


const bool battery_empty()
{
	return (bms.batteryStatus.soc <= 1 || bms.batteryStatus.minVcell_mV <= BATTERY.V_CELL_MIN_PROTECT_mV);
}

const bool battery_almost_empty()
{
	return (bms.batteryStatus.soc <= 8 || bms.batteryStatus.minVcell_mV <= BATTERY.V_CELL_MIN_PROTECT_mV);
}

const bool battery_almost_full()
{
	return (bms.batteryStatus.soc == 100 || bms.batteryStatus.maxVcell_mV >= BATTERY.V_CELL_MAX_POWER_REDUCE_mV);
}

const bool battery_full()
{
	return (bms.batteryStatus.soc == 100 || bms.batteryStatus.maxVcell_mV >= BATTERY.V_CELL_MAX_PROTECT_mV);
}
#endif //SYSTEM_HAS_BATTERY


void battery_update_request()
{
	update_request = true;
}


void battery_state_request(stateBattery_t state)
{
	// in case of empty battery, use ship mode and turnoff BMS
	if (state == BAT_OFF) {
		if (bms.batteryStatus.soc > 30) {
			state = BMS_ON__BAT_OFF;
		} else {
			state = BMS_OFF__BAT_OFF;
		}
	}


	stateBattery_next = state;
}


// returns true if there was an update request
bool async_battery_communication()
{
	bool return_code = false;
	static bool rate_limit = false;

#if SYSTEM_HAS_BATTERY == 1
	if ( (stateBattery_next != stateBattery) && !rate_limit) {
		rate_limit = true;
		switch (stateBattery_next) {
		  case BMS_OFF__BAT_OFF:
			  if (bms.batteryOff() == BMS_OK) {
				  stateBattery = BMS_ON__BAT_OFF;
				  bat_connected = false;
			  }
			  if (bms.shipmode(0b00111111) == BMS_OK) {
				  stateBattery = BMS_OFF__BAT_OFF;
				  bat_connected = false;
			  }
			  break;

		  case BMS_ON__BAT_ON:
		  {
			  BMS_StatusTypeDef status = bms.batteryOn();
			  if (status == BMS_OK) {
				  stateBattery = BMS_ON__BAT_ON;
			  } else if (status == BMS_NCK) {
				  stateBattery = BMS_ON__BAT_OFF;
			  }
			  break;
		  }

		  case BMS_ON__BAT_OFF:
		  default:
			  if (bms.batteryOff() == BMS_OK) {
				  stateBattery = BMS_ON__BAT_OFF;
				  bat_connected = false;
			  }
			  break;
		}
	}

	if ( stateBattery == BMS_ON__BAT_ON
	     && (abs(bms.batteryStatus.power_W) > 5
			 || (   bms.batteryStatus.voltage_100mV > get_v_dc_FBboost_filt50Hz_100mV()-5
				 && bms.batteryStatus.voltage_100mV < get_v_dc_FBboost_filt50Hz_100mV()+5)
		)
	) {
		bat_connected = true;
	}

	if (update_request) {
		rate_limit = false;
		update_request = false;
		if (stateBattery != BMS_OFF__BAT_OFF) {
			if (bms.get_summary() == BMS_OK) {
				bms.estimateSoC();
				bat_comm_fail_cnt = 0;
			} else {
				if (bat_comm_fail_cnt == 60) {
					set_sys_errorcode(EC_BATTERY_COMM_FAIL);
				} else {
					bat_comm_fail_cnt++;
				}
			}
		}
		return_code = true;
	}
#else
	if (update_request) {
		update_request = false;
		return_code = true;
	}
#endif //SYSTEM_HAS_BATTERY
	return return_code;
}
