
#include <algorithm>

#include "dc_control.h"
#include "battery.h"

#include "BatteryManagement/ETI_DualBMS.hpp"
extern ETI_DualBMS bms;

// does not compile with BaseBMS
//#include "battery/BaseBMS.hpp"
//#include "battery/ETI_DualBMS.hpp"
//extern BaseBMS bms;


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
	if (bms.batteryStatus.maxVcell_mV < bms.V_CELL_MAX_PROTECT_mV() ) {
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
	return (bms.batteryStatus.soc <= 1 || bms.batteryStatus.minVcell_mV <= bms.V_CELL_MIN_PROTECT_mV());
}

const bool battery_almost_empty()
{
	return (bms.batteryStatus.soc <= 8 || bms.batteryStatus.minVcell_mV <= bms.V_CELL_MIN_PROTECT_mV());
}

const bool battery_almost_full()
{
	return (bms.batteryStatus.soc == 100 || bms.batteryStatus.maxVcell_mV >= bms.V_CELL_MAX_POWER_REDUCE_mV());
}

const bool battery_full()
{
	return (bms.batteryStatus.soc == 100 || bms.batteryStatus.maxVcell_mV >= bms.V_CELL_MAX_PROTECT_mV());
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
			  if (bms.batteryOn() == BMS_OK) {
				  stateBattery = BMS_ON__BAT_ON;
			  }
			  break;

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
