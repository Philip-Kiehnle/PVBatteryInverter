
#include <algorithm>

#include "dc_control.h"
#include "battery.h"

#include "battery/ETI_DualBMS.hpp"
extern ETI_DualBMS bms;

// does not compile with BaseBMS
//#include "battery/BaseBMS.hpp"
//#include "battery/ETI_DualBMS.hpp"
//extern BaseBMS bms;


static volatile bool update_request;
static volatile bool state_change_request;
static volatile stateBattery_t stateBattery_next;
static stateBattery_t stateBattery;
static uint16_t bat_comm_fail_cnt;
static bool bat_connected;


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


void battery_update_request()
{
	update_request = true;
}


void battery_state_request(stateBattery_t state)
{
	stateBattery_next = state;
	state_change_request = true;
}


// returns true if there was an update request
bool async_battery_communication()
{

#if SYSTEM_HAS_BATTERY == 1
	if (state_change_request || (stateBattery_next != stateBattery)) {
		switch (stateBattery_next) {
		  case BMS_OFF__BAT_OFF:
			  if (bms.batteryOff() == 0) {
				  stateBattery = BMS_ON__BAT_OFF;
				  bat_connected = false;
			  }
			  if (bms.shipmode(0b00111111) == 0) {
				  stateBattery = BMS_OFF__BAT_OFF;
				  bat_connected = false;
			  }
			  break;

		  case BMS_ON__BAT_ON:
			  if (bms.batteryOn() == 0) {
				  stateBattery = BMS_ON__BAT_ON;
			  }
			  break;

		  case BMS_ON__BAT_OFF:
		  default:
			  if (bms.batteryOff() == 0) {
				  stateBattery = BMS_ON__BAT_OFF;
				  bat_connected = false;
			  }
			  break;
		}

		state_change_request = false;
	}

	if ( stateBattery == BMS_ON__BAT_ON
	     && (abs(bms.batteryStatus.power_W) > 2
		 || (   bms.batteryStatus.voltage_100mV > get_v_dc_FBboost_filt50Hz_100mV()-5
		     && bms.batteryStatus.voltage_100mV < get_v_dc_FBboost_filt50Hz_100mV()+5)
		)
	) {
		bat_connected = true;
	}

	bool return_code = false;

	if (update_request) {
		update_request = false;
		if (stateBattery != BMS_OFF__BAT_OFF) {
			if (bms.get_summary() == 0) {
				bms.estimateSoC();
				bat_comm_fail_cnt = 0;
			} else {
				if (bat_comm_fail_cnt == 60) {
					set_sys_errorcode(EC_BATTERY_COMM_FAIL);
				} else {
					bat_comm_fail_cnt++;
				}
			}
			return_code = true;
		}
	}


#endif
	return return_code;
}
