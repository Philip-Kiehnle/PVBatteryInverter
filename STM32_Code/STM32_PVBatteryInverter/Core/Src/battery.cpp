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
		bms.get_summary();
	}
#endif
}
