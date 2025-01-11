
#include <algorithm>

#include "gpio.h"
#include "sys_mode_controller.h"
#include "dc_control.h"

#include "battery.h"
#include "battery_config.h"
#include "BatteryManagement/bms_types.h"
#include "BatteryManagement/STW_mBMS.hpp"


enum mode_t sys_mode = SYS_MODE;
bool locked = false;
static stateHYBRID_AC_t stateHYBRID_AC = HYB_AC_OFF;
static uint16_t p_bat_chg_max;
static uint16_t p_bat_chg_externalmax = UINT16_MAX;
static uint32_t cnt_rel_1Hz;
extern volatile uint32_t cnt_1Hz;  // overflow in 136 years
extern volatile uint32_t cntErr_1Hz;


#if SYSTEM_HAS_BATTERY == 1
STW_mBMS bms(0, 1.0, BATTERY);
#endif


uint16_t get_p_bat_chg_max()
{
	return p_bat_chg_max;
}


enum mode_t get_sys_mode()
{
	return sys_mode;
}


static inline void nextMode(enum mode_t next_mode)
{
	if (next_mode == PV2AC || next_mode == OFF) {
		contactorBattery(0);
		bmsPower(0);
		sys_mode_needs_battery = false;
#if SYSTEM_HAS_BATTERY == 1
		battery_state_request(BAT_OFF);
#endif //SYSTEM_HAS_BATTERY
	}

	if (sys_mode == OFF) {
		// stay in OFF mode for 5 minutes to discharge DC bus
		if ( cnt_1Hz > (cnt_rel_1Hz+(5*60)) && !locked) {
			sys_mode = next_mode;
			cnt_rel_1Hz = cnt_1Hz;
		}
	} else {
		sys_mode = next_mode;
		cnt_rel_1Hz = cnt_1Hz;
	}
	stateHYBRID_AC = HYB_AC_OFF;
}


void sys_mode_ctrl_step(control_ref_t* ctrl_ref)
{
#if SYSTEM_HAS_BATTERY == 1
	const batteryStatus_t* battery = get_batteryStatus();
	p_bat_chg_max = std::min(battery->p_charge_max, p_bat_chg_externalmax);
#endif //SYSTEM_HAS_BATTERY

	switch (sys_mode) {
		  case OFF:
			ctrl_ref->mode = AC_OFF;
			ctrl_ref->v_dc_100mV = 0;
			sys_mode_needs_battery = false;
			nextMode(SYS_MODE);
			break;

#if SYSTEM_HAS_BATTERY == 1
		  case BAT_SUPERVISOR_MODE:
			  sys_mode_needs_battery = true;
			  break;
		  case PV2BAT:
			// preload DC bus if battery is in deepsleep to test PV power
			if (battery->voltage_100mV == 0 || get_stateBattery() == BMS_OFF__BAT_OFF) {
				ctrl_ref->v_dc_100mV = (bms.V_MIN_PROTECT*10 + bms.V_MAX_PROTECT*10)/2;
				if (get_v_dc_FBboost_filt50Hz_100mV() >= ctrl_ref->v_dc_100mV ) {
					battery_state_request(BMS_ON__BAT_OFF);  // wakeup battery
				}
			} else {
				ctrl_ref->v_dc_100mV = battery->voltage_100mV;
			}

			ctrl_ref->mode = AC_OFF;
			if (ctrl_ref->v_dc_100mV > bms.V_MIN_PROTECT*10 && ctrl_ref->v_dc_100mV < bms.V_MAX_PROTECT*10) {
				sys_mode_needs_battery = true;
			} else {
				sys_mode_needs_battery = false;
			}
			//sys_mode_needs_battery = true; // todo remove this battery debug line
			break;
#endif //SYSTEM_HAS_BATTERY

		  default:
			break;
		}

}
