
#include <algorithm>

#include "gpio.h"
#include "sys_mode_controller.h"
#include "ac_control.h"
#include "dc_control.h"
#include "electricity_meter.h"

#include "battery.h"
#include "battery/bms_types.h"
//#include "battery/STW_mBMS.hpp"
#include "battery/ETI_DualBMS.hpp"


enum mode_t sys_mode = SYS_MODE;
static uint32_t cnt_rel_1Hz;
extern volatile uint32_t cnt_1Hz;


#if SYSTEM_HAS_BATTERY == 1
//static STW_mBMS bms(2);  // BMS address
ETI_DualBMS bms(15);  // BMS address
#endif

enum mode_t get_sys_mode()
{
	return sys_mode;
}

static inline void nextMode(enum mode_t mode)
{
	sys_mode = mode;
	cnt_rel_1Hz = cnt_1Hz;
}

void sys_mode_ctrl_step(control_ref_t* ctrl_ref)
{
#if SYSTEM_HAS_BATTERY == 1
	const batteryStatus_t* battery = get_batteryStatus();
#endif //SYSTEM_HAS_BATTERY

	static stateHYBRID_AC_t stateHYBRID_AC = HYB_AC_OFF;

	switch (sys_mode) {
		   case OFF:
			ctrl_ref->mode = AC_OFF;
			ctrl_ref->v_dc_100mV = 0;
			sys_mode_needs_battery = false;

			// stay in OFF mode for 5 minutes to discharge DC bus
			if ( cnt_1Hz > (cnt_rel_1Hz+(5*60)) ) {
				nextMode(SYS_MODE);
			}
			break;

		  case PV2AC:  // EnergyMeter: 29.6W 30.2VA with saturating inductor model
			ctrl_ref->v_dc_100mV = (1.02*VGRID_AMP*10);  // for init of PV DC controller
			if (stateDC > VOLTAGE_CONTROL) {
			    ctrl_ref->mode = VDC_VARIABLE_CONTROL;
			} else {
			    ctrl_ref->mode = VDC_CONTROL;
			}

			sys_mode_needs_battery = false;

			if (    cnt_1Hz > (cnt_rel_1Hz+5)  // stay in PV2AC mode for min 5 seconds
			     && (SYS_MODE != PV2AC)  // without battery, the only other mode is OFF
			     && ctrl_ref->p_pcc > 50 && ctrl_ref->p_pcc_prev > 50  // feedin required
#if SYSTEM_HAS_BATTERY == 1
				 && !bms.tempWarn()
#endif //SYSTEM_HAS_BATTERY
			   ) {
				nextMode(SYS_MODE);
				if (SYS_MODE == HYBRID_PCC_SENSOR) {
					ctrl_ref->mode = AC_PASSIVE;  // todo: check if switch to PAC_CONTROL_PCC is fast enough to prevent overvoltage
				}
			} else if (    cnt_1Hz > (cnt_rel_1Hz+120)  // stay in PV2AC mode for min 120 seconds before possible turnoff AC
					    && get_p_ac_filt1minute() < 5 && get_p_ac_filt50Hz() < 5
			          ) {
				nextMode(OFF);
			}
			break;

#if SYSTEM_HAS_BATTERY == 1
		  case PV2BAT:
			ctrl_ref->v_dc_100mV = battery->voltage_100mV;
			ctrl_ref->mode = AC_OFF;
			if (ctrl_ref->v_dc_100mV > bms.V_MIN_PROTECT()*10 && ctrl_ref->v_dc_100mV < bms.V_MAX_PROTECT()*10) {
				sys_mode_needs_battery = true;
			} else {
				sys_mode_needs_battery = false;
			}
			break;

		  case HYBRID_PCC_SENSOR:
			// preload DC bus if battery is in deepsleep to test PV power
			if (battery->voltage_100mV == 0) {
				ctrl_ref->v_dc_100mV = (bms.V_MIN_PROTECT()*10 + bms.V_MAX_PROTECT()*10)/2;
				if (get_v_dc_FBboost_filt50Hz_100mV() >= ctrl_ref->v_dc_100mV ) {
					battery_state_request(BMS_ON__BAT_OFF);
				}
		    } else {
				ctrl_ref->v_dc_100mV = battery->voltage_100mV;
		    }

			if (ctrl_ref->v_dc_100mV > bms.V_MIN_PROTECT()*10 && ctrl_ref->v_dc_100mV < bms.V_MAX_PROTECT()*10) {
				// battery charging
				sys_mode_needs_battery = true;

				// todo battery power controller for battery temperature

				switch (stateHYBRID_AC) {
					case HYB_AC_OFF:
						if (   battery_connected()
							&& (   battery->soc > 20  // enough energy for feedin
							    || battery_full()     // or battery is full
								|| bms.tempWarn()     // hot or cold battery -> PV2AC
							    || battery->power_W >= P_BAT_MAX)  // or battery charge power is large
							) {
								stateHYBRID_AC = HYB_AC_ALLOWED;
						}
						break;

					case HYB_AC_ALLOWED:
						// AC turnon if
						if (   (ctrl_ref->p_pcc > 50 && ctrl_ref->p_pcc_prev > 50)  // feedin required; filter 1 second spikes from freezer motor start
							|| battery_full()  // or battery is full
							|| bms.tempWarn()  // hot or cold battery -> PV2AC
							|| battery->power_W > P_BAT_MAX  // or battery charge power is large
							) {
							stateHYBRID_AC = HYB_AC_ON;
						}
						break;

					case HYB_AC_ON:
						if (!battery_connected()) {  // AC already on but battery reconnect because previous mode was PV2AC
							// wait until battery is connected
						} else if (   battery->maxVcell_mV >= bms.V_CELL_MAX_POWER_REDUCE_mV()  // Ppcc + extra power for constant battery voltage
								   || battery->power_W >= P_BAT_MAX  // Ppcc + extra power for constant battery power
							) {
							ctrl_ref->mode = PAC_CONTROL_V_P_BAT_CONST;
						} else {
							ctrl_ref->mode = PAC_CONTROL_PCC;
						}

						// check if battery should be disconnected (variable Vdc -> higher efficiency)
						if ( (bms.tempWarn())
							 || ( (ctrl_ref->p_pcc < -50 || electricity_meter_get_status() == EL_METER_CONN_ERR)  // other PV inverters produce enough for household or PCC sensor fail
							      && (battery->power_W > 10)  // minimum charging power to compensate switching loss for AC bridge
							      && battery_full()
							  	  )
							) {
							nextMode(PV2AC);
							contactorBattery(0);
							bmsPower(0);
							sys_mode_needs_battery = false;
							battery_state_request(BMS_ON__BAT_OFF);

						// AC turnoff in case of empty battery.
						// during day, AC stays connected, but AC energy packet control turns off gatepulses if no feedin required
						} else if ( battery_connected()
									&& (  battery->soc < 10
									    || battery->minVcell_mV < bms.V_CELL_MIN_POWER_REDUCE_mV()
									    || bms.tempHighWarn()
									   )
						   ) {
							ctrl_ref->mode = AC_OFF;
							sys_mode_needs_battery = false;
							contactorBattery(0);
							bmsPower(0);
							//battery_state_request(BMS_OFF__BAT_OFF);
							battery_state_request(BMS_ON__BAT_OFF);
							nextMode(OFF);
							stateHYBRID_AC = HYB_AC_OFF;
						}
						break;
					default:
						stateHYBRID_AC = HYB_AC_OFF;
						break;
				}
			} else {
				sys_mode_needs_battery = false;
				ctrl_ref->mode = AC_OFF;
			}
			break;

		  case HYBRID_REMOTE_CONTROLLED:  // todo implement control interface
			ctrl_ref->mode = PAC_CONTROL_PCC;
			ctrl_ref->v_dc_100mV = battery->voltage_100mV;
			sys_mode_needs_battery = true;
			break;
#endif //SYSTEM_HAS_BATTERY

		  default:
			break;
		}

}
