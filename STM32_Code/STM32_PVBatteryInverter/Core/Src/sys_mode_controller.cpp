
#include <algorithm>

#include "gpio.h"
#include "sys_mode_controller.h"
#include "ac_control.h"
#include "dc_control.h"
#include "fan_control.h"
#include "electricity_meter.h"

#include "battery.h"
#include "config_battery.h"
#include "BatteryManagement/bms_types.h"
#include "BatteryManagement/STW_mBMS.hpp"
//#include "BatteryManagement/ETI_DualBMS.hpp"
#include <modbus_hybridinverter.h>


enum mode_t sys_mode = SYS_MODE;
bool locked = false;
static stateHYBRID_AC_t stateHYBRID_AC = HYB_AC_OFF;
static uint16_t p_bat_chg_max;
static uint32_t cnt_rel_1Hz;
extern volatile uint32_t cnt_1Hz;  // overflow in 136 years
extern volatile uint32_t cntErr_1Hz;


#if SYSTEM_HAS_BATTERY == 1
STW_mBMS bms(0, 1.0, BATTERY, KALMAN);
//ETI_DualBMS bms(15, 1.0, BATTERY);  // BMS address=15
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


void apply_sys_mode_cmd(control_ref_t* ctrl_ref)
{
	static uint32_t cnt_1Hz_prev_ext_cmd;

	if (modbus_reg_rw.cmd == CMD_INVERTER_RESET) {
		GPIOB->BRR = (1<<0);  // enable red LED
		if (get_sys_errorcode() != EC_NO_ERROR) {
			if (cntErr_1Hz > 120) {  // reset possible after 120 seconds in case of error
				NVIC_SystemReset();
			}
		} else {
			shutdownAll();  // todo: battery off sends to late
			NVIC_SystemReset();
		}
	} else if (modbus_reg_rw.cmd == CMD_INVERTER_OFF) {
		locked = true;
		shutdownAll();
		nextMode(OFF);
	} else if (modbus_reg_rw.cmd == CMD_INVERTER_ON) {
		locked = false;

	} else if (modbus_reg_rw.cmd == CMD_P_AC_EXT_OFF) {
		ctrl_ref->ext_ctrl_mode = EXT_OFF;
	}

	modbus_reg_rw.cmd = CMD_INVALID;  // reset to invalid command

	static int16_t p_ac_unclamped = 0;

	if (modbus_p_ac_soft_update) {
		p_ac_unclamped = modbus_reg_rw.p_ac_soft_W;
		ctrl_ref->ext_ctrl_mode = EXT_AC_SOFT;
		cnt_1Hz_prev_ext_cmd = cnt_1Hz;

	} else if (modbus_p_ac_hard_update) {
		p_ac_unclamped = modbus_reg_rw.p_ac_hard_W;
		ctrl_ref->ext_ctrl_mode = EXT_AC_HARD;
		cnt_1Hz_prev_ext_cmd = cnt_1Hz;
	}

	if ( ctrl_ref->ext_ctrl_mode != EXT_OFF && sys_mode != OFF ) {
		ctrl_ref->p_ac_external = std::clamp(p_ac_unclamped, (int16_t)(-P_AC_MAX), (int16_t)P_AC_MAX);
		nextMode(HYBRID_REMOTE_CONTROLLED);
	}

	if (cnt_1Hz > cnt_1Hz_prev_ext_cmd+60) {  // internal control after 60 sec without command
		ctrl_ref->ext_ctrl_mode = EXT_OFF;
	}

	if (ctrl_ref->ext_ctrl_mode == EXT_OFF && sys_mode == HYBRID_REMOTE_CONTROLLED) {
		nextMode(HYBRID_PCC_SENSOR);
	}

	modbus_p_ac_soft_update = false;
	modbus_p_ac_hard_update = false;
}


void sys_mode_ctrl_step(control_ref_t* ctrl_ref)
{
#if SYSTEM_HAS_BATTERY == 1
	const batteryStatus_t* battery = get_batteryStatus();
	bms.read_current = true;
	p_bat_chg_max = std::min(battery->p_charge_max, modbus_reg_rw.p_bat_chg_max_W);
#endif //SYSTEM_HAS_BATTERY
	static uint32_t cnt_1Hz_chargeDC;

	fan_control_ac(cnt_1Hz, get_p_ac_filt50Hz());

	switch (sys_mode) {
		  case OFF:
			ctrl_ref->mode = AC_OFF;
			ctrl_ref->v_dc_100mV = 0;
			sys_mode_needs_battery = false;
			nextMode(SYS_MODE);
			break;

		  case PV2AC:  // EnergyMeter: 29.6W 30.2VA with saturating inductor model
#ifndef USE_TRAFO_33V
#error "USE_TRAFO_33V has to be defined as 0 or 1"
#endif
#if USE_TRAFO_33V == 1  // high dc voltage with low voltage trafo; sigmadelta software processing shows >100Vdc at 0Vdc
			ctrl_ref->v_dc_100mV = (320*10);  // for init of PV DC controller
			ctrl_ref->mode = VDC_CONTROL;
#else
			ctrl_ref->v_dc_100mV = (1.02*VGRID_AMP*10);  // for init of PV DC controller
			if (stateDC > VOLTAGE_CONTROL) {
			    //ctrl_ref->mode = VDC_VARIABLE_CONTROL;  // increases voltage too much with 2x1kW grid resistance for debugging
			    ctrl_ref->mode = VDC_CONTROL;
			} else {
			    ctrl_ref->mode = VDC_CONTROL;
			}
#endif

			sys_mode_needs_battery = false;

			if (    cnt_1Hz > (cnt_rel_1Hz+5)  // stay in PV2AC mode for min 5 seconds
			     && (SYS_MODE != PV2AC)  // in PV2AC general mode, the only other mode is OFF
#if SYSTEM_HAS_BATTERY == 1
			     && ((ctrl_ref->p_pcc > 50 && ctrl_ref->p_pcc_prev > 50 && battery->minVcell_mV > (BATTERY.V_CELL_MIN_POWER_REDUCE_mV+100))  // more feedin required and battery not empty
					 || (ctrl_ref->p_pcc < -50 && battery->soc_percent < 98))  // battery recharge required; todo battery soc control curve, goal: >90% at sunset
				 && !bms.warn_temperature()
#endif //SYSTEM_HAS_BATTERY
			   ) {
				ctrl_ref->mode = VDC_CONTROL;
#if SYSTEM_HAS_BATTERY == 1
				ctrl_ref->v_dc_100mV = (bms.V_MIN_PROTECT*10 + bms.V_MAX_PROTECT*10)/2;  // todo: check this line
#else
				ctrl_ref->v_dc_100mV = (1.02*VGRID_AMP*10);
#endif //SYSTEM_HAS_BATTERY
				nextMode(SYS_MODE);
			} else if (    cnt_1Hz > (cnt_rel_1Hz+(3*60))  // stay in PV2AC mode for min 3 minutes in case of low PV power
					    && get_p_ac_filt1minute() < 5 && get_p_ac_filt50Hz() < 5  // todo: AC power seems larger than measured
			          ) {
				if (SYS_MODE == PV2AC) {
					nextMode(OFF);
				} else if (cnt_1Hz > (cnt_rel_1Hz+(10*60))) {  // limit toggling rate between battery charging and PV2AC
					// prevent short turnoff in the evening when battery is full
					ctrl_ref->mode = VDC_CONTROL;
#if SYSTEM_HAS_BATTERY == 1
					ctrl_ref->v_dc_100mV = (bms.V_MIN_PROTECT*10 + bms.V_MAX_PROTECT*10)/2;
#else
					ctrl_ref->v_dc_100mV = (1.02*VGRID_AMP*10);
#endif //SYSTEM_HAS_BATTERY
					nextMode(SYS_MODE);
				}
			}
			break;

#if SYSTEM_HAS_BATTERY == 1
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
			if (   get_v_dc_FBboost_filt50Hz_100mV() > bms.V_MIN_PROTECT*10
				&& get_v_dc_FBboost_filt50Hz_100mV() < bms.V_MAX_PROTECT*10
			) {
				sys_mode_needs_battery = true;
			} else {
				sys_mode_needs_battery = false;
			}
			break;

		  case HYBRID_REMOTE_CONTROLLED:
		  case HYBRID_PCC_SENSOR:
			// preload DC bus if battery is in deepsleep to test PV power
			if (battery->voltage_100mV == 0 || get_stateBattery() == BMS_OFF__BAT_OFF) {
				ctrl_ref->v_dc_100mV = (bms.V_MIN_PROTECT*10 + bms.V_MAX_PROTECT*10)/2;
				if (get_v_dc_FBboost_filt50Hz_100mV() >= ctrl_ref->v_dc_100mV ) {
					battery_state_request(BMS_ON__BAT_OFF);  // wakeup battery
				}
		    } else {
				ctrl_ref->v_dc_100mV = battery->voltage_100mV;
		    }

			if (ctrl_ref->v_dc_100mV > bms.V_MIN_PROTECT*10 && ctrl_ref->v_dc_100mV < bms.V_MAX_PROTECT*10) {
				// battery charging
				sys_mode_needs_battery = true;
				cnt_1Hz_chargeDC = cnt_1Hz;

				// todo interval charging at low bat temp to maximise resistive power loss and generate heat
				int p_bat_50Hz = get_p_dc_filt50Hz() - get_p_ac_filt50Hz();
				static uint16_t cnt_lowPV = 0;

				switch (stateHYBRID_AC) {
					case HYB_AC_OFF:
						if (   battery_connected()
							&& (   (battery->soc_percent > 10          // enough energy for feedin
								    && battery->minVcell_mV > (BATTERY.V_CELL_MIN_POWER_REDUCE_mV+80)
								    && (ctrl_ref->p_pcc > 50 && ctrl_ref->p_pcc_prev > 50))  // feedin required; filter 1 second spikes from freezer motor start
							    || bms.warn_temperature()              // hot or cold battery -> PV2AC
							    || battery_almost_full()               // or battery charge power has to be reduced
							    || p_bat_50Hz >= p_bat_chg_max         // or battery charge power is large and has to be reduced
							    || ctrl_ref->ext_ctrl_mode != EXT_OFF  // or external control
							   )
							) {
								stateHYBRID_AC = HYB_AC_ON;
								cnt_lowPV = 0;
						// Shutdown of already discharged battery when PV power is low
						} else if(   battery_connected()
								  && get_p_dc_filt50Hz() < P_BAT_MIN_CHARGE
								  && battery_almost_empty()
								  ) {
							cnt_lowPV++;
							if (cnt_lowPV == 500) {  // ~500 seconds, but function call has no defined interval
								nextMode(OFF);
								cnt_lowPV = 0;
							}
						} else {
							cnt_lowPV = 0;
						}
						break;

					case HYB_AC_ON:

						if (!battery_connected()) {  // AC already on but battery reconnect because previous mode was PV2AC
							// wait until battery is connected
						} else {
							ctrl_ref->mode = PAC_CONTROL;
						}

						// check if battery should be disconnected (variable Vdc -> higher efficiency)
						if ( (bms.warn_temperature())
							 || ( (ctrl_ref->p_pcc < -50 || electricity_meter_get_status() == EL_METER_CONN_ERR)  // other PV inverters produce enough for household or PCC sensor fail
							      && (battery->power_W > P_MIN_PV2AC)  // minimum charging power to compensate switching loss for AC bridge
							      && battery_full()
							  	  )
							) {
							nextMode(PV2AC);

						// AC turnoff in case of empty battery.
						// during day, AC stays connected, but AC energy packet control turns off gatepulses if no feedin required
						} else if ( battery_connected()
									&& (  battery_empty()
									    || bms.warn_temperature_cell_high()  // if battery hot, use PV2AC mode
									    || battery->voltage_100mV < get_v_ac_amp_filt50Hz_100mV()  // if battery voltage is lower than grid voltage
									    || get_p_ac_max_dc_lim() < 40  // if battery power became low because of heat or low voltage
									   )
						   ) {
							// prevent system shutdown during sunrise
							//if(time > time_sunrise && time < time_sundown)  // todo PLL-like time estimation for day and night
							if (get_p_dc_filt50Hz() >= P_MIN_PV2AC) {
								nextMode(PV2AC);
							} else if (get_p_dc_filt50Hz() >= P_BAT_MIN_CHARGE) {
								ctrl_ref->mode = AC_PASSIVE;
							} else {
								ctrl_ref->mode = AC_OFF;
								nextMode(OFF);
							}
						}
						break;

					default:
						stateHYBRID_AC = HYB_AC_OFF;
						break;
				}
			// if MPP Tracker does not provide enough power to charge DC bus, shutdown battery after 30sec
			} else if (cnt_1Hz > (cnt_1Hz_chargeDC+30)) {
				sys_mode_needs_battery = false;
				ctrl_ref->mode = AC_OFF;
			}
			break;
#endif //SYSTEM_HAS_BATTERY

		  default:
			break;
		}

}
