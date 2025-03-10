
#include <algorithm>

#include "gpio.h"
#include "dc_control.h"
#include "battery.h"
#include "config_battery.h"

#if IS_BATTERY_SUPERVISOR_PCB != 1
#include <modbus_hybridinverter.h>
#endif

#include "BatteryManagement/STW_mBMS.hpp"
extern STW_mBMS bms;

// does not compile with BaseBMS
//#include "battery/BaseBMS.hpp"
//#include "battery/ETI_DualBMS.hpp"
//extern BaseBMS bms;


static volatile bool update_request;
static volatile stateBattery_t stateBattery_next = DEFAULT_BATTERY_STATE;
static stateBattery_t stateBattery = DEFAULT_BATTERY_STATE;
static uint16_t bat_comm_fail_cnt;
static uint16_t bat_state_fail_cnt;  // for battery disconnects
static bool bat_connected;

#if SYSTEM_HAS_BATTERY == 1
const batteryStatus_t* get_batteryStatus()
{
	 return &bms.batteryStatus;
}


const uint16_t get_battery_nr_series_cells()
{
	return bms.get_nr_series_cells();
}


const int8_t get_battery_cell_temperature(uint8_t nr)
{
	return bms.cellStack.temperature_cell[nr];
}


const int8_t get_battery_PCB_temperature(uint8_t nr)
{
	return bms.cellStack.temperature_PCB[nr];
}


const uint16_t get_battery_vCell_mV(uint8_t nr)
{
	return bms.cellStack.vCell_mV[nr];
}


const bool get_battery_balancingState(uint8_t nr)
{
	return bms.cellStack.balancingState[nr];
}


const uint64_t get_battery_csc_err(uint8_t nr)
{
	return bms.cellStack.csc_err[nr];
}

const char* get_battery_csc_err_str(uint8_t bit)
{
	return csc_err_str[bit];
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
	bool cell_imbalance_flag =    (bms.batteryStatus.minVcell_mV < BATTERY.V_CELL_NOM_mV)
			                   && ((bms.batteryStatus.maxVcell_mV - bms.batteryStatus.minVcell_mV) > V_CELL_IMBALANCE_WARN_mV);

#if IS_BATTERY_SUPERVISOR_PCB == 1
	const float soc_ref = 2.0;
#else
	const float soc_ref = std::clamp(modbus_reg_rw.soc_min_protect_percent, (uint16_t)1, (uint16_t)90);
#endif
	return (bms.batteryStatus.soc_percent <= soc_ref || bms.batteryStatus.minVcell_mV <= BATTERY.V_CELL_MIN_PROTECT_mV || cell_imbalance_flag);
}

const bool battery_almost_empty()
{
	bool cell_imbalance_flag =    (bms.batteryStatus.minVcell_mV < BATTERY.V_CELL_NOM_mV)
			                   && ((bms.batteryStatus.maxVcell_mV - bms.batteryStatus.minVcell_mV) > V_CELL_IMBALANCE_INFO_mV);

#if IS_BATTERY_SUPERVISOR_PCB == 1
	const float soc_ref = 5.0;
#else
	const float soc_ref = std::clamp((uint16_t)(modbus_reg_rw.soc_min_protect_percent+5), (uint16_t)1, (uint16_t)90);
#endif
	return (bms.batteryStatus.soc_percent <= soc_ref || bms.batteryStatus.minVcell_mV <= (BATTERY.V_CELL_MIN_PROTECT_mV+BATTERY.V_CELL_MIN_POWER_REDUCE_mV)/2 || cell_imbalance_flag);
}

const bool battery_almost_full()
{
	bool cell_imbalance_flag =    (bms.batteryStatus.minVcell_mV > BATTERY.V_CELL_NOM_mV && bms.batteryStatus.maxVcell_mV > BATTERY.V_CELL_NOM_mV)
			                   && ((bms.batteryStatus.maxVcell_mV - bms.batteryStatus.minVcell_mV) > V_CELL_IMBALANCE_INFO_mV);

#if IS_BATTERY_SUPERVISOR_PCB == 1
	const float soc_ref = 98.0;
#else
	const float soc_ref = std::clamp((uint16_t)(modbus_reg_rw.soc_max_protect_percent-1), (uint16_t)10, (uint16_t)100);
#endif
	return (bms.batteryStatus.soc_percent >= soc_ref || bms.batteryStatus.maxVcell_mV >= BATTERY.V_CELL_MAX_POWER_REDUCE_mV || cell_imbalance_flag);
}

const bool battery_full()
{
	bool cell_imbalance_flag =    (bms.batteryStatus.maxVcell_mV > BATTERY.V_CELL_NOM_mV)
			                   && ((bms.batteryStatus.maxVcell_mV - bms.batteryStatus.minVcell_mV) > V_CELL_IMBALANCE_WARN_mV);

#if IS_BATTERY_SUPERVISOR_PCB == 1
	const float soc_ref = 100.0;
#else
	const float soc_ref = std::clamp(modbus_reg_rw.soc_max_protect_percent, (uint16_t)10, (uint16_t)100);
#endif
	return (bms.batteryStatus.soc_percent >= soc_ref || bms.batteryStatus.maxVcell_mV >= BATTERY.V_CELL_MAX_PROTECT_mV || cell_imbalance_flag);
}


bool battery_set_balancing(uint8_t stack_mask, uint16_t vCellBalTarget_mV)
{
	if (bms.set_balancing(stack_mask, vCellBalTarget_mV) == BMS_OK) {
		return true;
	} else {
		return false;
	}
}


static void check_bat_error()
{
	if (bms.fault_v_cell_min()) {
		set_sys_errorcode(EC_BATTERY_V_CELL_MIN);

	} else if (bms.fault_v_cell_max()) {
		set_sys_errorcode(EC_BATTERY_V_CELL_MAX);

	} else if (bms.fault_v_cell_imbalance()) {
		set_sys_errorcode(EC_BATTERY_V_CELL_IMBALANCE);

	} else if (bms.fault_i_charge_max()) {
		set_sys_errorcode(EC_BATTERY_I_CHARGE_RMS_MAX);

	} else if (bms.fault_i_discharge_max()) {
		set_sys_errorcode(EC_BATTERY_I_DISCHARGE_RMS_MAX);

	} else if (bms.fault_temperature_cell_min()) {
		set_sys_errorcode(EC_BATTERY_TEMPERATURE_CELL_MIN);

	} else if (bms.fault_temperature_cell_max()) {
		set_sys_errorcode(EC_BATTERY_TEMPERATURE_CELL_MAX);

	} else if (bms.fault_temperature_PCB_max()) {
		set_sys_errorcode(EC_BATTERY_TEMPERATURE_PCB_MAX);

	} else if (bms.fault_other()) {
		set_sys_errorcode(EC_BATTERY_OTHER);
	}
}


void battery_state_request(stateBattery_t state)
{
	// in case of empty battery, use ship mode and turnoff BMS
	if (state == BAT_OFF) {
		if (   bms.batteryStatus.soc_percent > 30
		    && get_sys_errorcode() == EC_NO_ERROR
		) {
			state = BMS_ON__BAT_OFF;
		} else {
			state = BMS_OFF__BAT_OFF;
		}
	}

	stateBattery_next = state;
}

// check if sum of cell voltages deviates from DC bus voltage e.g. +-10Volt (~0.1V per Cell)
bool battery_bus_voltage_match_coarse()
{
	if (   bms.batteryStatus.voltage_100mV > get_v_dc_FBboost_filt50Hz_100mV()-BAT_BUS_VOLTAGE_TOLERANCE_COARSE_100mV
	    && bms.batteryStatus.voltage_100mV < get_v_dc_FBboost_filt50Hz_100mV()+BAT_BUS_VOLTAGE_TOLERANCE_COARSE_100mV
	) {
		return true;
	}
	return false;
}


static bool battery_bus_voltage_match_fine()
{
	if (   bms.batteryStatus.voltage_100mV > get_v_dc_FBboost_filt50Hz_100mV()-BAT_BUS_VOLTAGE_TOLERANCE_FINE_100mV
	    && bms.batteryStatus.voltage_100mV < get_v_dc_FBboost_filt50Hz_100mV()+BAT_BUS_VOLTAGE_TOLERANCE_FINE_100mV
	) {
		return true;
	}
	return false;
}
#endif //SYSTEM_HAS_BATTERY


void battery_update_request()
{
	update_request = true;
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
			  bmsPower(0);  // for batteries with supply voltage control
			  if (bms.batteryOff(0b01) == BMS_OK) {
				  stateBattery = BMS_ON__BAT_OFF;
				  bat_connected = false;
			  }
			  if (bms.shipmode(0b00111111) == BMS_OK) {
				  stateBattery = BMS_OFF__BAT_OFF;
				  bat_connected = false;
			  }
			  bms.batteryStatus = {0};
			  break;

		  case BMS_ON__BAT_ON:
		  {
			  bmsPower(1);  // for batteries with supply voltage control
			  BMS_StatusTypeDef status = bms.batteryOn(0b01);
			  if (status == BMS_OK) {
				  stateBattery = BMS_ON__BAT_ON;
			  } else if (status == BMS_NCK) {
				  stateBattery = BMS_ON__BAT_OFF;
			  }
			  break;
		  }

		  case BMS_ON__BAT_OFF:
		  default:
			  bmsPower(1);  // for batteries with supply voltage control
			  if (bms.batteryOff(0b01) == BMS_OK) {
				  stateBattery = BMS_ON__BAT_OFF;
				  bat_connected = false;
			  }
			  break;
		}
	}

	if ( stateBattery == BMS_ON__BAT_ON
	     && (abs(bms.batteryStatus.power_W) > 5
			 || battery_bus_voltage_match_fine()
		)
	) {
		bat_connected = true;
	}

	if (update_request) {
		rate_limit = false;
		update_request = false;
		if (   stateBattery != BMS_OFF__BAT_OFF
		    && get_sys_errorcode() == EC_NO_ERROR
		) {
			if (bms.get_summary() == BMS_OK) {
#if IS_BATTERY_SUPERVISOR_PCB != 1  // supervisor does not need SoC
				bms.estimateSoC();  // estimator has dt=1sec, so call it every second
#endif
				check_bat_error();
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

		if ( stateBattery == BMS_ON__BAT_ON) {
			// check for battery disconnect
			if (bms.batteryStatus.power_W == 0) {
				if (bat_state_fail_cnt == 180) {  // 3 minutes
					if (!battery_bus_voltage_match_fine()) {
						bat_state_fail_cnt = 0;
						stateBattery = BMS_ON__BAT_OFF;
						//NVIC_SystemReset();  // todo check negative effects
					}
				} else {
					bat_state_fail_cnt++;
				}
			} else {
				bat_state_fail_cnt = 0;
			}
		}
	}
#else
	if (update_request) {
		update_request = false;
		return_code = true;
	}
#endif //SYSTEM_HAS_BATTERY
	return return_code;
}
