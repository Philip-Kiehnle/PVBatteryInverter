#ifndef _MODBUS_REG_HYBRIDINVERTER_H_
#define _MODBUS_REG_HYBRIDINVERTER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


typedef enum {
	CMD_INVALID,
	CMD_INVERTER_OFF, CMD_INVERTER_ON, CMD_INVERTER_RESET,
	CMD_P_AC_EXT_OFF,
	CMD_AC_LOCK_INACTIVE, CMD_AC_LOCK_SOFT, CMD_AC_LOCK_HARD,  // AC softlock gets disabled by excess PV power
	CMD_DC_LOCK_INACTIVE, CMD_DC_LOCK_SOFT, CMD_DC_LOCK_HARD,  // DC softlock gets disabled by PV timer set in modbus register
} modbus_cmd_t;

typedef struct {
	// General
	uint16_t cmd;
	uint16_t fan_test;

	// PV related
	uint16_t temperature_outdoor_celsius;  // used to calculate PV start voltage
	uint16_t number_of_pv_modules_stringA;  // used to optimize MPP tracking
	uint16_t number_of_bypass_diodes_per_module_stringA;  // used to optimize MPP tracking
	uint16_t number_of_pv_modules_stringB;
	uint16_t number_of_bypass_diodes_per_module_stringB;
	uint16_t interval_glob_mppt_regular_sec;
	uint16_t interval_glob_mppt_trig_event_sec;  // when power drops
	uint16_t pv_ref_v_100mV;  // keeps pv voltage at a defined level for testing MPPT performance
	uint16_t pv_ref_duration_sec;  // duration until MPPT takes over again
	uint16_t pv_dc_softlock_duration_minutes;  // duration until softlock cmd is released automatically

	// battery related
	uint16_t soc_min_protect_percent;
	uint16_t soc_max_protect_percent;
	uint16_t p_bat_chg_max_W;
	uint16_t p_bat_dischg_max_W;
	uint16_t bat_cell_v_bal_target_mV;

	// AC related
	uint16_t ext_ctrl_mode;  // ext_ctrl_mode_t
	uint16_t p_ac_soft_W;
	uint16_t p_ac_hard_W;
} modbus_reg_rw_t;


typedef struct {
	uint16_t fan_state;

	// battery related: last 8 elements of batteryStatus_t
	uint16_t bat_soc_10mPercent;
	uint16_t bat_voltage_100mV;
	int16_t bat_power_W;
	uint16_t bat_p_charge_max;
	uint16_t bat_p_discharge_max;
	uint16_t bat_minVcell_mV;
	uint16_t bat_maxVcell_mV;
	uint16_t bat_minTempCell_maxTempCell;  // [minTemp; maxTemp]
	uint16_t bat_maxTempPCB;

	uint16_t bat_cell_v_mV[96];
	uint16_t bat_cell_balancing[8];
	uint16_t bat_cell_temperature[4*12];
	uint16_t bat_PCB_temperature[4*2];
} modbus_reg_ro_t;

#ifdef __cplusplus
}
#endif

#endif // _MODBUS_REG_HYBRIDINVERTER_H_
