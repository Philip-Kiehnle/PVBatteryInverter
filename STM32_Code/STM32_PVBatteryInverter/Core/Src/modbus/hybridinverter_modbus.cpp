#include <modbus_hybridinverter.h>
#include <BatteryManagement/bms_types.h>
#include "battery.h"
#include "config_pv.h"
#include "mpptracker.hpp"
#include "dc_control.h"
#include "fan_control.h"
#include <common.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern          "C"
{
#endif

modbus_reg_rw_t modbus_reg_rw = {
	// General
	.cmd = CMD_INVALID,
	.fan_test = 0,

	// PV related
	.temperature_outdoor_celsius = 10,  // used to calculate PV start voltage
	.number_of_pv_modules_stringA = MPPTPARAM.nr_pv_modules,  // used to optimize MPP tracking; todo: implement runtime change in MPPT
	.number_of_bypass_diodes_per_module_stringA = PVMODULE.nr_bypassDiodes,  // used to optimize MPP tracking; todo: implement runtime change in MPPT
	.number_of_pv_modules_stringB = MPPTPARAM.nr_pv_modules,
	.number_of_bypass_diodes_per_module_stringB = PVMODULE.nr_bypassDiodes,
	.interval_glob_mppt_regular_sec = GMPPTPARAM.interval_glob_mppt_regular_sec,
	.interval_glob_mppt_trig_event_sec = 0,
	.pv_ref_v_100mV = 200*10,
	.pv_ref_duration_sec = 10,

	// battery related
	.soc_min_protect_percent = 8,
	.soc_max_protect_percent = 90,
	.p_bat_chg_max_W = 350*4,

	// AC related
	//ext_ctrl_mode;  // ext_ctrl_mode_t
	.p_ac_soft_W = 0,
	.p_ac_hard_W = 0
};

bool modbus_p_ac_soft_update;
bool modbus_p_ac_hard_update;

extern MPPTracker mppTracker;

//Function for initialization modbus on you device
mbus_t mbus_hybridinverter_open(Modbus_Conf_t *pconf)
{
	 mbus_t mb;

	 pconf->devaddr = 10;
	 pconf->device = (void*) 0;

	//Define read callback function
    pconf->read = &mbus_hybridinverter_read;
	
	//Define write callback function
    pconf->write = &mbus_hybridinverter_write;

    //Open modbus context
	mb = mbus_open(pconf);

    if (mb < 0 ) return (mbus_t)MBUS_ERROR;

    return mb;
}


/* It's modbus request on read register by logical address (la) */
uint16_t mbus_hybridinverter_read(uint32_t la)
{
	// read only registers
	constexpr uint16_t OFFSET = 30001;
	if (la >= OFFSET && la < OFFSET+sizeof(modbus_reg_ro_t)/2) {
		int addr = la-OFFSET;

		constexpr uint16_t REG_START_bat_cell_v_mV = offsetof(modbus_reg_ro_t, bat_cell_v_mV)/2;
		constexpr uint16_t REG_END_bat_cell_v_mV = REG_START_bat_cell_v_mV + sizeof(modbus_reg_ro_t::bat_cell_v_mV)/2;

		constexpr uint16_t REG_START_bat_cell_balancing = offsetof(modbus_reg_ro_t, bat_cell_balancing)/2;
		constexpr uint16_t REG_END_bat_cell_balancing = REG_START_bat_cell_balancing + sizeof(modbus_reg_ro_t::bat_cell_balancing)/2;

		if (addr >= REG_START_bat_cell_v_mV && addr < REG_END_bat_cell_v_mV) {
			return get_battery_vCell_mV(addr-REG_START_bat_cell_v_mV);

		} else if (addr >= REG_START_bat_cell_balancing && addr < REG_END_bat_cell_balancing) {
			uint8_t id_stack = addr-REG_START_bat_cell_balancing;
			uint16_t bal_state = 0;
			for (int i=0; i<12; i++) {  // 12 cells per stack
				bal_state |= get_battery_balancingState(12*id_stack + i) << i;
			}
			return bal_state;

		} else if (addr == offsetof(modbus_reg_ro_t, fan_state)/2) {
			return fan_control_get_state();
		}

	// read and write registers
	} else if (la >= 40001 && la < 40001+sizeof(modbus_reg_rw)/2) {
		int addr = la-40001;
		uint16_t* regs = (uint16_t*) &modbus_reg_rw;
		return regs[addr];
	}

	//return mbus_response(mb_context, MBUS_RESPONSE_ILLEGAL_DATA_ADDRESS);
	return la;
}


//Function for write to some register by logic address
uint16_t mbus_hybridinverter_write(uint32_t la, uint16_t value)
{
    //printf("We write: %d %d\n",la, value);
	constexpr uint16_t OFFSET = 40001;

	if (la >= OFFSET && la < OFFSET+sizeof(modbus_reg_rw)/2) {
		int addr = la-OFFSET;

		uint16_t* regs = (uint16_t*)&modbus_reg_rw;
		regs[addr] = value;

		if (addr == offsetof(modbus_reg_rw_t, p_ac_soft_W)/2) {
			modbus_p_ac_soft_update = true;

		} else if (addr == offsetof(modbus_reg_rw_t, p_ac_hard_W)/2) {
			modbus_p_ac_hard_update = true;

		} else if (addr == offsetof(modbus_reg_rw_t, pv_ref_v_100mV)/2) {
			mppTracker.set_voltage( ((float)modbus_reg_rw.pv_ref_duration_sec)/10, ((float)modbus_reg_rw.pv_ref_v_100mV)/10, get_v_dc_filt50Hz());

		} else if (addr == offsetof(modbus_reg_rw_t, bat_cell_v_bal_target_mV)/2) {
			uint16_t v_bal_mV = modbus_reg_rw.bat_cell_v_bal_target_mV;
			if (v_bal_mV >= 3600 && v_bal_mV <= 4200) {
				battery_set_balancing(0b1111, v_bal_mV);  // all CSCs
			}
		} else if (addr == offsetof(modbus_reg_rw_t, fan_test)/2) {
			fan_control_test(regs[addr]);
		}
	}

    return value;
}



#ifdef __cplusplus
}
#endif
