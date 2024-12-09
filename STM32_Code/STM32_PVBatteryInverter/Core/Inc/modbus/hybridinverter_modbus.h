

#ifndef _HYBRIDINVERTER_MODBUS_H_
#define _HYBRIDINVERTER_MODBUS_H_

#ifdef __cplusplus
extern          "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#include "modbus.h"


typedef enum {CMD_INVALID, CMD_INVERTER_OFF, CMD_INVERTER_ON, CMD_INVERTER_RESET, CMD_P_AC_EXT_OFF} modbus_cmd_t;

typedef struct {
	// General
	uint16_t cmd;

	// PV related
	uint16_t temperature_outdoor_celsius;  // used to calculate PV start voltage
	uint16_t number_of_pv_modules_stringA;  // used to optimize MPP tracking
	uint16_t number_of_bypass_diodes_per_module_stringA;  // used to optimize MPP tracking
	uint16_t number_of_pv_modules_stringB;
	uint16_t number_of_bypass_diodes_per_module_stringB;

	// battery related
	uint16_t soc_min_protect_percent;
	uint16_t soc_max_protect_percent;
	uint16_t p_bat_chg_max_W;

	// AC related
	uint16_t ext_ctrl_mode;  // ext_ctrl_mode_t
	uint16_t p_ac_soft_W;
	uint16_t p_ac_hard_W;
} modbus_param_rw_t;

#ifdef __cplusplus  // modbus default param only usable in C++ files
constexpr modbus_param_rw_t MODBUS_PARAM_RW_DEFAULT = {
	// General
	.cmd = CMD_INVALID,

	// PV related
	.temperature_outdoor_celsius = 10,  // used to calculate PV start voltage
	.number_of_pv_modules_stringA = 8,  // used to optimize MPP tracking
	.number_of_bypass_diodes_per_module_stringA = 3,  // used to optimize MPP tracking
	.number_of_pv_modules_stringB = 8,
	.number_of_bypass_diodes_per_module_stringB = 3,

	// battery related
	.soc_min_protect_percent = 8,
	.soc_max_protect_percent = 90,
	.p_bat_chg_max_W = 40,

	// AC related
	//ext_ctrl_mode;  // ext_ctrl_mode_t
	.p_ac_soft_W = 0,
	.p_ac_hard_W = 0
};
#endif

extern modbus_param_rw_t modbus_param_rw;

extern bool modbus_p_ac_soft_update;
extern bool modbus_p_ac_hard_update;

/*
 * function mbus_isma_open()
 * open new modbus context for new port
 * return: MODBUS_ERROR - if can't open context
*/

mbus_status_t mbus_somedev_read_3xxxx(mbus_t mb_context);

mbus_status_t mbus_somedev_read_4xxxx(mbus_t mb_context);

uint16_t mbus_hybridinverter_read(uint32_t la);

uint16_t mbus_hybridinverter_write(uint32_t la, uint16_t value);

mbus_t mbus_hybridinverter_open(Modbus_Conf_t* pconf);


#ifdef __cplusplus
}
#endif

#endif // _HYBRIDINVERTER_MODBUS_H_
