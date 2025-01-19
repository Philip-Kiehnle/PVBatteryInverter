#ifndef _MODBUS_HYBRIDINVERTER_H_
#define _MODBUS_HYBRIDINVERTER_H_

#ifdef __cplusplus
extern          "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#include "modbus.h"
#include "modbus_reg_hybridinverter.h"


extern modbus_reg_rw_t modbus_reg_rw;

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

#endif // _MODBUS_HYBRIDINVERTER_H_
