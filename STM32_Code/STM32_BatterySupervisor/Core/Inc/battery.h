#ifndef INC_BATTERY_H
#define INC_BATTERY_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "common.h"
#include "BatteryManagement/bms_types.h"


typedef enum {BMS_OFF__BAT_OFF, BMS_ON__BAT_OFF, BMS_ON__BAT_ON, BAT_OFF} stateBattery_t;

const bool battery_connected();
const bool battery_empty();
const bool battery_almost_empty();
const bool battery_almost_full();
const bool battery_full();
const bool battery_maxVcell_OK();
const stateBattery_t get_stateBattery();
const uint16_t get_p_bat_discharge_max();
const batteryStatus_t* get_batteryStatus();

// for debug printing
const int8_t    get_battery_temperature(uint8_t nr);
const uint16_t  get_battery_vCell_mV(uint8_t nr);
const bool      get_battery_balancingState(uint8_t nr);
const uint64_t  get_battery_csc_err(uint8_t nr);
const char*     get_battery_csc_err_str(uint8_t bit);

void battery_update_request();
void battery_state_request(stateBattery_t state);
bool async_battery_communication();
bool battery_set_balancing(uint8_t stack_mask, uint16_t vCellBalTarget_mV);

#ifdef __cplusplus
}
#endif

#endif /* INC_BATTERY_H */
