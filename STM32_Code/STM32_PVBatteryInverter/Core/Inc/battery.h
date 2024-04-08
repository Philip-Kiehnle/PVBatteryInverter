#ifndef INC_BATTERY_H
#define INC_BATTERY_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "common.h"
#include "battery/bms_types.h"


typedef enum {BMS_OFF__BAT_OFF, BMS_ON__BAT_OFF, BMS_ON__BAT_ON, BAT_OFF} stateBattery_t;

const bool battery_connected();
const bool battery_full();
const bool battery_maxVcell_OK();
const stateBattery_t get_stateBattery();
const uint16_t get_p_bat_discharge_max();
const batteryStatus_t* get_batteryStatus();
void battery_update_request();
void battery_state_request(stateBattery_t state);
bool async_battery_communication();


#ifdef __cplusplus
}
#endif

#endif /* INC_BATTERY_H */
