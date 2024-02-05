#ifndef INC_BATTERY_H
#define INC_BATTERY_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "common.h"
#include "battery/bms_types.h"

#define BATTERY_ON true
#define BATTERY_OFF false

const batteryStatus_t* get_batteryStatus();
void shutdownBattery();
void battery_update_request();
void battery_state_request(bool state);
void async_battery_communication();


#ifdef __cplusplus
}
#endif

#endif /* INC_BATTERY_H */
