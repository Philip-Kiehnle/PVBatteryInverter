#ifndef INC_CAN_BUS_H
#define INC_CAN_BUS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#include "BatteryManagement/bms_types.h"

void can_bus_init();
void can_bus_set_filter(uint32_t filter_type, uint32_t filter_id1, uint32_t filter_id2);

uint8_t* getRxMsg_8byte_blocking(uint32_t* id);
uint8_t* getRxMsg_8byte_singleID_blocking(uint32_t filter_id);

bool addTxMsg_8byte(uint32_t id, uint8_t* tx_data);


#ifdef __cplusplus
}
#endif

#endif /* INC_CAN_BUS_H */
