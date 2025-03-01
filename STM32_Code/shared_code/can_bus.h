#ifndef INC_CAN_BUS_H
#define INC_CAN_BUS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#include "BatteryManagement/bms_types.h"

void can_bus_set_filter(uint8_t fifo_nr, uint8_t filter_index, uint32_t filter_type, uint32_t filter_id1, uint32_t filter_id2);
void can_bus_disable_filter(uint8_t filter_index);
void can_bus_FIFO_drop_msgs(uint8_t fifo_nr, uint8_t nr_msgs_left);
uint8_t* can_bus_getRxMsg_8byte(uint8_t fifo_nr, uint32_t* id, bool blocking);
bool addTxMsg_8byte(uint32_t id, uint8_t* tx_data);
void can_bus_stop(uint8_t fifo_nr);


#ifdef __cplusplus
}
#endif

#endif /* INC_CAN_BUS_H */
