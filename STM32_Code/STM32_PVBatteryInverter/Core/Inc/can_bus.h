#ifndef INC_CAN_BUS_H
#define INC_CAN_BUS_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#include "battery/bms_types.h"

void can_bus_read();

extern struct CellStack cellStack;

#ifdef __cplusplus
}
#endif

#endif /* INC_CAN_BUS_H */
