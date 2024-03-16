#ifndef INC_ELECTRICITY_METER_H
#define INC_ELECTRICITY_METER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define EL_METER_RX_BUF_TYP_SIZE 236

enum el_meter_status_t {EL_METER_OKAY, EL_METER_CONN_WARN, EL_METER_CONN_ERR};
typedef enum el_meter_status_t el_meter_status_t;

el_meter_status_t electricity_meter_get_status();
el_meter_status_t electricity_meter_read(UART_HandleTypeDef* huart);
int electricity_meter_get_power();

#ifdef __cplusplus
}
#endif

#endif /* INC_ELECTRICITY_METER_H */
