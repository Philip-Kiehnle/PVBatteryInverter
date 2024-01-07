/*
 * bms_types.h
 *
 *  Created on: 23.02.2023
 *      Author: Philip Kiehnle
 */

#ifndef SRC_BMS_TYPES_H_
#define SRC_BMS_TYPES_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>


// upper status byte (errors detected in microcontroller code)
//#define reserved                    (1<<7)
#define CELL_IMBALANCE_FAULT          (1<<6)
#define MAX_AMBIENT_TEMPERATURE_FAULT (1<<5)
#define MIN_AMBIENT_TEMPERATURE_FAULT (1<<4)
#define MAX_MOSFET_TEMPERATURE_FAULT  (1<<3)
#define MIN_MOSFET_TEMPERATURE_FAULT  (1<<2)
#define MAX_BAT_TEMPERATURE_FAULT     (1<<1)
#define MIN_BAT_TEMPERATURE_FAULT     (1<<0)

// lower status byte (status and errors detected in BQ chip)
#define FET_DSG_ON    (1<<7)
#define FET_CHG_ON    (1<<6)
// SYS_STAT bit masks
#define DEVICE_XREADY (1<<5)
#define OVRD_ALERT    (1<<4)
#define UV            (1<<3)
#define OV            (1<<2)
#define SCD           (1<<1)
#define OCD           (1<<0)


typedef struct {
    uint8_t status;
    volatile uint8_t faultsDetectedByMicrocontroller;
    uint8_t soc;
    int8_t temperatureBatteryMAX;
    int8_t temperatureBatteryMIN;
    int8_t temperatureMOSFET;
    int16_t current_mA;
    uint32_t voltage_mV;
    uint16_t vCell_min_mV;
    uint16_t vCell_max_mV;
} __attribute__((__packed__)) battery_t;

struct DualBMS {
    battery_t battery[2];
    uint8_t crc;
} __attribute__((__packed__));


struct CellStack {
    uint16_t vCell_mV[96];
    bool balancingState[96];
    int8_t temperature[14*4];
    uint64_t csc_err[4];
} __attribute__((__packed__));


#ifdef __cplusplus
}
#endif

#endif /* SRC_BMS_TYPES_H_ */
