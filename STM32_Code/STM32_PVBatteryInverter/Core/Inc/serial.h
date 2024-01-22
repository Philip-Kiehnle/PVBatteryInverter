#ifndef SERIAL_H
#define SERIAL_H

#include <stdint.h>

#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint16_t header;
    uint16_t srcAddr;
    uint16_t dstAddr;
    uint8_t ctrl;
    uint8_t fct;
    uint8_t dLen;
    uint8_t data[255];
    uint16_t checksum;
} __attribute__((__packed__)) msg_t;

void serial_write(int serial_fd, const char *data, int size);
int serial_read(int serial_fd, char *data, int size, int timeout_usec);

#ifdef __cplusplus
}
#endif

#endif  // SERIAL_H
