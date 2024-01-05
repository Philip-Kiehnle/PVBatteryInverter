#ifndef INC_GPIO_H
#define INC_GPIO_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

void gatedriverDC(bool state);
void gatedriverAC(bool state);
void contactorAC(bool state);
void contactorBattery(bool state);
void bmsPower(bool state);

#ifdef __cplusplus
}
#endif

#endif /* INC_GPIO_H */
