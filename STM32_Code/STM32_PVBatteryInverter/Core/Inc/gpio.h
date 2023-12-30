#ifndef INC_GPIO_H
#define INC_GPIO_H

#include <stdint.h>
#include <stdbool.h>

void gatedriverDC(bool state);
void gatedriverAC(bool state);
void contactorAC(bool state);
void contactorBattery(bool state);


#endif /* INC_GPIO_H */
