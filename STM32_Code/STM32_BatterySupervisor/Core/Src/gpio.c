
#include "stm32g4xx_hal.h"

#include "gpio.h"


void contactorBattery(bool state)
{
	// Bat2DCboost
	if ( state ) {
		GPIOC->BSRR = (1<<13);
	} else {
		GPIOC->BRR = (1<<13);
	}
}
