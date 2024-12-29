
#include "stm32g4xx_hal.h"

#include "gpio.h"
#include "common.h"


void gatedriverDC(bool state)
{
	if ( state && get_sys_errorcode()==EC_NO_ERROR ) {
		GPIOB->BSRR = (1<<7);  // enable boost gatedriver
	} else {
		GPIOB->BRR = (1<<7);  // disable boost gatedriver
	}
}


void contactorBattery(bool state)
{
	// Bat2DCboost
	if ( state ) {
		GPIOC->BSRR = (1<<13);
	} else {
		GPIOC->BRR = (1<<13);
	}
}

void bmsPower(bool state)
{
	// Mosfet for 12V BMS supply
	if ( state ) {
		GPIOC->BSRR = (1<<0);
	} else {
		GPIOC->BRR = (1<<0);
	}
}

