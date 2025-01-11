
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

void gatedriverAC(bool state)
{
	if ( state && get_sys_errorcode()==EC_NO_ERROR ) {
		GPIOB->BSRR = (1<<14);  // enable grid FB gatedriver
	} else {
		GPIOB->BRR = (1<<14);  // disable grid FB gatedriver
	}
}

void contactorAC(bool state)
{
	if ( state && get_sys_errorcode()==EC_NO_ERROR ) {
		GPIOB->BSRR = (1<<10);
	} else {
		GPIOB->BRR = (1<<10);
	}
}

void contactorBattery(bool state)
{
	// Bat2DCboost -> Now used for negative battery contactor. Only connected to BatterySupervisor.
	if ( state && get_sys_errorcode()==EC_NO_ERROR ) {
		GPIOC->BSRR = (1<<13);
	} else {
		GPIOC->BRR = (1<<13);
	}

	// Bat2Inverter -> Now used for positive battery contactor. Only connected to PVBatteryInverter.
	if ( state && get_sys_errorcode()==EC_NO_ERROR ) {
		GPIOC->BSRR = (1<<14);
	} else {
		GPIOC->BRR = (1<<14);
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
