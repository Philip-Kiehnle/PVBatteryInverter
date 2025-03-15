
#include "stm32g4xx_hal.h"

#include "gpio.h"
#include "common.h"


inline void gatedriverDC(bool state)
{
	if ( state && get_sys_errorcode()==EC_NO_ERROR ) {
		GPIOB->BSRR = (1<<7);  // enable boost gatedriver
	} else {
		GPIOB->BRR = (1<<7);  // disable boost gatedriver
	}
}

inline void gatedriverAC(bool state)
{
	if ( state && get_sys_errorcode()==EC_NO_ERROR ) {
		GPIOB->BSRR = (1<<14);  // enable grid FB gatedriver
	} else {
		GPIOB->BRR = (1<<14);  // disable grid FB gatedriver
	}
}

inline void contactorAC(bool state)
{
	if ( state && get_sys_errorcode()==EC_NO_ERROR ) {
		GPIOB->BSRR = (1<<10);
	} else {
		GPIOB->BRR = (1<<10);
	}
}

inline void contactorBattery(bool state)
{
#if IS_BATTERY_SUPERVISOR_PCB == 1
	// Bat2DCboost -> Now used for negative battery contactor in BatterySupervisor. On Main Control PCB, pin is used for fan control.
	if ( state && get_sys_errorcode()==EC_NO_ERROR ) {
		GPIOC->BSRR = (1<<13);  // enable contactor
		GPIOB->BRR = (1<<1);  // enable green LED
	} else {
		GPIOC->BRR = (1<<13);  // disable contactor
		GPIOB->BSRR = (1<<1);  // disable green LED
	}
#endif // IS_BATTERY_SUPERVISOR_PCB == 1

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

void fanAC(bool state)
{
	// Pin name in KiCAD: Bat2DCboost
	if ( state ) {
		GPIOC->BSRR = (1<<13);
	} else {
		GPIOC->BRR = (1<<13);
	}
}
