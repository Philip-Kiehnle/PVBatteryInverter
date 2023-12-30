
#include "stm32g4xx_hal.h"

#include <stdio.h>

#include "common.h"
#include "gpio.h"


static char strerror_buf[1024];
volatile error_t sys_errcode;


struct _sys_errordesc {
    int  errcode;
    char *message;
} sys_errordesc[] = {
    { EC_NO_ERROR, "EC_NO_ERROR : No error" },
    { EC_EMERGENCY_STOP, "EC_EMERGENCY_STOP : Emergency button pressed" },
    { EC_V_DC_MAX, "EC_V_DC_MAX : DC voltage too high" },
	{ EC_V_DC_SENSOR, "EC_V_DC_SENSOR : Vdc sigma delta pulse count invalid"}
};


char *strerror(int errcode)
{
	int sys_nerr = sizeof(sys_errordesc) / sizeof(sys_errordesc[0]);
	for (int i=0; i<sys_nerr; i++) {
		if (sys_errordesc[i].errcode == errcode) {
			return sys_errordesc[i].message;
		}
	}

	sprintf(strerror_buf, "Unknown error %d", errcode);
    return strerror_buf;
}


static void checkEmergencyStop()
{
	//(GPIOA->IDR & GPIO_PIN_15)
  	if ( !HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15) ) {
  		// prevent noise
        if ( !HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15) ) {
        	shutdown();
        	sys_errcode = EC_EMERGENCY_STOP;
    		GPIOB->BRR = (1<<0);  // enable red LED
        }
    }
}

void shutdown()
{
	gatedriverAC(0);
	gatedriverDC(0);
	contactorAC(0);
	contactorBattery(0);
}

void checkErrors()
{

	checkEmergencyStop();

	if (sys_errcode != 0 ) {
		shutdown();
	}
}
