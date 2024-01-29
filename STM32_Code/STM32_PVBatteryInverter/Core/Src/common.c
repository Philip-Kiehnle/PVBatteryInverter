
#include "stm32g4xx_hal.h"

#include <stdio.h>

#include "common.h"
#include "gpio.h"


struct _sys_modedesc {
    int  sys_mode;
    char *message;
} sys_modedesc[] = {
    { OFF, "OFF : System inactive." },
    { PV2AC, "PV2AC (default): PV power completely fed into the AC grid." },
	{ PV2BAT, "PV2BAT : PV power completely fed into the battery." },
	{ HYBRID_OFFLINE, "HYBRID_OFFLINE : PCC controlled to zero. Without PCC sensor: PV power 50% into battery and 50% into grid. 100W into AC grid if no PV power."},
	{ HYBRID_ONLINE, "HYBRID_ONLINE : Externally controlled power distribution, regarding limits: Pbat_charge=1.6kW (0.25C), Pac=1.5kW."}
};

volatile enum mode_t sys_mode;

static char strerror_buf[1024];
volatile errorPVBI_t sys_errcode;

struct _sys_errordesc {
    int  errcode;
    char *message;
} sys_errordesc[] = {
    { EC_NO_ERROR, "EC_NO_ERROR : No error" },
    { EC_EMERGENCY_STOP, "EC_EMERGENCY_STOP : Emergency button pressed" },
    { EC_V_DC_MAX_FB_BOOST, "EC_V_DC_MAX_FB_BOOST : DC voltage too high (Full-bridge PV boost)" },
	{ EC_V_DC_MAX_FB_GRID, "EC_V_DC_MAX_FB_GRID : DC voltage too high (Full-bridge AC-grid)" },
	{ EC_V_DC_SENSOR_FB_BOOST, "EC_V_DC_SENSOR_FB_BOOST : Vdc sigma delta pulse count invalid (Full-bridge PV boost)"},
	{ EC_V_DC_SENSOR_FB_GRID, "EC_V_DC_SENSOR_FB_GRID : Vdc sigma delta pulse count invalid (Full-bridge AC-grid)"},
	{ EC_I_DC_MAX, "EC_I_DC_MAX : DC current too high" },
	{ EC_I_AC_MAX, "EC_I_AC_MAX : AC current too high" },
	{ EC_GRID_SYNC_LOST, "EC_GRID_SYNC_LOST : grid parameters out of range while connected to grid"}
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
        	shutdownALL();
        	sys_errcode = EC_EMERGENCY_STOP;
        }
    }
}


void checkErrors()
{

	checkEmergencyStop();

	if (sys_errcode != 0 ) {
		shutdownALL();
		GPIOB->BRR = (1<<0);  // enable red LED
	}
}
