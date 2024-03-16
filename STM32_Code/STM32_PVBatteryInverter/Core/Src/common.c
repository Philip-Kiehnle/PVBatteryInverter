
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
	{ HYBRID_PCC_SENSOR, "HYBRID_PCC_SENSOR : PCC controlled to zero. Without PCC sensor: PV power 50% into battery and 50% into grid. 100W into AC grid if no PV power."},
	{ HYBRID_REMOTE_CONTROLLED, "HYBRID_REMOTE_CONTROLLED : Externally controlled power distribution, regarding limits: Pbat_charge=1.6kW (0.25C), Pac=1.5kW."}
};


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
	{ EC_I_AC_DC_OFFSET, "EC_I_AC_DC_OFFSET : DC offset in AC current too high" },
	{ EC_GRID_SYNC_LOST, "EC_GRID_SYNC_LOST : grid parameters out of range while connected to grid"},
	{ EC_WATCHDOG_RESET, "EC_WATCHDOG_RESET : watchdog counter caused reset" },
	{ EC_BATTERY_COMM_FAIL, "EC_BATTERY_COMM_FAIL : no battery communication for 60 sec" },

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


void set_sys_errorcode(errorPVBI_t err)
{
	if (   sys_errcode == EC_NO_ERROR  // prevent override of previous error
	    && err != EC_NO_ERROR) {  // prevent reset of error. Error needs hardware reset.
		sys_errcode = err;
	}
}


errorPVBI_t get_sys_errorcode()
{
	return sys_errcode;
}


static void checkEmergencyStop()
{
	//(GPIOA->IDR & GPIO_PIN_15)
  	if ( !HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15) ) {
  		// prevent noise
        if ( !HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_15) ) {
        	shutdownAll();
        	sys_errcode = EC_EMERGENCY_STOP;
        }
    }
}


void checkErrors()
{

	checkEmergencyStop();

	if (sys_errcode != 0 ) {
		shutdownAll();
		GPIOB->BRR = (1<<0);  // enable red LED
	}
}

uint16_t lowpass4(uint16_t in, uint16_t* prev)
{
    uint16_t filt = ((uint32_t)in + prev[0] + prev[1] + prev[2]) >> 2;
    prev[2] = prev[1];
    prev[1] = prev[0];
    prev[0] = in;
    return filt;
}
