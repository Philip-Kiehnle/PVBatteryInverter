
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
	{ HYBRID_REMOTE_CONTROLLED, "HYBRID_REMOTE_CONTROLLED : Externally controlled Pac. todo: power distribution, regarding limits: Pbat_charge=1.6kW (0.25C), Pac=1.5kW."}
};


static char strerror_buf[1024];
volatile errorPVBI_t sys_errcode;

struct _sys_errordesc {
    int  errcode;
    char *message;
} sys_errordesc[] = {
	{ EC_NO_ERROR,                  "EC_NO_ERROR : No error" },
	{ EC_EMERGENCY_STOP,            "EC_EMERGENCY_STOP : Emergency button pressed" },
	{ EC_WATCHDOG_RESET,            "EC_WATCHDOG_RESET : Watchdog counter caused reset" },
	{ EC_V_BUS_V_BATTERY_DEVIATION, "EC_V_BUS_V_BATTERY_DEVIATION : Battery voltage deviates from bus voltage" },
	{ EC_V_DC_MAX_BUS,              "EC_V_DC_MAX_BUS : DC voltage too high (DC bus sensor)" },
	{ EC_I_DC_SENSOR,               "EC_I_DC_SENSOR : DC current sensor 60A overcurrent fault. 5V reset required." },
	{ EC_I_AC_SENSOR,               "EC_I_AC_SENSOR : AC current sensor 60A overcurrent fault. 5V reset required." },
	{ EC_V_SUPPLY_5V_LOW,           "EC_V_SUPPLY_5V_LOW : 5V supply voltage dropped. Prevented powerstage from low gate voltages." },

	{ EC_V_DC_MAX_FB_BOOST,         "EC_V_DC_MAX_FB_BOOST : DC voltage too high (Full-bridge PV boost)" },
	{ EC_V_DC_SENSOR_FB_BOOST,      "EC_V_DC_SENSOR_FB_BOOST : Vdc sigma delta pulse count invalid (Full-bridge PV boost)"},
	{ EC_TEMPERATURE_FB_BOOST,      "EC_TEMPERATURE_FB_BOOST : Temperature too high (Full-bridge PV boost)"},

	{ EC_V_DC_MAX_FB_GRID,          "EC_V_DC_MAX_FB_GRID : DC voltage too high (Full-bridge AC-grid)" },
	{ EC_V_DC_SENSOR_FB_GRID,       "EC_V_DC_SENSOR_FB_GRID : Vdc sigma delta pulse count invalid (Full-bridge AC-grid)"},
	{ EC_TEMPERATURE_FB_GRID,       "EC_TEMPERATURE_FB_GRID : Temperature too high (Full-bridge AC-grid)"},

	{ EC_GRID_SYNC_LOST,            "EC_GRID_SYNC_LOST : Grid parameters out of range while connected to grid"},
	{ EC_V_AC_LOW,                  "EC_V_AC_LOW : AC voltage too low "},
	{ EC_V_AC_HIGH,                 "EC_V_AC_HIGH : AC voltage too high "},
	{ EC_FREQ_AC_LOW,               "EC_FREQ_AC_LOW : AC frequency too low "},
	{ EC_FREQ_AC_HIGH,              "EC_FREQ_AC_HIGH : AC frequency too high "},
	{ EC_I_AC_RMS_MAX,              "EC_I_AC_RMS_MAX : AC current exceeded rms max in multiple control periods" },
	{ EC_I_AC_PULSE_MAX,            "EC_I_AC_PULSE_MAX : AC current exceeded pulse max in one control period" },
	{ EC_I_AC_DC_OFFSET,            "EC_I_AC_DC_OFFSET : DC offset in AC current too high" },

	{ EC_BATTERY_COMM_FAIL,             "EC_BATTERY_COMM_FAIL : No battery communication for 60 sec" },
	{ EC_BATTERY_V_CELL_MIN,            "EC_BATTERY_V_CELL_MIN : Battery cell voltage below min voltage" },
	{ EC_BATTERY_V_CELL_MAX,            "EC_BATTERY_V_CELL_MAX : Battery cell voltage above max voltage" },
	{ EC_BATTERY_V_CELL_IMBALANCE,      "EC_BATTERY_V_CELL_IMBALANCE : Min and max battery cell voltage difference too high" },
	{ EC_BATTERY_I_CHARGE_RMS_MAX,      "EC_BATTERY_I_CHARGE_RMS_MAX : Max rms charge current exceeded" },
	{ EC_BATTERY_I_CHARGE_PULSE_MAX,    "EC_BATTERY_I_CHARGE_PULSE_MAX : Max pulse charge current exceeded" },
	{ EC_BATTERY_I_DISCHARGE_RMS_MAX,   "EC_BATTERY_I_DISCHARGE_RMS_MAX : Max rms discharge current exceeded" },
	{ EC_BATTERY_I_DISCHARGE_PULSE_MAX, "EC_BATTERY_I_DISCHARGE_PULSE_MAX : Max pulse discharge current exceeded" },
	{ EC_BATTERY_TEMPERATURE_MIN,       "EC_BATTERY_TEMPERATURE_MIN : Battery temperature below minimum" },
	{ EC_BATTERY_TEMPERATURE_MAX,       "EC_BATTERY_TEMPERATURE_MAX : Battery temperature above maximum" },
	{ EC_BATTERY_OTHER,                 "EC_BATTERY_OTHER : Other BMS error, see BMS registers" },
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
	if ( !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) ) {
		// prevent noise
		if ( !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) ) {
			shutdownAll();
			sys_errcode = EC_EMERGENCY_STOP;  // overwrites existing errors!
		}
	}

#if IS_BATTERY_SUPERVISOR_PCB == 1
	// Second parallel emergency pin because of slightly bended microcontroller pin A15.
	//(GPIOC->IDR & GPIO_PIN_9)
	if ( !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) ) {
		// prevent noise
		if ( !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) ) {
			shutdownAll();
			sys_errcode = EC_EMERGENCY_STOP;  // overwrites existing errors!
		}
	}
#endif  // IS_BATTERY_SUPERVISOR_PCB == 1
}


static void checkCurrentSensors()
{
	if ( !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) ) {
		shutdownAll();
		sys_errcode = EC_I_DC_SENSOR;  // overwrites existing errors!
	}

#if IS_BATTERY_SUPERVISOR_PCB != 1
	if ( !HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) ) {
		shutdownAll();
		sys_errcode = EC_I_AC_SENSOR;  // overwrites existing errors!
	}
#endif  // IS_BATTERY_SUPERVISOR_PCB != 1
}


void checkTemperatureSensors()
{
#if IS_BATTERY_SUPERVISOR_PCB != 1  // Supervisor has no temperature sensors
	if ( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) ) {
		shutdownAll();
		if (sys_errcode == EC_NO_ERROR) {
			sys_errcode = EC_TEMPERATURE_FB_BOOST;
		}
	}

	if ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) ) {
		shutdownAll();
		if (sys_errcode == EC_NO_ERROR) {
			sys_errcode = EC_TEMPERATURE_FB_GRID;
		}
	}
#endif  // IS_BATTERY_SUPERVISOR_PCB != 1
}


void checkErrors()
{
	checkEmergencyStop();
	checkCurrentSensors();
	checkTemperatureSensors();

	if (sys_errcode != EC_NO_ERROR) {
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
