#ifndef INC_COMMON_H
#define INC_COMMON_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include <config.h>


#define V_TO_100mV(x)  (10*x)

enum mode_t {OFF, PV2AC, PV2BAT, HYBRID_PCC_SENSOR, HYBRID_REMOTE_CONTROLLED, BAT_SUPERVISOR_MODE};
enum stateHYBRID_AC_t {HYB_AC_OFF, HYB_AC_ON};

#if IS_BATTERY_SUPERVISOR_PCB == 1
enum stateDC_t {INIT_DC, WAIT_BUS_VOLTAGE, WAIT_BATTERY_STATUS, BAT_CONNECTED, CONTACTOR_PROTECTION};
#else
enum stateDC_t {INIT_DC, WAIT_PV_VOLTAGE, VOLTAGE_CONTROL, WAIT_CONTACTOR_DC, MPPT};
#endif // IS_BATTERY_SUPERVISOR_PCB == 1

enum stateAC_t {INIT_AC, WAIT_AC_DC_VOLTAGE, WAIT_ZERO_CROSSING, CLOSE_CONTACTOR_AC, WAIT_CONTACTOR_AC, GRID_CONNECTING, GRID_SYNC};

typedef enum
{
	AC_OFF,
	AC_PASSIVE,
	FFWD_ONLY,
	VDC_CONTROL,
	VDC_VARIABLE_CONTROL,  // keeps DC voltage just above AC amplitude
	PAC_CONTROL,  // controls point of common coupling to zero or external power cmd or keeps max battery charge power or Vcell below protect limit
} ac_ctrl_mode_t;

typedef enum
{
	EXT_OFF,
	EXT_AC_SOFT,  // e.g. GRID2AC is ignored if Ppcc consumes power from grid
	EXT_AC_HARD,
} ext_ctrl_mode_t;

typedef enum
{
	EXT_LOCK_INACTIVE,
	EXT_LOCK_SOFT,
	EXT_LOCK_HARD,
} ext_lock_t;

typedef struct {
	ac_ctrl_mode_t mode;
	uint16_t v_dc_100mV;
	int16_t p_pcc;  // power at point of common coupling (electricity meter)
	int16_t p_pcc_prev;
	int16_t p_ac_pccCtrl;  // power from PCC controller
	ext_ctrl_mode_t ext_ctrl_mode;
	ext_lock_t ext_ac_lock;
	ext_lock_t ext_dc_lock;
	int16_t p_ac_external;
	int16_t p_ac_rms;  // power for AC control
	uint16_t p_ac_low_power_mode_enter;
	uint16_t p_ac_low_power_mode_exit;
} control_ref_t;

typedef enum
{
	EC_NO_ERROR                  = 0,
	EC_EMERGENCY_STOP            = -1,
	EC_WATCHDOG_RESET            = -2,
	EC_V_BUS_V_BATTERY_DEVIATION = -3,
	EC_V_DC_MAX_BUS              = -4,
	EC_I_DC_SENSOR               = -5,
	EC_I_AC_SENSOR               = -6,
	EC_V_SUPPLY_5V_LOW           = -7,

	EC_V_DC_MAX_FB_BOOST         = -10,
	EC_V_DC_SENSOR_FB_BOOST      = -11,
	EC_TEMPERATURE_FB_BOOST      = -12,

	EC_V_DC_MAX_FB_GRID          = -20,
	EC_V_DC_SENSOR_FB_GRID       = -21,
	EC_TEMPERATURE_FB_GRID       = -22,

	EC_GRID_SYNC_LOST            = -50,
	EC_V_AC_LOW                  = -51,
	EC_V_AC_HIGH                 = -52,
	EC_FREQ_AC_LOW               = -53,
	EC_FREQ_AC_HIGH              = -54,
	EC_I_AC_RMS_MAX              = -55,
	EC_I_AC_PULSE_MAX            = -56,
	EC_I_AC_DC_OFFSET            = -57,

	EC_BATTERY_COMM_FAIL             = -100,
	EC_BATTERY_V_CELL_MIN            = -101,
	EC_BATTERY_V_CELL_MAX            = -102,
	EC_BATTERY_V_CELL_IMBALANCE      = -103,
	EC_BATTERY_I_CHARGE_RMS_MAX      = -104,
	EC_BATTERY_I_CHARGE_PULSE_MAX    = -105,
	EC_BATTERY_I_DISCHARGE_RMS_MAX   = -106,
	EC_BATTERY_I_DISCHARGE_PULSE_MAX = -107,
	EC_BATTERY_TEMPERATURE_CELL_MIN  = -108,
	EC_BATTERY_TEMPERATURE_CELL_MAX  = -109,
	EC_BATTERY_TEMPERATURE_PCB_MAX   = -110,
	EC_BATTERY_OTHER                 = -120,
} errorPVBI_t;


extern volatile enum stateDC_t stateDC;
extern volatile enum stateAC_t stateAC;

extern void shutdownAll();
extern void Error_Handler();

char *strerror(int errcode);
void set_sys_errorcode(errorPVBI_t err);
errorPVBI_t get_sys_errorcode();

void checkTemperatureSensors();
void checkErrors();

uint16_t lowpass4(uint16_t in, uint16_t* prev);

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMON_H */
