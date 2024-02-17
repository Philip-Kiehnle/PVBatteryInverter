#ifndef INC_COMMON_H
#define INC_COMMON_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define E_VDC_MAX_100mV 58*10   // worst case for battery if BMS fails: 58V / 15cells = 3.87V
#define E_VDC_MAX_MPPT_100mV 54*10  // 54V / 15cells = 3.6V
//#define E_VDC_MAX_FB_GRID_100mV 4000  // 400/96 = 4.16Vcell
#define E_VDC_MAX_FB_GRID_100mV 58*10  // todo

//#define E_IAC_MAX_10mA (7.3 * 100)  // 1200W÷230V×sqrt(2) amplitude
//#define E_IAC_MAX_10mA (10 * 100)  // 1625W÷230V×sqrt(2) amplitude
#define E_IAC_MAX_10mA (12.3 * 100)  // 2000W÷230V×sqrt(2) amplitude

#define P_AC_MIN 0 // feed into grid only -> no AC2BAT for now
#define P_AC_MAX 180  // todo implement anti windup in power controller regarding IAC_AMP_MAX_10mA

#define DEF_MPPT_DUTY_ABSMAX 4250

#define DC_CTRL_FREQ 20000
#define DC_CTRL_FREQ_MPPT 50
#define CYCLES_CNT_50HZ (DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT)

#define AC_CTRL_FREQ DC_CTRL_FREQ

#define COMM_READ_ELECTRICITY_METER 1  // listen for smart meter data and send inverterdata after reception
#define SYSTEM_HAS_BATTERY 1

enum mode_t {OFF, PV2AC, PV2BAT, HYBRID_OFFLINE, HYBRID_ONLINE};
extern volatile enum mode_t sys_mode;

enum stateDC_t {INIT_DC, WAIT_PV_VOLTAGE, VOLTAGE_CONTROL, WAIT_CONTACTOR_DC, MPPT};
enum stateAC_t {INIT_AC, WAIT_AC_DC_VOLTAGE, WAIT_ZERO_CROSSING, CLOSE_CONTACTOR_AC, WAIT_CONTACTOR_AC, GRID_CONNECTING, GRID_SYNC};

typedef enum
{
	AC_OFF = 0,
	FFWD_ONLY = 1,
	VDC_CONTROL = 2,
	VDC_VARIABLE_CONTROL = 3,  // keeps DC voltage just above AC amplitude
	PAC_CONTROL = 4
} ac_ctrl_mode_t;

typedef struct {
	ac_ctrl_mode_t mode;
	uint16_t v_dc_100mV;
	float p_ac_rms;
} control_ref_t;

extern volatile enum stateDC_t stateDC;
extern volatile enum stateAC_t stateAC;


enum _errorcode
{
	EC_NO_ERROR = 0,
	EC_EMERGENCY_STOP = -1,
	EC_V_DC_MAX_FB_BOOST = -2,
	EC_V_DC_MAX_FB_GRID = -3,
	EC_V_DC_SENSOR_FB_BOOST = -4,
	EC_V_DC_SENSOR_FB_GRID = -5,
	EC_I_DC_MAX = -6,
	EC_I_AC_MAX = -7,
	EC_GRID_SYNC_LOST = -8,
	EC_WATCHDOG_RESET = -9
};

typedef enum _errorcode errorPVBI_t;
char *strerror(int errcode);

extern volatile errorPVBI_t sys_errcode;

void shutdown();
void checkErrors();

uint16_t lowpass4(uint16_t in, uint16_t* prev);

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMON_H */
