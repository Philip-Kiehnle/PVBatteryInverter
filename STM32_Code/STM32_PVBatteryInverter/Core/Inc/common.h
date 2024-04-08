#ifndef INC_COMMON_H
#define INC_COMMON_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define SYS_MODE HYBRID_PCC_SENSOR
//#define SYS_MODE PV2AC

#define COMM_READ_ELECTRICITY_METER 1  // listen for smart meter data and send inverterdata after reception
#define SYSTEM_HAS_BATTERY 1

#define E_VDC_MAX_100mV 58*10   // worst case for battery if BMS fails: 58V / 15cells = 3.87V
#define E_VDC_MAX_MPPT_100mV 54*10  // 54V / 15cells = 3.6V
//#define E_VDC_MAX_FB_GRID_100mV 4000  // 400/96 = 4.16Vcell
#define E_VDC_MAX_FB_GRID_100mV 58*10  // todo

//#define E_IAC_MAX_10mA (7.3 * 100)  // 1200W÷230V×sqrt(2) amplitude
//#define E_IAC_MAX_10mA (10 * 100)  // 1625W÷230V×sqrt(2) amplitude
#define E_I_AC_MAX_10mA (12.3 * 100)  // 2000W÷230V×sqrt(2) amplitude
#define E_I_AC_DC_OFFSET_MAX_10mA (0.8 * 100)  // 800mA in E_I_AC_DC_OFFSET_CYCLES consecutive 50Hz periods  todo decrease
#define E_I_AC_DC_OFFSET_CYCLES 8  // number of consecutive 50Hz periods for DC current fault

#define P_AC_MIN 0 // feed into grid only -> no AC2BAT for now
//#define P_AC_MAX ((5*32)/1.41) //113W  // todo implement anti windup in power controller regarding IAC_AMP_MAX_10mA
#define P_AC_MAX (210)  // 210W minimum for energy packet controller
#define P_BAT_MIN_CHARGE 2

#define DEF_MPPT_DUTY_ABSMAX 4250

#define DC_CTRL_FREQ 20000
#define DC_CTRL_FREQ_MPPT 50
#define CYCLES_cnt20kHz_20ms (DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT)

#define AC_CTRL_FREQ DC_CTRL_FREQ

#define V_TO_100mV(x)  (10*x)


enum mode_t {OFF, PV2AC, PV2BAT, HYBRID_PCC_SENSOR, HYBRID_REMOTE_CONTROLLED};
enum stateHYBRID_AC_t {HYB_AC_OFF, HYB_AC_ALLOWED, HYB_AC_ON};

enum stateDC_t {INIT_DC, WAIT_PV_VOLTAGE, VOLTAGE_CONTROL, WAIT_CONTACTOR_DC, MPPT};
enum stateAC_t {INIT_AC, WAIT_AC_DC_VOLTAGE, WAIT_ZERO_CROSSING, CLOSE_CONTACTOR_AC, WAIT_CONTACTOR_AC, GRID_CONNECTING, GRID_SYNC};

typedef enum
{
	AC_OFF,
	AC_PASSIVE,
	FFWD_ONLY,
	VDC_CONTROL,
	VDC_VARIABLE_CONTROL,  // keeps DC voltage just above AC amplitude
	PAC_CONTROL_PCC,  // control point of common coupling to zero
	PAC_CONTROL_V_P_BAT_CONST  // keeps max battery charge power or Vcell below protect limit
} ac_ctrl_mode_t;

typedef struct {
	ac_ctrl_mode_t mode;
	uint16_t v_dc_100mV;
	int16_t p_pcc;  // power at point of common coupling (electricity meter)
	int16_t p_pcc_prev;
	int16_t p_ac_rms_pccCtrl;  // power from PCC controller
	int16_t p_ac_rms;  // power for AC control
} control_ref_t;

typedef enum
{
	EC_NO_ERROR = 0,
	EC_EMERGENCY_STOP = -1,
	EC_V_DC_MAX_FB_BOOST = -2,
	EC_V_DC_MAX_FB_GRID = -3,
	EC_V_DC_SENSOR_FB_BOOST = -4,
	EC_V_DC_SENSOR_FB_GRID = -5,
	EC_I_DC_MAX = -6,
	EC_I_AC_MAX = -7,
	EC_I_AC_DC_OFFSET = -8,
	EC_GRID_SYNC_LOST = -9,
	EC_WATCHDOG_RESET = -10,
	EC_BATTERY_COMM_FAIL = -11
} errorPVBI_t;


extern volatile enum stateDC_t stateDC;
extern volatile enum stateAC_t stateAC;

extern void shutdownAll();
extern void Error_Handler();

char *strerror(int errcode);
void set_sys_errorcode(errorPVBI_t err);
errorPVBI_t get_sys_errorcode();

void checkErrors();

uint16_t lowpass4(uint16_t in, uint16_t* prev);

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMON_H */
