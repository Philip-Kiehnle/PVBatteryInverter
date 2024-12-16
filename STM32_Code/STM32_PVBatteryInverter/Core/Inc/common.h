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

#define E_I_AC_RMS_MAX_CNT 10  // 10 samples @ 20kHz -> 500us
#define E_I_AC_RMS_MAX_10mA (12.3 * 100)  // 2000W÷230V×sqrt(2) amplitude for E_I_AC_RMS_MAX_CNT samples
#define E_I_AC_PULSE_MAX_10mA (16.0 * 100)  // 2600W÷230V×sqrt(2) amplitude for 1 sample

#define PERMIL_V_DFFW_MIN  100  // 10% grid voltage direct feedforward, 90% PLL
#define PERMIL_V_DFFW_MAX  700  // 70% maximum in case of high current due to distorted grid
#define PERMIL_V_DFFW_INCR 60  // 60 equals 6% per control cycle -> 60% in 500µs
#define PERMIL_V_DFFW_DECR 1  // 1 equals 0.1% per control cycle; @70% direct feedforward, it takes 600*50µs=30ms to come back to 10%
// always 50% has higher inductor sound
// always 80% has even higher inductor sound

#define E_I_AC_DC_OFFSET_MAX_10mA (0.8 * 100)  // 800mA in E_I_AC_DC_OFFSET_CYCLES consecutive 50Hz periods  todo decrease
#define E_I_AC_DC_OFFSET_CYCLES 8  // number of consecutive 50Hz periods for DC current fault

#define P_AC_MIN 0 // feed into grid only -> no AC2BAT for now
//#define P_AC_MAX ((5*32)/1.41) //113W  // todo implement anti windup in power controller regarding IAC_AMP_MAX_10mA
#define P_AC_MAX (210)  // 210W minimum for energy packet controller
#define P_BAT_MIN_CHARGE 2
#define P_MIN_PV2AC 10

#define DEF_MPPT_DUTY_ABSMAX 4250

#define DC_CTRL_FREQ 20000
#define DC_CTRL_FREQ_MPPT 50
#define CYCLES_cnt20kHz_20ms (DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT)

#define AC_CTRL_FREQ DC_CTRL_FREQ

#define V_TO_100mV(x)  (10*x)


enum mode_t {OFF, PV2AC, PV2BAT, HYBRID_PCC_SENSOR, HYBRID_REMOTE_CONTROLLED};
enum stateHYBRID_AC_t {HYB_AC_OFF, HYB_AC_ON};

enum stateDC_t {INIT_DC, WAIT_PV_VOLTAGE, VOLTAGE_CONTROL, WAIT_CONTACTOR_DC, MPPT};
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

typedef struct {
	ac_ctrl_mode_t mode;
	uint16_t v_dc_100mV;
	int16_t p_pcc;  // power at point of common coupling (electricity meter)
	int16_t p_pcc_prev;
	int16_t p_ac_pccCtrl;  // power from PCC controller
	ext_ctrl_mode_t ext_ctrl_mode;
	int16_t p_ac_external;
	int16_t p_ac_rms;  // power for AC control
} control_ref_t;

typedef enum
{
	EC_NO_ERROR                  = 0,
	EC_EMERGENCY_STOP            = -1,
	EC_V_DC_MAX_FB_BOOST         = -2,
	EC_V_DC_MAX_FB_GRID          = -3,
	EC_V_DC_SENSOR_FB_BOOST      = -4,
	EC_V_DC_SENSOR_FB_GRID       = -5,
	EC_I_DC_MAX                  = -6,
	EC_I_AC_PULSE_MAX            = -7,
	EC_I_AC_RMS_MAX              = -8,
	EC_I_AC_DC_OFFSET            = -9,
	EC_GRID_SYNC_LOST            = -10,
	EC_WATCHDOG_RESET            = -11,
	EC_BATTERY_COMM_FAIL         = -12,
	EC_BATTERY_V_CELL_MIN        = -13,
	EC_BATTERY_V_CELL_MAX        = -14,
	EC_BATTERY_V_CELL_IMBALANCE  = -15,
	EC_BATTERY_I_CHARGE_MAX      = -16,
	EC_BATTERY_I_DISCHARGE_MAX   = -17,
	EC_BATTERY_TEMPERATURE_MIN   = -18,
	EC_BATTERY_TEMPERATURE_MAX   = -19,
	EC_BATTERY_OTHER             = -20,
	EC_V_AC_LOW                  = -21,
	EC_V_AC_HIGH                 = -22,
	EC_FREQ_AC_LOW               = -23,
	EC_FREQ_AC_HIGH              = -24,
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
