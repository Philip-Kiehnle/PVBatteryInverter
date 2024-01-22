#ifndef INC_COMMON_H
#define INC_COMMON_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

enum mode_t {OFF, PV2AC, PV2BAT, HYBRID_OFFLINE, HYBRID_ONLINE};
extern volatile enum mode_t sys_mode;

enum stateDC_t {INIT_DC, WAIT_PV_VOLTAGE, VOLTAGE_CONTROL, WAIT_CONTACTOR_DC, MPPT};
enum stateAC_t {INIT_AC, WAIT_AC_DC_VOLTAGE, WAIT_ZERO_CROSSING, CLOSE_CONTACTOR_AC, WAIT_CONTACTOR_AC, GRID_CONNECTING, GRID_SYNC};

#define E_VDC_MAX_100mV 58*10   // worst case for battery if BMS fails: 58V / 15cells = 3.87V
#define E_VDC_MAX_MPPT_100mV 54*10  // 54V / 15cells = 3.6V
//#define E_VDC_MAX_FB_GRID_100mV 4000  // 400/96 = 4.16Vcell
#define E_VDC_MAX_FB_GRID_100mV 58*10  // todo


typedef struct {
	uint16_t ID;
	int16_t sys_errcode;
	uint16_t stateDC;
	uint16_t duty;
	float pdc_filt50Hz;
	float v_pv_filt50Hz;
	float v_dc_filt50Hz;
	uint16_t stateAC;
	int16_t f_ac_10mHz;
	int16_t v_ac_rms_100mV;
	int16_t v_amp_pred_100mV;
	int16_t i_ac_amp_10mA;
	uint16_t VdcFBgrid_sincfilt_100mV;
} __attribute__((__packed__)) monitor_vars_t;

extern volatile monitor_vars_t monitor_vars;

extern volatile enum stateDC_t stateDC;
extern volatile enum stateAC_t stateAC;

extern volatile int16_t debug_f_ac_10mHz;
extern volatile int16_t debug_v_ac_rms_100mV;
extern volatile int16_t debug_v_amp_pred_100mV;
extern volatile int16_t debug_i_ac_amp_10mA;

void send_monitor_vars();


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
	EC_GRID_SYNC_LOST = -8
};

typedef enum _errorcode errorPVBI_t;
char *strerror(int errcode);

extern volatile errorPVBI_t sys_errcode;

void shutdown();
void checkErrors();

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMON_H */
