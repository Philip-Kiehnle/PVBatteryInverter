/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DC_CONTROL_H
#define __DC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "monitoring.h"


//#define E_VDC_MAX_100mV 400*10  // 400/96 = 4.16Vcell
//#define VDC_BOOST_START_100mV 240*10  // 8*30V minimum voltage of DC bus to start DC booster stage
//#define VDC_BOOST_STOP_100mV 200*10  // 8*25V  minimum voltage of DC bus to keep booster stage running

// for debugging
//#define VDC_BOOST_START_100mV 32*10
//#define VDC_BOOST_STOP_100mV 25*10

// for autonomous operation
#define VDC_BOOST_START_100mV 35.4*10  // PV-Panel: Voc(STC)=46.30V Voc(NOCT)=43.70V; morning Voc_dcbus=35.7V; 28°C cloudy Voc_dcbus=35.5V Ppv=22W
#define VDC_BOOST_STOP_100mV 26*10

#define VDC_TOLERANCE_100mV 2*10  // 2V tolerance for connecting battery

enum dcdc_mode_t {INACTIVE, DCDC_HB1, DCDC_HB2, DCDC_INTERLEAVED};

extern volatile enum dcdc_mode_t dcdc_mode;

extern volatile uint16_t debug_v_dc_FBboost_sincfilt_100mV;
extern volatile bool sys_mode_needs_battery;

uint16_t get_p_ac_bat_chg_reduction();
uint16_t get_p_ac_max_dc_lim();
uint16_t get_v_dc_FBboost_sincfilt_100mV();
uint16_t get_v_dc_FBboost_filt50Hz_100mV();
float get_p_dc_filt50Hz();
void shutdownDC();
void calc_async_dc_control();
void fill_monitor_vars_dc(monitor_vars_t* mon_vars);
void measVdcFBboost();
errorPVBI_t checkDCLimits();
int16_t dcControlStep(uint16_t cnt20kHz_20ms, uint16_t v_dc_ref_100mV, int16_t i_dc_filt_10mA);


#ifdef __cplusplus
}
#endif

#endif /* __DC_CONTROL_H */
