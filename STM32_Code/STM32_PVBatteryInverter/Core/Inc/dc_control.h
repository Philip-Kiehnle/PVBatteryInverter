/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DC_CONTROL_H
#define __DC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"


//#define E_VDC_MAX_100mV 400*10  // 400/96 = 4.16Vcell
//#define VDC_BOOST_START_100mV 240*10  // 8*30V minimum voltage of DC bus to start DC booster stage
//#define VDC_BOOST_STOP_100mV 200*10  // 8*25V  minimum voltage of DC bus to keep booster stage running


#define VDC_BOOST_START_100mV 32*10
#define VDC_BOOST_STOP_100mV 25*10

#define VDC_TOLERANCE_100mV 2*10  // 2V tolerance for connecting battery

enum dcdc_mode_t {INACTIVE, DCDC_HB1, DCDC_HB2, DCDC_INTERLEAVED};

extern volatile enum dcdc_mode_t dcdc_mode;

extern volatile uint16_t VdcFBboost_sincfilt_100mV;
extern volatile int16_t Idc_filt_10mA;
extern volatile bool sys_mode_needs_battery;

void shutdownDC(bool include_bms_bus_cmd);
void calc_async_dc_control(bool bus_comm_allowed);
void fill_monitor_vars_dc(monitor_vars_t* mon_vars);
void measVdcFBboost();
errorPVBI_t checkDCLimits();
int16_t dcControlStep(uint16_t cnt50Hz, uint16_t vdc_filt50Hz_100mV);



#ifdef __cplusplus
}
#endif

#endif /* __DC_CONTROL_H */
