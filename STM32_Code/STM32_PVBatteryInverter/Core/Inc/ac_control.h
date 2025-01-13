/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AC_CONTROL_H
#define __AC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "common.h"
#include "controller.h"
#include "monitoring.h"

//#define VGRID_AMP (325)
//#define VGRID_AMP_MIN_100mV (VGRID_AMP_100mV*10*0.9)
//#define VGRID_AMP_MAX_100mV (VGRID_AMP_100mV*10*1.1)
//
//#define VD_MIN_RAW ((1<<ADC_BITS)* 0.9 * VGRID_AMP/VGRID_ADCR)
//#define VD_MAX_RAW ((1<<ADC_BITS)* 1.15 * VGRID_AMP/VGRID_ADCR)

//#define FGRID 50
//#define FGRID_MIN_mHz (49*1000)
//#define FGRID_MAX_mHz (51*1000)

extern volatile uint16_t debug_v_dc_FBgrid_sincfilt_100mV;

void measVdcFBgrid();
const int16_t get_p_ac_filt50Hz();
const int16_t get_p_ac_filt1minute();
errorPVBI_t checkACLimits();
void calc_p_ac(control_ref_t* ctrl_ref);
int16_t acControlStep(uint16_t cnt20kHz_20ms, control_ref_t ctrl_ref, uint16_t v_dc_FBboost_sincfilt_100mV, uint16_t v_dc_FBboost_filt50Hz_100mV, int16_t v_ac_raw, uint16_t i_ac_raw);
void fill_monitor_vars_ac(monitor_vars_t* mon_vars);

inline int acControl_RAW_to_100mV(int v_ac_raw)
{
	return (v_ac_raw * (10*VGRID_ADCR) )/(1<<ADC_BITS_VGRID);
}


#ifdef __cplusplus
}
#endif

#endif /* __AC_CONTROL_H */
