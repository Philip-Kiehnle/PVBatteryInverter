/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AC_CONTROL_H
#define __AC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "common.h"
#include "controller.h"


#define E_IAC_MAX_10mA (7.3 * 100)  // 1200W÷230V×sqrt(2) amplitude

//#define VGRID_AMP (325)
//#define VGRID_AMP_MIN_100mV (VGRID_AMP_100mV*10*0.9)
//#define VGRID_AMP_MAX_100mV (VGRID_AMP_100mV*10*1.1)
//
//#define VD_MIN_RAW ((1<<ADC_BITS)* 0.9 * VGRID_AMP/VGRID_ADCR)
//#define VD_MAX_RAW ((1<<ADC_BITS)* 1.15 * VGRID_AMP/VGRID_ADCR)

//#define FGRID 50
//#define FGRID_MIN_mHz (49*1000)
//#define FGRID_MAX_mHz (51*1000)


extern volatile enum stateAC_t stateAC;
extern volatile uint16_t VdcFBgrid_sincfilt_100mV;
extern volatile int16_t iac_10mA;
extern volatile bool sys_mode_needs_ACside;

void measVdcFBgrid();
errorPVBI_t checkACLimits();
int16_t acControlStep(int16_t vac_raw, int16_t iac_raw, uint16_t vdc_sinc_mix_100mV, uint16_t v_dc_ref_100mV, float p_ac_rms_ref);

inline int acControl_RAW_to_100mV(int vac_raw)
{
	return (vac_raw * (10*VGRID_ADCR) )/(1<<ADC_BITS_VGRID);
}


#ifdef __cplusplus
}
#endif

#endif /* __AC_CONTROL_H */
