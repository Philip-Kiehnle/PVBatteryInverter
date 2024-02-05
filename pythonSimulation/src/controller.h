/*
 * sogi_pllFXP.h
 *
 *  Created on: 23.07.2022
 *      Author: philip
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#define CAPACITANCE (4*390e-6)
#define R (0.7005)   // see calc sheet

//#define ADC_BITS 12
//int32_t get_ADC_BITS();

//#define VIN_ADCR (1.04 * 3.3 * 190/10)  // adc range in volts [0,62.7V]  30V real shows as 29.6V -> 1,0135; 25,2÷24,8=1,016
#define VIN_ADCR (6554)  // voltage range [0, +6553.6] 16bit in 100mV
int32_t get_VIN_ADCR();
#define ADC_BITS_VIN 16

//#define VIN_MIN 24.5  //25
//#define VIN_MIN_RAW ((1<<ADC_BITS)*VIN_MIN/VIN_ADCR)
//#define VIN_START 26.0  //32
//#define VIN_START_RAW ((1<<ADC_BITS)*VIN_START/VIN_ADCR)
//#define VDC_REF_100mV 25.0*10  // 23.4V with 33k
//#define VDC_REF_100mV 27.0*10  // 25.1V with 33k todo
//#define VDC_REF_100mV 30.0*10  // 28.0V with 33k
//#define VDC_REF_100mV 50.0*10  //

//#define VDC_REF_RAW ((1<<ADC_BITS_VIN)*VDC_REF/VIN_ADCR)
//#define VIN_MAX 35.0
//#define VIN_MAX_RAW ((1<<ADC_BITS)*VIN_MAX/VIN_ADCR)

//#define VGRID_TRATIO 14  // transformer winding ratio
#define VGRID_TRATIO 7  // transformer winding ratio
#define VGRID_AMP 325/VGRID_TRATIO  // todo
int32_t get_VGRID_TRATIO();
float get_C();

//#define VGRID_ADCR (1.056 * 3.3 * 325/0.67)  // signed; additional transformer winding scales 325V down  // same RMS
// 0.9308 input in AMC3330 equals 325V
//Vref_ADC=3.3V; differential mode
//(2x2.49)÷(2x3.3)×2^12=3090LSB
//-> (2×436.45V)÷3090 = 0.28V resolution
// -> ADC-Range (2×436.45V)÷3090*2^12 = 1157.1
#define VGRID_ADCR (1157.1)  // adc range in volts
int32_t get_VGRID_ADCR();
#define ADC_BITS_VGRID 12

// for trafo grid 15-30V
#define VD_MIN_RAW ((1<<ADC_BITS_VGRID)* 0.8 * VGRID_AMP/VGRID_ADCR)
#define VD_MAX_RAW ((1<<ADC_BITS_VGRID)* 1.2 * VGRID_AMP/VGRID_ADCR)

// for 230V
//#define VD_MIN_RAW ((1<<ADC_BITS_VGRID)* 0.9 * VGRID_AMP/VGRID_ADCR)
//#define VD_MAX_RAW ((1<<ADC_BITS_VGRID)* 1.1 * VGRID_AMP/VGRID_ADCR)


#define SCALE_F2WRAW  ((1<<15) -1)
#define F_MIN 49
#define W_MIN_RAW F_MIN*SCALE_F2WRAW
#define F_MAX 51
#define W_MAX_RAW F_MAX*SCALE_F2WRAW

#define IGRID_ADCR (655)  // current range [-327.68, +327.67] 16bit in 10mA
//#define IGRID_ADCR 30  // [-15,+15A]; no grid current sensor is used at the moment
int32_t get_IGRID_ADCR();
#define ADC_BITS_IGRID 12
//#define IAC_AMP_MAX 7  //(5 * sqrt(2))
#define IAC_AMP_MAX_10mA 6.0*100  // 6Aamp*45V/2=135W
//#define IAC_AMP_MAX_RAW ((1<<ADC_BITS_IGRID) * IAC_AMP_MAX/IGRID_ADCR)
//#define I_REF_AMP_MIN 0.4
//#define I_REF_AMP_MIN_RAW ((1<<ADC_BITS_IGRID) * I_REF_AMP_MIN/IGRID_ADCR)


typedef struct {
	int32_t y;
	int32_t y_min;
	int32_t y_max;
	int32_t x_prev;
	int32_t c1;  // (Ki*T/2 + Kp)
	int32_t c2;  // (Ki*T/2 - Kp)
} piController;


extern piController piCtrl;
void pi_step(int32_t x, piController *ctrl);

int16_t calc_IacAmp2VacSecAmpDCscale(int32_t i_amp);
int16_t get_IacPhase();
int32_t step_predict_i(int32_t i_ref, int32_t i);
int16_t step_pi_Vdc2IacAmp(int32_t vdc_ref, int32_t vdc);
int16_t step_pi_Vdc2IacAmp_volt_comp(int32_t vdc_ref, int32_t vdc, int16_t phase);
int16_t step_pi_Vdc2IacAmp_charge_comp(int32_t vdc_ref, int32_t vdc, int32_t vac_sec_VDCscale, int32_t i_ref_VDCscale);

int16_t sin1(int16_t angle);
int16_t cos1(int16_t angle);

#ifdef __cplusplus
}
#endif

#endif /* INC_CONTROLLER_H_ */
