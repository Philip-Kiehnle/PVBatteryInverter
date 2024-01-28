
#include "stm32g4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <algorithm>

#include "common.h"
#include "gpio.h"
//#include "dc_control.h"
#include "ac_control.h"

#include "controller.h"
#include "sogi_pllFXP.h"

// single PWM step has 1/170MHz = 5.88ns
// deadtime is configured to 64ns (10k resistor)
//#define MIN_PULSE 18  // min pulse duration is 106ns - 64ns deadtime = 42ns

#define AC_CTRL_FREQ 20000

#define AC_DUTY_ABSMAX 8500
#define AC_DUTY_HALF (AC_DUTY_ABSMAX/2)


volatile uint16_t VdcFBgrid_sincfilt_100mV;
volatile int16_t iac_10mA;

volatile int16_t debug_v_ac_rms_100mV;
volatile int16_t debug_f_ac_10mHz;
volatile int16_t debug_v_amp_pred_100mV;
volatile int16_t debug_i_ac_amp_10mA;


volatile enum stateAC_t stateAC = INIT_AC;

volatile bool sys_mode_needs_ACside = false;

static uint32_t cnt_rel = 0;


void shutdownAC()
{
	gatedriverAC(0);
	contactorAC(0);
}


static inline void nextState(enum stateAC_t state)
{
	stateAC = state;
	cnt_rel = 0;
}


int16_t acControlStep(int16_t vac_raw, int16_t iac_raw, uint16_t vdc_sinc_mix_100mV, uint16_t v_dc_ref_100mV, float p_ac_rms_ref)
{
//GPIOC->BSRR = (1<<4);  // set Testpin TP201 PC4
	static int16_t duty = 0;
	static uint32_t cnt_pll_locked = 0;

	bool acGrid_valid = false;

	// Vgrid
	int32_t vd = pll_get_vd();
	int16_t v_ac_rms_100mV = (((int32_t)vd) * ((10*VGRID_ADCR)/M_SQRT2) )/(1<<ADC_BITS_VGRID);
	debug_v_ac_rms_100mV = v_ac_rms_100mV;
	int32_t w = pll_get_w();
	debug_f_ac_10mHz = (w*100)>>15; // todo: check if (2^15)-1

	if (vd > VD_MIN_RAW && vd < VD_MAX_RAW &&
	   w > W_MIN_RAW && w < W_MAX_RAW) {
	  if (cnt_pll_locked >= 0.2*AC_CTRL_FREQ) {  // 200ms
		  acGrid_valid = true;
	  } else {
		  cnt_pll_locked++;
	  }
	} else {
	  shutdownAC();
	  acGrid_valid = false;
	  if (stateAC == GRID_SYNC) {
		  sys_errcode = EC_GRID_SYNC_LOST;
	  }
	  stateAC = INIT_AC;
	  cnt_pll_locked = 0;
	}

	if (sys_errcode != EC_NO_ERROR ) {
		shutdownAC();
		stateAC = INIT_AC;
	} else if (sys_mode_needs_ACside == false) {
		gatedriverAC(0);  // get current to zero before contactor action. todo: check
		stateAC = INIT_AC;
	}

	cnt_rel++;

	// GPIOC->BRR = (1<<4);  // reset Testpin TP201 PC4   from function enter to here 3us

	int16_t phase = pll_singlephase_step(vac_raw);

	//GPIOC->BRR = (1<<4);  // reset Testpin TP201 PC4   from function enter to here 20us; 14us with -O1

	int16_t phase_shiftRL = 0;
	int16_t duty_v_amp_pred = 0;

	switch (stateAC) {
	  case INIT_AC:
		shutdownAC();
		if (sys_errcode == EC_NO_ERROR ) {
			if (sys_mode_needs_ACside) {
				nextState(WAIT_AC_DC_VOLTAGE);
			}
		}
		break;

	  case WAIT_AC_DC_VOLTAGE:
		if (   acGrid_valid
		    && VdcFBgrid_sincfilt_100mV > ( ((float)debug_v_ac_rms_100mV) * M_SQRT2 )  // no precharge resistors available
			&& cnt_rel >= 4*AC_CTRL_FREQ)  // wait at least 4 sec to avoid instabilities
		 {
			pll_set_phaseOffset((1<<15) * +10.0/20);  // zero crossing of grid and converter voltage matched
			nextState(WAIT_ZERO_CROSSING);
		}
		break;

	  case WAIT_ZERO_CROSSING:  // for LCL charge without overshoot
		if ( phase > (0.25*INT16_MAX) && phase < (0.27*INT16_MAX) ) {
			nextState(CLOSE_CONTACTOR_AC);
		}
		break;

	  case CLOSE_CONTACTOR_AC:
		gatedriverAC(1);
		//contactorAC(1);
		nextState(WAIT_CONTACTOR_AC);


	  case WAIT_CONTACTOR_AC:
		if (cnt_rel == 0.025*AC_CTRL_FREQ) {  // 25ms delay for contactor action
  			nextState(GRID_CONNECTING);
		}
		break;

	  case GRID_CONNECTING:
		  // todo implement islanding detection in GRID_CONNECTING
		piCtrl.y = 0;
		piCtrl.x_prev = 0;
		nextState(GRID_SYNC);
		break;

	  case GRID_SYNC:
	  {

//		if (cnt_rel >= 2.0*AC_CTRL_FREQ) {  // short turnon for testing
//			nextState(WAIT_AC_DC_VOLTAGE);
//			gatedriverAC(0);
//		}

		// trafo with
		// 2 winding systems in series and 5 windings in opposite direction has 44.7Vpeak at 227.0V grid and 1.6W loss
		// 2 winding systems in series and 5 windings in same direction has 47.9Vpeak at 227,0V grid
		// -> feedin down to 2.98Vcell

		// 0,5mm2 white wire inside inverter can carry max 12A : todo check

		// #########################
		// ### control algorithm ###
		// #########################

		// problems:
		// 1. high delay from grid current rise until voltage rise ~1ms. Maybe caused by LCL filter -> old
		// 2. unloaded LCL output and high dutycycle causes ringing with 1.66kHz 70Vpp
		//    starts with Umid1 not switching at all for 100us = 2PWM periods. Pulse before was 400us so no dutycycle limit
		//    starts also with Umid2 not switching at all for 54us = 1PWM periods
		//    problem exists in PWM 3.3V signal
		//    => solved by compare unit 1 config: "greater than" instead of "equal"
		// 3. clipping : if DC voltage is too small for clean sine wave, reduce current

//#define EXTEND_LOC 8
////#define SCALE_VAC2VDC (int32_t)( ((VGRID_ADCR/VGRID_TRATIO) / VIN_ADCR ) * (1<<EXTEND_LOC) )
//#define SCALE_VAC2VDC (int32_t)( ((VGRID_ADCR) / (VIN_ADCR) ) * (1<<EXTEND_LOC) )

		// grid current feedforward
		//if (cnt_rel >= 1.0*AC_CTRL_FREQ) {  // turnon for testing
			int16_t i_ac_amp_10mA = 0;
			if (v_dc_ref_100mV == 0) {
				// use power reference from power controller
				int i_ac_amp_10mA_unclamped = 100*M_SQRT2 * (p_ac_rms_ref) / ( ((float)v_ac_rms_100mV) / 10);
				i_ac_amp_10mA = std::clamp(i_ac_amp_10mA_unclamped, -(int)IAC_AMP_MAX_10mA, (int)IAC_AMP_MAX_10mA);
			} else {
				// DC voltage control
				i_ac_amp_10mA = step_pi_Vdc2IacAmp( v_dc_ref_100mV*10, vdc_sinc_mix_100mV*10 );
				//i_ref_amp = step_pi_Vdc2IacAmp_volt_comp( VDC_REF_RAW, Vdc_filt, phase );
			}

			debug_i_ac_amp_10mA = i_ac_amp_10mA;

			int16_t v_amp_pred_100mV = calc_IacAmp2VacSecAmpDCscale(i_ac_amp_10mA)/10;
			debug_v_amp_pred_100mV = v_amp_pred_100mV;
			duty_v_amp_pred = ( v_amp_pred_100mV * AC_DUTY_HALF ) / vdc_sinc_mix_100mV;
		//}
		phase_shiftRL = get_IacPhase();
		//int16_t phase_shiftRL = get_IacPhase()+ (int16_t)(((1<<15)*1.08)/20);
		//OLD: Strom(3,5Aamp) in Phase bei 40,5W/42,5VA (25,4Vdc bis 27,3Vdc)


		break;
	  }
      default:
          break;
      }

	// grid voltage feedforward
	int32_t v_ac_amp_100mV = (((int32_t)pll_get_vd()) * ((10*VGRID_ADCR)) )/(1<<ADC_BITS_VGRID);
	int16_t duty_pll = (v_ac_amp_100mV * AC_DUTY_HALF) / vdc_sinc_mix_100mV;

	duty = AC_DUTY_HALF + ( ( (duty_pll*(int32_t)cos1(phase) + duty_v_amp_pred*(int32_t)cos1(phase+phase_shiftRL)) ) >> 15 );
//	duty = AC_DUTY_HALF + ( ( (AC_DUTY_HALF)*(int32_t)cos1(phase) ) >> 15 );  // MAX amplitude test

	//duty = 8500 - 2*3; // PWMA 35.6ns low  -> 6*2*2.94ns
	//duty = 8500 - 5; // PWMA 29ns low  < 3 periods of the fHRTIM !
	//duty = 8500 - 4; // PWMA 23.8ns low  < 3 periods of the fHRTIM !
//	duty = 8500 - 3;  // PWMA always low !!,  PWM B high
//	duty = 0;  // PWMA always low, PWM B high


// single PWM step has 1/(2*170MHz) = 2.94ns  -> center aligned PWM -> 5.88ns
// deadtime is configured to 64ns (10k resistor)
#define MIN_PULSE_HRTIM 18  // min pulse duration is 106ns - 64ns deadtime = 42ns

	// 100% duty: neg wave 571us high and not even small pulses. pos wave has 23.6ns low pulses
	if (duty > (AC_DUTY_ABSMAX-MIN_PULSE_HRTIM/2)) {
	  //duty = AC_DUTY_ABSMAX-MIN_PULSE_HRTIM;
	  duty = AC_DUTY_ABSMAX-4;  // todo implement 100% duty: okay for now because gatedriver ignores small pulses
	} else if (duty > (AC_DUTY_ABSMAX-MIN_PULSE_HRTIM)){
	  duty = AC_DUTY_ABSMAX-MIN_PULSE_HRTIM;
	} else if (duty < MIN_PULSE_HRTIM){
	  //duty = MIN_PULSE_HRTIM;
	  duty = 0;
	}

	//GPIOC->BRR = (1<<4);  // reset Testpin TP201 PC4   from function enter to here 23us -> too much

	return duty;

}


errorPVBI_t checkACLimits() {
	if ( iac_10mA > E_IAC_MAX_10mA ) {
		return EC_I_AC_MAX;
	}
	if ( VdcFBgrid_sincfilt_100mV > E_VDC_MAX_FB_GRID_100mV ) {
		return EC_V_DC_MAX_FB_GRID;
	}
	return EC_NO_ERROR;
}


void measVdcFBgrid()
{

	// sigma delta bitstream with 10MHz
	// 10MHz/20kHz = 500 -> 250 high periods equal 0V
	// v1: gated counter -> not precise, tested with 50kHz
	//     100*100ns sampled with 170MHz -> cnt=1700 at 0V, measured 1760-1811
	// v2: Vdc is close to Vadc_max → only single zeros occur and can be counted with simple rising/falling edge counter
	//     0V = 250 edges
	//     e.g. voltage divider R1=1k R2=10k
	//     11V = 50 edges, because sigma delta outputs 90% ones at Vadc_max
//#define VIN_ADCR (9.67/9.81 * 11/1)  // adc range in volts ~[0,11.0V]  -> @9.67V,dec=16 : 9.66V to 9.70V
//#define VIN_ADCR (414.7/1)  // adc range in volts ~[0,414.7V]  -> @9.67V : dec=16 : 369.1 to 370.3V ; dec=32 : 369.1 to 370.3V -> same
//#define VIN_ADCR (450/1)  // adc range in volts ~[0,414.7V]  -> @9.67V :  dec=32 : 3s.1 to 3s3V -> same

	//define above
//#define VIN_ADCR (450/1)  // adc range in volts ~[0,414.7V]  -> 20kHz  dec=16 :
	// @0.00V Vdc=59.2-178.9V, re=136-231
	// @3.41V Vdc=173.8-174.9V, re=167-177
	// @4.81V Vdc=210.2-213.1V, re=155-158
	// @6.46V Vdc=271.9-272.9V, re=128-130
	// @7.78V Vdc=327.2-327.6V, re=103-104
	// @9.55V Vdc=400.6-401.2V, re=71-72
	// @12.57V Vdc=528.4-529.1V, re=14-16

	uint16_t sigma_delta_re = TIM2->CNT;
	//uint16_t sigma_delta_re = 250;  //for DC debugging todo remove
	TIM2->CNT = 0;

	if (sigma_delta_re < 40 || sigma_delta_re > 500) {
		sys_errcode = EC_V_DC_SENSOR_FB_GRID;
	}

	uint16_t filt_in = sigma_delta_re;

	// nominal voltage of battery: 96*3.7=355.2V
	// max voltage of ADC: 450V equals 90% ones
	// 50% + 355.2/450 * 40% = 81.6% ones
	// -> 8 of 10 bits are ones
	// => low probability of sequential zeros. Thus, edge counting should be sufficient!

	static int cnt_intr = 0;
	cnt_intr++;

	// Lowpass filter for Vdc

	// v1: averaging
//	static uint16_t Vdc_prev[3] = {0};
//	uint16_t Vdc_filt = (sigma_delta_cnt + Vdc_prev[0] + Vdc_prev[1] + Vdc_prev[2]);// >> 2;
//	debug_Vdc_filt = Vdc_filt;
//	Vdc_prev[2] = Vdc_prev[1];
//	Vdc_prev[1] = Vdc_prev[0];
//	Vdc_prev[0] = sigma_delta_cnt;

	// v2: sinc-2nd order
//	static unsigned int int_stage[2] = {0};
//	static unsigned int comb_stage[2] = {0};
//
//	int_stage[1] += int_stage[0];
//	int_stage[0] += sigma_delta_cnt;
//
//	if (cnt_intr == 32) {
//
//		unsigned int tmp = int_stage[1]-comb_stage[0];
//		comb_stage[0] = int_stage[1];
//
//		unsigned int Vdc_filt = tmp-comb_stage[1];
//		comb_stage[1] = tmp;
//		debug_Vdc_filt = Vdc_filt;
//
//		cnt_intr = 0;
//	}

	// v3: sinc-3rd order (dec32 32bit: 2,8% duty) (dec32 64bit: 4.3% duty)
	// init filter (~260Vdc) -> still need blanking time
//	static uint32_t int_stage[3] = {6873269, 100990673, 617986547};
//	static uint32_t comb_stage[3] = {617986547, 683719973, 1747908787};
	static uint32_t int_stage[3] = {0};
	static uint32_t comb_stage[3] = {0};

	int_stage[2] += int_stage[1];
	int_stage[1] += int_stage[0];
	int_stage[0] += filt_in;

	// Downsampling factor 16 -> 20kHz/16=1.25kHz
#define CIC_GAIN (16*16*16)  // decimation ld(16)=4, 3stages -> 4096 (12bit)
	if (cnt_intr == 16) {

		uint32_t tmp = int_stage[2]-comb_stage[0];
		comb_stage[0] = int_stage[2];

		uint32_t tmp2 = tmp-comb_stage[1];
		comb_stage[1] = tmp;

		uint32_t filt_out = (tmp2-comb_stage[2]);
		//int Vdc_filt = ((tmp2-comb_stage[2])-1847679234)/220668;
		comb_stage[2] = tmp2;

		// voltage calculation
		// 0V = 50% bitstream ones = 250 edges counted in 50us -> probability of doubles high -> less edges
		// 1V = 90% bitstream ones = 50 edges counted in 50us
		// pos value range from 50 to 250 -> div by 200

#define V_DC_MAX_FBgrid (1+59)  // 68kOhm 450×68÷(450+68)
#define V_DC_CALIB_FBgrid  992  // per mil for 68kOhm 450×68÷(450+68)
//#define V_DC_MAX_FBgrid (1+450)  // todo 450 voltage divider

#if (E_VDC_MAX_100mV/10) > ((V_DC_MAX_FBgrid*97)/100)
#error Choose E_VDC_MAX_100mV lower than FBgrid sensor range
#endif
		// decim=16 ->                                   7bit(100milliVolt) + 12bit(CIC_GAIN) + 8bit(signal) = 27bit
		uint32_t vdc_no_calib = (V_DC_MAX_FBgrid * ( (10             * ((CIC_GAIN*250-filt_out)/200) ))/CIC_GAIN);
		VdcFBgrid_sincfilt_100mV = (vdc_no_calib*V_DC_CALIB_FBgrid)/1000;

		cnt_intr = 0;
	}

}
