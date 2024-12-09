
#include "stm32g4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <algorithm>
#include <climits>

#include "gpio.h"
#include "common.h"
#include "monitoring.h"
#include "ac_control.h"
#include "dc_control.h"

#include "controller.h"
#include "sogi_pllFXP.h"

// single PWM step has 1/170MHz = 5.88ns
// deadtime is configured to 64ns (10k resistor)
//#define MIN_PULSE 18  // min pulse duration is 106ns - 64ns deadtime = 42ns

#define AC_DUTY_ABSMAX 8500
#define AC_DUTY_HALF (AC_DUTY_ABSMAX/2)


volatile uint16_t v_dc_FBgrid_sincfilt_100mV;
volatile int16_t i_ac_10mA;
volatile int16_t p_ac_filt50Hz;
volatile int16_t p_ac_filt1minute;

volatile int16_t debug_v_ac_rms_100mV;
volatile int16_t debug_f_ac_10mHz;
volatile int16_t debug_i_ac_amp_10mA;


volatile enum stateAC_t stateAC = INIT_AC;
bool sampling_trig_en = false;

static int16_t permil_v_dffw = PERMIL_V_DFFW_MIN;  // per mil of direct voltage feedforward

static uint32_t cnt_rel = 0;

static int cnt_intr = 0;  // for sigma_delta processing


const int16_t get_p_ac_filt50Hz()
{
	return p_ac_filt50Hz;
}


const int16_t get_p_ac_filt1minute()
{
	return p_ac_filt1minute;
}


void fill_monitor_vars_ac(monitor_vars_t* mon_vars)
{
	mon_vars->stateAC = stateAC;
	mon_vars->f_ac_10mHz = debug_f_ac_10mHz;
	mon_vars->v_ac_rms_100mV = debug_v_ac_rms_100mV;
	mon_vars->i_ac_amp_10mA = debug_i_ac_amp_10mA;
	mon_vars->p_ac_filt50Hz = p_ac_filt50Hz;

	mon_vars->v_dc_FBgrid_sincfilt_100mV = v_dc_FBgrid_sincfilt_100mV;
}


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


static inline bool check_zero_crossing(int16_t phase)
{
	if ( phase > (0.25*INT16_MAX) && phase < (0.27*INT16_MAX) ) {
		return true;
	}
	return false;
}

// todo: add controller for stationary accuracy for p_bat_reduction and p_external, e.g. later in code when P is converted to current
void calc_p_ac(control_ref_t* ctrl_ref)
{
	int16_t p_default = ctrl_ref->p_ac_pccCtrl + get_p_ac_bat_chg_reduction();

	if (ctrl_ref->ext_ctrl_mode == EXT_AC_HARD) {
		ctrl_ref->p_ac_rms = ctrl_ref->p_ac_external;  // risk of BMS action; useful for BMS test
	} else if (ctrl_ref->ext_ctrl_mode == EXT_AC_SOFT) {
		if (    ctrl_ref->p_ac_pccCtrl > 0  // ignore power setpoints smaller than grid consumption
		     || get_p_ac_bat_chg_reduction()  // handle high PV input; todo GRID2BAT interrupts grid current at Pbat~=60W
			) {
			ctrl_ref->p_ac_rms = std::max(ctrl_ref->p_ac_external, p_default);
		} else {
			ctrl_ref->p_ac_rms = ctrl_ref->p_ac_external;
		}
	} else {
		ctrl_ref->p_ac_rms = p_default;
	}
}


int16_t acControlStep(uint16_t cnt20kHz_20ms, control_ref_t ctrl_ref, uint16_t v_dc_FBboost_sincfilt_100mV, uint16_t v_dc_FBboost_filt50Hz_100mV, int16_t v_ac_raw, uint16_t i_ac_raw)
{
//GPIOC->BSRR = (1<<4);  // set Testpin TP201 PC4
	static int16_t duty = 0;
	static uint32_t cnt_pll_locked = 0;

	bool acGrid_valid = false;

	// Grid current
#define IAC_OFFSET_RAW 2629
#define IAC_mV_per_LSB (3300.0/4096)  // 3.3V 12bit
#define IAC_mV_per_A 35 // current sensor datasheet 35mV/A
#define IAC_RAW_TO_10mA (IAC_mV_per_LSB * 100.0/IAC_mV_per_A)
	i_ac_10mA = (i_ac_raw-IAC_OFFSET_RAW) * IAC_RAW_TO_10mA;

	// Grid voltage
	int16_t v_ac_100mV = (((int32_t)v_ac_raw) * (10*VGRID_ADCR) )/(1<<ADC_BITS_VGRID);
	int32_t vd = pll_get_vd();
	int16_t v_ac_amp_100mV = (((int32_t)vd) * (10*VGRID_ADCR) )/(1<<ADC_BITS_VGRID);
	int16_t v_ac_rms_100mV = (((int32_t)vd) * ((10*VGRID_ADCR)/M_SQRT2) )/(1<<ADC_BITS_VGRID);
	debug_v_ac_rms_100mV = v_ac_rms_100mV;

	//static int16_t v_ac_rms_filt50Hz_100mV = 0;
	static int16_t v_ac_amp_filt50Hz_100mV = 0;
	static int32_t v_ac_amp_sum = 0;
	v_ac_amp_sum += v_ac_amp_100mV;

	static int32_t i_ac_sum = 0;
	i_ac_sum += i_ac_10mA;
	static uint8_t cnt_dc_offset = 0;

	int v_dcCtrl_mV = 0;
	//static int v_dcCtrl_I_mV;
	//static int16_t i_ac_dc_offset_mA;

	if (cnt20kHz_20ms == 0) {
		v_ac_amp_filt50Hz_100mV = v_ac_amp_sum/CYCLES_cnt20kHz_20ms;
		//v_ac_rms_filt50Hz_100mV = ((float)v_ac_amp_filt50Hz_100mV) / M_SQRT2;
		v_ac_amp_sum = 0;

		//i_ac_dc_offset_mA = (i_ac_sum*10)/CYCLES_cnt20kHz_20ms;

		// check DC component in AC current
		if ( i_ac_sum > (E_I_AC_DC_OFFSET_MAX_10mA*CYCLES_cnt20kHz_20ms) ) {
			i_ac_sum = 0;
			cnt_dc_offset++;
			if (cnt_dc_offset >= E_I_AC_DC_OFFSET_CYCLES) {
				set_sys_errorcode(EC_I_AC_DC_OFFSET);
			}
		} else {
			cnt_dc_offset = 0;
		}
	}

	// DC voltage
	uint16_t v_dc_sinc_mix_100mV = (v_dc_FBboost_sincfilt_100mV + v_dc_FBgrid_sincfilt_100mV)/2;

	// no 100Hz ripple prediction + extra filter -> Pac ~100W: 45.6V to 47.0V instead of 44.9V to 47.8V FBgrid_sincfilt
	//static uint16_t v_dc_prev[3] = {0};
	//uint16_t v_dc_modulator_100mV = lowpass4(v_dc_sinc_mix_100mV, v_dc_prev);

	// 100Hz ripple prediction because sinc filter is too slow to provide precise modulator voltage
	// sinc filter adds average delay of 16/20kHz = 0.8ms. New sample 0.4ms up to 1.2ms
	static uint16_t v_dc_modulator_100mV;
	//static int16_t delta_v_prev8_10mV;
	//static int16_t delta_v_10mV;

	int p_ac_mW = -i_ac_10mA*v_ac_100mV;
#define BATTERY_FACTOR_PERCENT 20  // define part of 100Hz ripple which is taken by battery

//  fixed point capacitor ripple compensation
//	disadvantage: battery part is hard to estimate
//	              calculation time needed
//	              external events are delayed
//	=> todo: implement direct V_DC measurement without software sigma delta converter
//	int32_t p_cap_10mW = ((100-BATTERY_FACTOR_PERCENT) * ( p_ac_filt50Hz - (p_ac_mW/1000)) );  // p_cap >0 (charging) if actual p_ac < pac_rms
//	constexpr uint16_t COEF = AC_CTRL_FREQ*CAPACITANCE;  // 20000×4×390×10^−6 = 31.2
//	delta_v_10mV += (p_cap_10mW*10)/(v_dc_modulator_100mV*COEF);
//	if (cnt_intr == 0) {
//		delta_v_10mV = (delta_v_10mV-delta_v_prev8_10mV);  // compensates measurement delay
//	} else if (cnt_intr == 8) {
//		delta_v_prev8_10mV = delta_v_10mV;
//	}
//
//	v_dc_modulator_100mV = v_dc_sinc_mix_100mV + delta_v_10mV/10;

	// for stability during test
	//v_dc_modulator_100mV = 490;
	//v_dc_modulator_100mV = v_dc_FBboost_filt50Hz_100mV;
	v_dc_modulator_100mV = v_dc_sinc_mix_100mV;

	// grid frequency
	int32_t w = pll_get_w();
	debug_f_ac_10mHz = (w*100)>>15; // todo: check if (2^15)-1

#define DEBUG_OUTPUT_CURRENT 0  // for testing output current without grid
#if DEBUG_OUTPUT_CURRENT == 0
	if (vd >= VD_MIN_RAW && vd <= VD_MAX_RAW &&
	     w >= W_MIN_RAW && w <= W_MAX_RAW) {
	  if (cnt_pll_locked >= 0.2*AC_CTRL_FREQ) {  // 200ms
		  acGrid_valid = true;
	  } else {
		  cnt_pll_locked++;
	  }
	} else {
	  shutdownAC();
	  acGrid_valid = false;
	  if (stateAC == GRID_SYNC) {
		  if (vd < VD_MIN_RAW) {
			set_sys_errorcode(EC_V_AC_LOW);
		  } else if (vd > VD_MAX_RAW) {
			set_sys_errorcode(EC_V_AC_HIGH);
		  } else if (w < W_MIN_RAW) {
			set_sys_errorcode(EC_FREQ_AC_LOW);
		  } else if (w > W_MAX_RAW) {
			set_sys_errorcode(EC_FREQ_AC_HIGH);
		  } else {
		  	set_sys_errorcode(EC_GRID_SYNC_LOST);
	  	  }
	  }
	  stateAC = INIT_AC;
	  cnt_pll_locked = 0;
	}
	int16_t phase = pll_singlephase_step(v_ac_raw);
#else
	acGrid_valid = true;
	v_ac_amp_filt50Hz_100mV = 45*10;
	static int16_t phase;
	phase += SHRT_MAX/(50*8);
#endif //DEBUG_OUTPUT_CURRENT

	if (get_sys_errorcode() != EC_NO_ERROR ) {
		shutdownAC();
		stateAC = INIT_AC;
	} else if (ctrl_ref.mode == AC_OFF) {
		gatedriverAC(0);  // get current to zero before contactor action. todo: check
		stateAC = INIT_AC;
	}

	cnt_rel++;

	static int p_ac_sum_mW;  // max +-5.3kW
	p_ac_sum_mW += p_ac_mW;

	if (cnt20kHz_20ms == 0) {
		p_ac_filt50Hz = p_ac_sum_mW/CYCLES_cnt20kHz_20ms/1000;
		p_ac_sum_mW = 0;
	} else if (cnt20kHz_20ms == 1) {

		static uint16_t cnt50Hz_1min;
		cnt50Hz_1min++;

		// todo thermal model for AC side -> more output power for pulse loads and cloudy days 1.2kW nom 1.6kW max

		static int p_ac_sum_1minute;
		p_ac_sum_1minute += p_ac_filt50Hz;

		if (cnt50Hz_1min == 60*50) {
			cnt50Hz_1min = 0;
			p_ac_filt1minute = p_ac_sum_1minute/(60*50);
			p_ac_sum_1minute = 0;
		}
	}

	int16_t phaseshift_RL = 0;
	int v_amp_pred_100mV = 0;
	int v_prCtrl_10mV = 0;

	switch (stateAC) {
	  case INIT_AC:
		shutdownAC();
		if (get_sys_errorcode() == EC_NO_ERROR ) {
			if (ctrl_ref.mode != AC_OFF) {
				nextState(WAIT_AC_DC_VOLTAGE);
			}
		}
		break;

	  case WAIT_AC_DC_VOLTAGE:
		if (   acGrid_valid
		    && v_dc_FBgrid_sincfilt_100mV > v_ac_amp_100mV  // no precharge resistors available
		    && v_dc_FBgrid_sincfilt_100mV > v_ac_amp_filt50Hz_100mV
		    && cnt_rel >= 2*AC_CTRL_FREQ)  // wait at least 2 sec to avoid instabilities
		 {
			// zero crossing of grid and converter voltage tuning:
			//pll_set_phaseOffset((1<<15) * +10.05/20);  // i_rms 260-365mA
			pll_set_phaseOffset((1<<15) * +10.0/20);  // i_rms 130-169mA; grid leading 5;10;13us lagging 6;13;18;28;47us
			//pll_set_phaseOffset((1<<15) * +9.99/20);  // i_rms 121-147mA  sun/PV input has influence to grid: 260-320mA
			//pll_set_phaseOffset((1<<15) * +9.98/20);  // i_rms 119-135mA; 209-213mA with sun; grid leading 0;18.1-18.7;41;47us
			//pll_set_phaseOffset((1<<15) * +9.95/20);  // i_rms 120-138mA
			nextState(WAIT_ZERO_CROSSING);
		}
		break;

	  case WAIT_ZERO_CROSSING:  // for LCL charge without overshoot
		if ( check_zero_crossing(phase) ) {
			nextState(CLOSE_CONTACTOR_AC);
		}
		break;

	  case CLOSE_CONTACTOR_AC:
		gatedriverAC(1);
		contactorAC(1);
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
		//v_dcCtrl_I_mV = 0;
		gatedriverAC(1);
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
		// -> added another 4 windings in opposite direction 45.8Vpeak when feedin 4A

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
			static int16_t i_ac_amp_10mA = 0;
			switch (ctrl_ref.mode) {
			  case AC_PASSIVE:  // for battery reconnect to avoid DC voltage controller fighting AC Vdc voltage controller
				gatedriverAC(0);
				nextState(WAIT_CONTACTOR_AC);  // todo gatedriver is turned on for short period
				break;

			  case VDC_CONTROL:
				gatedriverAC(1);
				i_ac_amp_10mA = step_pi_Vdc2IacAmp( ctrl_ref.v_dc_100mV*10, v_dc_sinc_mix_100mV*10 );
				//i_ac_amp_10mA = step_pi_Vdc2IacAmp_volt_comp( ctrl_ref.v_dc_100mV*10, v_dc_sinc_mix_100mV*10, phase, 10*v_ac_amp_filt50Hz_100mV);
				break;

			  case VDC_VARIABLE_CONTROL:
			  {
				gatedriverAC(1);
				// sine in modern power grid is flattened -> no extra headroom needed
				uint32_t v_dc_ref_10mV = 10*v_ac_amp_filt50Hz_100mV + ((R*64)*i_ac_amp_10mA)/64;
				i_ac_amp_10mA = step_pi_Vdc2IacAmp( v_dc_ref_10mV, v_dc_sinc_mix_100mV*10 );
				//i_ac_amp_10mA = step_pi_Vdc2IacAmp_volt_comp( ctrl_ref.v_dc_100mV*10, v_dc_sinc_mix_100mV*10, phase, 10*v_ac_amp_filt50Hz_100mV);
				break;
			  }

			  case PAC_CONTROL:
			  {
				// use power reference from power controller
				int i_ac_amp_10mA_unclamped = 100 * (2*ctrl_ref.p_ac_rms*10) / v_ac_amp_filt50Hz_100mV;

				// V1: clamp
				//i_ac_amp_10mA = std::clamp(i_ac_amp_10mA_unclamped, -(int)IAC_AMP_MAX_10mA, (int)IAC_AMP_MAX_10mA);

				// V2: ramp: max decent per 50Hz period: 20kHz/50Hz * 10mA = 4A
				if (i_ac_amp_10mA_unclamped > i_ac_amp_10mA  && i_ac_amp_10mA < IAC_AMP_MAX_10mA) {
					i_ac_amp_10mA++;
				} else if (i_ac_amp_10mA_unclamped < i_ac_amp_10mA  && i_ac_amp_10mA > -IAC_AMP_MAX_10mA) {
					i_ac_amp_10mA--;
				}

#define ENABLE_LOW_POWER_ENERGY_PACKET_CONTROLLER 1
#if ENABLE_LOW_POWER_ENERGY_PACKET_CONTROLLER == 1 && SYSTEM_HAS_BATTERY == 1
constexpr uint16_t P_LOW_POWER_CTRL_REENABLE = 160;  // 200 leads to 400mWh sold in 10 minutes and 600mWh buyed
				// electricity meter stores energy in 100mWh
				//-> in 1 sec, 100mWh/1sec=360W offset can be tolerated
				// if feedin is included, 720Ws can be tolerated -> todo
				static uint32_t low_power_mode_cnt = 0;

				if (    ((ctrl_ref.p_ac_rms <= 50 && get_p_dc_filt50Hz() < 40) || ctrl_ref.p_ac_rms <= 20) // reduce number of switching events during PV production
					 && ctrl_ref.p_pcc <= 10 && (get_p_ac_max_dc_lim() > P_LOW_POWER_CTRL_REENABLE)
					 && check_zero_crossing(phase)  // turnoff during zero crossing reduces "click" noise in inductor
					 && ctrl_ref.ext_ctrl_mode == EXT_OFF  // do not use packet mode in case of external power control
				) {
					gatedriverAC(0);
					low_power_mode_cnt++;
				} else if (low_power_mode_cnt) {
					if(    ctrl_ref.p_ac_rms > P_LOW_POWER_CTRL_REENABLE  // 200/50 = 3sec off / 1sec on, depends on power_controller tuning, see calc sheet
						|| ( low_power_mode_cnt > (AC_CTRL_FREQ*3) && (get_p_ac_max_dc_lim() < P_LOW_POWER_CTRL_REENABLE) )  // en if power controller is limited by reduced battery power
						|| ctrl_ref.ext_ctrl_mode != EXT_OFF  // leave packet mode in case of external power control
					) {
						gatedriverAC(1);
						low_power_mode_cnt = 0;
						if (i_ac_amp_10mA > 100) {
							i_ac_amp_10mA = 100;  // clamp to 1A for starting point
						}
					}
				}
#endif

				break;
			  }
			  case FFWD_ONLY:
			  default:
				break;
			}

			debug_i_ac_amp_10mA = i_ac_amp_10mA;
			int i_ref_10mA = (i_ac_amp_10mA * (int32_t)cos1(phase))>>15;

#define USE_AC_DC_OFFSET_CONTROLLER 0
#if USE_AC_DC_OFFSET_CONTROLLER == 1
			if (cnt20kHz_20ms == 0) {
				// control DC offset in AC current
				// assume R = 0.7 todo use exact R
				// 2A offset needs V = 1.4V DC voltage

				// zu wenig
//				const uint8_t KP_DIV = 2;
//				const uint8_t KI_DIV = 4;

				const uint8_t KP_DIV = 1;
				const uint8_t KI_DIV = 2;
#define EXTRA_GAIN 4
		#define V_AC_DC_OFFSET_MAX_mV (16*1000)
				int16_t v_ac_dc_mV = (i_ac_dc_offset_mA*(EXTRA_GAIN*0.7*32))/32;

				v_dcCtrl_I_mV += v_ac_dc_mV / KI_DIV;
				v_dcCtrl_I_mV = std::clamp(v_dcCtrl_I_mV, (int)(-V_AC_DC_OFFSET_MAX_mV), (int)V_AC_DC_OFFSET_MAX_mV);
				v_dcCtrl_mV = v_ac_dc_mV/KP_DIV + v_dcCtrl_I_mV;
				v_dcCtrl_mV = std::clamp(v_dcCtrl_mV, (int)(-V_AC_DC_OFFSET_MAX_mV), (int)V_AC_DC_OFFSET_MAX_mV);
				//v_dcCtrl_mV = -4000; // -4.0A
				//v_dcCtrl_mV = -10000; // -3.9A depend on switch on?
			}
#endif

			// V1: linear inductor model
			//int16_t v_amp_pred_100mV = calc_IacAmp2VacSecAmpDCscale(i_ac_amp_10mA)/10;

			// V2: saturating inductor model; uses LUT with nonlinear inductance or remanence model
			//v_amp_pred_100mV = calc_v_amp_pred(i_ac_amp_10mA, i_ac_10mA/10)/10;  // i_meas -> audible noise 150Hz:-10dBV 250Hz:-23dBV
			v_amp_pred_100mV = calc_v_amp_pred(i_ac_amp_10mA, i_ref_10mA/10)/10;  // i_ref -> audible noise 150Hz:-11dBV 250Hz:-26dBV

		//}
		phaseshift_RL = get_IacPhase();
		//int16_t phase_shiftRL = get_IacPhase()+ (int16_t)(((1<<15)*1.08)/20);
		//OLD: Strom(3,5Aamp) in Phase bei 40,5W/42,5VA (25,4Vdc bis 27,3Vdc)

		// proportional resonant current controller
		// 16.02.2024: does not compensate saturating inductor -> using 1D saturation LUT feedforward only
//		v_prCtrl_10mV = pr_step(i_ref_10mA-i_ac_10mA);
//		v_prCtrl_10mV = std::clamp(v_prCtrl_10mV, -3*100, 3*100);

		break;
	  }
      default:
          break;
      }

	// grid voltage feedforward
#if DEBUG_OUTPUT_CURRENT == 0
	int32_t v_amp_pll_100mV = v_ac_amp_filt50Hz_100mV;
	int32_t v_pll = v_amp_pll_100mV*(int32_t)cos1(phase);  // range: +-6553V
	#define SCALE_FFW 6
	v_pll >>= SCALE_FFW;  // with permil gain in next step, range will be: *64/1000 -> +- 419V
	int32_t v_ffw = (permil_v_dffw*((-v_ac_100mV)<<(15-SCALE_FFW)) + (1000-permil_v_dffw)*v_pll)/1000;
	v_ffw <<= SCALE_FFW;
#else
	int32_t v_ffw = 0;
#endif

	int32_t vout = (v_ffw + v_amp_pred_100mV*(int32_t)cos1(phase+phaseshift_RL) + (v_prCtrl_10mV<<15)/10 + (v_dcCtrl_mV<<15)/100 ) / v_dc_modulator_100mV;

	duty = AC_DUTY_HALF + ( (AC_DUTY_HALF*vout) >> 15 );
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
	  duty = AC_DUTY_ABSMAX-4;  // todo implement 100% duty: okay for now because gatedriver ignores small pulses
	} else if (duty > (AC_DUTY_ABSMAX-MIN_PULSE_HRTIM)){
	  duty = AC_DUTY_ABSMAX-MIN_PULSE_HRTIM;
	} else if (duty < MIN_PULSE_HRTIM) {
		if (duty > MIN_PULSE_HRTIM/2) {
			duty = MIN_PULSE_HRTIM;
		} else {
			duty = 0;
		}
	}

	if (fast_mon_vars_trig) {
		if (frame_nr < FAST_MON_FRAMES) {
			//fast_monitor_vars[frame_nr].v_dc = v_dc_FBgrid_sincfilt_100mV;
			//fast_monitor_vars[frame_nr].v_dc = delta_v_10mV;
			fast_monitor_vars[frame_nr].v_dc = duty;
			fast_monitor_vars[frame_nr].v_dc_modulator_100mV = v_dc_modulator_100mV;
			frame_nr++;
		}
	}

	//GPIOC->BRR = (1<<4);  // reset Testpin TP201 PC4
	// from function enter to here
	// with -O3 and delta_v float calculation 25.58us
	// with -O3 and delta_v fixp calculation  19.08us  with -O2 19.14us
	// with -O3 without delta_v               17.76us

	return duty;
}


errorPVBI_t checkACLimits() {
	if ( v_dc_FBgrid_sincfilt_100mV > E_VDC_MAX_FB_GRID_100mV ) {
		return EC_V_DC_MAX_FB_GRID;
	}

	static uint8_t cnt_i_ac_rmslimit;  // todo: calculate rms limit based on actual chip temperature
	if ( i_ac_10mA > E_I_AC_PULSE_MAX_10mA || i_ac_10mA < -E_I_AC_PULSE_MAX_10mA) {
		return EC_I_AC_PULSE_MAX;
	} else if ( i_ac_10mA > E_I_AC_RMS_MAX_10mA || i_ac_10mA < -E_I_AC_RMS_MAX_10mA) {
		permil_v_dffw = std::clamp(permil_v_dffw+PERMIL_V_DFFW_INCR, PERMIL_V_DFFW_MIN, PERMIL_V_DFFW_MAX);

		if (cnt_i_ac_rmslimit < E_I_AC_RMS_MAX_CNT) {
			cnt_i_ac_rmslimit++;
		} else {
			return EC_I_AC_RMS_MAX;
		}
	} else {
		if (cnt_i_ac_rmslimit > 0) cnt_i_ac_rmslimit--;
	}

	if (cnt_i_ac_rmslimit == 0) {
		permil_v_dffw = std::clamp(permil_v_dffw-PERMIL_V_DFFW_DECR, PERMIL_V_DFFW_MIN, PERMIL_V_DFFW_MAX);
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
		set_sys_errorcode(EC_V_DC_SENSOR_FB_GRID);
	}

	uint16_t filt_in = sigma_delta_re;

	// nominal voltage of battery: 96*3.7=355.2V
	// max voltage of ADC: 450V equals 90% ones
	// 50% + 355.2/450 * 40% = 81.6% ones
	// -> 8 of 10 bits are ones
	// => low probability of sequential zeros. Thus, edge counting should be sufficient!

	//static int cnt_intr = 0;
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
		v_dc_FBgrid_sincfilt_100mV = (vdc_no_calib*V_DC_CALIB_FBgrid)/1000;

		cnt_intr = 0;
	}

}
