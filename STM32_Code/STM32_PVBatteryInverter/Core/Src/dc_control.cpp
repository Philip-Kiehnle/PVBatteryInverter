
#include "stm32g4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <algorithm>


#include "common.h"
#include "gpio.h"
#include "dc_control.h"
//#include "battery/STW_mBMS.hpp"
#include "battery/ETI_DualBMS.hpp"
#include "mpptracker.hpp"

// single PWM step has 1/170MHz = 5.88ns -> center aligned PWM -> 11.8ns
// deadtime is configured to 64ns (10k resistor)
#define MIN_PULSE 9  // min pulse duration is 106ns - 64ns deadtime = 42ns

#define DC_CTRL_FREQ 20000
#define DC_CTRL_FREQ_MPPT 50

bool monitoring_binary = false;
volatile bool sys_mode_needs_battery = false;
extern volatile uint16_t v_dc_ref_100mV;

volatile uint16_t debug_duty;

// configure PV system and PWM dutycycle parameters of microcontroller here:

// Jinko JKM405N-6RL3
// Voc(25°C)=46.3
// Vmpp(25°C)=36.48
// Voc(-20°C)=46,3×(1+45×0,28÷100)=52,13
// Vmpp(70°C)=46,3×(1+45×0,28÷100)=31,88
constexpr pvModule_t PVMODULE = pvModule_t{
	.v_mp = 36.48,
	.i_mp = 11.1,
	.coef_v_temp = -0.0028,  // The open-circuit voltage temperature coefficient of the module [%/K]
	.v_bypassDiode = 0.5,
	.nr_bypassDiodes = 3
};

//constexpr mpptParams_t MPPTPARAMS = mpptParams_t{
//	.vin_min = 62,  // 2 modules * 31V
//	.vin_max = 424,  // 8 modules * 53V
//	.vout_min = 256,  // 80 Li-cells * 3.2
//	.vout_max = 336,  // 80 Li-cells * 4.2
//	.nr_pv_modules = 8,
//	.nr_substring_search_per_interval = 6
//};

// PV emulator:
// 2*Voc = 2*46.3 = 92.6V
// 2*Vmp = 2*36.48 = 73.0V
// 30%*Isc = 0.3*11.84 = 3.55A
// 30%*Imp = 0.3*11.1 = 3.33A

constexpr mpptParams_t MPPTPARAMS = mpptParams_t{
	.vin_min = 0.33*31,  // 1/3 modules * 31V
	.vin_max = 2*53,  // 2 modules * 53V
	.vout_min = 40,
	.vout_max = 58,
	.nr_pv_modules = 2,
	.nr_substring_search_per_interval = 1
};


constexpr unsigned int MPPT_FREQ = DC_CTRL_FREQ_MPPT;
//constexpr unsigned int INTERVAL_GLOB_MPPT_REGULAR_SEC = 10*60;  // 10 minutes
//constexpr unsigned int INTERVAL_GLOB_MPPT_TRIG_EVENT_SEC = 2*60;  // 2 minutes when power drops
constexpr unsigned int INTERVAL_GLOB_MPPT_REGULAR_SEC = 60;  // 1 minute
constexpr unsigned int INTERVAL_GLOB_MPPT_TRIG_EVENT_SEC = 30;  // 30 sec when power drops

constexpr unsigned int MPPT_DUTY_ABSMAX = 4250;
constexpr unsigned int MPPT_DUTY_MIN_BOOTSTRAP = 0;  // High side has isolated supply and no bootstrap capacitor


volatile uint16_t VdcFBboost_sincfilt_100mV;
volatile int16_t Idc_filt_10mA;

volatile float pdc_filt50Hz;
volatile float v_pv_filt50Hz;
volatile float v_dc_filt50Hz;
volatile bool mppt_calc_request;
volatile bool mppt_calc_complete;

MPPTracker mppTracker(MPPTPARAMS, PVMODULE);

//static STW_mBMS bms(2);  // BMS address
ETI_DualBMS bms(15);  // BMS address
int battery_voltage_mV;

extern volatile uint16_t debug_sigma_delta_re;

volatile enum stateDC_t stateDC = INIT_DC;
volatile bool monitoring_request;

volatile bool bms_update_request;
volatile bool bms_battery_request;
volatile bool batteryON;


static uint32_t cnt_rel = 0;


void calc_async_dc_control()
{
	if (monitoring_binary && monitoring_request) {
		monitoring_request = false;
		monitor_vars.sys_errcode = sys_errcode;
		monitor_vars.stateDC = stateDC;
		monitor_vars.duty = debug_duty;
		monitor_vars.pdc_filt50Hz = pdc_filt50Hz;
		monitor_vars.v_pv_filt50Hz = v_pv_filt50Hz;
		monitor_vars.v_dc_filt50Hz = v_dc_filt50Hz;

		monitor_vars.stateAC = stateAC;
		monitor_vars.f_ac_10mHz = debug_f_ac_10mHz;
		monitor_vars.v_ac_rms_100mV = debug_v_ac_rms_100mV;
		monitor_vars.v_amp_pred_100mV = debug_v_amp_pred_100mV;
		monitor_vars.i_ac_amp_10mA = debug_i_ac_amp_10mA;

		send_monitor_vars();
	}
	if (mppt_calc_request) {
		mppTracker.step(pdc_filt50Hz, v_pv_filt50Hz, v_dc_filt50Hz);
		mppt_calc_request = false;
		mppt_calc_complete = true;
	}

#if SYSTEM_HAS_BATTERY == 1
	if (bms_update_request) {
		bms_update_request = false;
		bms.get_summary();
		battery_voltage_mV = bms.voltage_mV;
	}

	if (bms_battery_request) {
		if (batteryON) {
			bms.batteryOn();
		} else {
			bms.batteryOff();
		}
		bms_battery_request = false;
	}
#endif

}


void shutdownDC()
{
	gatedriverDC(0);
	contactorBattery(0);
	batteryON = false;
	bms_battery_request = true;
}


static inline void nextState(enum stateDC_t state)
{
	stateDC = state;
	cnt_rel = 0;
}

int16_t dcControlStep()
{
	GPIOC->BSRR = (1<<4);  // set Testpin TP201 PC4

	static int16_t dutyLS1 = 0;
	static uint16_t cnt50Hz;
	static uint16_t vdc_filt50Hz_100mV = 0;

	bool vdc_inRange = false;

	// 50Hz more robust: for critical increase from 400V to 500V in 20ms: 0.5*4*390uF*(500^2-400^2)/0.02 = 3.5kW
	if (vdc_filt50Hz_100mV > VDC_BOOST_STOP_100mV && vdc_filt50Hz_100mV < E_VDC_MAX_100mV) {
		if (vdc_filt50Hz_100mV > VDC_BOOST_START_100mV) {
//	if (VdcFBboost_sincfilt_100mV > VDC_BOOST_STOP_100mV && VdcFBboost_sincfilt_100mV < E_VDC_MAX_100mV) {
//		if (VdcFBboost_sincfilt_100mV > VDC_BOOST_START_100mV) {
		    vdc_inRange = true;
		}
	} else {
		shutdownDC();
		vdc_inRange = false;
		stateDC = INIT_DC;
	}

	if (sys_errcode != EC_NO_ERROR ) {
		shutdownDC();
		stateDC = INIT_DC;
	}

	cnt_rel++;
	cnt50Hz++;

	static uint32_t vdc_sum;
	vdc_sum += VdcFBboost_sincfilt_100mV;

	static float pdc_sum;
	pdc_sum += (Idc_filt_10mA*VdcFBboost_sincfilt_100mV)/1000.0;

	if (cnt50Hz >= (DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT) ) {
		cnt50Hz = 0;
		monitoring_request = true;
		monitor_vars.ID++;
		vdc_filt50Hz_100mV = vdc_sum/(DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT);
		vdc_sum = 0;
		pdc_filt50Hz = pdc_sum/(DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT);
		pdc_sum = 0;

		v_dc_filt50Hz = (float)vdc_filt50Hz_100mV/10.0;
		v_pv_filt50Hz = v_dc_filt50Hz * (1.0 - (float)mppTracker.duty_raw/MPPT_DUTY_ABSMAX);

		if (sys_mode_needs_battery) {
			bmsPower(1);
		} else {
			bmsPower(0);
		}

		static uint16_t cnt_1Hz_from_50Hz = 0;
		cnt_1Hz_from_50Hz++;
		if (cnt_1Hz_from_50Hz >= DC_CTRL_FREQ_MPPT ) {
			cnt_1Hz_from_50Hz = 0;
			bms_update_request = true;
		}
	}

	switch (stateDC) {
	  case INIT_DC:
		shutdownDC();
		nextState(WAIT_PV_VOLTAGE);
		break;

	  case WAIT_PV_VOLTAGE:
	  {
		if (vdc_inRange && cnt_rel >= 5*DC_CTRL_FREQ) {  // wait at least 5 sec to avoid instabilities
			dutyLS1 = 0;
			nextState(VOLTAGE_CONTROL);
		}
		break;
	  }
	  case VOLTAGE_CONTROL:
	  {  // curly braces to have scope for variable initialization
		gatedriverDC(1);

		if (cnt50Hz == 0) {
			if ( VdcFBboost_sincfilt_100mV < (v_dc_ref_100mV-VDC_TOLERANCE_100mV) ) {
				dutyLS1 += 2;
			} else if (VdcFBboost_sincfilt_100mV < v_dc_ref_100mV) {
				dutyLS1++;
			} else if ( VdcFBboost_sincfilt_100mV > (v_dc_ref_100mV+VDC_TOLERANCE_100mV/4) ) {
				dutyLS1 -= 2;
			}
			dutyLS1 = std::clamp(dutyLS1, (int16_t)0, (int16_t)MPPT_DUTY_ABSMAX);
		}

		if (   VdcFBboost_sincfilt_100mV > (v_dc_ref_100mV-VDC_TOLERANCE_100mV)
		    && VdcFBboost_sincfilt_100mV < (v_dc_ref_100mV+VDC_TOLERANCE_100mV)
		    && v_dc_filt50Hz*10          > (v_dc_ref_100mV-VDC_TOLERANCE_100mV)
		    && v_dc_filt50Hz*10          < (v_dc_ref_100mV+VDC_TOLERANCE_100mV)
			){

			if (sys_mode_needs_battery) {
				contactorBattery(1);
				batteryON = true;
				bms_battery_request = true;
				nextState(WAIT_CONTACTOR_DC);
			} else {
				mppTracker.duty_raw = dutyLS1;
				nextState(MPPT);
			}

		}
		break;
	  }
	  case WAIT_CONTACTOR_DC:
		//if (cnt_rel == 0.025*DC_CTRL_FREQ) {  // 25ms delay for contactor action
		if (cnt_rel == 0.2*DC_CTRL_FREQ) {  // 200ms delay for battery enable
			mppTracker.duty_raw = dutyLS1;
			nextState(MPPT);
		}
		break;

	  case MPPT:
		if ( VdcFBboost_sincfilt_100mV > E_VDC_MAX_MPPT_100mV ) {
			//dutyLS1 -= 0.15 * MPPT_DUTY_ABSMAX;  // triggers overvoltage fault
			dutyLS1 = 0;
			nextState(VOLTAGE_CONTROL);
		} else {
			if (cnt50Hz == 0) {
				mppt_calc_request = true;
			}

			if (mppt_calc_complete) {
				dutyLS1 = mppTracker.duty_raw;
				mppt_calc_complete = false;
			}
		}

		// todo
		// if no current comes from the PV part and in PV mode, turn off

		break;

      default:
          break;
      }

	  debug_duty = dutyLS1;
	  int16_t dutyB1 = MPPT_DUTY_ABSMAX - dutyLS1;

	  if (dutyB1 > ((int)MPPT_DUTY_ABSMAX-MIN_PULSE)) {
		  dutyB1 = MPPT_DUTY_ABSMAX;
	  } else if (dutyB1 < MIN_PULSE){
		  dutyB1 = 0;
	  }

	  GPIOC->BRR = (1<<4);  // reset Testpin TP201 PC4

	  return dutyB1;

//		  // A leg
//		  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, duty1);  //update pwm value
//
//		  // B leg
//		  int16_t duty2 = PWM_MAX_HALF+(PWM_MAX_HALF-duty1);
//		  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duty2);  //update pwm value
//		  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, duty2);  //update pwm value

}

errorPVBI_t checkDCLimits(){
	if ( VdcFBboost_sincfilt_100mV > E_VDC_MAX_100mV ) {
		return EC_V_DC_MAX_FB_BOOST;
	}
	return EC_NO_ERROR;
}


void measVdcFBboost()
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

	uint16_t sigma_delta_re = TIM4->CNT;
	//uint16_t sigma_delta_re = 250;  //for AC debugging todo remove
	TIM4->CNT = 0;

	if (sigma_delta_re < 40 || sigma_delta_re > 500) {
		sys_errcode = EC_V_DC_SENSOR_FB_BOOST;
	}

	uint16_t filt_in = sigma_delta_re;
	debug_sigma_delta_re = sigma_delta_re;

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
#define V_DC_MAX_FBboost (1+59)  // 68kOhm 450×68÷(450+68)
//#define V_DC_MAX_FBboost (1+450)  // todo 450 voltage divider

#if (E_VDC_MAX_100mV/10) > ((V_DC_MAX_FBboost*97)/100)
#error Choose E_VDC_MAX_100mV lower than FBboost sensor range
#endif
		// decim=16 ->                                   7bit(100milliVolt) + 12bit(CIC_GAIN) + 8bit(signal) = 27bit
		VdcFBboost_sincfilt_100mV = (V_DC_MAX_FBboost * ( (10             * ((CIC_GAIN*250-filt_out)/200) ))/CIC_GAIN);

		cnt_intr = 0;
	}

}
