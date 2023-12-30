
#include "stm32g4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#include "common.h"
#include "gpio.h"
#include "dc_control.h"

// defines for MPP tracking algorithm
#define I_ADCR 50
#define VIN_ADCR (450/1)

#define MPPT_PRECISION_BITS 16
#define MPPT_V_RANGE (6554)  // voltage range [0, +6553.6] 16bit in 100mV
#define MPPT_I_RANGE (655)  // current range [-327.68, +327.67] 16bit in 10mA

#define MPPT_VIN_MIN 62  // 2 modules * 31V
#define MPPT_VIN_MAX 320  // 8 modules * 40V
#define MPPT_VOUT_MIN 288  // 96 Li-cells * 3.0
#define MPPT_VOUT_MAX 384  // 96 Li-cells * 4.0

#define MPPT_DUTY_ABSMAX 4250
#define MPPT_DUTY_MIN_BOOTSTRAP 0  // High side has isolated supply and no bootstrap capacitor

// single PWM step has 1/170MHz = 5.88ns
// deadtime is configured to 64ns (10k resistor)
#define MIN_PULSE 18  // min pulse duration is 106ns - 64ns deadtime = 42ns
#include "mpptracker.h"

#define DC_CTRL_FREQ 20000
#define DC_CTRL_FREQ_MPPT 50

volatile uint16_t Vdc_sincfilt_100mV;
volatile int16_t Idc_filt_10mA;


extern volatile uint16_t debug_sigma_delta_re;

volatile enum stateDC_t stateDC = INIT;



void shutdownDC()
{
	gatedriverDC(0);
	contactorBattery(0);
}

uint32_t cnt_rel = 0;

static inline void nextState(enum stateDC_t state)
{
	stateDC = state;
	cnt_rel = 0;
}

int16_t dcControlStep()
{

	static int16_t dutyLS1 = 0;

	static MPPTracker mppTracker;

	bool vdc_inRange = false;

	if (Vdc_sincfilt_100mV > VDC_BOOST_STOP_100mV && Vdc_sincfilt_100mV < E_VDC_MAX_100mV) {
		if (Vdc_sincfilt_100mV > VDC_BOOST_START_100mV) {
		    vdc_inRange = true;
		}
	} else {
		shutdownDC();
		vdc_inRange = false;
		stateDC = INIT;
	}

	cnt_rel++;

	static uint16_t cnt50Hz;
	cnt50Hz++;

	uint16_t vdc_filt50Hz_100mV;
	static uint32_t vdc_sum;
	vdc_sum += Vdc_sincfilt_100mV;

	int16_t idc_filt50Hz_10mA;
	static int32_t idc_sum;
	idc_sum += Idc_filt_10mA;

	if (cnt50Hz >= (DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT) ) {
		cnt50Hz = 0;
		vdc_filt50Hz_100mV = vdc_sum/(DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT);
		vdc_sum = 0;
		idc_filt50Hz_10mA = idc_sum/(DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT);
		idc_sum = 0;
	}

	switch (stateDC) {
	  case INIT:
		shutdownDC();
		nextState(WAIT_PV_VOLTAGE);
		break;

	  case WAIT_PV_VOLTAGE:
		if (vdc_inRange && cnt_rel >= 5*DC_CTRL_FREQ) {  // wait at least 5 sec to avoid instabilities
			dutyLS1 = 0;
			nextState(VOLTAGE_CONTROL);
		}
		break;

	  case VOLTAGE_CONTROL:
		gatedriverDC(1);
		uint16_t vdc_ref_100mV = 2900;  // todo insert battery voltage

		if (cnt50Hz == 0) {
			if ( vdc_filt50Hz_100mV < (vdc_ref_100mV-VDC_TOLERANCE_100mV) ) {
				dutyLS1 += 2;
			} else if (vdc_filt50Hz_100mV < vdc_ref_100mV) {
				dutyLS1++;
			} else {
				dutyLS1--;
			}
		}


		if (   Vdc_sincfilt_100mV > (vdc_ref_100mV-VDC_TOLERANCE_100mV)
		    && Vdc_sincfilt_100mV < (vdc_ref_100mV+VDC_TOLERANCE_100mV)
			){

			contactorBattery(1);
			nextState(WAIT_CONTACTOR);
		}
		break;

	  case WAIT_CONTACTOR:
		if (cnt_rel == 0.025*DC_CTRL_FREQ) {  // 25ms delay for contactor action
			mppTracker.i_prev = idc_filt50Hz_10mA;
			mppTracker.v_prev = vdc_filt50Hz_100mV;
			mppTracker.duty_raw = dutyLS1;
			nextState(MPPT);
		}
		break;

	  case MPPT:
		if (cnt50Hz == 0) {
			GPIOC->BSRR = (1<<4);  // set Testpin TP201 PC4
			mppTracker_step(&mppTracker, vdc_filt50Hz_100mV, idc_filt50Hz_10mA);  // todo meas exec time; find better way for #defines in mppt
			dutyLS1 = mppTracker.duty_raw;
			GPIOC->BRR = (1<<4);  // reset Testpin TP201 PC4
		}
		break;

      default:
          break;
      }

	  int16_t dutyB1 = MPPT_DUTY_ABSMAX - dutyLS1;

	  if (dutyB1 > (MPPT_DUTY_ABSMAX-MIN_PULSE)) {
		  dutyB1 = MPPT_DUTY_ABSMAX;
	  } else if (dutyB1 < MIN_PULSE){
		  dutyB1 = 0;
	  }

	  return dutyB1;


//		  // A leg
//		  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3, duty1);  //update pwm value
//
//		  // B leg
//		  int16_t duty2 = PWM_MAX_HALF+(PWM_MAX_HALF-duty1);
//		  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, duty2);  //update pwm value
//		  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, duty2);  //update pwm value

}

int checkLimits(){
	if ( Vdc_sincfilt_100mV > E_VDC_MAX_100mV ) {
		return EC_V_DC_MAX;
	}
	return EC_NO_ERROR;
}


void measVdc()
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
	TIM4->CNT = 0;

	if (sigma_delta_re < 40 || sigma_delta_re > 500) {
		sys_errcode = EC_V_DC_SENSOR;
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
		// decim=16 ->                  7bit(100milliVolt) + 12bit(CIC_GAIN) + 8bit(signal) = 27bit
		// pos value range from 250 to 450 -> div by 200
		Vdc_sincfilt_100mV = (VIN_ADCR * ( (10             * ((CIC_GAIN*250-filt_out)/200) ))/CIC_GAIN);

		cnt_intr = 0;
	}

}
