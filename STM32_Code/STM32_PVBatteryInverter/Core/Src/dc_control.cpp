
#include "stm32g4xx_hal.h"

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <algorithm>
#include <climits>


#include "common.h"
#include "monitoring.h"
#include "gpio.h"
#include "dc_control.h"
#include "ac_control.h"
#include "battery.h"
#include "mpptracker.hpp"
#include "PICtrl.hpp"
#include "sys_mode_controller.h"

#include "BatteryManagement/STW_mBMS.hpp"
extern STW_mBMS bms;


// single PWM step has 1/170MHz = 5.88ns -> center aligned PWM -> 11.8ns
// deadtime is configured to 64ns (10k resistor)
#define MIN_PULSE 9  // min pulse duration is 106ns - 64ns deadtime = 42ns

volatile bool sys_mode_needs_battery = false;
extern volatile uint16_t v_dc_ref_100mV;

volatile uint16_t debug_dutyHS;

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

constexpr mpptParams_t MPPTPARAMS = mpptParams_t{
	.debug = false,
	.vin_min = 62,  // 2 modules * 31V
	.vin_max = 424,  // 8 modules * 53V
	.vout_min = 288,  // 96 Li-cells * 3.0
	.vout_max = 403,  // 96 Li-cells * 4.2
	.nr_pv_modules = 8,
	.nr_bypassDiodes_search_per_interval = 6
};

// PV emulator:
// 2*Voc = 2*46.3 = 92.6V
// 2*Vmp = 2*36.48 = 73.0V
// 30%*Isc = 0.3*11.84 = 3.55A
// 30%*Imp = 0.3*11.1 = 3.33A

//constexpr mpptParams_t MPPTPARAMS = mpptParams_t{
//	.debug = false,
////	.vin_min = 0.33*31,  // 1/3 module * 31V
//	.vin_min = 0.2*31,  // 20% * Vmodule
//	.vin_max = 1*53,  // 1 module * 53V
//	.vout_min = 38,
//	.vout_max = 58,
//	.nr_pv_modules = 1,
//	.nr_bypassDiodes_search_per_interval = 1
//};


constexpr unsigned int MPPT_FREQ = DC_CTRL_FREQ_MPPT/2;  // one cycle stabilisation, one cycle MPPT calculation
constexpr unsigned int INTERVAL_GLOB_MPPT_REGULAR_SEC = 10*60;  // 10 minutes
constexpr unsigned int INTERVAL_GLOB_MPPT_TRIG_EVENT_SEC = 2*60;  // 2 minutes when power drops
//constexpr unsigned int INTERVAL_GLOB_MPPT_REGULAR_SEC = 60;  // 1 minute
//constexpr unsigned int INTERVAL_GLOB_MPPT_TRIG_EVENT_SEC = 30;  // 30 sec when power drops

constexpr unsigned int MPPT_DUTY_ABSMAX = DEF_MPPT_DUTY_ABSMAX;
constexpr unsigned int MPPT_DUTY_MIN_BOOTSTRAP = 0;  // High side has isolated supply and no bootstrap capacitor

// shutdown parameters for PV booster stage. current is used, because power is affected by AC 100Hz ripple and phase shifted Vdc measurement
//constexpr uint16_t PV_LOW_CURRENT_mA = 80;  // if PV netto input current into DC bus is lower, switchoff counter is increased
//constexpr uint16_t PV_LOW_CURRENT_SEC = 1;  // switch of after low power for this amount of seconds
//constexpr uint16_t PV_WAIT_SEC = 5*60;  // wait this amount of seconds until booster stage is started again; max 21 minutes
//debug
constexpr uint16_t PV_LOW_CURRENT_mA = 40;  // 40mA*50V = 2W
constexpr uint16_t PV_LOW_CURRENT_SEC = 10;
constexpr uint16_t PV_WAIT_SEC = 1*30;
//constexpr uint16_t PV_LOW_POWER = 2;  todo use dc current because power is affected by AC 100Hz ripple
//constexpr uint16_t PV_LOW_POWER_SEC = 5;
//constexpr uint16_t PV_WAIT_SEC = 20;


volatile uint16_t v_dc_FBboost_sincfilt_100mV;
volatile uint16_t v_dc_FBboost_filt50Hz_100mV;

volatile float p_dc_filt50Hz;
volatile float v_pv_filt50Hz;
volatile float v_dc_filt50Hz;
volatile float i_pv_filt50Hz;
volatile bool mppt_calc_request;
volatile bool mppt_calc_complete;
volatile bool bat_protect_calc_request;

MPPTracker mppTracker(MPPTPARAMS, PVMODULE);

volatile enum stateDC_t stateDC = INIT_DC;
volatile enum dcdc_mode_t dcdc_mode;

static volatile uint32_t cnt_rel = 0;

static uint16_t p_ac_bat_chg_reduction = 0;  // if PV power is to high for battery, feed into AC grid

uint16_t get_p_ac_bat_chg_reduction()
{
	return p_ac_bat_chg_reduction;
}

uint16_t get_p_ac_max_dc_lim()
{
	return (bms.batteryStatus.p_discharge_max+p_dc_filt50Hz);
}


uint16_t get_v_dc_FBboost_sincfilt_100mV()
{
	return v_dc_FBboost_sincfilt_100mV;
}


uint16_t get_v_dc_FBboost_filt50Hz_100mV()
{
	return v_dc_FBboost_filt50Hz_100mV;
}


float get_p_dc_filt50Hz()
{
	return p_dc_filt50Hz;
}


void fill_monitor_vars_dc(monitor_vars_t* mon_vars)
{
	mon_vars->stateDC = stateDC;
	mon_vars->dcdc_mode = dcdc_mode;
	mon_vars->dutyDC_HS = debug_dutyHS;
	mon_vars->p_dc_filt50Hz = p_dc_filt50Hz;
	mon_vars->v_dc_filt50Hz = v_dc_filt50Hz;

	mon_vars->v_dc_FBboost_sincfilt_100mV = v_dc_FBboost_sincfilt_100mV;
}


void calc_async_dc_control()
{
	if (mppt_calc_request) {
		mppTracker.step(p_dc_filt50Hz, v_pv_filt50Hz, v_dc_filt50Hz);
		mppt_calc_request = false;
		mppt_calc_complete = true;
	}

	if (bat_protect_calc_request) {

		bat_protect_calc_request = false;

		constexpr float TC = 0.02;  // 20ms execution interval

		/**********************************************/
		/* Battery power limit -> AC power controller */
		/**********************************************/
		constexpr float T_Pbat_sensor_delay = 0.02;  // 20ms battery power estimation meas delay modelled as PT1-delay
		constexpr float T_sigma_Pbat = T_Pbat_sensor_delay + TC;

		// magnitude optimum method (Betragsoptimum) // ToDo: check if valid
		constexpr float kp_Pbat = 0.5;  // no gain in plant -> 0.5
		constexpr float ki_Pbat = 0.5 * 1/T_sigma_Pbat;
		static PICtrl piCtrl_Pac_Pbat(TC, kp_Pbat, ki_Pbat);

		int p_bat_50Hz = p_dc_filt50Hz - get_p_ac_filt50Hz();

//		uint16_t p_limit;

//		if (bms.batteryStatus.power_W > 0) {
//			p_limit = bms.p_charge_max();
//		} else {
//			p_limit = bms.p_discharge_max();
//		}

		piCtrl_Pac_Pbat.step( (p_bat_50Hz - get_p_bat_chg_max()), 0, P_AC_MAX);

#if 0  // use if linear Vcell power limit in BMS code causes ringing
	    /**********************************************/
	    /* Battery Vcell limit -> AC power controller */
	    /**********************************************/
	    constexpr float T_Vcell_sensor_delay = 1.0;  // 1.0 sec battery cell voltage meas delay modelled as PT1-delay
	    constexpr float T_sigma_Vcell = T_Vcell_sensor_delay + TC;

	    // magnitude optimum method (Betragsoptimum) // ToDo: check if valid
	    constexpr float kp_Vcell = 0.5;  // no gain in plant -> 0.5
	    constexpr float ki_Vcell = 0.5 * 1/T_sigma_Vcell;
	    static PICtrl piCtrl_Pac_Vcell(TC, kp_Vcell, ki_Vcell);

	    // to reduce cell voltage by 1mV, Pbat has to be reduced Vbat*Ibat=Vbat*(Vcell/Rcell)
	    // e.g. 360Vbat 8mOhmCell 1mV reduction -> 360V*(1mV/8mOhm) = 45W
	    constexpr float PBAT_REDUCE_PER_MV = ((float)V_BAT_NOM) / bms.R_CELL_mOHM();

	    const batteryStatus_t* battery = get_batteryStatus();
		piCtrl_Pac_Vcell.step( PBAT_REDUCE_PER_MV*(battery->maxVcell_mV - bms.V_CELL_MAX_POWER_REDUCE_mV()), 0, P_AC_MAX);
#endif

		p_ac_bat_chg_reduction = piCtrl_Pac_Pbat.y;
	}
}


void shutdownDC()
{
	gatedriverDC(0);
	contactorBattery(0);
}


static inline void nextState(enum stateDC_t state)
{
	stateDC = state;
	cnt_rel = 0;
}


int16_t dcControlStep(uint16_t cnt20kHz_20ms, uint16_t v_dc_ref_100mV, int16_t i_dc_filt_10mA)
{
	//GPIOC->BSRR = (1<<4);  // set Testpin TP201 PC4

	static int16_t dutyLS1 = 0;

	// PI controller
	constexpr float TE = 1.0/50;   // 20ms execution interval
	constexpr float KP = 0.6;      // 0.6 dutycycle change per 100mV difference; 300 per 50V; 1÷(1−(300÷4250))×310V=334V
	constexpr float KI = 0.01/TE;  // 0.01 dutycycle change per 100mV difference per cycle; 300 per 50V for 60cycles (60/50=1.2sec)
	static PICtrl piCtrl(TE, KP, KI);

	bool vdc_inRange = false;

	// 50Hz more robust: for critical increase from 400V to 500V in 20ms: 0.5*4*390uF*(500^2-400^2)/0.02 = 3.5kW
	if (   v_dc_FBboost_filt50Hz_100mV > VDC_BOOST_STOP_LOW_100mV
		&& v_dc_FBboost_filt50Hz_100mV < (E_VDC_MAX_FB_BOOST_100mV-VDC_TOLERANCE_100mV)
#if SYSTEM_HAS_BATTERY == 1
		&& battery_maxVcell_OK()
#endif //SYSTEM_HAS_BATTERY
	   ) {
		if (v_dc_FBboost_filt50Hz_100mV > VDC_BOOST_START_100mV) {
//	if (VdcFBboost_sincfilt_100mV > VDC_BOOST_STOP_100mV && VdcFBboost_sincfilt_100mV < E_VDC_MAX_100mV) {
//		if (VdcFBboost_sincfilt_100mV > VDC_BOOST_START_100mV) {
		    vdc_inRange = true;
		}
	} else {
		shutdownDC();
		vdc_inRange = false;
		nextState(INIT_DC);
	}

	if (get_sys_errorcode() != EC_NO_ERROR ) {
		shutdownDC();
#if SYSTEM_HAS_BATTERY == 1
		battery_state_request(BAT_OFF);
#endif //SYSTEM_HAS_BATTERY
		nextState(INIT_DC);
	}

	cnt_rel++;

	static uint32_t v_dc_sum;
	static int32_t i_dc_sum_10mA;
	static int16_t i_dc_filt50Hz_mA;  // +-32A max
	static float p_dc_sum;
	static uint16_t pv_probe_timer50Hz = (PV_WAIT_SEC-1)*50;  // speedup first start

	p_dc_sum += (v_dc_FBboost_sincfilt_100mV * i_dc_filt_10mA)/1000.0;
	v_dc_sum += v_dc_FBboost_sincfilt_100mV;
	i_dc_sum_10mA += i_dc_filt_10mA;

	if (cnt20kHz_20ms == 0) {
		if (pv_probe_timer50Hz < USHRT_MAX) {
			pv_probe_timer50Hz++;
		}

		monitoring_request = true;

		if (sys_mode_needs_battery) {
			bmsPower(1);
		} else {
			bmsPower(0);
		}

		// todo: delay of sigma delta processing caused lagging Vdc meas when 100Hz ripple occurs -> use voltage estimator or extra Vdc meas
		v_dc_filt50Hz = (float)v_dc_FBboost_filt50Hz_100mV/10.0;
		v_pv_filt50Hz = v_dc_filt50Hz * (1.0 - (float)mppTracker.duty_raw/MPPT_DUTY_ABSMAX);
		i_pv_filt50Hz = p_dc_filt50Hz/v_pv_filt50Hz;

		p_dc_filt50Hz = p_dc_sum/CYCLES_cnt20kHz_20ms;
		p_dc_sum = 0;

		i_dc_filt50Hz_mA = i_dc_sum_10mA/(CYCLES_cnt20kHz_20ms/10);
		i_dc_sum_10mA = 0;

		v_dc_FBboost_filt50Hz_100mV = v_dc_sum/CYCLES_cnt20kHz_20ms;
		debug_v_dc_FBboost_sincfilt_100mV = v_dc_FBboost_filt50Hz_100mV;
		v_dc_sum = 0;
	} else if (cnt20kHz_20ms == 1) {  // at 0 MPPT is calculated
		bat_protect_calc_request = true;
	}

	switch (stateDC) {
	  case INIT_DC:
		shutdownDC();
		nextState(WAIT_PV_VOLTAGE);
		break;

	  case WAIT_PV_VOLTAGE:
	  {
		gatedriverDC(0);
		if (   vdc_inRange
			&& cnt_rel >= 5*DC_CTRL_FREQ  // wait at least 5 sec to avoid instabilities
			&& pv_probe_timer50Hz >= PV_WAIT_SEC*50  // probe pv current from time to time during night mode when battery holds DC voltage
		) {
			dutyLS1 = 0;
			piCtrl.y = 0;
			nextState(VOLTAGE_CONTROL);
		}
		break;
	  }
	  case VOLTAGE_CONTROL:
	  {  // curly braces to have scope for variable initialization
		gatedriverDC(1);

		if (cnt20kHz_20ms == 0) {
			constexpr float MAX_LS_DUTY = 1.0 - (MPPTPARAMS.vin_min/MPPTPARAMS.vout_max);
			int16_t err = v_dc_ref_100mV-v_dc_FBboost_sincfilt_100mV;
			piCtrl.step(err, 0, MPPT_DUTY_ABSMAX*(float)MAX_LS_DUTY);
			dutyLS1 = piCtrl.y;
		}

		if (   (  v_dc_FBboost_sincfilt_100mV > (v_dc_ref_100mV-VDC_TOLERANCE_100mV)
		       && v_dc_FBboost_sincfilt_100mV < (v_dc_ref_100mV+VDC_TOLERANCE_100mV)
		       && v_dc_filt50Hz*10            > (v_dc_ref_100mV-VDC_TOLERANCE_100mV)
		       && v_dc_filt50Hz*10            < (v_dc_ref_100mV+VDC_TOLERANCE_100mV))
		    || (SYS_MODE==PV2AC && stateAC == GRID_SYNC)
			){

#if SYSTEM_HAS_BATTERY == 1
			const batteryStatus_t* battery = get_batteryStatus();
			if (    sys_mode_needs_battery
				&& v_dc_filt50Hz*10 > (battery->voltage_100mV-VDC_TOLERANCE_100mV)
				&& v_dc_filt50Hz*10 < (battery->voltage_100mV+VDC_TOLERANCE_100mV)
				){
				// wait for negative contactor which is controlled by battery supervisor PCB
				if (cnt_rel > 10*DC_CTRL_FREQ) {  // 10sec delay
					contactorBattery(1);
					battery_state_request(BMS_ON__BAT_ON);
					nextState(WAIT_CONTACTOR_DC);
				}
			} else {
#endif //SYSTEM_HAS_BATTERY
				cnt_rel = 0;
				mppTracker.duty_raw = dutyLS1;
				nextState(MPPT);
#if SYSTEM_HAS_BATTERY == 1
			}
#endif //SYSTEM_HAS_BATTERY
		} else {
			cnt_rel = 0;
		}
		break;
	  }
	  case WAIT_CONTACTOR_DC:
		//if (cnt_rel == 0.025*DC_CTRL_FREQ) {  // 25ms delay for contactor action
		if (cnt_rel >= 0.2*DC_CTRL_FREQ) {  // 200ms delay for battery enable. Hangs, if battery not connecting
			mppTracker.duty_raw = dutyLS1;
			nextState(MPPT);
		}
		break;

	  case MPPT:
#if SYSTEM_HAS_BATTERY == 1
		if (sys_mode_needs_battery && !battery_connected()) {
			nextState(VOLTAGE_CONTROL);
			break;
		}
#endif //SYSTEM_HAS_BATTERY

		if ( v_dc_FBboost_sincfilt_100mV > VDC_MAX_MPPT_100mV ) {
			//dutyLS1 -= 0.15 * MPPT_DUTY_ABSMAX;  // triggers overvoltage fault
			dutyLS1 = 0;
			nextState(VOLTAGE_CONTROL);
		} else {
			if (cnt20kHz_20ms == 0) {
				// V1 : run MPP-Tracker in every cycle
				// problem: Pbat=12W Vpv=32.5V-38.2V; estimated V_MPP 37.xV -> MPP missmatch loss
				// problem: Pbat=4W Vpv=26.3V-35.2V; V_OC=39V -> MPP missmatch loss
				// lower boundary can happen because MPP algorithm count the energy which is extracted form input capacitors
				// mppt_calc_request = true;

				// V2 : run MPP-Tracker in every second cycle
				// advantage: new dutycycle can be stabilised in first cycle
				// Pbat=4W Vpv=30.3V-36.2V;
				static bool stabilize_MPP;
				if (stabilize_MPP) {
					stabilize_MPP = false;
				} else {
					stabilize_MPP = true;
					mppt_calc_request = true;
				}

				// check_low_power todo check for other states also -> shift outside MPP case
				static uint16_t cnt_pv_low_current = 0;
				if (i_dc_filt50Hz_mA < PV_LOW_CURRENT_mA) {
					cnt_pv_low_current++;
					if (cnt_pv_low_current == 50*PV_LOW_CURRENT_SEC) {
						nextState(WAIT_PV_VOLTAGE);
						pv_probe_timer50Hz = 0;
						cnt_pv_low_current = 0;
					}
				} else {
					cnt_pv_low_current = 0;
				}
			}

			if (mppt_calc_complete) {
				dutyLS1 = mppTracker.duty_raw;
				mppt_calc_complete = false;
			}
		}

		break;

      default:
          break;
      }


//	GaN Booster Interleaved Mode for full PV panel current
//	25°C 50mOhm
//	50°C 60mOhm
//	70°C 72mOhm
//	115°C 100mOhm
//	max PV input current 12A
//	each HB 100mOhm * 6A^2 = 3,6W
//	switching loss assumption: 20kHz -> 1 W for each HB
//	-> 2,3 W maximum loss for each transistor
//
//	Switching to interleaved mode at 5A PV input current:
//	Ploss_1HB = 80mOhm * 5A^2 + 1W = 2W + 1W = 3W                 (/2 -> 1.5W each GaN)
//	Ploss_2HB = 2 * 70mOhm * 2,5A^2  + 2*1W = 0.875W+2W = 2,875W  (/4 -> 0.72W each GaN)
//	-> more loss in single HB mode
//
//	Switching back to single mode at 4A (see code)
//	Ploss_1HB = 75mOhm * 4A^2 + 1W = 1,2W+1W = 2,2W               (/2 -> 1.1W each GaN)
//	Ploss_2HB = 2 * 65mOhm * 2,0A^2 + 2*1W = 0.52W+2W = 2,52W     (/4 -> 0.63W each GaN)
//	-> more loss in interleaved mode

	  static uint16_t cnt_interleaved_mode = 0;

	  if (dcdc_mode == DCDC_INTERLEAVED) {
		  cnt_interleaved_mode++;
	  } else {
		  cnt_interleaved_mode = 0;
	  }

	  static dcdc_mode_t hb_prev;

	  // no discontinuous mode can occur, because of high currents
	  if ( i_pv_filt50Hz > 5.0 ) {
		  if (dcdc_mode != DCDC_INTERLEAVED) {
			  dcdc_mode = DCDC_INTERLEAVED;
		  }
	  } else if ( (i_pv_filt50Hz < 4.0) && (cnt_interleaved_mode >= 1.0*DC_CTRL_FREQ) ) {  // min 1sec in interleaved mode
		  if (dcdc_mode != DCDC_HB1 && dcdc_mode != DCDC_HB2) {
			  if ( hb_prev == DCDC_HB1 ) {  // equal distribution between both halfbridges
				  dcdc_mode = DCDC_HB2;
				  hb_prev = DCDC_HB2;
			  } else {
				  dcdc_mode = DCDC_HB1;
				  hb_prev = DCDC_HB1;
			  }
	  	  }
	  }

	  int16_t dutyB1 = MPPT_DUTY_ABSMAX - dutyLS1;

	  if (dutyB1 > ((int)MPPT_DUTY_ABSMAX-MIN_PULSE)) {
		  dutyB1 = MPPT_DUTY_ABSMAX;
	  } else if (dutyB1 < MIN_PULSE){
		  dutyB1 = 0;
	  }

	  debug_dutyHS = dutyB1;


	  //GPIOC->BRR = (1<<4);  // reset Testpin TP201 PC4
	// from function enter to here
	// 21.04.2024: with debug:-g3  -O3     <12.75us for 2min
	// 21.04.2024: with debug:none -Ofast  < 8.83us for 2min
	// 21.04.2024: with debug:-g   -Ofast  < 8.84us for 2min (minimal different main loop)
	// 21.04.2024: with debug:-g3  -Ofast  < 8.55us for 2min (minimal different main loop)

	  return dutyB1;
}

errorPVBI_t checkDCLimits()
{
	if ( v_dc_FBboost_sincfilt_100mV > E_VDC_MAX_FB_BOOST_100mV ) {
		return EC_V_DC_MAX_FB_BOOST;
	}

#if SYSTEM_HAS_BATTERY == 1
	// check if sum of cell voltages deviates from DC bus voltage e.g. +-10Volt (~0.1V per Cell)
	else if ( battery_connected() && !battery_bus_voltage_match_coarse() ) {
		return EC_V_BUS_V_BATTERY_DEVIATION;
	}
#endif //SYSTEM_HAS_BATTERY

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

	// if sensor sees more than 1V, 90% high increase up to only a single zero in 128cycles, which equals 1.25V
	if (sigma_delta_re < 32 || sigma_delta_re > 500) {
		set_sys_errorcode(EC_V_DC_SENSOR_FB_BOOST);
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
//#define V_DC_MAX_FBboost (1+59)  // 68kOhm 450×68÷(450+68)
//#define V_DC_CALIB_FBboost  995  // per mil for 68kOhm 450×68÷(450+68)

#define V_DC_MAX_FBboost (1+375)  // 375k voltage divider; 4.1Vcell*96=393.6V -> Vadc=1.05V (40edges); 4.25Vcell*96=408V -> Vadc=1.088V (32edges)
#define V_DC_CALIB_FBboost  995  // 310.1V is shown as 310.3V

#ifndef E_VDC_MAX_FB_BOOST_100mV
#error "Define E_VDC_MAX_FB_BOOST_100mV"
#endif

#if (E_VDC_MAX_FB_BOOST_100mV/10) > ((V_DC_MAX_FBboost*110)/100)  // 125% is abs max
#error Choose E_VDC_MAX_FB_BOOST_100mV lower than FBboost sensor range
#endif

		// decim=16 ->                                   7bit(100milliVolt) + 12bit(CIC_GAIN) + 8bit(signal) = 27bit
		uint32_t vdc_no_calib = (V_DC_MAX_FBboost * ( (10             * ((CIC_GAIN*250-filt_out)/200) ))/CIC_GAIN);
		v_dc_FBboost_sincfilt_100mV = (vdc_no_calib*V_DC_CALIB_FBboost)/1000;

		cnt_intr = 0;
	}

}
