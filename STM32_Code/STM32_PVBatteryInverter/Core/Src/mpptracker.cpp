#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <algorithm>
#include <cassert>
#include "mpptracker.hpp"


MPPTracker::MPPTracker(const mpptParams_t MPPTPARAMS, const pvModule_t PVMODULE)
{
    // d = 1-Vin/Vout
    int duty_min_tmp = MPPT_DUTY_ABSMAX - (MPPT_DUTY_ABSMAX*MPPTPARAMS.vin_max)/MPPTPARAMS.vout_min;
    int duty_max_tmp = MPPT_DUTY_ABSMAX - (MPPT_DUTY_ABSMAX*MPPTPARAMS.vin_min)/MPPTPARAMS.vout_max;

	const int DUTY_MAX = MPPT_DUTY_ABSMAX-MPPT_DUTY_MIN_BOOTSTRAP;

	duty_min = std::clamp(duty_min_tmp, (int)MPPT_DUTY_MIN_BOOTSTRAP, DUTY_MAX);
	duty_max = std::clamp(duty_max_tmp, (int)MPPT_DUTY_MIN_BOOTSTRAP, DUTY_MAX);

    assert((void("Invalid duty_min"), duty_min >= MPPT_DUTY_MIN_BOOTSTRAP));
	assert((void("Invalid duty_min/duty_max"), duty_min < duty_max));
    assert((void("Invalid duty_max"), duty_max <= MPPT_DUTY_ABSMAX));

	nr_bypassDiodes_total = MPPTPARAMS.nr_pv_modules * PVMODULE.nr_bypassDiodes;

	v_prev=0;
	p_prev=0;
	duty_raw=(duty_min+duty_max)/2;

	nr_substring_search_per_interval = MPPTPARAMS.nr_substring_search_per_interval;

	substring_v_mp = PVMODULE.v_mp/PVMODULE.nr_bypassDiodes;
	i_mp = PVMODULE.i_mp;
	v_bypassDiode = PVMODULE.v_bypassDiode;
    coef_v_temp = PVMODULE.coef_v_temp;
    substring_voltage_estim = (pv_temperature-25)*coef_v_temp*substring_v_mp;
}

void MPPTracker::nextMode(mppTrackerMode_t nextMode)
{
    modeRuntime = 0;
    mode_ = nextMode;
}

void MPPTracker::setDuty(int duty_raw_unclamped)
{
	duty_raw = std::clamp(duty_raw_unclamped, (int)duty_min, (int)duty_max);
}

// global + local MPP Tracking
void MPPTracker::step(float p, float v, float vdc)
{


	// global MPP Tracker using a priori information about the solar plant
	// advantages:
	//   less scan time -> higher efficiency
	//   system knowledge allows adjusted scan intervalls -> higher efficiency
	//   more power is extracted from the PV system -> cells run cooler and lifetime is improved: ToDo: check, because bypass diodes are used more often
	//   Improved Search Skip and judge looses 35% power during 1sec scan -> every 5min: 35%*1/(5*60) = 0.12% loss [article: Improved SSJ‑MPPT Method for Maximum Power Point Tracking of Photovoltaic Inverter Under Partial Shadow Condition]
	//   this method scans 6 bypass diodes in ~0.6sec and looses only 15% -> every 5min: 15%*0.6/(5*60) = 0.03% loss 
	// disadvantages:
	//   a priori knowledge necessary
	//   other influcences may degrade performance
	//
	// algorithm:
	//   Instead of a global MPP search, a so called extended search is used.
	//   The extended search changes the PV string voltage in multiples of a module substring voltage, e.g. 12V for 36V panel with 3 bypass diodes.
	//   A temperature estimation is used to adjust this value.
	//   The number of search steps in each execution interval can be limited according to the PV installation. Default of 6 is used, which equals to 2 standard PV panels.
	//   Every 10 minutes, a regular extended search is triggered.
	//   Every 2 minutes, an external event can trigger an extended search. This event can be a drop in PV power of 20% for example.
	//   If the PV current is below 15% of nominal current, e.g. 0.15*11.1A=1.67A no GMPP is tracked, because diffuse light only. This also prevents setting of voltages in discontinuous current mode.


	static int nr_bypassDiodes_test = 0;
	static float p_sum = 0;

	static unsigned int filt_cnt_1sec = 0;
	static float p_filt_1sec_sum = 0;
	static float p_filt_1sec = 0;
	static float p_filt_10sec = 0;
	static float p_filt_100sec = 0;

	p_filt_1sec_sum += p;
	filt_cnt_1sec++;

	if (filt_cnt_1sec >= MPPT_FREQ) {
		p_filt_1sec = p_filt_1sec_sum/MPPT_FREQ;
		p_filt_1sec_sum = 0;
		filt_cnt_1sec = 0;


		static unsigned int filt_cnt_10sec = 0;
		static float p_filt_10sec_sum = 0;

		p_filt_10sec_sum += p_filt_1sec;
		filt_cnt_10sec++;

		if (filt_cnt_10sec >= 10) {
			p_filt_10sec = p_filt_10sec_sum/10;
			p_filt_10sec_sum = 0;
			filt_cnt_10sec = 0;
		}

	}

	modeRuntime++;
	debug_printf("mode: %d\n", mode_);
    switch (mode_)
    {

    case MPP_localSearch:
		step_localMPPT(p, v);
		if (modeRuntime > 1.0*MPPT_FREQ) {
			nextMode(MPP_found);
		}
		break;

	case MPP_found:
	{
		step_localMPPT(p, v);
		bool trig = false;

		debug_printf("p_filt_1sec %f\n", p_filt_1sec);
		debug_printf("p_filt_10sec %f\n", p_filt_10sec);
		if ( p_filt_1sec < (0.8 * p_filt_10sec) ) {  // debug
		//if ( p_filt_10sec < (0.8 * p_filt_100sec) ) {  // production
			trig = true;
		}
#define ENABLE_GLOBAL_MPPT 0
#if ENABLE_GLOBAL_MPPT
		if (    ((p/v) > (0.15*i_mp))
			 && (    (modeRuntime > INTERVAL_GLOB_MPPT_REGULAR_SEC*MPPT_FREQ)
			      || ((modeRuntime > 9.5*MPPT_FREQ) && trig )  // debug
		     	//|| ((modeRuntime > interval_globMPP_trg_sec*freq) && trig )  // production
				)
		){
			nextMode(MPP_globalSearch);
			nr_bypassDiodes_test = 1;
			gmpp.p_avg = p;
			gmpp.v_start = v;
			gmpp.duty_raw = duty_raw;
		}
#endif
		break;
	}
	case MPP_globalSearch:
	{
	    float substring_voltage_estim = substring_v_mp * (1.0 + (pv_temperature-25)*coef_v_temp);
		uint16_t bpd = nr_bypassDiodes_test+nr_bypassDiodes_conducting;
		float v_pv_ref =  substring_voltage_estim*(nr_bypassDiodes_total - bpd) - v_bypassDiode * bpd;

		debug_printf("v_pv_ref = %f\n",v_pv_ref);
		
		// calculate dutycycle analytically
		// (1-d) * Vout = Vin
		// (1-d) = Vin/Vout
		// -> todo cope with discontinuous current mode (use current setting of duty cycle)
		float duty = 1.0 - v_pv_ref/vdc;
		//duty_raw_fix = (duty_raw + duty_raw_prev)/2;
		duty_raw = duty*MPPT_DUTY_ABSMAX;

#define STEPS_SHORT_SEARCH 4
		if (nr_bypassDiodes_test != 1) {
			float p_avg = (p_sum+p)/(STEPS_SHORT_SEARCH+1);
			p_sum = 0;
			if (p_avg > gmpp.p_avg) {
				gmpp.p_avg = p_avg;
				gmpp.duty_raw = duty_raw;
				if ( v < (gmpp.v_start - 0.9*nr_bypassDiodes_test*substring_voltage_estim)) {
					gmpp.nr_bypassDiodes_conducting = nr_bypassDiodes_test;
				}
			}
		}

		if (nr_bypassDiodes_test >= nr_substring_search_per_interval) {
			duty_raw = gmpp.duty_raw;
			nr_bypassDiodes_conducting = gmpp.nr_bypassDiodes_conducting;  // todo: when shading disappears, this value is not updated
			debug_printf("found GMPP with nr_bypassDiodes_conducting = %d\n",nr_bypassDiodes_conducting);
			nextMode(MPP_localSearch);
		} else {
			nextMode(MPP_shortLocalSearch);
		}
		nr_bypassDiodes_test++;  // todo: implement search in other direction (less bypassdiodes than before)
	
		break;
	}
	case MPP_shortLocalSearch:
		step_localMPPT(p, v);
		p_sum += p;
		if (modeRuntime >= STEPS_SHORT_SEARCH) {  // 4 steps local search + 1 step in global mode -> 6 steps for each bypass diode
			nextMode(MPP_globalSearch);
		}
		break;

    default:
        break;
    }
}


// MPP-Tracker with method of
// Incremental conductance (IC)
//
// dp/dv = 0, At MPP
// dp/dv > 0, Left of MPP
// dp/dv < 0, Right of MPP
void MPPTracker::step_localMPPT(float p, float v)
{
    float dv = v - v_prev;
	float dp = p - p_prev;
    int16_t dVpv = 0;
    uint32_t abs_duty_delta = 1;
	debug_printf("dv %f\n", dv);

	if (dv == 0) {
		if (dp > 0) {  // pos. infinity -> Left of MPP
	        dVpv = 1;  // increase Vpv
	    //} else if (di < 0) {  // zero does not change anything
		} else { // neg. infinity -> Right of MPP; zero keeps tracking
	        dVpv = -1;  // decrease Vpv
	    }
	} else {
		
		if (p != 0) {
			//decent = abs(dp/(p>>(MPPT_DUTY_BITS-3)));  // factor for power normalisation -3 for production
			//decent = abs(dp/(p>>(MPPT_DUTY_BITS-1)));  // factor for power normalisation -1 for simulation
			
			abs_duty_delta = abs( (dp * MPPT_DUTY_ABSMAX) / p );  // 4250 duty_absmax
			//abs_duty_delta = abs( (dp * MPPT_DUTY_ABSMAX/2) / p );  // 4250 duty_absmax  /2 for test


			debug_printf("abs_duty_delta %d\n", (int)abs_duty_delta);

			const uint16_t DUTY_DELTA_MAX  = (0.02*MPPT_DUTY_ABSMAX);  // max 2% duty change
			if (abs_duty_delta == 0) {
			    abs_duty_delta = 1;
		    } else if (abs_duty_delta > DUTY_DELTA_MAX) {
			    abs_duty_delta = DUTY_DELTA_MAX;
    			debug_printf("DUTY_DELTA_MAX!!!!!!!\n");
		    }
		}

	    if (dp/dv > 0) {  // Left of MPP
			dVpv = 1;  // increase Vpv
			debug_printf("increase Vpv %d\n", dVpv);
	    } else if (dp/dv < 0) {  // Right of MPP
			dVpv = -1;  // decrease Vpv
			debug_printf("decrease Vpv %d\n", dVpv);
        } else {
        	debug_printf("no change\n");
		}
	}


	int16_t duty_delta = -dVpv*abs_duty_delta;  // increase Vpv -> reduce dutycycle in boost converter
	debug_printf("dp/p = %f/%f   duty_delta=%d\n",dp,p,duty_delta);
    
    setDuty((int)duty_raw+duty_delta);
	v_prev = v;
	p_prev = p;
}
