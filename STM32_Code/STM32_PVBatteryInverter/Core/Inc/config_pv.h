#ifndef INC_CONFIG_PV_H
#define INC_CONFIG_PV_H

#include <config.h>
#include "mpptracker.hpp"

/*****************************************************************************/
/* configure PV system and PWM dutycycle parameters of microcontroller here: */
/*****************************************************************************/

// Jinko JKM405N-6RL3
// Voc(25°C)=46.3
// Vmpp(25°C)=36.48
// Voc(-20°C)=46,3×(1+45×0,28÷100)=52,13
// Vmpp(70°C)=46,3×(1+45×0,28÷100)=31,88
constexpr PV_module_t PVMODULE = PV_module_t{
	.v_mp = 36.48,
	.i_mp = 11.1,
	.coef_v_temp = -0.0028,  // The open-circuit voltage temperature coefficient of the module [%/K]
	.v_bypassDiode = 0.5,
	.nr_bypassDiodes = 3
};

constexpr MPPT_parameter_t MPPTPARAM = MPPT_parameter_t{
	.debug = false,
	.vin_min = 62,  // 2 modules * 31V
	.vin_max = 424,  // 8 modules * 53V
	.vout_min = 288,  // 96 Li-cells * 3.0
	.vout_max = 403,  // 96 Li-cells * 4.2
	.nr_pv_modules = 8,
	.mppt_freq = DC_CTRL_FREQ_MPPT/2,  // one cycle stabilisation, one cycle MPPT calculation
	.mppt_duty_absmax = DEF_MPPT_DUTY_ABSMAX,
    .mppt_duty_min_bootstrap = 0  // High side has isolated supply and no bootstrap capacitor
};

constexpr GMPPT_parameter_t GMPPTPARAM = GMPPT_parameter_t{
	.nr_bypassDiodes_search_per_interval = 6,
	.interval_glob_mppt_regular_sec = 10*60,  // 10 minutes
	.interval_glob_mppt_trig_event_sec = 2*60  // 2 minutes when power drops
};

// shutdown parameters for PV booster stage. current is used, because power is affected by AC 100Hz ripple and phase shifted Vdc measurement
//constexpr uint16_t PV_LOW_CURRENT_mA = 80;  // if PV netto input current into DC bus is lower, switchoff counter is increased
//constexpr uint16_t PV_LOW_CURRENT_SEC = 1;  // switch of after low power for this amount of seconds
//constexpr uint16_t PV_WAIT_SEC = 5*60;  // wait this amount of seconds until booster stage is started again; max 21 minutes
//debug
constexpr uint16_t PV_LOW_CURRENT_mA = 20;  // 20mA*350V = 7W
constexpr uint16_t PV_LOW_CURRENT_SEC = 30;
constexpr uint16_t PV_WAIT_SEC = 1*30;
//constexpr uint16_t PV_LOW_POWER = 2;  todo use dc current because power is affected by AC 100Hz ripple
//constexpr uint16_t PV_LOW_POWER_SEC = 5;
//constexpr uint16_t PV_WAIT_SEC = 20;


#endif /* INC_CONFIG_PV_H */
