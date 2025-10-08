// power controller
// controls smart meter (point of common coupling) to zero

//#include "Filter.hpp"
#include "PICtrl.hpp"
#include "power_controller.h"
#include "common.h"


int power_controller_step(int p_pcc_ref, int p_pcc, uint16_t p_ac_max)
{

	// control schematic
	// smart meter uses load power notation ( pos = power from grid)
	// inverter uses generator power notation ( pos = power to grid)
	//                 ___               _________                ___________
	// p_pcc_ref (0W) |   | err         |         |  p_inv_ref   |           |  p_inv
	//       -------> | + |----(*-1)--> | PI-ctrl |------------> | PT1-delay | --------
	//                |_-_|             |_________|              |___________|         |
	//                  ^                                                              |
	//                  | p_meter                                                      |
	//                  |                                                              |
	//                 ___                                                             |
	//   p_pv (neg)   |   |                                                            |
	//   -----------> | + |<-----------------------------------------------------------
	//                |___|
	//                  ^
	//                  |
	//                 p_load (pos)

	constexpr float TE = 1.0;                // 1sec execution interval determined by smart meter

	// V1: magnitude optimum method (Betragsoptimum) // ToDo: check if valid, because T_inverter_delay > TA
//	constexpr float T_inverter_delay = 0.1;  // 0.1sec modelled as PT1-delay
//	constexpr float T_sigma = T_inverter_delay + TE;
//	constexpr float kp = 0.5;  // no gain in plant -> 0.5
//	constexpr float ki = 0.5 * 1/T_sigma;

	// V2: faster PI-controller because overshoot is not critical
	constexpr float kp = 0.5;
	constexpr float ki = 0.6;
	// 100Watt load step, also seen in second interval because of delay. Todo: check if this assumption is valid
	// err 100W: p-part: 50W  i-part:60W         -> p_inv_ref=110Watt
	// err 100W: p-part: 50W  i-part:60W+60Watt  -> p_inv_ref=170Watt
	// err -10W: p-part: -5W  i-part:120W-3Watt  -> p_inv_ref=112Watt

	static PICtrl piCtrl(TE, kp, ki);
	piCtrl.step(-(p_pcc_ref-p_pcc), P_AC_MIN, p_ac_max);
	int p_ref = piCtrl.y;

	// V3: matched controller with Smith predictor
	// todo

	return p_ref;
}
