// power controller
// controls smart meter (point of common coupling) to zero

//#include "Filter.hpp"
#include "PICtrl.hpp"
#include "power_controller.h"
#include "common.h"


int power_controller_step(int powerPCC)
{

    // control schematic
	// smart meter uses load power notation ( pos = power from grid)
	// inverter uses generator power notation ( pos = power to grid)
    //                 ___               _________                ___________
    //           0W   |   | err         |         |  P_inv_ref   |           |  P_inv
    //       -------> | + |----(*-1)--> | PI-ctrl |------------> | PT1-delay | --------
    //                |_-_|             |_________|              |___________|         |
    //                  ^                                                              |
    //                  | P_meter                                                      |
    //                  |                                                              |
    //                 ___                                                             |
    //   P_pv (neg)   |   |                                                            |
    //   -----------> | + |<-----------------------------------------------------------
    //                |___|
    //                  ^
    //                  |
    //                 P_load (pos)
    constexpr float TE = 1.0;                // 1sec execution interval determined by smart meter
    constexpr float T_inverter_delay = 0.1;  // 0.1sec modelled as PT1-delay
    constexpr float T_sigma = T_inverter_delay + TE;

    // magnitude optimum method (Betragsoptimum) // ToDo: check if valid, because T_inverter_delay > TA
    constexpr float kp = 0.5;  // no gain in plant -> 0.5
    constexpr float ki = 0.5 * 1/T_sigma;
    static PICtrl piCtrl(TE, kp, ki);

	piCtrl.step(-(0-powerPCC), P_AC_MIN, P_AC_MAX);

	int p_ref = piCtrl.y;

    return p_ref;
}
