#include <algorithm>

#include "PICtrl.hpp"


PICtrl::PICtrl(float T, float kp, float ki)
{
    Kp = kp;
    q = ki*T/2;
}

void PICtrl::step(float x, float y_min, float y_max)
{
	float kp_x = Kp * x;
	float ki_x = q * (x + x_prev);  // q = Ki*T/2

	y = kp_x + integrator + ki_x;

	// output and integrator saturation
	// This implementation respects the proportional part and clamps integrator to lower value.
	// Thus providing fast recovery.
	if (y > y_max) {
		y = y_max;
		// check how much of ki_x was needed to reach saturation
		if(x > 0) {
			ki_x = std::max(0.0f, y_max - (kp_x+integrator));  // todo: what happens if integrator is negative
		}
	} else if (y < y_min) {
		y = y_min;
		// check how much of ki_x was needed to reach saturation
		if(x < 0) {
			ki_x = std::min(0.0f, y_min - (kp_x+integrator));  // todo: what happens if integrator is negative
		}
	}

	integrator += ki_x;
	x_prev = x;
}

