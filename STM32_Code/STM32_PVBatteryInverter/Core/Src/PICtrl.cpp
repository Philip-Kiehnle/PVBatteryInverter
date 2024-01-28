#include "PICtrl.hpp"



PICtrl::PICtrl(float T, float kp, float ki)
{
    c1 = ki*T/2 + kp;
    c2 = ki*T/2 - kp;
}

void PICtrl::step(float x, float y_min, float y_max)
{
    y += x * c1 + x_prev * c2;

	if (y > y_max) {
		y = y_max;
	} else if (y < y_min) {
		y = y_min;
	}
	x_prev = x;
}

