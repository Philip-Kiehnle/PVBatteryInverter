#ifndef PICtrl_H
#define PICtrl_H


class PICtrl
{
private:

// tustin approximation: s = 2/T * (z-1)/(z+1)
// y(k) = y(k-1) + Kp * (x(k)-x(k-1)) + Ki * T/2 * (x(k)+x(k-1))
// y(k) = y(k-1) + x(k) * (Ki*T/2 + Kp) + x(k-1) * (Ki*T/2 - Kp)

    float x_prev;
    float integrator;
    float Kp;
    float q;  // q = Ki*T/2

    float y_min;
    float y_max;


public:
    PICtrl(float T, float kp, float ki);
    void step(float x, float y_min, float y_max);

    float y;

};


#endif
