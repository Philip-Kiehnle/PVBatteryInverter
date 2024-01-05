#include <math.h>

float pll_singlephase_step(float v);
float pll_get_w();
float pll_get_va();
float pll_get_vb();
float pll_get_vd();
float pll_get_vq();

// internal use
void sogi_step(float x);
void dq_step();
float pi_step(float x);

/* constants for PLL */
const float F_SAMPLE = 40e3;
const float T = 1/F_SAMPLE;
const float Kp = 88.857658;  // 10Hz Felix Kammerer
const float Ki = 3947.8417;  // 10Hz Felix Kammerer
// const float Kp = 44.4288;  // 10Hz Philip Kiehnle
// const float Ki = 986.9604;  // 10Hz Philip Kiehnle
const float wn = 50 * 2*M_PI;

/* Global Variables for PLL */
float w = 0;
float phi = 0;


/* Global Variables for SOGI */
float va = 0;
float vb = 0;
float wscale = T * wn;

/* Global Variables for DQ trafo */
float vd = 0;
float vq = 0;

/* Global Variables for PI controller */
float w_pictrl = 0;



// Three phase
//                 _______                     ___
//         w      |       | v1_a              |   |
//        ------> |       |-----------------> | + |
//                | SOGI1 |                   |   |          ________
//         v_a    |       | v1_b              |   |  va     |        |
//        ------> |       |----->     v2_b -> | - | ------> |        |
// v_ab  |        |_______|                   |___|         |        |
// ------|                                                  | DQ_PLL | -->
//       |         _______                     ___          |        |
//       | v_b    |       | v2_a              |   |  vb     |        |
//        ------> |       |-----------------> | + | ------> |        |
//                | SOGI2 |                   |   |         |________|
//         w      |       | v2_b              |   |
//        ------> |       |----->     v1_b -> | + |
//                |_______|                   |___|

//float pll_threephase_step(float v) {

// Single phase
//                 _______                 ________        _________
//         w      |       | va            |        |      |         |
//        ------> |       |-----> ------> |        |      |         |
//                | SOGI1 |               |        |  vq  | PI-ctrl | --> w_pictrl  ...---> phi
//         v      |       | vb            | DQ_PLL | ---> |         |
//        ------> |       |-----> ------> |        |      |_________|
//                |_______|               |________|             

float pll_singlephase_step(float v)
{

	sogi_step(v);
	dq_step();

	w_pictrl = pi_step(vq);
	w = wn + w_pictrl;

	phi = (phi + T*w);
	if ( phi > (2*M_PI) )
		phi -= (2*M_PI);  

	return phi;
}

float pll_get_w()
{
	return w;
}

float pll_get_va()
{
	return va;
}

float pll_get_vb()
{
	return vb;
}

float pll_get_vd()
{
	return vd;
}

float pll_get_vq()
{
	return vq;
}


typedef struct {
	float x1;
	float x2;
	float x3;
} thirdOrderIntegr;

float integrate(thirdOrderIntegr *integrState, float x)
{
	// source: https://imperix.com/doc/implementation/sogi-pll

	// V2: works better with one delay step removed
	integrState->x3 = integrState->x2;
	integrState->x2 = integrState->x1;
	integrState->x1 += (x * T) / 12;

	float y = 23*integrState->x1 - 16*integrState->x2 + 5*integrState->x3;

	// V1: original implementation according to source
	// integrState->x3 = integrState->x2;
	// integrState->x2 = integrState->x1;
	// integrState->x1 += (x * T) / 12;

	return y;
}


// SOGI

//          <------------------------------------------------------------------------------
//       __|__         _______        _____             _____                              |
//      |     |       |       |      |     |  wscale-> |     |          _____      _____   |
//      |  -  |  x2   |       |----> |  +  |           |     |         |     | v  |     |  |
//  x   |     |-----> | X 1.5 |      |     |---------> |  X  |-------> | 1/s |--->|z^-1 |-----> va
// ---> |  +  |       |       |      |  -  |           |     |         |_____|    |_____|  |
//      |_____|       |_______|      |_____|           |_____|                             |
//                                      ^                                                  |
//                                      |                             _____                |
//                                      |              _____         |     |               |
//                                      |   vb        |     |        |     | <-------------
//                                      |-------------| 1/s | <----- |  X  |
//                                      |             |_____|        |     | <- wscale
//                                      |                            |_____|
//                                      |
//                                      |
//                                       ---------------------------------------> vb
//
//             using backward euler integration; backward euler approximation: s = (z-1)/(z*T)

void sogi_step(float x)
{
	static float v = 0;
	float x2 = x - va;

	// simple integrator
	// v += wscale * (1.5*x2 - vb);
	// vb += wscale * va;


	// third order integrator
	static thirdOrderIntegr integrStateVA = {}, integrStateVB = {};
	v = integrate( &integrStateVA, wn * (1.5*x2 - vb) );
	vb = integrate( &integrStateVB, wn * va );

	va = v;
}


void dq_step()
{
	vd = cos(phi)*va + sin(phi)*vb;
	vq = -sin(phi)*va + cos(phi)*vb;
}


// tustin approximation: s = 2/T * (z-1)/(z+1)
// y(k) = y(k-1) + Kp * (x(k)-x(k-1)) + Ki * T/2 * (x(k)+x(k-1))
// y(k) = y(k-1) + x(k) * (Ki*T/2 + Kp) + x(k-1) * (Ki*T/2 - Kp)
float pi_step(float x)
{
	static float y = 0;
	static float x_prev = 0;

	y += x*(Ki*T/2 + Kp) + x_prev * (Ki*T/2 - Kp);

	x_prev = x;
	return y;
}
