#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "controller.h"

#define FGRID 50
#define L (10.2e-3)  // see calc sheet
#define ZL (2*M_PI*FGRID*L)
#define T (1/20e3)

#define L_LINEAR 0      // linear inductor
#define L_SATURATING 1  // nonlinear inductor
#define L_REMANENT 2    // nonlinear inductor with remanence

#define INDUCTOR L_SATURATING


//#define EXTEND 30
//
//const int L_EXT = L * (1<<EXTEND);
//const int R_EXT = R * (1<<EXTEND);

int32_t get_VGRID_TRATIO()
{
	return VGRID_TRATIO;
}

float get_C()
{
	return CAPACITANCE;
}

//int32_t get_ADC_BITS()
//{
//	return ADC_BITS;
//}

int32_t get_VIN_ADCR()
{
	return VIN_ADCR;
}

int32_t get_VGRID_ADCR()
{
	return VGRID_ADCR;
}

int32_t get_IGRID_ADCR()
{
	return IGRID_ADCR;
}

// generic PI controller:
// tustin approximation: s = 2/T * (z-1)/(z+1)
// y(k) = y(k-1) + Kp * (x(k)-x(k-1)) + Ki * T/2 * (x(k)+x(k-1))
// y(k) = y(k-1) + x(k) * (Ki*T/2 + Kp) + x(k-1) * (Ki*T/2 - Kp)
void pi_step(int32_t x, piController *ctrl)
{
	ctrl->y += x * ctrl->c1 + ctrl->x_prev * ctrl->c2;
	if (ctrl->y > ctrl->y_max) {
		ctrl->y = ctrl->y_max;
	} else if (ctrl->y < ctrl->y_min) {
		ctrl->y = ctrl->y_min;
	}
	ctrl->x_prev = x;
}


// PI voltage controller for DC-link capacitor
//
// model predictive current controller as PT1 element
//#define Tsigma (3*T)
#define Tsigma (3*T + 0.005)  // instead of 100Hz filter, increase Tsigma
//#define Tsigma (3*T + 0.01)  // instead of 100Hz filter, increase Tsigma 43ms rise with a=3  76ms rise with a=6
#define EXTEND_PI_VDC 16

// Go(s) = 1/C * 1/s * (Kp+Ki/s) * 1/(1+Tsigma*s)
// symmetric optimum:
#define a 2.0  // a>1; typical value is 2;
//#define a 3.0  // a>1; typical value is 2;
#define VR (CAPACITANCE / (Tsigma*a))
const int Kp_Vdc = 1.0*VR * (1<<EXTEND_PI_VDC);
#define Tn (a*a * Tsigma)
const int Ki_Vdc = 1.0*VR/Tn * (1<<EXTEND_PI_VDC);

volatile int32_t debug_vdc_comp = 0;

volatile bool current_increasing;


// damped PR controller for grid current

// from flexQgrid-Hybrid-MMC/Matlab-Simulink/DQ-_vs_PR-controller/controller_design.m
// CAUTION: Matlab array indexing starts at 1!
// y[k] = cx(1)*x[k] + cx(2)*x[k-1] + cx(3)*x[k-2] + cy(1)*y[k-1] + cy(2)*y[k-2]

#define TA T
#define w0 2*M_PI*50
#define wc 2*M_PI*1.0
#define T_sigma ( 1.6e-06 + 2.0*TA )  //Tmeas_Iarm + ctrl.i.ac.TA
#define L_pr (0.1*L)  // inductor saturation
#define Kp (L_pr/(2*T_sigma))
#define Ki (R/(2*T_sigma))

#define EXTEND_BITS 12

// V1:
int cx[3] = { 	(1<<EXTEND_BITS)*	(Kp + (4*Ki*TA*wc)/(pow(TA*w0,2) + 4*wc*TA + 4)),
				(1<<EXTEND_BITS)*	(2*Kp - (16*Kp + 8*Kp*TA*wc)/(pow(TA*w0,2) + 4*wc*TA + 4)),
				(1<<EXTEND_BITS)*	(Kp - (4*Ki*TA*wc + 8*Kp*TA*wc)/(pow(TA*w0,2) + 4*wc*TA + 4))
};

int cy[2] = {	(1<<EXTEND_BITS)*  (-(2*pow(TA*w0,2) - 8)/(pow(TA*w0,2) + 4*wc*TA + 4)),
				(1<<EXTEND_BITS)*	(1 - (2*pow(TA*w0,2) + 8)/(pow(TA*w0,2) + 4*wc*TA + 4))
};

// V2: https://imperix.com/doc/implementation/proportional-resonant-controller
int a1 = (1<<EXTEND_BITS)* (4*Ki*TA*wc);
int b0 = (1<<EXTEND_BITS)* (pow(TA*w0,2) + (4*TA*wc) + 4);
int b1 = (1<<EXTEND_BITS)* (2*pow(TA*w0,2) - 8);
int b2 = (1<<EXTEND_BITS)* (pow(TA*w0,2) - (4*TA*wc) + 4);

int pr_step(int x)
{
	static int x_prev[2];
	static int y_prev[2];

	// V1:
	int y = (cx[0]*x + cx[1]*x_prev[0] + cx[2]*x_prev[1] + cy[0]*y_prev[0] + cy[1]*y_prev[1])>>EXTEND_BITS;

	// offset compensation
	static int16_t v_offset_comp = 0;
	static uint32_t cnt=0;
	static int32_t i_sum = 0;
	cnt++;
	i_sum += x;
#define CNT_MAX (20000/50)
	if (cnt == CNT_MAX) {
		int i_avg_offset = i_sum/CNT_MAX;
		v_offset_comp += i_avg_offset;
		i_sum = 0;
		cnt = 0;
	}


	// V2: https://imperix.com/doc/implementation/proportional-resonant-controller
	//int y = (a1*x - a1*x_prev[1] - b1*y_prev[0] - b2*y_prev[1])/b0;

	y_prev[1] = y_prev[0];
	x_prev[1] = x_prev[0];

	y_prev[0] = y;
	x_prev[0] = x;

	return y+v_offset_comp;
}


static uint16_t lut_pos_i = 0;

#if INDUCTOR == L_SATURATING
#define FIXP_SCALE_BITS 14
uint16_t Ztot_LUT[105] = {
54286,54286,54286,54286,54286,54286,53867,53447,53028,52609,  // 0.0A, 0.1A, ...
52191,51772,51354,50936,50518,50101,49684,49267,48851,48435,
48019,47603,47188,46773,46359,45945,45531,45118,44705,44292,
43880,43468,43057,42646,42235,41826,41416,41007,40599,40191,
39784,39377,38971,38565,38160,37756,37352,36949,36547,36146,
35745,35345,34946,34548,34150,33754,33358,32964,32570,32178,
31786,31396,31007,30619,30232,29846,29462,29080,28698,28319,
27941,27564,27189,26816,26445,26076,25709,25344,24981,24621,
24263,24198,24133,24069,24004,23940,23876,23811,23747,23683,
23619,23555,23492,23428,23364,23301,23238,23174,23111,23048,
22985,22985,22985,22985,22985  // ... 10.4A
};
int16_t ZLphase_LUT[105] = {
6850,6850,6850,6850,6850,6850,6840,6829,6818,6806,  // 0.0A, 0.1A, ...
6795,6783,6772,6760,6748,6735,6723,6710,6697,6684,
6670,6657,6643,6629,6614,6600,6585,6569,6554,6538,
6522,6506,6489,6472,6455,6437,6419,6400,6382,6362,
6343,6323,6303,6282,6261,6239,6217,6194,6171,6147,
6123,6098,6073,6047,6021,5994,5966,5937,5908,5878,
5848,5817,5784,5752,5718,5683,5648,5611,5574,5535,
5496,5455,5413,5371,5326,5281,5235,5187,5137,5086,
5034,5024,5015,5005,4995,4985,4975,4965,4955,4945,
4935,4925,4915,4904,4894,4884,4873,4863,4852,4841,
4831,4831,4831,4831,4831  // ... 10.4A
};

#elif INDUCTOR == L_REMANENT
#define FIXP_SCALE_BITS 13
uint16_t Ztot_iINCR_LUT[105] = {
34876,36341,37784,39136,40326,41284,41954,42296,42287,41930,  // 0.0A, 0.1A, ...
41249,40288,39106,37772,36356,34928,33544,32247,31067,30013,
29085,28266,27536,26869,26240,25627,25015,24394,23761,23120,
22477,21843,21228,20643,20094,19589,19130,18718,18350,18023,
17733,17473,17240,17027,16831,16647,16473,16305,16143,15985,
15829,15676,15524,15373,15223,15074,14927,14779,14633,14488,
14343,14199,14056,13913,13772,13631,13491,13352,13214,13076,
12940,12804,12669,12535,12401,12269,12137,12007,11877,11748,
11620,11493,11367,11242,11118,10995,10872,10751,10631,10512,
10393,10276,10160,10045,9931,9818,9706,9596,9486,9378,
9271,9213,9156,9100,9044  // ... 10.4A
};
int16_t ZLphase_iINCR_LUT[105] = {
7333,7368,7399,7427,7450,7467,7479,7484,7484,7478,  // 0.0A, 0.1A, ...
7466,7449,7426,7399,7368,7334,7298,7262,7226,7192,
7160,7130,7101,7073,7046,7018,6989,6958,6924,6889,
6850,6811,6770,6728,6687,6647,6609,6573,6539,6508,
6480,6453,6429,6406,6384,6364,6343,6324,6304,6284,
6265,6245,6225,6205,6184,6163,6142,6120,6099,6076,
6054,6031,6007,5984,5959,5935,5910,5884,5858,5832,
5805,5778,5750,5722,5693,5664,5634,5604,5573,5541,
5509,5477,5444,5410,5375,5340,5304,5268,5231,5193,
5155,5115,5075,5035,4993,4951,4908,4864,4819,4773,
4727,4701,4675,4649,4623  // ... 10.4A
};
uint16_t Ztot_iDECR_LUT[105] = {
34876,33578,32396,31341,30411,29591,28860,28191,27561,26947,  // 0.0A, 0.1A, ...
26333,25710,25075,24432,23787,23150,22533,21945,21394,20886,
20424,20009,19639,19310,19017,18756,18520,18306,18108,17922,
17746,17578,17414,17254,17097,16941,16788,16635,16484,16333,
16183,16034,15886,15739,15592,15446,15301,15156,15012,14869,
14727,14585,14444,14304,14165,14026,13888,13751,13615,13480,
13345,13211,13078,12946,12814,12684,12554,12425,12297,12169,
12043,11918,11793,11669,11546,11424,11303,11183,11064,10945,
10828,10712,10596,10482,10368,10256,10144,10034,9924,9816,
9709,9602,9497,9393,9290,9188,9087,8988,8889,8792,
8696,8647,8599,8552,8505  // ... 10.4A
};
int16_t ZLphase_iDECR_LUT[105] = {
7333,7299,7266,7235,7205,7178,7152,7127,7102,7077,  // 0.0A, 0.1A, ...
7050,7022,6992,6960,6926,6890,6854,6817,6781,6746,
6712,6681,6651,6624,6599,6576,6555,6535,6517,6499,
6481,6464,6447,6430,6414,6397,6379,6362,6345,6327,
6309,6290,6272,6253,6234,6214,6195,6175,6154,6134,
6113,6091,6070,6048,6025,6002,5979,5956,5932,5908,
5883,5858,5832,5806,5780,5753,5726,5698,5670,5641,
5612,5583,5552,5522,5491,5459,5426,5393,5360,5326,
5291,5256,5220,5183,5146,5108,5070,5030,4990,4950,
4908,4866,4823,4780,4735,4690,4644,4597,4549,4500,
4451,4425,4400,4374,4348  // ... 10.4A
};
#endif

int16_t get_IacPhase()
{
#if INDUCTOR == L_LINEAR
	return (int16_t)( (1<<15) * atan(ZL/R) / (2*M_PI));  // calculated at compile time

#elif INDUCTOR == L_SATURATING
	return ZLphase_LUT[lut_pos_i];

#elif INDUCTOR == L_REMANENT
	if (current_increasing) {
		return ZLphase_iINCR_LUT[lut_pos_i];
	} else {
		return ZLphase_iDECR_LUT[lut_pos_i];
	}
#endif
}


int16_t calc_IacAmp2VacSecAmpDCscale(int32_t i_amp)  // returns amplitude at secondary side
{
	#define Ztot sqrt(R*R + ZL*ZL)
	const int32_t coeff = Ztot*(1<<14);  // DC scale
	int32_t v_amp = (coeff*i_amp)>>14;
	return v_amp;
}


uint16_t calc_v_amp_pred(uint32_t i_amp, int32_t i_ac_100mA)
{
	int32_t v_amp;
#if INDUCTOR == L_SATURATING
	if (i_ac_100mA < 0)
		i_ac_100mA *= -1;

	// 6A max -> 6×sin(2×π×50×1÷20000) = 94mA
	if (i_ac_100mA > lut_pos_i && lut_pos_i < sizeof(Ztot_LUT)/sizeof(Ztot_LUT[0])-2 ) {  // -2 for possible alignment issues
		lut_pos_i++;
	} else if (lut_pos_i>0) {
		lut_pos_i--;
	}

	v_amp = (Ztot_LUT[lut_pos_i]*i_amp)>>FIXP_SCALE_BITS;

#elif INDUCTOR == L_REMANENT
	static int32_t i_ac_100mA_prev;
	if (i_ac_100mA > i_ac_100mA_prev) {
		current_increasing = true;
	} else {
		current_increasing = false;
	}
	i_ac_100mA_prev = i_ac_100mA;

	if (i_ac_100mA < 0) {
		i_ac_100mA *= -1;
	}

	// 6A max -> 6×sin(2×π×50×1÷20000) = 94mA
	if (i_ac_100mA > lut_pos_i && lut_pos_i < sizeof(Ztot_iINCR_LUT)/sizeof(Ztot_iINCR_LUT[0])-2 ) {  // -2 for possible alignment issues
		lut_pos_i++;
	} else if (lut_pos_i>0) {
		lut_pos_i--;
	}

	// todo: smooth switch if sign of decent changes, also in phase calculation.
	//       But voltage is low in this moment, so may not be a problem.
	if (   (current_increasing && i_ac_100mA >= 0)
	    || (!current_increasing && i_ac_100mA < 0)
	) {
		v_amp = (Ztot_iINCR_LUT[lut_pos_i]*i_amp)>>FIXP_SCALE_BITS;
	} else {
		v_amp = (Ztot_iDECR_LUT[lut_pos_i]*i_amp)>>FIXP_SCALE_BITS;
	}

#endif
	return v_amp;
}


// scale i_dc to i_ac_rms to i_ac_amp
piController piCtrl = { .y=0, .y_min=0, .y_max=IAC_AMP_MAX_10mA*(1<<EXTEND_PI_VDC), .x_prev=0,  // PV to grid
//piController piCtrl = { .y=0, .y_min=-IAC_AMP_MAX_RAW*(1<<EXTEND_PI_VDC), .y_max=0, .x_prev=0,  // Grid to battery
//piController piCtrl = { .y=0, .y_min=-IAC_AMP_MAX_10mA*(1<<EXTEND_PI_VDC), .y_max=IAC_AMP_MAX_10mA*(1<<EXTEND_PI_VDC), .x_prev=0,  // bidi
								.c1=sqrt(2)*(Ki_Vdc*T/2 + Kp_Vdc),
								.c2=sqrt(2)*(Ki_Vdc*T/2 - Kp_Vdc) };


int16_t step_pi_Vdc2IacAmp(int32_t vdc_ref, int32_t vdc)
{
	pi_step((vdc-vdc_ref), &piCtrl);  // no ripple comp

	return (piCtrl.y>>EXTEND_PI_VDC);
}

int32_t get_vdc_comp()
{
	return debug_vdc_comp;
}


// predictive current controller for RL-load
//          _____        _____
//     ----|_____|------|_____|----
//    |       R            L       |
//    V                            |
//    |                            |
//    –                            –
//

// int32_t step_predict_i(int32_t i_ref, int32_t i)
// {
// 	int64_t v = (L_EXT/T)*((int64_t)i_ref-(int64_t)i) + R_EXT*(int64_t)i;
// 	return (int32_t)(v>>EXTEND);
// }


// *****************
// sine lookup table
// *****************

/*
 * The mask: all bit belonging to the table
 * are 1, the all above 0.
 */
//#define TABLE_BITS  (5)  //"5 bit" large table = 32+1 values. 
#define TABLE_BITS  (7)  //"7 bit" large table = 128+1 values. 
#define TABLE_SIZE  (1<<TABLE_BITS)
#define TABLE_MASK  (TABLE_SIZE-1)

/**
 * "5 bit" lookup table for the offsets. These are the sines for exactly
 * at 0deg, 11.25deg, 22.5deg etc. The values are from -1 to 1 in Q15.
 */
// static int16_t SIN90[TABLE_SIZE+1] = {
//   0x0000,0x0647,0x0c8b,0x12c7,0x18f8,0x1f19,0x2527,0x2b1e,
//   0x30fb,0x36b9,0x3c56,0x41cd,0x471c,0x4c3f,0x5133,0x55f4,
//   0x5a81,0x5ed6,0x62f1,0x66ce,0x6a6c,0x6dc9,0x70e1,0x73b5,
//   0x7640,0x7883,0x7a7c,0x7c29,0x7d89,0x7e9c,0x7f61,0x7fd7,
//   0x7fff
// };

// --> quarter wave sine lookup table -> 512 phase increments -> 9bit total phase
static int16_t SIN90[TABLE_SIZE+1] = {
0, 402, 804, 1206, 1607, 2009, 2410, 2811, 3211, 3611, 
4011, 4409, 4807, 5205, 5601, 5997, 6392, 6786, 7179, 7571, 
7961, 8351, 8739, 9126, 9511, 9895, 10278, 10659, 11038, 11416, 
11792, 12166, 12539, 12909, 13278, 13645, 14009, 14372, 14732, 15090, 
15446, 15799, 16150, 16499, 16845, 17189, 17530, 17868, 18204, 18537, 
18867, 19194, 19519, 19840, 20159, 20474, 20787, 21096, 21402, 21705, 
22004, 22301, 22594, 22883, 23169, 23452, 23731, 24006, 24278, 24546, 
24811, 25072, 25329, 25582, 25831, 26077, 26318, 26556, 26789, 27019, 
27244, 27466, 27683, 27896, 28105, 28309, 28510, 28706, 28897, 29085, 
29268, 29446, 29621, 29790, 29955, 30116, 30272, 30424, 30571, 30713, 
30851, 30984, 31113, 31236, 31356, 31470, 31580, 31684, 31785, 31880, 
31970, 32056, 32137, 32213, 32284, 32350, 32412, 32468, 32520, 32567, 
32609, 32646, 32678, 32705, 32727, 32744, 32757, 32764, 32767
};

/*
 * The lookup table is to 90DEG, the input can be -360 to 360 DEG, where negative
 * values are transformed to positive before further processing. We need two
 * additional bits (*4) to represent 360 DEG:
 */
#define LOOKUP_BITS (TABLE_BITS+2)
#define LOOKUP_MASK ((1<<LOOKUP_BITS)-1)
#define FLIP_BIT    (1<<TABLE_BITS)
#define NEGATE_BIT  (1<<(TABLE_BITS+1))
#define INTERP_BITS (16-1-LOOKUP_BITS)
#define INTERP_MASK ((1<<INTERP_BITS)-1)



/**
 * Sine calculation using interpolated table lookup.
 * Instead of radiants or degrees we use "turns" here. Means this
 * sine does NOT return one phase for 0 to 2*PI, but for 0 to 1.
 * Input: -1 to 1 as int16 Q15  == -32768 to 32767.
 * Output: -1 to 1 as int16 Q15 == -32768 to 32767.
 *
 * See the full description at www.AtWillys.de for the detailed
 * explanation.
 *
 * @param int16_t angle Q15
 * @return int16_t Q15
 */
int16_t sin1(int16_t angle)
{
  int16_t v0, v1;
  if (angle < 0) {
	angle += INT16_MAX;
	angle += 1;
  }
  v0 = (angle >> INTERP_BITS);
  if (v0 & FLIP_BIT) {
	v0 = ~v0;
	v1 = ~angle;
  } else {
	v1 = angle;
  }
  v0 &= TABLE_MASK;
  v1 = SIN90[v0] + (int16_t) (((int32_t) (SIN90[v0+1]-SIN90[v0]) * (v1 & INTERP_MASK)) >> INTERP_BITS);
  if((angle >> INTERP_BITS) & NEGATE_BIT) v1 = -v1;
  return v1;
}
 
/**
 * Cosine calculation using interpolated table lookup.
 * Instead of radiants or degrees we use "turns" here. Means this
 * cosine does NOT return one phase for 0 to 2*PI, but for 0 to 1.
 * Input: -1 to 1 as int16 Q15  == -32768 to 32767.
 * Output: -1 to 1 as int16 Q15 == -32768 to 32767.
 *
 * @param int16_t angle Q15
 * @return int16_t Q15
 */
int16_t cos1(int16_t angle)
{
  if (angle < 0) {
	angle += INT16_MAX;
	angle += 1;
  }
  return sin1(angle - (int16_t)(((int32_t)INT16_MAX * 270) / 360));
}
