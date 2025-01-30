#ifndef INC_CONFIG_H
#define INC_CONFIG_H

//#define SYS_MODE HYBRID_PCC_SENSOR
#define SYS_MODE PV2AC

#define COMM_READ_ELECTRICITY_METER 1  // listen for smart meter data and send inverterdata after reception
#define SYSTEM_HAS_BATTERY 1

#define USE_TRAFO_33V 0  // 0: high voltage grid. 1: test with 33V trafo (~30.5V with 9 extra windings for 15 cell LFP battery)

#define VDC_MAX_MPPT_100mV 360*10  // 360V / 96cells = 3.75V
#define E_VDC_MAX_FB_BOOST_100mV 374*10   // worst case for battery if BMS fails: 384V / 96cells = 4.0V
#define E_VDC_MAX_FB_GRID_100mV 374*10   // worst case for battery if BMS fails: 404V / 96cells = 4.208V

/***********/
/* AC grid */
/***********/
#ifndef USE_TRAFO_33V
#error "USE_TRAFO_33V has to be defined as 0 or 1"
#endif

#if USE_TRAFO_33V == 1
#define VGRID_TRATIO 7  // transformer winding ratio
#elif USE_TRAFO_33V == 0
#define VGRID_TRATIO 1  // no transformer
#endif

#define VGRID_AMP 325/VGRID_TRATIO

//VDE 4105:
// 47.5 Hz to 49.0 Hz  ≥30 min
// 49.0 Hz to 51.0 Hz  unlimited
// 51.0 Hz to 51.5 Hz  ≥30 min
#define F_MIN 48
#define F_MAX 51

#define E_I_AC_RMS_MAX_CNT 10  // 10 samples @ 20kHz -> 500us
//#define E_I_AC_RMS_MAX_AMP_10mA (12.3 * 100)  // 2000W÷230V×sqrt(2) amplitude for E_I_AC_RMS_MAX_CNT samples
#define E_I_AC_PULSE_MAX_AMP_10mA (16.0 * 100)  // 2600W÷230V×sqrt(2) amplitude for 1 sample
#define E_I_AC_RMS_MAX_AMP_10mA (8.0 * 100)  // 1300W÷230V×sqrt(2) amplitude for E_I_AC_RMS_MAX_CNT samples
//#define E_I_AC_PULSE_MAX_AMP_10mA (10.5 * 100)  // 1700W÷230V×sqrt(2) amplitude for 1 sample

#define PERMIL_V_DFFW_MIN  200  // 20% grid voltage direct feedforward, 80% PLL
//#define PERMIL_V_DFFW_MIN  500  // 50% grid voltage direct feedforward, 50% PLL; Test at 31Vac trafo did not cause LCL oscillation
#define PERMIL_V_DFFW_MAX  500  // 50% maximum in case of high current due to distorted grid
#define PERMIL_V_DFFW_INCR 60  // 60 equals 6% per control cycle -> 60% in 500µs
#define PERMIL_V_DFFW_DECR 1  // 1 equals 0.1% per control cycle; @70% direct feedforward, it takes 600*50µs=30ms to come back to 10%
// always 50% has higher inductor sound
// always 80% has even higher inductor sound

#define E_I_AC_DC_OFFSET_MAX_10mA (0.8 * 100)  // 800mA in E_I_AC_DC_OFFSET_CYCLES consecutive 50Hz periods  todo decrease
#define E_I_AC_DC_OFFSET_CYCLES 8  // number of consecutive 50Hz periods for DC current fault

#define P_AC_MIN 0 // feed into grid only -> no AC2BAT for now
//#define P_AC_MAX ((5*32)/1.41) //113W  // todo implement anti windup in power controller regarding IAC_AMP_MAX_10mA
#define P_AC_MAX (210)  // 210W minimum for energy packet controller
#define P_BAT_MIN_CHARGE 2
#define P_MIN_PV2AC 10

/***************/
/* Fan control */
/***************/
//#define FAN_P_AC_START 1200
//#define FAN_P_AC_STOP 1000
//#define FAN_RUNTIME_MINIMUM_SEC 30

#define FAN_P_AC_START 300
#define FAN_P_AC_STOP 200
#define FAN_RUNTIME_MINIMUM_SEC 15


// 20kHz PWM, 20kHz controlfreq -> repetition counter = 1
// 20kHz / ((1+1)/2) = 20kHz
#define DEF_MPPT_DUTY_ABSMAX 4250

#define DC_CTRL_FREQ 20000
#define DC_CTRL_FREQ_MPPT 50
#define CYCLES_cnt20kHz_20ms (DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT)

#define AC_CTRL_FREQ DC_CTRL_FREQ


#endif /* INC_CONFIG_H */
