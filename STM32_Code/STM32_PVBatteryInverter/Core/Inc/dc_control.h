/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DC_CONTROL_H
#define __DC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif


#define E_VDC_MAX_100mV 4000  // 400/96 = 4.16Vcell

//#define VDC_BOOST_START_100mV 2400  // 8*30V minimum voltage of DC bus to start DC booster stage
//#define VDC_BOOST_STOP_100mV 2000  // 8*25V  minimum voltage of DC bus to keep booster stage running

#define VDC_BOOST_START_100mV 1600
#define VDC_BOOST_STOP_100mV 1200

#define VDC_TOLERANCE_100mV 50  // 5V tolerance for closing DC contactor


enum stateDC_t {INIT, WAIT_PV_VOLTAGE, VOLTAGE_CONTROL, WAIT_CONTACTOR, MPPT};
//enum stateAC_t {INIT, GRID_CONNECTING, GRID_SYNC};

extern volatile enum stateDC_t stateDC;
extern volatile uint16_t Vdc_sincfilt_100mV;
extern volatile int16_t Idc_filt_10mA;

void measVdc();
int checkLimits();
int16_t dcControlStep();



#ifdef __cplusplus
}
#endif

#endif /* __DC_CONTROL_H */
