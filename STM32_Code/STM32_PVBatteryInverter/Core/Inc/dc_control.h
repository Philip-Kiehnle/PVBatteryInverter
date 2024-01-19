/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DC_CONTROL_H
#define __DC_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif


//#define E_VDC_MAX_100mV 400*10  // 400/96 = 4.16Vcell
//#define VDC_BOOST_START_100mV 240*10  // 8*30V minimum voltage of DC bus to start DC booster stage
//#define VDC_BOOST_STOP_100mV 200*10  // 8*25V  minimum voltage of DC bus to keep booster stage running

#define E_VDC_MAX_100mV 140*10
#define VDC_BOOST_START_100mV 50*10
#define VDC_BOOST_STOP_100mV 40*10

#define VDC_TOLERANCE_100mV 5*10  // 5V tolerance for closing DC contactor


enum stateDC_t {INIT_DC, WAIT_PV_VOLTAGE, VOLTAGE_CONTROL, WAIT_CONTACTOR_DC, MPPT};

extern volatile enum stateDC_t stateDC;
extern volatile uint16_t VdcFBboost_sincfilt_100mV;
extern volatile int16_t Idc_filt_10mA;

void calc_and_wait(uint32_t delay, UART_HandleTypeDef *huart);
void measVdcFBboost();
error_t checkDCLimits();
int16_t dcControlStep();



#ifdef __cplusplus
}
#endif

#endif /* __DC_CONTROL_H */
