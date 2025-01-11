#ifndef INC_CONFIG_H
#define INC_CONFIG_H

// Define a system mode or uncomment IS_BATTERY_SUPERVISOR_PCB
//#define SYS_MODE PV2BAT
#define IS_BATTERY_SUPERVISOR_PCB 1

#if IS_BATTERY_SUPERVISOR_PCB == 1
#define SYS_MODE BAT_SUPERVISOR_MODE
#endif

#define COMM_READ_ELECTRICITY_METER 0  // listen for smart meter data and send inverterdata after reception
#define SYSTEM_HAS_BATTERY 1

#define E_VDC_BUS_MAX_100mV 404*10  // limit for ADC with voltage divider
#define E_VDC_MAX_100mV 375*10
//#define E_VDC_MAX_100mV 384*10   // worst case for battery if BMS fails: 384V / 96cells = 4.0V
//#define E_VDC_MAX_100mV 404*10   // worst case for battery if BMS fails: 404V / 96cells = 4.208V
#define VDC_MAX_MPPT_100mV 360*10  // 360V / 96cells = 3.75V
//#define VDC_MAX_MPPT_100mV 375*10  // 375V / 96cells = 3.91V
//#define VDC_MAX_MPPT_100mV 384*10  // 384V / 96cells = 4.0V

//#define E_IAC_MAX_10mA (7.3 * 100)  // 1200W÷230V×sqrt(2) amplitude
//#define E_IAC_MAX_10mA (10 * 100)  // 1625W÷230V×sqrt(2) amplitude

#define P_BAT_MIN_CHARGE 2
#define P_MIN_PV2AC 10

// 40kHz PWM, 20kHz controlfreq -> repetition counter = 3
// 40kHz / ((3+1)/2) = 20kHz
#define DEF_MPPT_DUTY_ABSMAX 2125  // Counter Period = 2124

#define DC_CTRL_FREQ 20000
#define DC_CTRL_FREQ_MPPT 50
#define CYCLES_cnt20kHz_20ms (DC_CTRL_FREQ/DC_CTRL_FREQ_MPPT)


#endif /* INC_CONFIG_H */
