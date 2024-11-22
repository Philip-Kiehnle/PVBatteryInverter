#ifndef INC_MONITORING_H
#define INC_MONITORING_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "stm32g4xx_hal.h"


typedef struct {
	uint16_t id;  // sys variables
	uint8_t sys_mode;
	int8_t sys_errcode;
	uint8_t stateDC;  // DC variables
	uint8_t dcdc_mode;
	uint16_t dutyDC_HS;
	float p_dc_filt50Hz;
	float v_dc_filt50Hz;
	uint16_t stateAC;  // AC variables
	int16_t f_ac_10mHz;
	int16_t v_ac_rms_100mV;
	int16_t i_ac_amp_10mA;
	int16_t p_ac_filt50Hz;
	int16_t p_ac_ref;  // for debugging
	int16_t bat_p;
	uint16_t soc_10mPercent;
	uint16_t v_dc_FBboost_sincfilt_100mV;
	uint16_t v_dc_FBgrid_sincfilt_100mV;
} __attribute__((__packed__)) monitor_vars_t;

typedef struct {
	uint32_t header;
	uint32_t crc;
	monitor_vars_t monitor_vars;
} __attribute__((__packed__)) monitor_packet_t;

typedef struct {
	uint16_t v_dc;
	uint16_t v_dc_modulator_100mV;
 } __attribute__((__packed__)) fast_monitor_vars_t;


//extern volatile int16_t debug_f_ac_10mHz;
//extern volatile int16_t debug_v_ac_rms_100mV;
//extern volatile int16_t debug_v_amp_pred_100mV;
//extern volatile int16_t debug_i_ac_amp_10mA;

extern bool monitoring_binary_en;
extern volatile bool monitoring_request;

extern volatile bool fast_mon_vars_trig;
extern volatile uint16_t frame_nr;

#define FAST_MON_FRAMES 400
#define FAST_MON_BYTES (FAST_MON_FRAMES*sizeof(fast_monitor_vars_t))
extern volatile fast_monitor_vars_t fast_monitor_vars[];

void async_monitor_check(UART_HandleTypeDef *huart);


#ifdef __cplusplus
}
#endif

#endif /* INC_MONITORING_H */
