#ifndef __FAN_CONTROL_H
#define __FAN_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

bool fan_control_get_state();
void fan_control_test(uint16_t test_mode);
void fan_control_ac(uint32_t cnt_1Hz, int16_t p_ac);


#ifdef __cplusplus
}
#endif

#endif /* __FAN_CONTROL_H */
