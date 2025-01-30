
#include <stdlib.h>

#include "stm32g4xx_hal.h"

#include "gpio.h"
#include "common.h"
#include "fan_control.h"

static uint32_t cnt_1Hz_start = 0;
static bool test_mode_ = 0;

bool fan_control_get_state()
{
	return HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
}


void fan_control_test(uint16_t test_mode)
{
	test_mode_ = test_mode;
}


void fan_control_ac(uint32_t cnt_1Hz, int16_t p_ac)
{
    if (   abs(p_ac) > FAN_P_AC_START
		|| test_mode_) {
        fanAC(1);
        cnt_1Hz_start = cnt_1Hz;
        test_mode_ = 0;
    } else if (    abs(p_ac) < FAN_P_AC_STOP
                && cnt_1Hz > (cnt_1Hz_start+FAN_RUNTIME_MINIMUM_SEC)
    ) {
        fanAC(0);
    }
}
