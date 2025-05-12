#ifndef INC_POWER_CONTROLLER_H
#define INC_POWER_CONTROLLER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


int power_controller_step(int p_pcc_ref, int p_pcc, uint16_t p_ac_max);


#ifdef __cplusplus
}
#endif

#endif /* INC_POWER_CONTROLLER_H */
