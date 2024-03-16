#ifndef INC_SYS_MODE_CONTROLLER_H
#define INC_SYS_MODE_CONTROLLER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

#include "common.h"

enum mode_t get_sys_mode();
void sys_mode_ctrl_step(control_ref_t* ctrl_ref);


#ifdef __cplusplus
}
#endif

#endif /* INC_SYS_MODE_CONTROLLER_H */
