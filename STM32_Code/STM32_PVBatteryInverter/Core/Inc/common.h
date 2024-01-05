#ifndef INC_COMMON_H
#define INC_COMMON_H

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

enum mode_t {OFF, PV2AC, PV2BAT, HYBRID_OFFLINE, HYBRID_ONLINE};
extern volatile enum mode_t sys_mode;

//enum stateAC_t {INIT, GRID_CONNECTING, GRID_SYNC};

enum _errorcode
{
	EC_NO_ERROR = 0,
	EC_EMERGENCY_STOP = -1,
	EC_V_DC_MAX_FB_BOOST = -2,
	EC_V_DC_MAX_FB_GRID = -3,
	EC_V_DC_SENSOR_FB_BOOST = -4,
	EC_V_DC_SENSOR_FB_GRID = -5,
	EC_I_DC_MAX = -6,
	EC_I_AC_MAX = -7,
	EC_GRID_SYNC_LOST = -8
};

typedef enum _errorcode error_t;
char *strerror(int errcode);

extern volatile error_t sys_errcode;

void shutdown();
void checkErrors();

#ifdef __cplusplus
}
#endif

#endif /* INC_COMMON_H */
