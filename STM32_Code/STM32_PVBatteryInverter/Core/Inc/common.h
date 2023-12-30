#ifndef INC_COMMON_H
#define INC_COMMON_H

#include <stdint.h>


enum _errorcode
{
	EC_NO_ERROR = 0,
	EC_EMERGENCY_STOP = -1,
	EC_V_DC_MAX = -2,
	EC_V_DC_SENSOR = -3
};

typedef enum _errorcode error_t;
char *strerror(int errcode);

extern volatile error_t sys_errcode;

void shutdown();
void checkErrors();


#endif /* INC_COMMON_H */
