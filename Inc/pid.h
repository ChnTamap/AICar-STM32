#ifndef _PID_H_
#define _PID_H_

#include "stm32f1xx_hal.h"

/* PID Base */
typedef struct
{
	double P;
	double I;
	double D;
	int lastDiv;
	int addI;
} PID_typedef;
int16_t funPID(int16_t div, PID_typedef *pid);

#endif // !_PID_H_
