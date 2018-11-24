#include "pid.h"

int16_t funPID(int16_t div, PID_typedef *pid,uint16_t timeTick)
{
	int16_t value = div * pid->P;
	pid->addI += div * pid->I * timeTick;
	if (pid->addI > value)
	{
		pid->addI = value;
	}
	value += pid->addI;
	value += (div - pid->lastDiv) * pid->D;
	pid->lastDiv = div;
	return value;
}
