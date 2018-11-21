#include "pid.h"

int16_t funPID(int16_t div, PID_typedef *pid)
{
	int16_t value = div * pid->P;
	pid->addI += div * pid->I;
	if (pid->addI > value)
	{
		pid->addI = value;
	}
	value += pid->addI;
	value += (pid->lastDiv - div) * pid->D;
	pid->lastDiv = div;
	return value;
}
