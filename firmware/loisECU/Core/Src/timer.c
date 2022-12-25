#include "timer.h"

void TIMER_Start(TIMER_t* t)
{
	t->start = HAL_GetTick();
}

uint32_t TIMER_Stop(TIMER_t* t)
{
	return HAL_GetTick() - t->start;
}

bool TIMER_IsExpired(TIMER_t* t, uint32_t period)
{
	return (TIMER_Stop(t) > period);
}

