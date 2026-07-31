#ifndef __TIMEBASE_H
#define __TIMEBASE_H

#include "stm32f10x.h"

/* 1 ms system time used by the camera link and the PID controller. */
void timebase_init(void);
void timebase_tick_isr(void);
uint32_t timebase_millis(void);

#endif
