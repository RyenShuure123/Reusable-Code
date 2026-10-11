#ifndef MSPM0G3507_TIMER_PID_TIMER_H
#define MSPM0G3507_TIMER_PID_TIMER_H

#include <stdint.h>

void ControlTimer_Init(void);
uint32_t ControlTimer_GetTickCount(void);
int8_t ControlTimer_GetImuStatus(void);

#endif //MSPM0G3507_TIMER_PID_TIMER_H
