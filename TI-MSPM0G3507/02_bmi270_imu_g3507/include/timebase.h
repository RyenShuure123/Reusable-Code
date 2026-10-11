#ifndef TIMEBASE_H
#define TIMEBASE_H

#include <stdbool.h>
#include <stdint.h>

bool Timebase_Init(void);
uint32_t Timebase_GetMilliseconds(void);

#endif
