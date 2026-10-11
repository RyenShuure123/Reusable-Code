#include "timebase.h"

#include "ti_msp_dl_config.h"

#define TIMEBASE_TICKS_PER_SECOND 1000U

static volatile uint32_t g_timebaseMilliseconds;

bool Timebase_Init(void)
{
    g_timebaseMilliseconds = 0U;
    return SysTick_Config(CPUCLK_FREQ / TIMEBASE_TICKS_PER_SECOND) == 0U;
}

uint32_t Timebase_GetMilliseconds(void)
{
    return g_timebaseMilliseconds;
}

void SysTick_Handler(void)
{
    g_timebaseMilliseconds++;
}
