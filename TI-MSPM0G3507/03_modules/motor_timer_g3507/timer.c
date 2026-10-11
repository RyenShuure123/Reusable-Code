#include "timer.h"

#include "imu.h"
#include "ti_msp_dl_config.h"

#define CONTROL_TIMER_PERIOD_SECONDS 0.01f

static volatile uint32_t g_controlTickCount;
static volatile int8_t g_imuStatus;

void ControlTimer_Init(void)
{
    g_controlTickCount = 0U;
    g_imuStatus = IMU_OK;

    DL_TimerG_clearInterruptStatus(
        TIMER_CONTROL_INST,
        DL_TIMERG_INTERRUPT_ZERO_EVENT);
    NVIC_ClearPendingIRQ(TIMER_CONTROL_INST_INT_IRQN);
    NVIC_SetPriority(TIMER_CONTROL_INST_INT_IRQN, 0U);
    NVIC_EnableIRQ(TIMER_CONTROL_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_CONTROL_INST);
}

uint32_t ControlTimer_GetTickCount(void)
{
    return g_controlTickCount;
}

int8_t ControlTimer_GetImuStatus(void)
{
    return g_imuStatus;
}

void TIMER_CONTROL_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_CONTROL_INST)) {
        case DL_TIMERG_IIDX_ZERO:
            g_controlTickCount++;

            if (IMU_IsReady()) {
                g_imuStatus = IMU_Update(CONTROL_TIMER_PERIOD_SECONDS);
            }
            break;

        default:
            break;
    }
}
