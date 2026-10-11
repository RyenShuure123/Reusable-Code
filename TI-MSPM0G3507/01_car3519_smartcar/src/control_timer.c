#include "control_timer.h"
#include "button.h"
#include "drive_control.h"
#include "ti_msp_dl_config.h"

void ControlTimer_Init(void)
{
    NVIC_EnableIRQ(TIMER_PID_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_PID_INST);
}

void TIMER_PID_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(TIMER_PID_INST)) {
    case DL_TIMER_IIDX_ZERO:
        Button_Tick10ms();
        DriveControl_Tick10ms();
        TrackControl_Tick10ms();
        AngleControl_Tick10ms();

        break;

    default:
        break;
    }
}
