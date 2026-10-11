#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "encoder.h"

void Encoder_Init(void)
{
    Encoder_Reset();
    DL_TimerG_startCounter(QEI_LEFT_INST);
    DL_TimerG_startCounter(QEI_RIGHT_INST);
}

void Encoder_Reset(void)
{
    DL_TimerG_setTimerCount(QEI_LEFT_INST, 0U);
    DL_TimerG_setTimerCount(QEI_RIGHT_INST, 0U);
}

int16_t Read_Encoder_Left(void)
{
    int16_t count;

    count = (int16_t) DL_TimerG_getTimerCount(QEI_LEFT_INST);
    DL_TimerG_setTimerCount(QEI_LEFT_INST, 0U);

    return count;
}

int16_t Read_Encoder_Right(void)
{
    int16_t count;

    count = (int16_t) DL_TimerG_getTimerCount(QEI_RIGHT_INST);
    DL_TimerG_setTimerCount(QEI_RIGHT_INST, 0U);

    return -count;
}
