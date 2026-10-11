#include "motor.h"

#include "ti_msp_dl_config.h"

#define MOTOR_PWM_PERIOD_TICKS 2000U

static uint16_t Motor_ClampDutyPermille(int16_t dutyPermille)
{
    int32_t magnitude = dutyPermille;

    if (magnitude < 0) {
        magnitude = -magnitude;
    }

    if (magnitude > MOTOR_DUTY_PERMILLE_MAX) {
        magnitude = MOTOR_DUTY_PERMILLE_MAX;
    }

    return (uint16_t)magnitude;
}

static void Motor_SetLeftPwmDuty(uint32_t channelIndex, uint16_t dutyPermille)
{
    uint32_t activeTicks =
        ((uint32_t)MOTOR_PWM_PERIOD_TICKS * dutyPermille) /
        MOTOR_DUTY_PERMILLE_MAX;
    DL_TimerA_setCaptureCompareValue(
        PWM_MOTOR_LEFT_INST,
        activeTicks,
        channelIndex);
}

static void Motor_SetRightPwmDuty(uint32_t channelIndex, uint16_t dutyPermille)
{
    uint32_t activeTicks =
        ((uint32_t)MOTOR_PWM_PERIOD_TICKS * dutyPermille) /
        MOTOR_DUTY_PERMILLE_MAX;
    DL_TimerG_setCaptureCompareValue(
        PWM_MOTOR_RIGHT_INST,
        activeTicks,
        channelIndex);
}

void Motor_Init(void)
{
    Motor_Stop();
    DL_TimerA_startCounter(PWM_MOTOR_LEFT_INST);
    DL_TimerG_startCounter(PWM_MOTOR_RIGHT_INST);
}

void Motor_Stop(void)
{
    Motor_SetLeftPwmDuty(GPIO_PWM_MOTOR_LEFT_C0_IDX, 0U);
    Motor_SetLeftPwmDuty(GPIO_PWM_MOTOR_LEFT_C1_IDX, 0U);
    Motor_SetRightPwmDuty(GPIO_PWM_MOTOR_RIGHT_C0_IDX, 0U);
    Motor_SetRightPwmDuty(GPIO_PWM_MOTOR_RIGHT_C1_IDX, 0U);
}

void Motor_SetLeftOutput(int16_t dutyPermille)
{
    uint16_t magnitude = Motor_ClampDutyPermille(dutyPermille);

    Motor_SetLeftPwmDuty(GPIO_PWM_MOTOR_LEFT_C0_IDX, 0U);
    Motor_SetLeftPwmDuty(GPIO_PWM_MOTOR_LEFT_C1_IDX, 0U);

    if (dutyPermille > 0) {
        Motor_SetLeftPwmDuty(GPIO_PWM_MOTOR_LEFT_C0_IDX, magnitude);
    } else if (dutyPermille < 0) {
        Motor_SetLeftPwmDuty(GPIO_PWM_MOTOR_LEFT_C1_IDX, magnitude);
    }
}

void Motor_SetRightOutput(int16_t dutyPermille)
{
    uint16_t magnitude = Motor_ClampDutyPermille(dutyPermille);

    Motor_SetRightPwmDuty(GPIO_PWM_MOTOR_RIGHT_C0_IDX, 0U);
    Motor_SetRightPwmDuty(GPIO_PWM_MOTOR_RIGHT_C1_IDX, 0U);

    if (dutyPermille > 0) {
        Motor_SetRightPwmDuty(GPIO_PWM_MOTOR_RIGHT_C1_IDX, magnitude);
    } else if (dutyPermille < 0) {
        Motor_SetRightPwmDuty(GPIO_PWM_MOTOR_RIGHT_C0_IDX, magnitude);
    }
}

void Motor_SetOutputs(int16_t leftDutyPermille, int16_t rightDutyPermille)
{
    Motor_SetLeftOutput(leftDutyPermille);
    Motor_SetRightOutput(rightDutyPermille);
}
