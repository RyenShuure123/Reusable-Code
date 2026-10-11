#include "motor.h"
#include "ti_msp_dl_config.h"

static bool leftReverse;
static bool rightReverse;

// duty限幅函数
static uint16_t Motor_LimitDuty(uint16_t dutyTicks)
{
    if (dutyTicks > MOTOR_PWM_PERIOD_TICKS) {
        return MOTOR_PWM_PERIOD_TICKS;
    }

    return dutyTicks;
}

// 返回duty比较值
static uint16_t Motor_CompareValueFromDuty(uint16_t dutyTicks)
{
    return (uint16_t) (MOTOR_PWM_PERIOD_TICKS - dutyTicks);
}

// 命令限幅函数
static int16_t Motor_LimitCommand(int16_t command)
{
    if (command > (int16_t) MOTOR_PWM_PERIOD_TICKS) {
        return (int16_t) MOTOR_PWM_PERIOD_TICKS;
    }

    if (command < -(int16_t) MOTOR_PWM_PERIOD_TICKS) {
        return -(int16_t) MOTOR_PWM_PERIOD_TICKS;
    }

    return command;
}

static uint16_t Motor_CommandMagnitude(int16_t command)
{
    return (uint16_t) (command < 0 ? -command : command);
}

static void Motor_SetLeftPwmOutputEnabled(bool enabled)
{
    DL_TimerG_setCCPOutputDisabled(PWM_LEFT_INST,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL,
        enabled ? DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL : DL_TIMER_CCP_DIS_OUT_LOW);
}

static void Motor_SetRightPwmOutputEnabled(bool enabled)
{
    DL_TimerG_setCCPOutputDisabled(PWM_RIGHT_INST,
        enabled ? DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL : DL_TIMER_CCP_DIS_OUT_LOW,
        DL_TIMER_CCP_DIS_OUT_SET_BY_OCTL);
}

void Motor_Init(void)
{
    Motor_Stop();
    DL_TimerG_startCounter(PWM_LEFT_INST);
    DL_TimerG_startCounter(PWM_RIGHT_INST);
}

void Motor_Stop(void)
{
    Motor_SetLeftDuty(0U);
    Motor_SetRightDuty(0U);
    Motor_SetLeftPhase(false);
    Motor_SetRightPhase(false);
    leftReverse = false;
    rightReverse = false;
}

void Motor_SetLeftDuty(uint16_t dutyTicks)
{
    dutyTicks = Motor_LimitDuty(dutyTicks);

    if (dutyTicks == 0U) {
        Motor_SetLeftPwmOutputEnabled(false);
        return;
    }

    DL_TimerG_setCaptureCompareValue(PWM_LEFT_INST,
        Motor_CompareValueFromDuty(dutyTicks), GPIO_PWM_LEFT_C1_IDX);
    Motor_SetLeftPwmOutputEnabled(true);
}

void Motor_SetRightDuty(uint16_t dutyTicks)
{
    dutyTicks = Motor_LimitDuty(dutyTicks);

    if (dutyTicks == 0U) {
        Motor_SetRightPwmOutputEnabled(false);
        return;
    }

    DL_TimerG_setCaptureCompareValue(PWM_RIGHT_INST,
        Motor_CompareValueFromDuty(dutyTicks), GPIO_PWM_RIGHT_C0_IDX);
    Motor_SetRightPwmOutputEnabled(true);
}

void Motor_SetLeftPhase(bool phaseHigh)
{
    if (phaseHigh) {
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_LEFT_DIR_PIN);
    } else {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_LEFT_DIR_PIN);
    }
}

void Motor_SetRightPhase(bool phaseHigh)
{
    if (phaseHigh) {
        DL_GPIO_setPins(GPIO_MOTOR_PORT, GPIO_MOTOR_RIGHT_DIR_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_MOTOR_PORT, GPIO_MOTOR_RIGHT_DIR_PIN);
    }
}

void Motor_SetDuty(uint16_t left, uint16_t right)
{
    Motor_SetLeftDuty(left);
    Motor_SetRightDuty(right);
}

static void Motor_SetLeftCommand(int16_t command)
{
    bool reverse;
    uint16_t dutyTicks;

    command = Motor_LimitCommand(command);
    reverse = command < 0;
    dutyTicks = Motor_CommandMagnitude(command);

    if (dutyTicks == 0U) {
        Motor_SetLeftDuty(0U);
        return;
    }

    if (reverse != leftReverse) {
        Motor_SetLeftDuty(0U);
        Motor_SetLeftPhase(reverse);
        leftReverse = reverse;
        return;
    }

    Motor_SetLeftDuty(dutyTicks);
}

static void Motor_SetRightCommand(int16_t command)
{
    bool reverse;
    uint16_t dutyTicks;

    command = Motor_LimitCommand(command);
    reverse = command < 0;
    dutyTicks = Motor_CommandMagnitude(command);

    if (dutyTicks == 0U) {
        Motor_SetRightDuty(0U);
        return;
    }

    if (reverse != rightReverse) {
        Motor_SetRightDuty(0U);
        Motor_SetRightPhase(reverse);
        rightReverse = reverse;
        return;
    }

    Motor_SetRightDuty(dutyTicks);
}

void Motor_SetCommand(int16_t leftCommand, int16_t rightCommand)
{
    Motor_SetLeftCommand(leftCommand);
    Motor_SetRightCommand(rightCommand);
}
