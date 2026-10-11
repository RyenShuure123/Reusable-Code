#ifndef MSPM0G3507_TIMER_PID_MOTOR_H
#define MSPM0G3507_TIMER_PID_MOTOR_H

#include <stdint.h>

#define MOTOR_DUTY_PERMILLE_MAX 1000

void Motor_Init(void);
void Motor_Stop(void);
void Motor_SetLeftOutput(int16_t dutyPermille);
void Motor_SetRightOutput(int16_t dutyPermille);
void Motor_SetOutputs(int16_t leftDutyPermille, int16_t rightDutyPermille);

#endif //MSPM0G3507_TIMER_PID_MOTOR_H
