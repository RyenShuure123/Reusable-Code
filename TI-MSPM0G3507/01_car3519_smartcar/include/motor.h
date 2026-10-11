#ifndef CAR3519NEW_MOTOR_H
#define CAR3519NEW_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#define MOTOR_PWM_PERIOD_TICKS 1000U

void Motor_Init(void);
void Motor_Stop(void);
void Motor_SetLeftDuty(uint16_t dutyTicks);
void Motor_SetRightDuty(uint16_t dutyTicks);
void Motor_SetLeftPhase(bool phaseHigh);
void Motor_SetRightPhase(bool phaseHigh);
void Motor_SetDuty(uint16_t left, uint16_t right);

/* +1000: forward full duty, -1000: reverse full duty, 0: stop. */
void Motor_SetCommand(int16_t leftCommand, int16_t rightCommand);

#endif
