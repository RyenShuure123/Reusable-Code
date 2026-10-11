#ifndef MSPM0G3507_SERVO_LEARNING_SERVO_H
#define MSPM0G3507_SERVO_LEARNING_SERVO_H
#include <stdint.h>

void Servo_Init(void);
void Servo_SetPulseUs(uint16_t pulseUs);
void Servo_SetAngle(uint8_t angle);

#endif //MSPM0G3507_SERVO_LEARNING_SERVO_H
