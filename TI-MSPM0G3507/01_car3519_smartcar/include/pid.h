#ifndef CAR3519NEW_PID_H
#define CAR3519NEW_PID_H

#include <stdint.h>

typedef struct {
    float Kp;
    float Ki;
    float error;
    float last_error;
    float integral;
    int pwm_out;
} Speed_PI_t;

typedef struct {
    float Kp;
    float Kd;
    float error;
    float last_error;
} Track_PD_t;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float error;
    float last_error;
    float integral;
} Angle_PID_t;

#define PULSES_PER_METER 268209
#define dt 0.01f

extern Speed_PI_t left_speed;
extern Speed_PI_t right_speed;
extern Track_PD_t Track_PD;
extern Angle_PID_t Angle_PID;

int16_t SpeedPI_Control(Speed_PI_t *pi,
    float targetSpeed, float measuredSpeed);
void SpeedPI_Differential(float leftTargetSpeed, float rightTargetSpeed,
    int16_t *leftPulses, int16_t *rightPulses);
void SpeedPI_Straight(float targetSpeed, int16_t *leftPulses,
    int16_t *rightPulses);
void SpeedPI_Stop(void);

float Track_PD_Control(Track_PD_t *pd, float current_error);
float Angle_PID_Control(Angle_PID_t *pid, float currentError,
    float yawRateDps, float dtSeconds, float outputLimit);

#endif
