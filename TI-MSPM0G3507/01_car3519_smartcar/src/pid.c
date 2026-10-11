#include "pid.h"
#include <stddef.h>
#include "encoder.h"
#include "motor.h"

Speed_PI_t left_speed = {2.6f, 0.104f, 0.0f, 0.0f, 0.0f, 0.0f};
Speed_PI_t right_speed = {2.6f, 0.104f, 0.0f, 0.0f, 0.0f, 0.0f};
Track_PD_t Track_PD = {0.0022f, 0.0010f, 0, 0};
Angle_PID_t Angle_PID = {0.008f, 0.002f, 0.001f, 0.0f, 0.0f, 0.0f};

static float Get_LeftWheel_Speed(int16_t *pulses)
{
    *pulses = Read_Encoder_Left();

    return ((float) *pulses / (float) PULSES_PER_METER) / dt;
}

static float Get_RightWheel_Speed(int16_t *pulses)
{
    *pulses = Read_Encoder_Right();

    return ((float) *pulses / (float) PULSES_PER_METER) / dt;
}

int16_t SpeedPI_Control(Speed_PI_t *pi,
    float targetSpeed, float measuredSpeed)
{
    float canIng;
    float outputMin;
    float outputMax;
    float outputUns;
    float outputLimit;

    pi->error = targetSpeed - measuredSpeed;
    canIng = pi->integral + pi->error * dt;
    outputUns = pi->Kp * pi->error +
        pi->Ki * canIng;
    outputLimit = outputUns;

    outputMin = targetSpeed < 0.0f ? -1.0f : 0.0f;
    outputMax = targetSpeed > 0.0f ? 1.0f : 0.0f;

    if (outputLimit > outputMax) {
        outputLimit = outputMax;
    } else if (outputLimit < outputMin) {
        outputLimit = outputMin;
    }

    if ((outputUns >= outputMin && outputUns <= outputMax) ||
        (outputUns > outputMax && pi->error < 0.0f) ||
        (outputUns < outputMin && pi->error > 0.0f)) {
        pi->integral = canIng;
    }

    pi->last_error = pi->error;
    pi->pwm_out = (int) (outputLimit * MOTOR_PWM_PERIOD_TICKS +
        (outputLimit >= 0.0f ? 0.5f : -0.5f));

    return (int16_t) pi->pwm_out;
}

void SpeedPI_Stop(void)
{
    left_speed.error = 0.0f;
    left_speed.last_error = 0.0f;
    left_speed.integral = 0.0f;
    right_speed.error = 0.0f;
    right_speed.last_error = 0.0f;
    right_speed.integral = 0.0f;
    left_speed.pwm_out = 0;
    right_speed.pwm_out = 0;
    Motor_Stop();
}

void SpeedPI_Differential(float leftTargetSpeed, float rightTargetSpeed,
    int16_t *leftPulses, int16_t *rightPulses)
{
    int16_t localLeftPulses;
    int16_t localRightPulses;
    int16_t leftCommand;
    int16_t rightCommand;

    if (leftPulses == NULL) {
        leftPulses = &localLeftPulses;
    }

    if (rightPulses == NULL) {
        rightPulses = &localRightPulses;
    }

    leftCommand = SpeedPI_Control(&left_speed, leftTargetSpeed,
        Get_LeftWheel_Speed(leftPulses));
    rightCommand = SpeedPI_Control(&right_speed, rightTargetSpeed,
        Get_RightWheel_Speed(rightPulses));

    Motor_SetCommand(leftCommand, rightCommand);
}

void SpeedPI_Straight(float targetSpeed, int16_t *leftPulses,
    int16_t *rightPulses)
{
    if (leftPulses != NULL) {
        *leftPulses = 0;
    }

    if (rightPulses != NULL) {
        *rightPulses = 0;
    }

    if (targetSpeed == 0.0f) {
        SpeedPI_Stop();
        return;
    }

    SpeedPI_Differential(targetSpeed, targetSpeed,
        leftPulses, rightPulses);
}

float Track_PD_Control(Track_PD_t *pd, float current_error)
{
    float turn_out;

    pd->error = current_error;
    turn_out = pd->Kp * pd->error +
        pd->Kd * (pd->error - pd->last_error);
    pd->last_error = pd->error;

    return turn_out;
}

float Angle_PID_Control(Angle_PID_t *pid, float currentError,
    float yawRateDps, float dtSeconds, float outputLimit)
{
    const float integralOutputLimit = 0.03f;
    float candidateIntegral;
    float integralLimit;
    float proportionalOutput;
    float integralOutput;
    float derivativeOutput;
    float unclampedOutput;
    float output;

    pid->error = currentError;
    proportionalOutput = pid->Kp * pid->error;
    derivativeOutput = -pid->Kd * yawRateDps;
    candidateIntegral = pid->integral + pid->error * dtSeconds;

    if (pid->Ki > 0.0f) {
        integralLimit = integralOutputLimit / pid->Ki;
        if (candidateIntegral > integralLimit) {
            candidateIntegral = integralLimit;
        } else if (candidateIntegral < -integralLimit) {
            candidateIntegral = -integralLimit;
        }
    } else {
        candidateIntegral = 0.0f;
    }

    integralOutput = pid->Ki * candidateIntegral;
    unclampedOutput = proportionalOutput + integralOutput + derivativeOutput;

    if (unclampedOutput > outputLimit) {
        output = outputLimit;
    } else if (unclampedOutput < -outputLimit) {
        output = -outputLimit;
    } else {
        output = unclampedOutput;
    }

    if ((unclampedOutput == output) ||
        (unclampedOutput > outputLimit && pid->error < 0.0f) ||
        (unclampedOutput < -outputLimit && pid->error > 0.0f)) {
        pid->integral = candidateIntegral;
    }

    pid->last_error = pid->error;

    return output;
}
