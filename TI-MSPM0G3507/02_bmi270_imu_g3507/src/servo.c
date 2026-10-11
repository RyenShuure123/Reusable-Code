#include "servo.h"
#include "ti_msp_dl_config.h"

#define SERVO_PERIOD_US         20000U
#define SERVO_MIN_PULSE_US       1000U
#define SERVO_CENTER_PULSE_US    1500U
#define SERVO_MAX_PULSE_US       2000U
#define SERVO_MIN_ANGLE_DEG         0U
#define SERVO_MAX_ANGLE_DEG       180U

void Servo_SetPulseUs(uint16_t pulseUs)
{
    if (pulseUs < SERVO_MIN_PULSE_US) {
        pulseUs = SERVO_MIN_PULSE_US;
    } else if (pulseUs > SERVO_MAX_PULSE_US) {
        pulseUs = SERVO_MAX_PULSE_US;
    }

    DL_TimerA_setCaptureCompareValue(
        PWM_SERVO_INST,
        SERVO_PERIOD_US - pulseUs,
        GPIO_PWM_SERVO_C0_IDX);
}

void Servo_Init(void)
{
    Servo_SetPulseUs(SERVO_CENTER_PULSE_US);
    DL_TimerA_startCounter(PWM_SERVO_INST);
}

void Servo_SetAngle(uint8_t angle)
{
    uint16_t pulseUs;

    if (angle > SERVO_MAX_ANGLE_DEG) {
        angle = SERVO_MAX_ANGLE_DEG;
    }

    pulseUs = SERVO_MIN_PULSE_US +
        (uint16_t)(((uint32_t)angle *
                    (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US)) /
                   SERVO_MAX_ANGLE_DEG);

    Servo_SetPulseUs(pulseUs);
}
