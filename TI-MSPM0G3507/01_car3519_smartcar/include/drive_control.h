#ifndef CAR3519NEW_DRIVE_CONTROL_H
#define CAR3519NEW_DRIVE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t leftPulses10ms;
    int16_t rightPulses10ms;
    uint32_t distancePulses;
    uint32_t sampleCount;
    bool active;
} DriveControlTelemetry;

void DriveControl_StartStraight(float targetSpeed, float distanceMeters);
void DriveControl_Tick10ms(void);
void DriveControl_Stop(void);
void DriveControl_GetTelemetry(DriveControlTelemetry *telemetry);

void TrackControl_Start(void);
void TrackControl_Stop(void);
void TrackControl_Tick10ms(void);
void TrackControl_ProcessTuning(void);

void AngleControl_StartStraight(float targetSpeed, float distanceMeters);
void AngleControl_StartStraightToAngle(float targetSpeed, float distanceMeters,
    float targetAngleDegrees);
void AngleControl_Stop(void);
void AngleControl_Tick10ms(void);
void AngleControl_TurnAngle(float angleDegrees);
void AngleControl_TurnToAngle(float targetAngleDegrees);

#endif
