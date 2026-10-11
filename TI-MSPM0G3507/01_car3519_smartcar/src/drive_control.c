#include "drive_control.h"
#include "encoder.h"
#include "loop.h"
#include "imu_uart.h"
#include "pid.h"
#include "uart_bluetooth.h"
#include <string.h>
#include "ti_msp_dl_config.h"

#define TRACK_BASE_SPEED 0.30f
#define TRACK_MIN_SPEED 0.18f
#define TRACK_TURN_LIMIT 0.20f
#define TRACK_ERROR_DEADBAND 15
#define TRACK_TUNING_COMMAND_LENGTH 48U
#define TRACK_TUNING_GAIN_MAX 1.0f

#define ANGLE_STRAIGHT_TURN_SPEED_LIMIT 0.08f
#define ANGLE_TURN_SPEED_LIMIT 0.40f
#define ANGLE_TURN_MIN_SPEED 0.015f   // 原来 0.03，减半
#define ANGLE_TURN_TOLERANCE_DEG 1.5f
#define ANGLE_TURN_SETTLE_RATE_DPS 4.0f
#define ANGLE_TURN_SETTLE_COUNT 8U
#define ANGLE_IMU_STALE_TICK_LIMIT 3U

static volatile bool driveControlActive;
static volatile bool trackControlActive;
static volatile bool angleControlActive;
static volatile bool angleControlTurnInPlace;
static volatile int16_t leftPulse10ms;
static volatile int16_t rightPulse10ms;
static volatile uint32_t distancePulseCount;
static volatile uint32_t distanceTargetPulses;
static volatile uint32_t controlSampleCount;
static volatile float driveTargetSpeedMps;
static volatile bool driveDistanceLimited;
static volatile bool angleDistanceLimited;
static volatile uint32_t angleDistanceTargetPulses;
static volatile float angleTargetSpeedMps;
static volatile float angleTargetYawDeg;
static volatile uint8_t angleTurnStableCount;
static volatile uint8_t angleLastFeedbackSequence;
static volatile uint8_t angleFeedbackStaleTickCount;
static volatile float angleTurnSpeedCommand;
static char trackTuningCommand[TRACK_TUNING_COMMAND_LENGTH];
static uint8_t trackTuningCommandLength;

static uint16_t AbsPulseCount(int16_t pulses)
{
    if (pulses < 0) {
        return (uint16_t) -(int32_t) pulses;
    }

    return (uint16_t) pulses;
}

static float Track_Limit(float value, float min, float max)
{
    if (value > max) {
        return max;
    }

    if (value < min) {
        return min;
    }

    return value;
}

static float AngleControl_NormalizeAngle(float angleDegrees)
{
    while (angleDegrees > 180.0f) {
        angleDegrees -= 360.0f;
    }

    while (angleDegrees <= -180.0f) {
        angleDegrees += 360.0f;
    }

    return angleDegrees;
}

static float AngleControl_Absolute(float value)
{
    return value < 0.0f ? -value : value;
}

static void AngleControl_ResetPid(float initialError)
{
    Angle_PID.error = initialError;
    Angle_PID.last_error = initialError;
    Angle_PID.integral = 0.0f;
}

static int TrackControl_ApplyErrorDeadband(int trackError)
{
    if (trackError >= -TRACK_ERROR_DEADBAND &&
        trackError <= TRACK_ERROR_DEADBAND) {
        return 0;
    }

    return trackError;
}

static float TrackControl_GetBaseSpeed(int trackError)
{
    int absoluteError = trackError < 0 ? -trackError : trackError;

    if (absoluteError <= 10) {
        return TRACK_BASE_SPEED;
    }

    if (absoluteError <= 20) {
        return 0.27f;
    }

    if (absoluteError <= 30) {
        return 0.24f;
    }

    if (absoluteError <= 40) {
        return 0.21f;
    }

    return TRACK_MIN_SPEED;
}

static bool TrackControl_IsCommandSeparator(char character)
{
    return character == ' ' || character == '\t' || character == ',' ||
        character == ';';
}

static bool TrackControl_ParseGain(const char *text, char **end, float *value)
{
    bool hasDigit = false;
    float parsedValue = 0.0f;
    float decimalPlace = 0.1f;

    if (*text == '+') {
        text++;
    }

    while (*text >= '0' && *text <= '9') {
        parsedValue = parsedValue * 10.0f + (float) (*text - '0');
        hasDigit = true;
        text++;
    }

    if (*text == '.') {
        text++;
        while (*text >= '0' && *text <= '9') {
            parsedValue += (float) (*text - '0') * decimalPlace;
            decimalPlace *= 0.1f;
            hasDigit = true;
            text++;
        }
    }

    if (!hasDigit) {
        return false;
    }

    *end = (char *) text;
    *value = parsedValue;
    return true;
}

static bool DriveControl_ParseTuningCommand(char *command,
    bool trackTuningEnabled, bool angleTuningEnabled)
{
    char *cursor = command;
    bool updateTrackKp = false;
    bool updateTrackKd = false;
    bool updateAngleKp = false;
    bool updateAngleKi = false;
    bool updateAngleKd = false;
    float trackKp = 0.0f;
    float trackKd = 0.0f;
    float angleKp = 0.0f;
    float angleKi = 0.0f;
    float angleKd = 0.0f;

    while (*cursor != '\0') {
        char *end;
        float value;

        while (TrackControl_IsCommandSeparator(*cursor)) {
            cursor++;
        }

        if (*cursor == '\0') {
            break;
        }

        if (strncmp(cursor, "KP:", 3U) == 0 && trackTuningEnabled) {
            if (!TrackControl_ParseGain(cursor + 3, &end, &value) ||
                value > TRACK_TUNING_GAIN_MAX) {
                return false;
            }

            trackKp = value;
            updateTrackKp = true;
            cursor = end;
        } else if (strncmp(cursor, "KD:", 3U) == 0 && trackTuningEnabled) {
            if (!TrackControl_ParseGain(cursor + 3, &end, &value) ||
                value > TRACK_TUNING_GAIN_MAX) {
                return false;
            }

            trackKd = value;
            updateTrackKd = true;
            cursor = end;
        } else if (strncmp(cursor, "AP:", 3U) == 0 && angleTuningEnabled) {
            if (!TrackControl_ParseGain(cursor + 3, &end, &value) ||
                value > TRACK_TUNING_GAIN_MAX) {
                return false;
            }

            angleKp = value;
            updateAngleKp = true;
            cursor = end;
        } else if (strncmp(cursor, "AI:", 3U) == 0 && angleTuningEnabled) {
            if (!TrackControl_ParseGain(cursor + 3, &end, &value) ||
                value > TRACK_TUNING_GAIN_MAX) {
                return false;
            }

            angleKi = value;
            updateAngleKi = true;
            cursor = end;
        } else if (strncmp(cursor, "AD:", 3U) == 0 && angleTuningEnabled) {
            if (!TrackControl_ParseGain(cursor + 3, &end, &value) ||
                value > TRACK_TUNING_GAIN_MAX) {
                return false;
            }

            angleKd = value;
            updateAngleKd = true;
            cursor = end;
        } else {
            return false;
        }
    }

    if (updateTrackKp || updateTrackKd || updateAngleKp || updateAngleKi ||
        updateAngleKd) {
        uint32_t interruptState = __get_PRIMASK();

        __disable_irq();
        if (updateTrackKp) {
            Track_PD.Kp = trackKp;
        }
        if (updateTrackKd) {
            Track_PD.Kd = trackKd;
        }
        if (updateTrackKp || updateTrackKd) {
            Track_PD.last_error = Track_PD.error;
        }
        if (updateAngleKp) {
            Angle_PID.Kp = angleKp;
        }
        if (updateAngleKi) {
            Angle_PID.Ki = angleKi;
        }
        if (updateAngleKd) {
            Angle_PID.Kd = angleKd;
        }
        if (updateAngleKp || updateAngleKi || updateAngleKd) {
            AngleControl_ResetPid(Angle_PID.error);
        }
        __set_PRIMASK(interruptState);
    }

    return updateTrackKp || updateTrackKd || updateAngleKp || updateAngleKi ||
        updateAngleKd;
}

static uint32_t DriveControl_DistanceMetersToPulses(float distanceMeters)
{
    float pulseCount = distanceMeters * (float) PULSES_PER_METER;

    if (pulseCount >= (float) UINT32_MAX) {
        return UINT32_MAX;
    }

    if (pulseCount < 1.0f) {
        return 1U;
    }

    return (uint32_t) (pulseCount + 0.5f);
}

static void DriveControl_Start(float targetSpeed, float distanceMeters)
{
    uint32_t interruptState;
    bool distanceLimited = distanceMeters > 0.0f;

    if (targetSpeed == 0.0f) {
        DriveControl_Stop();
        return;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    SpeedPI_Straight(0.0f, NULL, NULL);
    Encoder_Reset();
    leftPulse10ms = 0;
    rightPulse10ms = 0;
    distancePulseCount = 0U;
    distanceTargetPulses = distanceLimited ?
        DriveControl_DistanceMetersToPulses(distanceMeters) : 0U;
    controlSampleCount = 0U;
    driveTargetSpeedMps = targetSpeed;
    driveDistanceLimited = distanceLimited;
    angleControlActive = false;
    trackControlActive = false;
    driveControlActive = true;
    __set_PRIMASK(interruptState);
}

void DriveControl_StartStraight(float targetSpeed, float distanceMeters)
{
    DriveControl_Start(targetSpeed, distanceMeters);
}

void DriveControl_Tick10ms(void)
{
    uint32_t pulseIncrement;
    int16_t leftPulses;
    int16_t rightPulses;

    if (!driveControlActive) {
        return;
    }

    SpeedPI_Straight(driveTargetSpeedMps, &leftPulses, &rightPulses);

    leftPulse10ms = leftPulses;
    rightPulse10ms = rightPulses;
    pulseIncrement = ((uint32_t) AbsPulseCount(leftPulses) +
        AbsPulseCount(rightPulses)) / 2U;
    distancePulseCount += pulseIncrement;
    controlSampleCount++;

    if (driveDistanceLimited &&
        distancePulseCount >= distanceTargetPulses) {
        driveControlActive = false;
        driveTargetSpeedMps = 0.0f;
        SpeedPI_Straight(0.0f, NULL, NULL);
    }
}

void DriveControl_Stop(void)
{
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    driveControlActive = false;
    trackControlActive = false;
    angleControlActive = false;
    driveTargetSpeedMps = 0.0f;
    __set_PRIMASK(interruptState);

    SpeedPI_Straight(0.0f, NULL, NULL);
}

void DriveControl_GetTelemetry(DriveControlTelemetry *telemetry)
{
    uint32_t interruptState;

    if (telemetry == NULL) {
        return;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    telemetry->leftPulses10ms = leftPulse10ms;
    telemetry->rightPulses10ms = rightPulse10ms;
    telemetry->distancePulses = distancePulseCount;
    telemetry->sampleCount = controlSampleCount;
    telemetry->active = driveControlActive || trackControlActive ||
        angleControlActive;
    __set_PRIMASK(interruptState);
}

static bool AngleControl_Start(float targetSpeed, float targetYawDegrees,
    float distanceMeters, bool turnInPlace)
{
    YawControlFeedback feedback;
    float initialError;
    uint32_t interruptState;

    if (!IMU_UART_GetFeedback(&feedback)) {
        return false;
    }

    initialError = AngleControl_NormalizeAngle(
        targetYawDegrees - feedback.headingRelativeDeg);
    interruptState = __get_PRIMASK();
    __disable_irq();
    SpeedPI_Stop();
    Encoder_Reset();
    leftPulse10ms = 0;
    rightPulse10ms = 0;
    distancePulseCount = 0U;
    controlSampleCount = 0U;
    driveControlActive = false;
    trackControlActive = false;
    angleDistanceLimited = distanceMeters > 0.0f;
    angleDistanceTargetPulses = angleDistanceLimited ?
        DriveControl_DistanceMetersToPulses(distanceMeters) : 0U;
    angleTargetSpeedMps = targetSpeed;
    angleTargetYawDeg = AngleControl_NormalizeAngle(targetYawDegrees);
    angleControlTurnInPlace = turnInPlace;
    angleTurnStableCount = 0U;
    angleLastFeedbackSequence = feedback.sequence;
    angleFeedbackStaleTickCount = 0U;
    angleTurnSpeedCommand = 0.0f;
    AngleControl_ResetPid(initialError);
    angleControlActive = true;
    __set_PRIMASK(interruptState);

    return true;
}

void AngleControl_StartStraight(float targetSpeed, float distanceMeters)
{
    YawControlFeedback feedback;

    if (targetSpeed == 0.0f) {
        AngleControl_Stop();
        return;
    }

    if (!IMU_UART_GetFeedback(&feedback)) {
        AngleControl_Stop();
        return;
    }

    (void) AngleControl_Start(targetSpeed, feedback.headingRelativeDeg,
        distanceMeters, false);
}

void AngleControl_StartStraightToAngle(float targetSpeed, float distanceMeters,
    float targetAngleDegrees)
{
    if (targetSpeed == 0.0f ||
        !AngleControl_Start(targetSpeed,
            AngleControl_NormalizeAngle(targetAngleDegrees), distanceMeters,
            false)) {
        AngleControl_Stop();
    }
}

void AngleControl_Stop(void)
{
    DriveControl_Stop();
    angleTurnStableCount = 0U;
    angleFeedbackStaleTickCount = 0U;
    angleTurnSpeedCommand = 0.0f;
    AngleControl_ResetPid(0.0f);
}

void AngleControl_TurnAngle(float angleDegrees)
{
    YawControlFeedback feedback;

    if (!IMU_UART_GetFeedback(&feedback)) {
        AngleControl_Stop();
        return;
    }

    (void) AngleControl_Start(0.0f,
        AngleControl_NormalizeAngle(feedback.headingRelativeDeg + angleDegrees),
        0.0f, true);
}

void AngleControl_TurnToAngle(float targetAngleDegrees)
{
    if (!AngleControl_Start(0.0f,
        AngleControl_NormalizeAngle(targetAngleDegrees), 0.0f, true)) {
        AngleControl_Stop();
    }
}

void AngleControl_Tick10ms(void)
{
    YawControlFeedback feedback;
    int16_t leftPulses;
    int16_t rightPulses;
    float angleError;
    float turnSpeed;
    float leftTargetSpeed;
    float rightTargetSpeed;
    float turnSpeedLimit;
    float feedbackDtSeconds;
    bool hasNewFeedback;

    if (!angleControlActive) {
        return;
    }

    if (!IMU_UART_GetFeedback(&feedback)) {
        AngleControl_Stop();
        return;
    }

    hasNewFeedback = feedback.sequence != angleLastFeedbackSequence;
    if (hasNewFeedback) {
        feedbackDtSeconds = (float) (angleFeedbackStaleTickCount + 1U) * dt;
        angleLastFeedbackSequence = feedback.sequence;
        angleFeedbackStaleTickCount = 0U;
        angleError = AngleControl_NormalizeAngle(
            angleTargetYawDeg - feedback.headingRelativeDeg);

        if (angleControlTurnInPlace) {
            if (AngleControl_Absolute(angleError) <= ANGLE_TURN_TOLERANCE_DEG &&
                AngleControl_Absolute(feedback.yawRateDps) <=
                    ANGLE_TURN_SETTLE_RATE_DPS) {
                if (angleTurnStableCount < ANGLE_TURN_SETTLE_COUNT) {
                    angleTurnStableCount++;
                }
            } else {
                angleTurnStableCount = 0U;
            }

            if (angleTurnStableCount >= ANGLE_TURN_SETTLE_COUNT) {
                AngleControl_Stop();
                return;
            }
        }

        turnSpeedLimit = angleControlTurnInPlace ? ANGLE_TURN_SPEED_LIMIT :
            ANGLE_STRAIGHT_TURN_SPEED_LIMIT;
        turnSpeed = Angle_PID_Control(&Angle_PID, angleError,
            feedback.yawRateDps, feedbackDtSeconds, turnSpeedLimit);

        if (angleControlTurnInPlace) {
            if (AngleControl_Absolute(angleError) > ANGLE_TURN_TOLERANCE_DEG &&
                AngleControl_Absolute(turnSpeed) < ANGLE_TURN_MIN_SPEED) {
                turnSpeed = angleError >= 0.0f ? ANGLE_TURN_MIN_SPEED :
                    -ANGLE_TURN_MIN_SPEED;
            }
        }

        angleTurnSpeedCommand = turnSpeed;
    } else {
        if (angleFeedbackStaleTickCount < UINT8_MAX) {
            angleFeedbackStaleTickCount++;
        }

        if (angleFeedbackStaleTickCount > ANGLE_IMU_STALE_TICK_LIMIT) {
            AngleControl_Stop();
            return;
        }
    }

    turnSpeed = angleTurnSpeedCommand;
    if (angleControlTurnInPlace) {
        leftTargetSpeed = -turnSpeed;
        rightTargetSpeed = turnSpeed;
    } else {
        leftTargetSpeed = angleTargetSpeedMps - turnSpeed;
        rightTargetSpeed = angleTargetSpeedMps + turnSpeed;
    }

    SpeedPI_Differential(leftTargetSpeed, rightTargetSpeed,
        &leftPulses, &rightPulses);
    leftPulse10ms = leftPulses;
    rightPulse10ms = rightPulses;
    controlSampleCount++;

    if (angleDistanceLimited) {
        distancePulseCount += ((uint32_t) AbsPulseCount(leftPulses) +
            AbsPulseCount(rightPulses)) / 2U;
        if (distancePulseCount >= angleDistanceTargetPulses) {
            AngleControl_Stop();
        }
    }
}

void TrackControl_Start(void)
{
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    SpeedPI_Stop();
    Encoder_Reset();
    leftPulse10ms = 0;
    rightPulse10ms = 0;
    distancePulseCount = 0U;
    controlSampleCount = 0U;
    driveControlActive = false;
    angleControlActive = false;
    driveTargetSpeedMps = 0.0f;
    Track_PD.error = 0.0f;
    Track_PD.last_error = 0.0f;
    trackControlActive = true;
    __set_PRIMASK(interruptState);
}

void TrackControl_Stop(void)
{
    DriveControl_Stop();
    Track_PD.error = 0.0f;
    Track_PD.last_error = 0.0f;
}

void TrackControl_ProcessTuning(void)
{
    uint8_t data;

    while (UART_BLUETOOTH_TryReceiveByte(&data)) {
        if (!trackControlActive && !angleControlActive) {
            trackTuningCommandLength = 0U;
            continue;
        }

        if (data == '\r' || data == '\n') {
            if (trackTuningCommandLength != 0U) {
                trackTuningCommand[trackTuningCommandLength] = '\0';
                (void) DriveControl_ParseTuningCommand(trackTuningCommand,
                    trackControlActive, angleControlActive);
                trackTuningCommandLength = 0U;
            }
        } else if (data >= 0x20U && data <= 0x7EU) {
            if (trackTuningCommandLength <
                (TRACK_TUNING_COMMAND_LENGTH - 1U)) {
                trackTuningCommand[trackTuningCommandLength] = (char) data;
                trackTuningCommandLength++;
            } else {
                trackTuningCommandLength = 0U;
            }
        } else {
            trackTuningCommandLength = 0U;
        }
    }
}

void TrackControl_Tick10ms(void)
{
    int trackError;
    int16_t leftPulses;
    int16_t rightPulses;
    float baseSpeed;
    float turnSpeed;
    float leftTargetSpeed;
    float rightTargetSpeed;

    if (!trackControlActive) {
        return;
    }

    trackError = Get_Track_Error();
    baseSpeed = TrackControl_GetBaseSpeed(trackError);
    trackError = TrackControl_ApplyErrorDeadband(trackError);
    turnSpeed = Track_PD_Control(&Track_PD, (float) trackError);
    turnSpeed = Track_Limit(turnSpeed,
        -TRACK_TURN_LIMIT, TRACK_TURN_LIMIT);

    leftTargetSpeed = baseSpeed + turnSpeed;
    rightTargetSpeed = baseSpeed - turnSpeed;

    SpeedPI_Differential(leftTargetSpeed, rightTargetSpeed,
        &leftPulses, &rightPulses);
    leftPulse10ms = leftPulses;
    rightPulse10ms = rightPulses;
    controlSampleCount++;
}
