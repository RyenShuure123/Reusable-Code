#include "task1.h"

#include "drive_control.h"
#include "imu_uart.h"

#define TASK1_STRAIGHT_SPEED 0.30f
#define TASK1_SQUARE_SIDE_METERS 1.0f
#define TASK1_HEADING_FIRST_SIDE 0.0f
#define TASK1_HEADING_SECOND_SIDE 90.0f
#define TASK1_HEADING_THIRD_SIDE 180.0f
#define TASK1_HEADING_FOURTH_SIDE -90.0f

typedef enum {
    TASK1_STEP_FIRST_SIDE,
    TASK1_STEP_FIRST_LEFT_TURN,
    TASK1_STEP_SECOND_SIDE,
    TASK1_STEP_SECOND_LEFT_TURN,
    TASK1_STEP_THIRD_SIDE,
    TASK1_STEP_THIRD_LEFT_TURN,
    TASK1_STEP_FOURTH_SIDE,
    TASK1_STEP_FOURTH_LEFT_TURN,
    TASK1_STEP_COUNT
} Task1Step;

static bool task1Active;
static Task1Step task1NextStep;

static void Task1_StartNextStep(void)
{
    switch (task1NextStep) {
    case TASK1_STEP_FIRST_SIDE:
        AngleControl_StartStraightToAngle(TASK1_STRAIGHT_SPEED,
            TASK1_SQUARE_SIDE_METERS, TASK1_HEADING_FIRST_SIDE);
        break;

    case TASK1_STEP_FIRST_LEFT_TURN:
        AngleControl_TurnToAngle(TASK1_HEADING_SECOND_SIDE);
        break;

    case TASK1_STEP_SECOND_SIDE:
        AngleControl_StartStraightToAngle(TASK1_STRAIGHT_SPEED,
            TASK1_SQUARE_SIDE_METERS, TASK1_HEADING_SECOND_SIDE);
        break;

    case TASK1_STEP_SECOND_LEFT_TURN:
        AngleControl_TurnToAngle(TASK1_HEADING_THIRD_SIDE);
        break;

    case TASK1_STEP_THIRD_SIDE:
        AngleControl_StartStraightToAngle(TASK1_STRAIGHT_SPEED,
            TASK1_SQUARE_SIDE_METERS, TASK1_HEADING_THIRD_SIDE);
        break;

    case TASK1_STEP_THIRD_LEFT_TURN:
        AngleControl_TurnToAngle(TASK1_HEADING_FOURTH_SIDE);
        break;

    case TASK1_STEP_FOURTH_SIDE:
        AngleControl_StartStraightToAngle(TASK1_STRAIGHT_SPEED,
            TASK1_SQUARE_SIDE_METERS, TASK1_HEADING_FOURTH_SIDE);
        break;

    case TASK1_STEP_FOURTH_LEFT_TURN:
        AngleControl_TurnToAngle(TASK1_HEADING_FIRST_SIDE);
        break;

    default:
        Task1_Stop();
        return;
    }

    task1NextStep = (Task1Step) ((task1NextStep + 1U) % TASK1_STEP_COUNT);
}

void Task1_Start(void)
{
    AngleControl_Stop();

    if (!IMU_UART_HasValidFeedback()) {
        task1Active = false;
        return;
    }

    task1Active = true;
    task1NextStep = TASK1_STEP_FIRST_SIDE;
    Task1_StartNextStep();
}


void Task1_Stop(void)
{
    task1Active = false;
    AngleControl_Stop();
}

void Task1_Tick(void)
{
    DriveControlTelemetry telemetry;

    if (!task1Active) {
        return;
    }

    if (!IMU_UART_HasValidFeedback()) {
        Task1_Stop();
        return;
    }

    DriveControl_GetTelemetry(&telemetry);
    if (telemetry.active) {
        return;
    }

    Task1_StartNextStep();
}
