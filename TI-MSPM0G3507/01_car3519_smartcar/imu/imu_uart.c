#include "imu_uart.h"

#include "ti_msp_dl_config.h"
#include "vofa_justfloat.h"

static YawUartReceiver s_receiver;
static YawControlFeedback s_feedback;
static volatile bool s_feedbackValid;
static volatile bool s_vofaEnabled;
static volatile bool s_vofaPending;
static volatile bool s_vofaStarted;
static volatile uint32_t s_receivedByteCount;
static float s_yawZeroDeg;
static bool s_yawZeroInitialized;

static float IMU_UART_NormalizeAngle(float angleDeg)
{
    while (angleDeg > 180.0f) {
        angleDeg -= 360.0f;
    }

    while (angleDeg <= -180.0f) {
        angleDeg += 360.0f;
    }

    return angleDeg;
}

static void IMU_UART_StoreReceivedByte(uint8_t data)
{
    YawUartPacket packet;
    YawControlFeedback feedback;
    bool feedbackValid;
    uint32_t interruptState;

    s_receivedByteCount++;
    if (!YawUartReceiver_PushByte(&s_receiver, data) ||
        !YawUartReceiver_GetLatestPacket(&s_receiver, &packet)) {
        return;
    }

    YawUartReceiver_PacketToFeedback(&packet, &feedback);
    feedbackValid = feedback.ready && !feedback.error;

    if (feedbackValid) {
        if (!s_yawZeroInitialized) {
            s_yawZeroDeg = feedback.headingRelativeDeg;
            s_yawZeroInitialized = true;
        }

        feedback.headingRelativeDeg = IMU_UART_NormalizeAngle(
            feedback.headingRelativeDeg - s_yawZeroDeg);
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    s_feedback = feedback;
    s_feedbackValid = feedbackValid;
    if (s_vofaEnabled && feedbackValid) {
        if (feedback.still) {
            s_vofaStarted = true;
        }

        if (s_vofaStarted) {
            s_vofaPending = true;
        }
    }
    __set_PRIMASK(interruptState);
}

void IMU_UART_Init(void)
{
    YawUartReceiver_Init(&s_receiver);
    s_feedbackValid = false;
    s_vofaEnabled = false;
    s_vofaPending = false;
    s_vofaStarted = false;
    s_receivedByteCount = 0U;
    s_yawZeroDeg = 0.0f;
    s_yawZeroInitialized = false;
    NVIC_EnableIRQ(UART_IMU_INST_INT_IRQN);
}

void IMU_UART_SetVofaEnabled(bool enabled)
{
    s_vofaEnabled = enabled;
    if (!enabled) {
        s_vofaPending = false;
        s_vofaStarted = false;
    }
}

void IMU_UART_Process(void)
{
    YawControlFeedback feedback;
    bool shouldSend = false;
    uint32_t interruptState = __get_PRIMASK();

    __disable_irq();
    if (s_vofaPending && s_feedbackValid) {
        feedback = s_feedback;
        s_vofaPending = false;
        shouldSend = true;
    }
    __set_PRIMASK(interruptState);

    if (!shouldSend) {
        return;
    }

    if (s_vofaEnabled) {
        (void) VOFA_JustFloat_Send3(
            feedback.rollDeg,
            feedback.pitchDeg,
            feedback.headingRelativeDeg);
    }
}

bool IMU_UART_GetFeedback(YawControlFeedback *feedback)
{
    uint32_t interruptState;

    if (feedback == NULL || !s_feedbackValid) {
        return false;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    *feedback = s_feedback;
    __set_PRIMASK(interruptState);
    return true;
}

bool IMU_UART_HasValidFeedback(void)
{
    return s_feedbackValid;
}

void IMU_UART_GetDiagnostics(IMU_UART_Diagnostics *diagnostics)
{
    uint32_t interruptState;

    if (diagnostics == NULL) {
        return;
    }

    interruptState = __get_PRIMASK();
    __disable_irq();
    diagnostics->receivedByteCount = s_receivedByteCount;
    diagnostics->validFrameCount = s_receiver.validFrameCount;
    diagnostics->checksumErrorCount = s_receiver.checksumErrorCount;
    diagnostics->formatErrorCount = s_receiver.formatErrorCount;
    diagnostics->lostFrameCount = s_receiver.lostFrameCount;
    diagnostics->feedbackValid = s_feedbackValid;
    diagnostics->ready = s_feedback.ready;
    diagnostics->error = s_feedback.error;
    diagnostics->errorCode = s_feedback.errorCode;
    diagnostics->vofaEnabled = s_vofaEnabled;
    diagnostics->rxFifoHasData =
        !DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST);
    diagnostics->rxInterruptPending =
        DL_UART_Main_getRawInterruptStatus(UART_IMU_INST,
            DL_UART_MAIN_INTERRUPT_RX) != 0U;
    __set_PRIMASK(interruptState);
}

void UART_IMU_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_IMU_INST)) {
    case DL_UART_MAIN_IIDX_RX:
        while (!DL_UART_Main_isRXFIFOEmpty(UART_IMU_INST)) {
            IMU_UART_StoreReceivedByte(
                DL_UART_Main_receiveData(UART_IMU_INST));
        }
        break;

    default:
        break;
    }
}
