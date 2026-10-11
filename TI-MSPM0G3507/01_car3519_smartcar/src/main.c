#include "button.h"
#include "control_timer.h"
#include "drive_control.h"
#include "LED.h"
#include "encoder.h"
#include "imu_uart.h"
#include "motor.h"
#include "oled.h"
#include "task1.h"
#include "uart_bluetooth.h"
#include <stdio.h>
#include "ti_msp_dl_config.h"

static void Main_ShowDiagnostics(const IMU_UART_Diagnostics *imuDiagnostics)
{
    char receivedText[22];
    char errorText[22];
    char statusText[22];
    char bluetoothText[22];

    snprintf(receivedText, sizeof(receivedText), "RX:%lu F:%lu",
        (unsigned long) imuDiagnostics->receivedByteCount,
        (unsigned long) imuDiagnostics->validFrameCount);
    snprintf(errorText, sizeof(errorText), "CK:%lu FM:%lu",
        (unsigned long) imuDiagnostics->checksumErrorCount,
        (unsigned long) imuDiagnostics->formatErrorCount);
    snprintf(statusText, sizeof(statusText), "OK:%u RD:%u E:%d",
        imuDiagnostics->feedbackValid ? 1U : 0U,
        imuDiagnostics->ready ? 1U : 0U,
        (int) imuDiagnostics->errorCode);
    snprintf(bluetoothText, sizeof(bluetoothText), "FI:%u IR:%u B:%c V:%u",
        imuDiagnostics->rxFifoHasData ? 1U : 0U,
        imuDiagnostics->rxInterruptPending ? 1U : 0U,
        UART_BLUETOOTH_IsDmaTransmitBusy() ? 'B' : '-',
        imuDiagnostics->vofaEnabled ? 1U : 0U);

    OLED_NewFrame();
    OLED_PrintASCIIString(0U, 0U, receivedText, &afont8x6,
        OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0U, 16U, errorText, &afont8x6,
        OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0U, 32U, statusText, &afont8x6,
        OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0U, 48U, bluetoothText, &afont8x6,
        OLED_COLOR_NORMAL);
    OLED_ShowFrame();
}

int main(void)
{
    DriveControlTelemetry telemetry;
    IMU_UART_Diagnostics imuDiagnostics;
    uint32_t lastDisplayedSample = 0U;
    uint32_t lastDisplayedReceivedBytes = UINT32_MAX;
    uint32_t lastDisplayedValidFrames = UINT32_MAX;
    uint32_t lastDisplayedChecksumErrors = UINT32_MAX;
    uint32_t lastDisplayedFormatErrors = UINT32_MAX;
    bool lastDisplayedFeedbackValid = false;
    bool lastDisplayedVofaEnabled = false;
    bool lastDisplayedBluetoothBusy = false;
    bool lastDisplayedImuFifoHasData = false;
    bool lastDisplayedImuInterruptPending = false;
    bool displayRequested = true;

    SYSCFG_DL_init();
    Button_Init();
    Motor_Init();
    Encoder_Init();
    UART_BLUETOOTH_Init();
    IMU_UART_Init();
    IMU_UART_SetVofaEnabled(true);
    OLED_Init();
    ControlTimer_Init();


    while (1) {
        uint32_t buttonEvents = Button_PollEvents();

        if ((buttonEvents & BUTTON_EVENT_KEY0_PRESSED) != 0U) {
            Task1_Start();
            LED_Blue_On();
            displayRequested = true;
        }

        if ((buttonEvents & BUTTON_EVENT_KEY1_PRESSED) != 0U) {
            Task1_Stop();
            TrackControl_Start();
            LED_Green_On();
            displayRequested = true;
        }

        if ((buttonEvents & BUTTON_EVENT_KEY2_PRESSED) != 0U) {
            Task1_Stop();
            TrackControl_Stop();
            LED_Red_On();
            displayRequested = true;
        }

        TrackControl_ProcessTuning();
        IMU_UART_Process();
        Task1_Tick();

        DriveControl_GetTelemetry(&telemetry);
        IMU_UART_GetDiagnostics(&imuDiagnostics);
        if (displayRequested ||
            (imuDiagnostics.receivedByteCount != lastDisplayedReceivedBytes &&
            (imuDiagnostics.receivedByteCount - lastDisplayedReceivedBytes) >=
                10U) ||
            (imuDiagnostics.validFrameCount != lastDisplayedValidFrames &&
            (imuDiagnostics.validFrameCount - lastDisplayedValidFrames) >= 10U) ||
            imuDiagnostics.checksumErrorCount != lastDisplayedChecksumErrors ||
            imuDiagnostics.formatErrorCount != lastDisplayedFormatErrors ||
            imuDiagnostics.feedbackValid != lastDisplayedFeedbackValid ||
            imuDiagnostics.vofaEnabled != lastDisplayedVofaEnabled ||
            imuDiagnostics.rxFifoHasData != lastDisplayedImuFifoHasData ||
            imuDiagnostics.rxInterruptPending !=
                lastDisplayedImuInterruptPending ||
            UART_BLUETOOTH_IsDmaTransmitBusy() != lastDisplayedBluetoothBusy ||
            (telemetry.sampleCount != lastDisplayedSample &&
            (!telemetry.active ||
            (telemetry.sampleCount - lastDisplayedSample) >= 10U))) {
            Main_ShowDiagnostics(&imuDiagnostics);
            lastDisplayedSample = telemetry.sampleCount;
            lastDisplayedReceivedBytes = imuDiagnostics.receivedByteCount;
            lastDisplayedValidFrames = imuDiagnostics.validFrameCount;
            lastDisplayedChecksumErrors = imuDiagnostics.checksumErrorCount;
            lastDisplayedFormatErrors = imuDiagnostics.formatErrorCount;
            lastDisplayedFeedbackValid = imuDiagnostics.feedbackValid;
            lastDisplayedVofaEnabled = imuDiagnostics.vofaEnabled;
            lastDisplayedImuFifoHasData = imuDiagnostics.rxFifoHasData;
            lastDisplayedImuInterruptPending =
                imuDiagnostics.rxInterruptPending;
            lastDisplayedBluetoothBusy = UART_BLUETOOTH_IsDmaTransmitBusy();
            displayRequested = false;
        }
    }
}
