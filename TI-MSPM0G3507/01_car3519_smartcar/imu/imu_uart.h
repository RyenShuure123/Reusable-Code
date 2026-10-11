#ifndef CAR3519NEW_IMU_UART_H
#define CAR3519NEW_IMU_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "yaw_uart_receiver.h"

typedef struct {
    uint32_t receivedByteCount;
    uint32_t validFrameCount;
    uint32_t checksumErrorCount;
    uint32_t formatErrorCount;
    uint32_t lostFrameCount;
    bool feedbackValid;
    bool ready;
    bool error;
    int8_t errorCode;
    bool vofaEnabled;
    bool rxFifoHasData;
    bool rxInterruptPending;
} IMU_UART_Diagnostics;

void IMU_UART_Init(void);
void IMU_UART_Process(void);
void IMU_UART_SetVofaEnabled(bool enabled);
bool IMU_UART_GetFeedback(YawControlFeedback *feedback);
bool IMU_UART_HasValidFeedback(void);
void IMU_UART_GetDiagnostics(IMU_UART_Diagnostics *diagnostics);

#endif
