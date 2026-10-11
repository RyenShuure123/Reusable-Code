#ifndef CAR3519NEW_YAW_UART_RECEIVER_H
#define CAR3519NEW_YAW_UART_RECEIVER_H

#include <stdbool.h>
#include <stdint.h>

#include "yaw_uart_protocol.h"

typedef struct {
    uint8_t sequence;
    bool ready;
    bool still;
    bool error;
    int8_t errorCode;
    float headingRelativeDeg;
    float yawRateDps;
    float rollDeg;
    float pitchDeg;
} YawControlFeedback;

typedef struct {
    uint8_t frameBuffer[YAW_UART_FRAME_LEN];
    uint8_t index;
    uint8_t lastSequence;
    uint32_t validFrameCount;
    uint32_t checksumErrorCount;
    uint32_t formatErrorCount;
    uint32_t lostFrameCount;
    bool hasPacket;
    bool fresh;
    YawUartPacket lastPacket;
} YawUartReceiver;

void YawUartReceiver_Init(YawUartReceiver *receiver);
bool YawUartReceiver_PushByte(YawUartReceiver *receiver, uint8_t data);
bool YawUartReceiver_HasFreshPacket(const YawUartReceiver *receiver);
void YawUartReceiver_ClearFresh(YawUartReceiver *receiver);
bool YawUartReceiver_GetLatestPacket(
    const YawUartReceiver *receiver, YawUartPacket *packet);
void YawUartReceiver_PacketToFeedback(
    const YawUartPacket *packet, YawControlFeedback *feedback);

#endif
