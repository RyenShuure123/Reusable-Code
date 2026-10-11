#include "yaw_uart_receiver.h"

#include <string.h>

static uint8_t YawUartReceiver_ResetByHeader(uint8_t data)
{
    return data == YAW_UART_FRAME_HEAD_0 ? 1U : 0U;
}

static int16_t YawUartReceiver_ReadInt16(const uint8_t *data)
{
    return (int16_t) ((uint16_t) data[0] | ((uint16_t) data[1] << 8U));
}

static bool YawUartReceiver_ParseFrame(
    const uint8_t *frame, YawUartPacket *packet)
{
    if (frame[0] != YAW_UART_FRAME_HEAD_0 ||
        frame[1] != YAW_UART_FRAME_HEAD_1 ||
        frame[2] != YAW_UART_PROTOCOL_VERSION ||
        frame[3] != YAW_UART_FRAME_LEN ||
        frame[YAW_UART_FRAME_LEN - 1U] !=
            YawUart_Checksum(frame, YAW_UART_FRAME_LEN - 1U)) {
        return false;
    }

    packet->seq = frame[4];
    packet->status = frame[5];
    packet->headingRelativeCdeg = YawUartReceiver_ReadInt16(&frame[6]);
    packet->yawRateCdegps = YawUartReceiver_ReadInt16(&frame[8]);
    packet->rollCdeg = YawUartReceiver_ReadInt16(&frame[10]);
    packet->pitchCdeg = YawUartReceiver_ReadInt16(&frame[12]);
    packet->errorCode = (int8_t) frame[14];
    return true;
}

void YawUartReceiver_Init(YawUartReceiver *receiver)
{
    memset(receiver, 0, sizeof(*receiver));
}

bool YawUartReceiver_PushByte(YawUartReceiver *receiver, uint8_t data)
{
    YawUartPacket packet;

    switch (receiver->index) {
    case 0U:
        if (data == YAW_UART_FRAME_HEAD_0) {
            receiver->frameBuffer[receiver->index++] = data;
        }
        return false;

    case 1U:
        if (data != YAW_UART_FRAME_HEAD_1) {
            receiver->formatErrorCount++;
            receiver->index = YawUartReceiver_ResetByHeader(data);
            if (receiver->index != 0U) {
                receiver->frameBuffer[0] = data;
            }
            return false;
        }
        break;

    case 2U:
        if (data != YAW_UART_PROTOCOL_VERSION) {
            receiver->formatErrorCount++;
            receiver->index = YawUartReceiver_ResetByHeader(data);
            if (receiver->index != 0U) {
                receiver->frameBuffer[0] = data;
            }
            return false;
        }
        break;

    case 3U:
        if (data != YAW_UART_FRAME_LEN) {
            receiver->formatErrorCount++;
            receiver->index = YawUartReceiver_ResetByHeader(data);
            if (receiver->index != 0U) {
                receiver->frameBuffer[0] = data;
            }
            return false;
        }
        break;

    default:
        break;
    }

    receiver->frameBuffer[receiver->index++] = data;
    if (receiver->index < YAW_UART_FRAME_LEN) {
        return false;
    }

    receiver->index = 0U;
    if (receiver->frameBuffer[YAW_UART_FRAME_LEN - 1U] !=
        YawUart_Checksum(receiver->frameBuffer, YAW_UART_FRAME_LEN - 1U)) {
        receiver->checksumErrorCount++;
        return false;
    }

    if (!YawUartReceiver_ParseFrame(receiver->frameBuffer, &packet)) {
        receiver->formatErrorCount++;
        return false;
    }

    if (receiver->hasPacket &&
        (uint8_t) (packet.seq - receiver->lastSequence) > 1U) {
        receiver->lostFrameCount +=
            (uint8_t) (packet.seq - receiver->lastSequence - 1U);
    }

    receiver->lastSequence = packet.seq;
    receiver->lastPacket = packet;
    receiver->hasPacket = true;
    receiver->fresh = true;
    receiver->validFrameCount++;
    return true;
}

bool YawUartReceiver_HasFreshPacket(const YawUartReceiver *receiver)
{
    return receiver->fresh;
}

void YawUartReceiver_ClearFresh(YawUartReceiver *receiver)
{
    receiver->fresh = false;
}

bool YawUartReceiver_GetLatestPacket(
    const YawUartReceiver *receiver, YawUartPacket *packet)
{
    if (!receiver->hasPacket) {
        return false;
    }

    *packet = receiver->lastPacket;
    return true;
}

void YawUartReceiver_PacketToFeedback(
    const YawUartPacket *packet, YawControlFeedback *feedback)
{
    feedback->sequence = packet->seq;
    feedback->ready = (packet->status & YAW_UART_STATUS_READY) != 0U;
    feedback->still = (packet->status & YAW_UART_STATUS_STILL) != 0U;
    feedback->error = (packet->status & YAW_UART_STATUS_ERROR) != 0U;
    feedback->errorCode = packet->errorCode;
    feedback->headingRelativeDeg =
        YawUart_CentiDegToDeg(packet->headingRelativeCdeg);
    feedback->yawRateDps = YawUart_CentiDegToDeg(packet->yawRateCdegps);
    feedback->rollDeg = YawUart_CentiDegToDeg(packet->rollCdeg);
    feedback->pitchDeg = YawUart_CentiDegToDeg(packet->pitchCdeg);
}
