#ifndef CAR3519NEW_YAW_UART_PROTOCOL_H
#define CAR3519NEW_YAW_UART_PROTOCOL_H

#include <stdint.h>

#define YAW_UART_PROTOCOL_VERSION 0x01U
#define YAW_UART_FRAME_HEAD_0 0xA5U
#define YAW_UART_FRAME_HEAD_1 0x5AU
#define YAW_UART_FRAME_LEN 16U

#define YAW_UART_STATUS_READY 0x01U
#define YAW_UART_STATUS_STILL 0x02U
#define YAW_UART_STATUS_ERROR 0x80U

typedef struct {
    uint8_t seq;
    uint8_t status;
    int16_t headingRelativeCdeg;
    int16_t yawRateCdegps;
    int16_t rollCdeg;
    int16_t pitchCdeg;
    int8_t errorCode;
} YawUartPacket;

static inline uint8_t YawUart_Checksum(const uint8_t *data, uint32_t length)
{
    uint8_t checksum = 0U;
    uint32_t index;

    for (index = 0U; index < length; index++) {
        checksum = (uint8_t) (checksum + data[index]);
    }

    return checksum;
}

static inline float YawUart_CentiDegToDeg(int16_t value)
{
    return (float) value * 0.01f;
}

#endif
