#include "vofa_justfloat.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "uart_bluetooth.h"

#define VOFA_JUSTFLOAT_FRAME_TAIL_SIZE 4U
#define VOFA_JUSTFLOAT_MAX_FLOAT_COUNT 6U
#define VOFA_JUSTFLOAT_FRAME_SIZE \
    ((VOFA_JUSTFLOAT_MAX_FLOAT_COUNT * sizeof(float)) + \
        VOFA_JUSTFLOAT_FRAME_TAIL_SIZE)

static const uint8_t g_vofaJustFloatFrameTail[VOFA_JUSTFLOAT_FRAME_TAIL_SIZE] = {
    0x00U,
    0x00U,
    0x80U,
    0x7FU,
};

static uint8_t g_vofaJustFloatDmaBuffer[VOFA_JUSTFLOAT_FRAME_SIZE];

static bool VOFA_JustFloat_Send(const float *values, uint8_t valueCount)
{
    uint8_t index;
    uint16_t payloadSize;
    uint16_t frameSize;

    if ((values == NULL) || (valueCount > VOFA_JUSTFLOAT_MAX_FLOAT_COUNT) ||
        UART_BLUETOOTH_IsDmaTransmitBusy()) {
        return false;
    }

    for (index = 0U; index < valueCount; index++) {
        memcpy(
            &g_vofaJustFloatDmaBuffer[index * sizeof(float)],
            &values[index],
            sizeof(float));
    }

    payloadSize = (uint16_t) valueCount * sizeof(float);
    memcpy(
        &g_vofaJustFloatDmaBuffer[payloadSize],
        g_vofaJustFloatFrameTail,
        VOFA_JUSTFLOAT_FRAME_TAIL_SIZE);
    frameSize = payloadSize + VOFA_JUSTFLOAT_FRAME_TAIL_SIZE;

    return UART_BLUETOOTH_StartDmaTransmit(
        g_vofaJustFloatDmaBuffer,
        frameSize);
}

bool VOFA_JustFloat_Send4(float f1, float f2, float f3, float f4)
{
    const float values[4] = {f1, f2, f3, f4};

    return VOFA_JustFloat_Send(values, 4U);
}

bool VOFA_JustFloat_Send3(float f1, float f2, float f3)
{
    const float values[3] = {f1, f2, f3};

    return VOFA_JustFloat_Send(values, 3U);
}

bool VOFA_JustFloat_Send6(
    float f1, float f2, float f3, float f4, float f5, float f6)
{
    const float values[6] = {f1, f2, f3, f4, f5, f6};

    return VOFA_JustFloat_Send(values, 6U);
}
