#include "loop.h"
#include "ti_msp_dl_config.h"

int active_sensors = 0;

static uint8_t loopSensors[8];

static void Loop_SelectChannel(uint8_t channel)
{
    const uint32_t addressPins = GPIO_LOOP_AD0_PIN |
        GPIO_LOOP_AD1_PIN |
        GPIO_LOOP_AD2_PIN;
    uint32_t addressValue = 0U;

    if ((channel & 0x01U) != 0U) {
        addressValue |= GPIO_LOOP_AD0_PIN;
    }

    if ((channel & 0x02U) != 0U) {
        addressValue |= GPIO_LOOP_AD1_PIN;
    }

    if ((channel & 0x04U) != 0U) {
        addressValue |= GPIO_LOOP_AD2_PIN;
    }

    DL_GPIO_clearPins(GPIO_LOOP_PORT, addressPins);
    DL_GPIO_setPins(GPIO_LOOP_PORT, addressValue);
}

void Read_XJSensor(void)
{
    uint8_t channel;

    for (channel = 0U; channel < 8U; channel++) {
        uint16_t delay;

        Loop_SelectChannel(channel);

        for (delay = 0U; delay < 800U; delay++) {
            __NOP();
        }

        loopSensors[channel] =
            (DL_GPIO_readPins(GPIO_LOOP_PORT, GPIO_LOOP_INPUT_PIN) != 0U) ?
                1U : 0U;
    }
}

int Get_Track_Error(void)
{
    static const int weights[8] = {-40, -30, -20, -10, 10, 20, 30, 40};
    static int lastError = 0;
    int sumWeights = 0;
    int currentError;
    uint8_t channel;

    Read_XJSensor();
    active_sensors = 0;

    for (channel = 0U; channel < 8U; channel++) {
        if (loopSensors[channel] != 0U) {
            sumWeights += weights[channel];
            active_sensors++;
        }
    }

    if (active_sensors > 0) {
        currentError = sumWeights / active_sensors;
        lastError = currentError;
    } else if (lastError > 0) {
        currentError = 50;
    } else if (lastError < 0) {
        currentError = -50;
    } else {
        currentError = 0;
    }

    return currentError;
}
