#include "main.h"
#include "loop.h"
#include "vofa+.h"

static LoopSensor_t loop_sensor = {
    .value = {0},
    .active_count = 0,
    .lost_line_cycles = 0,
    .last_error = 0
};

static const int loop_weights[LOOP_SENSOR_NUM] = {-42, -18, 0, 18, 42};

static GPIO_TypeDef *const loop_sensor_ports[LOOP_SENSOR_NUM] = {
    LINE5_LEFT_OUT_GPIO_Port,
    LINE5_LEFT_IN_GPIO_Port,
    LINE5_CENTER_OUT_GPIO_Port,
    LINE5_RIGHT_IN_GPIO_Port,
    LINE5_RIGHT_OUT_GPIO_Port
};

static const uint16_t loop_sensor_pins[LOOP_SENSOR_NUM] = {
    LINE5_LEFT_OUT_Pin,
    LINE5_LEFT_IN_Pin,
    LINE5_CENTER_OUT_Pin,
    LINE5_RIGHT_IN_Pin,
    LINE5_RIGHT_OUT_Pin
};

static int Loop_Calculate_Error(void)
{
    int sum_weights = 0;

    for (uint8_t index = 0; index < LOOP_SENSOR_NUM; index++) {
        if (loop_sensor.value[index] != 0U) {
            sum_weights += loop_weights[index];
        }
    }

    if (loop_sensor.active_count == 0U) {
        if (loop_sensor.lost_line_cycles < LOOP_LOST_LINE_TOLERANCE_CYCLES) {
            ++loop_sensor.lost_line_cycles;
            return loop_sensor.last_error;
        }

        if (loop_sensor.last_error > 0) {
            return 50;
        }
        if (loop_sensor.last_error < 0) {
            return -50;
        }
        return 0;
    }

    /* Four or five active sensors usually means a wide line or junction. */
    if (loop_sensor.active_count >= 4U) {
        loop_sensor.lost_line_cycles = 0U;
        return 0;
    }

    return sum_weights / loop_sensor.active_count;
}

void Read_XJSensor(void)
{
    loop_sensor.active_count = 0U;

    for (uint8_t index = 0; index < LOOP_SENSOR_NUM; index++) {
        GPIO_PinState pin_state = HAL_GPIO_ReadPin(
            loop_sensor_ports[index], loop_sensor_pins[index]);

        if (pin_state == LOOP_SENSOR_ACTIVE_LEVEL) {
            loop_sensor.value[index] = 1U;
            loop_sensor.active_count++;
        } else {
            loop_sensor.value[index] = 0U;
        }
    }
}

int Get_Track_Error(void)
{
    int current_error;

    Read_XJSensor();
    current_error = Loop_Calculate_Error();

    if (loop_sensor.active_count > 0U && loop_sensor.active_count < 4U) {
        loop_sensor.lost_line_cycles = 0U;
        loop_sensor.last_error = current_error;
    }

    return current_error;
}

void Loop_VOFA_Debug_Send(void)
{
    static uint16_t vofa_cnt = 0U;
    vofa_cnt++;

    Read_XJSensor();

    if (vofa_cnt >= 10U) {
        vofa_cnt = 0U;
        VOFA_Send_8Float_DMA(
            (float)loop_sensor.value[0],
            (float)loop_sensor.value[1],
            (float)loop_sensor.value[2],
            (float)loop_sensor.value[3],
            (float)loop_sensor.value[4],
            (float)loop_sensor.active_count,
            (float)Loop_Calculate_Error(),
            (float)loop_sensor.last_error);
    }
}
