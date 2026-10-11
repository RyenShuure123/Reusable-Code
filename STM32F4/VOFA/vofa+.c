#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "imu_receive.h"
#include "jetson_vision.h"
#include "Motor.h"
#include "stepper_motor.h"
#include "stepper_pid.h"
#include "usart.h"

#define VOFA_FRAME_TAIL_SIZE 4U
#define VOFA_MAX_FLOAT_COUNT 8U
#define VOFA_COMMAND_BUFFER_SIZE 32U
#define VOFA_COMMAND_IDLE_TIMEOUT_MS 20U

static uint8_t vofa_dma_buffer[VOFA_MAX_FLOAT_COUNT * sizeof(float) + VOFA_FRAME_TAIL_SIZE];
static const uint8_t vofa_frame_tail[VOFA_FRAME_TAIL_SIZE] = {0x00U, 0x00U, 0x80U, 0x7FU};
static uint8_t vofa_rx_byte;
static char vofa_rx_buffer[VOFA_COMMAND_BUFFER_SIZE];
static char vofa_command_buffer[VOFA_COMMAND_BUFFER_SIZE];
static volatile uint8_t vofa_rx_length;
static volatile uint8_t vofa_rx_overflow;
static volatile uint8_t vofa_command_ready;
static volatile uint32_t vofa_last_rx_ms;

static uint8_t VOFA_ParseFloat(const char *text, float *value)
{
    float integer_part = 0.0f;
    float fraction_part = 0.0f;
    float divisor = 1.0f;
    float sign = 1.0f;
    uint8_t has_digit = 0U;

    if ((text == NULL) || (value == NULL)) {
        return 0U;
    }

    if ((*text == '+') || (*text == '-')) {
        if (*text == '-') {
            sign = -1.0f;
        }
        text++;
    }

    while ((*text >= '0') && (*text <= '9')) {
        integer_part = integer_part * 10.0f + (float)(*text - '0');
        text++;
        has_digit = 1U;
    }

    if (*text == '.') {
        text++;
        while ((*text >= '0') && (*text <= '9')) {
            fraction_part = fraction_part * 10.0f + (float)(*text - '0');
            divisor *= 10.0f;
            text++;
            has_digit = 1U;
        }
    }

    if ((has_digit == 0U) || (*text != '\0')) {
        return 0U;
    }

    *value = sign * (integer_part + fraction_part / divisor);
    return 1U;
}

static void VOFA_FinalizeReceivedCommand(void)
{
    if ((vofa_rx_overflow == 0U) && (vofa_rx_length > 0U) &&
        (vofa_command_ready == 0U)) {
        vofa_rx_buffer[vofa_rx_length] = '\0';
        memcpy(vofa_command_buffer, vofa_rx_buffer, (size_t)vofa_rx_length + 1U);
        vofa_command_ready = 1U;
    }
    vofa_rx_length = 0U;
    vofa_rx_overflow = 0U;
}

static void VOFA_ProcessReceivedByte(uint8_t byte)
{
    uint8_t index;

    if ((byte == '\r') || (byte == '\n')) {
        VOFA_FinalizeReceivedCommand();
        return;
    }

    index = vofa_rx_length;
    if (index < (VOFA_COMMAND_BUFFER_SIZE - 1U)) {
        vofa_rx_buffer[index] = (char)byte;
        vofa_rx_length = index + 1U;
        vofa_last_rx_ms = HAL_GetTick();
    } else {
        vofa_rx_overflow = 1U;
    }
}

__weak void VOFA_CommandReceivedCallback(const char *command, float value)
{
    (void)command;
    (void)value;
}

void VOFA_Receive_Init(void)
{
    vofa_rx_length = 0U;
    vofa_rx_overflow = 0U;
    vofa_command_ready = 0U;
    vofa_last_rx_ms = 0U;

    (void)HAL_UART_Receive_IT(&huart1, &vofa_rx_byte, 1U);
}

void VOFA_ProcessCommands(void)
{
    char command[VOFA_COMMAND_BUFFER_SIZE];
    float value;
    uint32_t primask;

    primask = __get_PRIMASK();
    __disable_irq();
    if ((vofa_command_ready == 0U) && (vofa_rx_length > 0U) &&
        ((uint32_t)(HAL_GetTick() - vofa_last_rx_ms) >= VOFA_COMMAND_IDLE_TIMEOUT_MS)) {
        VOFA_FinalizeReceivedCommand();
    }
    __set_PRIMASK(primask);

    if (vofa_command_ready == 0U) {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    memcpy(command, vofa_command_buffer, sizeof(command));
    vofa_command_ready = 0U;
    __set_PRIMASK(primask);

    if (strncmp(command, "KP:", 3U) == 0) {
        if (VOFA_ParseFloat(&command[3], &value) != 0U) {
            track_pd.Kp = value;
            VOFA_CommandReceivedCallback("KP", value);
        }
    } else if (strncmp(command, "KD:", 3U) == 0) {
        if (VOFA_ParseFloat(&command[3], &value) != 0U) {
            track_pd.Kd = value;
            VOFA_CommandReceivedCallback("KD", value);
        }
    } else if (strncmp(command, "BKP:", 4U) == 0) {
        if (VOFA_ParseFloat(&command[4], &value) != 0U) {
            StepperPID_SetBallKp(value);
            VOFA_CommandReceivedCallback("BKP", value);
        }
    } else if (strncmp(command, "BKI:", 4U) == 0) {
        if (VOFA_ParseFloat(&command[4], &value) != 0U) {
            StepperPID_SetBallKi(value);
            VOFA_CommandReceivedCallback("BKI", value);
        }
    } else if (strncmp(command, "BKD:", 4U) == 0) {
        if (VOFA_ParseFloat(&command[4], &value) != 0U) {
            StepperPID_SetBallKd(value);
            VOFA_CommandReceivedCallback("BKD", value);
        }
    } else if (strncmp(command, "XENA:", 5U) == 0) {
        if (VOFA_ParseFloat(&command[5], &value) != 0U) {
            if (value > 0.5f) {
                (void)StepperMotor_Enable();
            } else {
                (void)StepperMotor_Disable();
            }
            VOFA_CommandReceivedCallback("XENA", value);
        }
    } else if (strncmp(command, "XADDR:", 6U) == 0) {
        if (VOFA_ParseFloat(&command[6], &value) != 0U) {
            StepperMotor_SetAddress((uint8_t)value);
            VOFA_CommandReceivedCallback("XADDR", value);
        }
    } else if (strncmp(command, "XSTOP:", 6U) == 0) {
        if (VOFA_ParseFloat(&command[6], &value) != 0U) {
            (void)StepperMotor_StopNow();
            VOFA_CommandReceivedCallback("XSTOP", value);
        }
    } else if (strncmp(command, "XZERO:", 6U) == 0) {
        if (VOFA_ParseFloat(&command[6], &value) != 0U) {
            (void)StepperMotor_ResetCurrentPositionToZero();
            VOFA_CommandReceivedCallback("XZERO", value);
        }
    } else if (strncmp(command, "XPULSE:", 7U) == 0) {
        if (VOFA_ParseFloat(&command[7], &value) != 0U) {
            (void)StepperMotor_MoveToPulse((int32_t)value, 30U, 10U);
            VOFA_CommandReceivedCallback("XPULSE", value);
        }
    }
}

uint8_t VOFA_SendText(const char *text)
{
    size_t length;

    if ((text == NULL) || (huart1.gState != HAL_UART_STATE_READY))
    {
        return 0U;
    }

    length = strlen(text);
    if ((length == 0U) || (length > 160U))
    {
        return 0U;
    }

    return (HAL_UART_Transmit(&huart1,
                              (uint8_t *)text,
                              (uint16_t)length,
                              20U) == HAL_OK) ? 1U : 0U;
}

static uint8_t VOFA_SendFloatFrame(const float *values, uint8_t count)
{
    uint16_t payload_size;

    if ((values == NULL) || (count == 0U) || (count > VOFA_MAX_FLOAT_COUNT)) {
        return 0U;
    }

    if (huart1.gState != HAL_UART_STATE_READY) {
        return 0U;
    }

    payload_size = (uint16_t)count * sizeof(float);
    memcpy(vofa_dma_buffer, values, payload_size);
    memcpy(&vofa_dma_buffer[payload_size], vofa_frame_tail, VOFA_FRAME_TAIL_SIZE);

    return (HAL_UART_Transmit_DMA(&huart1,
                                  vofa_dma_buffer,
                                  payload_size + VOFA_FRAME_TAIL_SIZE) == HAL_OK) ? 1U : 0U;
}

void VOFA_Send_6Float_DMA(float f1, float f2, float f3, float f4, float f5, float f6)
{
    const float values[6] = {f1, f2, f3, f4, f5, f6};

    (void)VOFA_SendFloatFrame(values, 6U);
}

void VOFA_Send_8Float_DMA(float f1, float f2, float f3, float f4,
                           float f5, float f6, float f7, float f8)
{
    const float values[8] = {f1, f2, f3, f4, f5, f6, f7, f8};

    (void)VOFA_SendFloatFrame(values, 8U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART1)) {
        VOFA_ProcessReceivedByte(vofa_rx_byte);
        (void)HAL_UART_Receive_IT(&huart1, &vofa_rx_byte, 1U);
    } else if ((huart != NULL) && (huart->Instance == USART6)) {
        Chassis_IMU_UART_RxCpltCallback(huart);
    } else if ((huart != NULL) && (huart->Instance == USART2)) {
        StepperMotor_UartRxCpltCallback(huart);
    } else if ((huart != NULL) && (huart->Instance == UART4)) {
        JetsonVision_UartRxCpltCallback(huart);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    StepperMotor_UartTxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if ((huart != NULL) && (huart->Instance == USART1)) {
        (void)HAL_UART_AbortReceive(&huart1);
        (void)HAL_UART_Receive_IT(&huart1, &vofa_rx_byte, 1U);
    } else if ((huart != NULL) && (huart->Instance == USART6)) {
        Chassis_IMU_UART_ErrorCallback(huart);
    } else if ((huart != NULL) && (huart->Instance == USART2)) {
        StepperMotor_UartErrorCallback(huart);
    } else if ((huart != NULL) && (huart->Instance == UART4)) {
        JetsonVision_UartErrorCallback(huart);
    }
}
