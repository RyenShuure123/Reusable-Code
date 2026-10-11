#include "button.h"
#include "imu.h"
#include "motor.h"
#include "oled.h"
#include "timer.h"
#include "ti_msp_dl_config.h"
#include "timebase.h"
#include "uart_bluetooth.h"
#include "uart_pc.h"

#define MAIN_GYRO_CALIBRATION_SAMPLES 600U
#define MAIN_GYRO_CALIBRATION_INTERVAL_MS 5U

int main(void)
{
    int8_t imuResult;
    uint8_t chipId = 0U;
    uint8_t echoByte;
    uint8_t car_start = 0U;

    SYSCFG_DL_init();
    UART_PC_Init();
    UART_BLUETOOTH_Init();
    Timebase_Init();
    NVIC_SetPriority(SysTick_IRQn, 1U);
    NVIC_SetPriority(UART_PC_INST_INT_IRQN, 1U);
    NVIC_SetPriority(UART_BLUETOOTH_INST_INT_IRQN, 1U);
    Button_Init();
    OLED_Init();
    Motor_Init();

    UART_PC_SendString("\r\nBMI270 SPI learning project\r\n");
    (void)IMU_ReadChipId(&chipId);
    UART_PC_SendString("BMI270 Chip ID: 0x");
    UART_PC_SendHex8(chipId);
    UART_PC_SendString(" (expected 0x24)\r\n");
    UART_PC_SendString("Keep BMI270 still...\r\n");

    imuResult = IMU_Init();
    if (imuResult == IMU_OK) {
        imuResult = IMU_CalibrateGyro(
            MAIN_GYRO_CALIBRATION_SAMPLES,
            MAIN_GYRO_CALIBRATION_INTERVAL_MS);
    }

    OLED_NewFrame();
    OLED_ShowString(1, 1, "MSPM0G3507");
    if (imuResult == IMU_OK) {
        OLED_ShowString(2, 1, "BMI270 OK");
        UART_PC_SendString("BMI270 init and gyro calibration OK\r\n");
    } else {
        OLED_ShowString(2, 1, "BMI270 FAIL");
        UART_PC_SendString("BMI270 init failed, code: 0x");
        UART_PC_SendHex8((uint8_t)imuResult);
        UART_PC_SendString("\r\n");
    }
    OLED_ShowFrame();

    if (imuResult == IMU_OK) {
        ControlTimer_Init();
    }

    while (1) {
        Button_Update();

        if (Button_WasPressed()) {
            car_start = (car_start == 0U) ? 1U : 0U;

            if (car_start != 0U) {
                Motor_SetOutputs(220, -220);
            } else {
                Motor_Stop();
            }
        }

        if (!UART_PC_IsDmaTransmitBusy() &&
            UART_PC_TryReceiveByte(&echoByte)) {
            UART_PC_StartDmaTransmit(&echoByte, 1U);
            DL_GPIO_togglePins(GPIO_GRP_LED_PORT, GPIO_GRP_LED_PIN_LED_0_PIN);
        }
    }
}
