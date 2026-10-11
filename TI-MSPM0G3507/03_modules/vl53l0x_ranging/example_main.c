#include <stdint.h>

#include "imu.h"
#include "oled.h"
#include "ti_msp_dl_config.h"
#include "timebase.h"
#include "uart_bluetooth.h"
#include "uart_pc.h"
#include "vl53l0x.h"
#include "vofa_justfloat.h"

#define IMU_UPDATE_PERIOD_MS 5U
#define MILLISECONDS_TO_SECONDS 0.001f
#define GYRO_CALIBRATION_SAMPLES 600U
#define GYRO_CALIBRATION_INTERVAL_MS 5U
#define IMU_REPORT_PERIOD_MS 100U
#define VL53L0X_REPORT_PERIOD_MS 100U

int main(void)
{
    int8_t imuResult;
    uint8_t chipId = 0U;
    uint8_t echoByte;
    uint8_t vl53l0xModelId = 0U;
    uint16_t distanceMm = VL53L0X_INVALID_DISTANCE;
    uint32_t lastImuUpdateMs;
    uint32_t lastImuReportMs;
    uint32_t lastVl53l0xReportMs;
    bool vl53l0xReady = false;
    VL53L0X_Dev_t vl53l0x;

    SYSCFG_DL_init();
    UART_PC_Init();
    UART_BLUETOOTH_Init();
    (void)Timebase_Init();
    OLED_Init();

    UART_PC_SendString("\r\nBMI270 SPI learning project\r\n");

    (void)IMU_ReadChipId(&chipId);
    UART_PC_SendString("BMI270 Chip ID: 0x");
    UART_PC_SendHex8(chipId);
    UART_PC_SendString(" (expected 0x24)\r\n");

    UART_PC_SendString("Keep BMI270 still...\r\n");
    imuResult = IMU_Init();
    if (imuResult == IMU_OK) {
        imuResult = IMU_CalibrateGyro(
            GYRO_CALIBRATION_SAMPLES,
            GYRO_CALIBRATION_INTERVAL_MS);
    }

    VL53L0X_HardwareReset();
    if (!VL53L0X_Probe(&vl53l0xModelId)) {
        UART_PC_SendString("VL53L0X I2C probe failed\r\n");
    } else {
        UART_PC_SendString("VL53L0X Model ID: 0x");
        UART_PC_SendHex8(vl53l0xModelId);
        UART_PC_SendString(" (expected 0xEE)\r\n");

        if (vl53l0xModelId == 0xEEU) {
            vl53l0xReady = VL53L0X_Init(&vl53l0x, true);
            if (vl53l0xReady) {
                VL53L0X_StartContinuous(&vl53l0x, 0U);
                UART_PC_SendString("VL53L0X continuous ranging started\r\n");
            } else {
                UART_PC_SendString("VL53L0X initialization failed\r\n");
            }
        } else {
            UART_PC_SendString("VL53L0X unexpected model ID\r\n");
        }
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
    OLED_ShowString(3, 1, vl53l0xReady ? "VL53L0X OK" : "VL53L0X FAIL");
    OLED_ShowFrame();

    lastImuUpdateMs = Timebase_GetMilliseconds();
    lastImuReportMs = lastImuUpdateMs;
    lastVl53l0xReportMs = lastImuUpdateMs;

    while (1) {
        uint32_t nowMs = Timebase_GetMilliseconds();

        if ((imuResult == IMU_OK) &&
            ((nowMs - lastImuUpdateMs) >= IMU_UPDATE_PERIOD_MS)) {
            imuResult = IMU_Update(
                (float)(nowMs - lastImuUpdateMs) * MILLISECONDS_TO_SECONDS);
            lastImuUpdateMs = nowMs;
        }

        if (vl53l0xReady &&
            ((nowMs - lastVl53l0xReportMs) >= VL53L0X_REPORT_PERIOD_MS)) {
            distanceMm = VL53L0X_ReadRangeContinuousMillimeters(&vl53l0x);
            lastVl53l0xReportMs = nowMs;

            OLED_NewFrame();
            OLED_ShowString(1, 1, "MSPM0G3507");
            OLED_ShowString(2, 1, (imuResult == IMU_OK) ? "BMI270 OK" : "BMI270 FAIL");
            OLED_ShowString(3, 1, "VL53L0X OK");
            if (distanceMm == VL53L0X_INVALID_DISTANCE) {
                OLED_ShowString(4, 1, "D: INVALID");
            } else {
                OLED_ShowString(4, 1, "D:");
                OLED_ShowNum(4, 3, distanceMm);
                OLED_ShowString(4, 9, "mm");
            }
            OLED_ShowFrame();
        }

        if ((imuResult == IMU_OK) &&
            ((nowMs - lastImuReportMs) >= IMU_REPORT_PERIOD_MS) &&
            !UART_PC_IsDmaTransmitBusy()) {
            float rollDegrees;
            float pitchDegrees;
            float yawDegrees;

            IMU_GetAngles(&rollDegrees, &pitchDegrees, &yawDegrees);
            (void)VOFA_JustFloat_Send6(
                rollDegrees,
                pitchDegrees,
                yawDegrees,
                (distanceMm == VL53L0X_INVALID_DISTANCE) ? -1.0f : (float)distanceMm,
                0.0f,
                0.0f);
            lastImuReportMs = nowMs;
        }

        if (!UART_PC_IsDmaTransmitBusy() &&
            UART_PC_TryReceiveByte(&echoByte)) {
            (void)UART_PC_StartDmaTransmit(&echoByte, 1U);
            DL_GPIO_togglePins(GPIO_GRP_LED_PORT, GPIO_GRP_LED_PIN_LED_0_PIN);
        }
    }
}
