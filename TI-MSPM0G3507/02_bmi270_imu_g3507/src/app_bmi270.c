#include "app_bmi270.h"

#include "imu.h"
#include "oled.h"
#include "ti_msp_dl_config.h"
#include "timebase.h"
#include "uart_pc.h"

#define APP_BMI270_UPDATE_PERIOD_MS 5U
#define APP_BMI270_MILLISECONDS_TO_SECONDS 0.001f
#define APP_BMI270_GYRO_CALIBRATION_SAMPLES 600U
#define APP_BMI270_GYRO_CALIBRATION_INTERVAL_MS 5U
#define APP_BMI270_REPORT_PERIOD_MS 100U

static int8_t g_imuResult;
static uint32_t g_lastImuUpdateMs;
static uint32_t g_lastImuReportMs;
static uint8_t g_echoByte;

static void App_BMI270_SendUnsignedDecimal(uint32_t value)
{
    uint8_t digits[10];
    uint8_t digitCount = 0U;

    do {
        digits[digitCount] = (uint8_t)(value % 10U);
        value /= 10U;
        digitCount++;
    } while (value > 0U);

    while (digitCount > 0U) {
        digitCount--;
        UART_PC_SendByte((uint8_t)('0' + digits[digitCount]));
    }
}

static void App_BMI270_LogStartupTimestamp(const char *stage)
{
    UART_PC_SendString("[T=");
    App_BMI270_SendUnsignedDecimal(Timebase_GetMilliseconds());
    UART_PC_SendString(" ms] ");
    UART_PC_SendString(stage);
    UART_PC_SendString("\r\n");
}

static void App_BMI270_SendAngle(float angleDegrees)
{
    int32_t tenths;
    uint32_t absoluteTenths;

    tenths = (angleDegrees >= 0.0f) ?
                 (int32_t)(angleDegrees * 10.0f + 0.5f) :
                 (int32_t)(angleDegrees * 10.0f - 0.5f);

    if (tenths < 0) {
        UART_PC_SendByte((uint8_t)'-');
        absoluteTenths = (uint32_t)(-tenths);
    } else {
        UART_PC_SendByte((uint8_t)'+');
        absoluteTenths = (uint32_t)tenths;
    }

    App_BMI270_SendUnsignedDecimal(absoluteTenths / 10U);
    UART_PC_SendByte((uint8_t)'.');
    UART_PC_SendByte((uint8_t)('0' + (absoluteTenths % 10U)));
}

static void App_BMI270_ReportAngles(void)
{
    float rollDegrees;
    float pitchDegrees;
    float yawDegrees;

    IMU_GetAngles(&rollDegrees, &pitchDegrees, &yawDegrees);
    UART_PC_SendString("R=");
    App_BMI270_SendAngle(rollDegrees);
    UART_PC_SendString(" P=");
    App_BMI270_SendAngle(pitchDegrees);
    UART_PC_SendString(" Y=");
    App_BMI270_SendAngle(yawDegrees);
    UART_PC_SendString("\r\n");
}

static void App_BMI270_ShowInitializationResult(void)
{
    OLED_NewFrame();
    OLED_ShowString(1, 1, "MSPM0G3507");

    if (g_imuResult == IMU_OK) {
        OLED_ShowString(2, 1, "BMI270 OK");
        UART_PC_SendString("BMI270 init and gyro calibration OK\r\n");
    } else {
        OLED_ShowString(2, 1, "BMI270 FAIL");
        UART_PC_SendString("BMI270 init failed, code: 0x");
        UART_PC_SendHex8((uint8_t)g_imuResult);
        UART_PC_SendString("\r\n");
    }

    OLED_ShowFrame();
}

static void App_BMI270_InitializeImu(void)
{
    uint8_t chipId = 0U;

    App_BMI270_LogStartupTimestamp("Chip ID read start");
    IMU_ReadChipId(&chipId);
    App_BMI270_LogStartupTimestamp("Chip ID read complete");
    UART_PC_SendString("BMI270 Chip ID: 0x");
    UART_PC_SendHex8(chipId);
    UART_PC_SendString(" (expected 0x24)\r\n");

    UART_PC_SendString("Keep BMI270 still...\r\n");
    App_BMI270_LogStartupTimestamp("BMI270 initialization start");
    g_imuResult = IMU_Init();
    App_BMI270_LogStartupTimestamp("BMI270 initialization complete");
    if (g_imuResult == IMU_OK) {
        App_BMI270_LogStartupTimestamp("Gyro calibration start");
        g_imuResult = IMU_CalibrateGyro(
            APP_BMI270_GYRO_CALIBRATION_SAMPLES,
            APP_BMI270_GYRO_CALIBRATION_INTERVAL_MS);
        App_BMI270_LogStartupTimestamp("Gyro calibration complete");
    }
}

static void App_BMI270_UpdateImu(uint32_t nowMs)
{
    uint32_t elapsedMs = nowMs - g_lastImuUpdateMs;

    if ((g_imuResult == IMU_OK) &&
        (elapsedMs >= APP_BMI270_UPDATE_PERIOD_MS)) {
        g_imuResult = IMU_Update(
            (float)elapsedMs * APP_BMI270_MILLISECONDS_TO_SECONDS);
        g_lastImuUpdateMs = nowMs;
    }
}

static void App_BMI270_ReportImu(uint32_t nowMs)
{
    if ((g_imuResult == IMU_OK) &&
        ((nowMs - g_lastImuReportMs) >= APP_BMI270_REPORT_PERIOD_MS) &&
        !UART_PC_IsDmaTransmitBusy()) {
        App_BMI270_ReportAngles();
        g_lastImuReportMs = nowMs;
    }
}

static void App_BMI270_EchoUart(void)
{
    if (!UART_PC_IsDmaTransmitBusy() &&
        UART_PC_TryReceiveByte(&g_echoByte)) {
        UART_PC_StartDmaTransmit(&g_echoByte, 1U);
        DL_GPIO_togglePins(GPIO_GRP_LED_PORT, GPIO_GRP_LED_PIN_LED_0_PIN);
    }
}

void App_BMI270_Init(void)
{
    SYSCFG_DL_init();
    UART_PC_Init();
    Timebase_Init();

    UART_PC_SendString("\r\nBMI270 SPI learning project\r\n");
    App_BMI270_LogStartupTimestamp("Startup initialized");

    App_BMI270_LogStartupTimestamp("OLED initialization start");
    OLED_Init();
    App_BMI270_LogStartupTimestamp("OLED initialization complete");
    App_BMI270_InitializeImu();
    App_BMI270_ShowInitializationResult();
    App_BMI270_LogStartupTimestamp("Startup sequence complete");

    g_lastImuUpdateMs = Timebase_GetMilliseconds();
    g_lastImuReportMs = g_lastImuUpdateMs;
}

void App_BMI270_RunOnce(void)
{
    uint32_t nowMs = Timebase_GetMilliseconds();

    App_BMI270_UpdateImu(nowMs);
    App_BMI270_ReportImu(nowMs);
    App_BMI270_EchoUart();
}
