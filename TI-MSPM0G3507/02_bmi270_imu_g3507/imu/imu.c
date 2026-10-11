#include "imu.h"

#include <math.h>

#include "bmi270_minimal.h"
#include "mahonyRK4.h"

#define IMU_ACCEL_LSB_PER_G 4096.0f
#define IMU_GYRO_LSB_PER_DPS 16.4f
#define IMU_DEG_TO_RAD 0.0174532925f
#define IMU_ACCEL_NORM_MIN_G 0.90f
#define IMU_ACCEL_NORM_MAX_G 1.10f
#define IMU_US_PER_MS 1000U

static BMI270_RawData g_imuRawData;
static float g_gyroOffsetX;
static float g_gyroOffsetY;
static float g_gyroOffsetZ;
static bool g_imuReady;

int8_t IMU_ReadChipId(uint8_t *chipId)
{
    return BMI270_ReadChipId(chipId);
}

int8_t IMU_Init(void)
{
    int8_t result;
    result = BMI270_InitDevice();
    if (result != BMI270_OK) {
        return result;
    }

    g_gyroOffsetX = 0.0f;
    g_gyroOffsetY = 0.0f;
    g_gyroOffsetZ = 0.0f;
    g_imuReady = true;
    Mahony_Init();
    return IMU_OK;
}

int8_t IMU_CalibrateGyro(uint16_t sampleCount, uint32_t sampleIntervalMs)
{
    int64_t sumX = 0;
    int64_t sumY = 0;
    int64_t sumZ = 0;
    uint16_t validSampleCount = 0U;
    uint16_t index;
    int8_t result;

    if ((!g_imuReady) || (sampleCount == 0U)) {
        return IMU_E_INVALID_INPUT;
    }

    for (index = 0U; index < sampleCount; index++) {
        result = BMI270_ReadRawData(&g_imuRawData);
        if (result == BMI270_OK) {
            sumX += g_imuRawData.gyrX;
            sumY += g_imuRawData.gyrY;
            sumZ += g_imuRawData.gyrZ;
            validSampleCount++;
        }

        BMI270_DelayUs(sampleIntervalMs * IMU_US_PER_MS);
    }

    if (validSampleCount == 0U) {
        return IMU_E_COMMUNICATION;
    }

    g_gyroOffsetX = (float)sumX / (float)validSampleCount;
    g_gyroOffsetY = (float)sumY / (float)validSampleCount;
    g_gyroOffsetZ = (float)sumZ / (float)validSampleCount;
    return IMU_OK;
}

int8_t IMU_Update(float dtSeconds)
{
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float accelerationNorm;
    int8_t result;

    if ((!g_imuReady) || (dtSeconds <= 0.0f)) {
        return IMU_E_INVALID_INPUT;
    }

    result = BMI270_ReadRawData(&g_imuRawData);
    if (result != BMI270_OK) {
        return result;
    }

    ax = (float)g_imuRawData.accX / IMU_ACCEL_LSB_PER_G;
    ay = (float)g_imuRawData.accY / IMU_ACCEL_LSB_PER_G;
    az = (float)g_imuRawData.accZ / IMU_ACCEL_LSB_PER_G;

    gx = (((float)g_imuRawData.gyrX - g_gyroOffsetX) /
          IMU_GYRO_LSB_PER_DPS) * IMU_DEG_TO_RAD;
    gy = (((float)g_imuRawData.gyrY - g_gyroOffsetY) /
          IMU_GYRO_LSB_PER_DPS) * IMU_DEG_TO_RAD;
    gz = (((float)g_imuRawData.gyrZ - g_gyroOffsetZ) /
          IMU_GYRO_LSB_PER_DPS) * IMU_DEG_TO_RAD;

    accelerationNorm = sqrtf(ax * ax + ay * ay + az * az);
    if ((accelerationNorm >= IMU_ACCEL_NORM_MIN_G) &&
        (accelerationNorm <= IMU_ACCEL_NORM_MAX_G)) {
        Mahony_Update_IMU(gx, gy, gz, ax, ay, az, dtSeconds);
    } else {
        Mahony_Update_GyroOnly(gx, gy, gz, dtSeconds);
    }

    return IMU_OK;
}

bool IMU_IsReady(void)
{
    return g_imuReady;
}

void IMU_GetAngles(float *rollDegrees,
                   float *pitchDegrees,
                   float *yawDegrees)
{
    Mahony_GetAngles(rollDegrees, pitchDegrees, yawDegrees);
}
