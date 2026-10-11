#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

#define IMU_OK 0
#define IMU_E_NULL_POINTER (-1)
#define IMU_E_COMMUNICATION (-2)
#define IMU_E_DEVICE_NOT_FOUND (-3)
#define IMU_E_CONFIG_LOAD (-9)
#define IMU_E_INVALID_INPUT (-20)

int8_t IMU_Init(void);
int8_t IMU_ReadChipId(uint8_t *chipId);
int8_t IMU_CalibrateGyro(uint16_t sampleCount, uint32_t sampleIntervalMs);
int8_t IMU_Update(float dtSeconds);
bool IMU_IsReady(void);
void IMU_GetAngles(float *rollDegrees,
                   float *pitchDegrees,
                   float *yawDegrees);

#endif
