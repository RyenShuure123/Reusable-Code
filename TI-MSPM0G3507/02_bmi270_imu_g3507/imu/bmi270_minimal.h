#ifndef BMI270_MINIMAL_H
#define BMI270_MINIMAL_H

#include <stdint.h>

#define BMI270_OK 0
#define BMI270_E_NULL_POINTER (-1)
#define BMI270_E_COMMUNICATION (-2)
#define BMI270_E_DEVICE_NOT_FOUND (-3)
#define BMI270_E_CONFIG_LOAD (-9)

typedef struct {
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;
} BMI270_RawData;

int8_t BMI270_ReadChipId(uint8_t *chipId);
int8_t BMI270_InitDevice(void);
int8_t BMI270_ReadRawData(BMI270_RawData *rawData);
void BMI270_DelayUs(uint32_t delayUs);

#endif
