#include "bmi270_minimal.h"

#include <stdbool.h>
#include <stddef.h>

#include "bmi270_config.h"
#include "bmi270_port.h"

#define BMI270_CHIP_ID 0x24U
#define BMI270_SPI_READ_MASK 0x80U
#define BMI270_SPI_WRITE_MASK 0x7FU

#define BMI270_REG_CHIP_ID 0x00U
#define BMI270_REG_ACC_DATA 0x0CU
#define BMI270_REG_INTERNAL_STATUS 0x21U
#define BMI270_REG_ACC_CONF 0x40U
#define BMI270_REG_GYR_CONF 0x42U
#define BMI270_REG_INIT_CTRL 0x59U
#define BMI270_REG_INIT_ADDR_0 0x5BU
#define BMI270_REG_INIT_DATA 0x5EU
#define BMI270_REG_PWR_CONF 0x7CU
#define BMI270_REG_PWR_CTRL 0x7DU
#define BMI270_REG_COMMAND 0x7EU

#define BMI270_COMMAND_SOFT_RESET 0xB6U
#define BMI270_CONFIG_LOADED_MASK 0x0FU
#define BMI270_CONFIG_LOADED_VALUE 0x01U
#define BMI270_CONFIG_CHUNK_SIZE 256U

#define BMI270_ACC_CONF_200HZ_AVG4 0x29U
#define BMI270_ACC_RANGE_8G 0x02U
#define BMI270_GYR_CONF_200HZ_NORMAL 0x29U
#define BMI270_GYR_RANGE_2000DPS 0x00U
#define BMI270_ACCEL_GYRO_ENABLE_MASK 0x06U

static int8_t BMI270_ReadRegisters(uint8_t regAddr,
                                   uint8_t *data,
                                   uint8_t length)
{
    uint8_t transferData[13];
    uint8_t index;

    if ((data == NULL) || (length == 0U) || (length > 12U)) {
        return BMI270_E_NULL_POINTER;
    }

    if (BMI270_PortSpiRead(regAddr | BMI270_SPI_READ_MASK,
                           transferData,
                           (uint32_t)length + 1U,
                           NULL) != BMI270_OK) {
        return BMI270_E_COMMUNICATION;
    }

    for (index = 0U; index < length; index++) {
        data[index] = transferData[index + 1U];
    }

    return BMI270_OK;
}

static int8_t BMI270_WriteRegisters(uint8_t regAddr,
                                    const uint8_t *data,
                                    uint16_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return BMI270_E_NULL_POINTER;
    }

    if (BMI270_PortSpiWrite(regAddr & BMI270_SPI_WRITE_MASK,
                            data,
                            length,
                            NULL) != BMI270_OK) {
        return BMI270_E_COMMUNICATION;
    }

    return BMI270_OK;
}

static int8_t BMI270_SetRegisterBit(uint8_t regAddr, uint8_t bitMask, bool enable)
{
    uint8_t value;
    int8_t result;

    result = BMI270_ReadRegisters(regAddr, &value, 1U);
    if (result != BMI270_OK) {
        return result;
    }
    BMI270_PortDelayUs(450U, NULL);

    if (enable) {
        value |= bitMask;
    } else {
        value &= (uint8_t)~bitMask;
    }

    result = BMI270_WriteRegisters(regAddr, &value, 1U);
    if (result == BMI270_OK) {
        BMI270_PortDelayUs(450U, NULL);
    }

    return result;
}

static int8_t BMI270_UploadConfigFile(void)
{
    uint16_t index;
    uint16_t chunkLength;
    uint8_t configAddress[2];
    uint8_t internalStatus;
    int8_t result;

    result = BMI270_SetRegisterBit(BMI270_REG_PWR_CONF, 0x01U, false);
    if (result != BMI270_OK) {
        return result;
    }
    BMI270_PortDelayUs(450U, NULL);

    result = BMI270_SetRegisterBit(BMI270_REG_INIT_CTRL, 0x01U, false);
    if (result != BMI270_OK) {
        return result;
    }

    for (index = 0U; index < bmi270_config_file_size; index += chunkLength) {
        chunkLength = bmi270_config_file_size - index;
        if (chunkLength > BMI270_CONFIG_CHUNK_SIZE) {
            chunkLength = BMI270_CONFIG_CHUNK_SIZE;
        }

        configAddress[0] = (uint8_t)((index / 2U) & 0x0FU);
        configAddress[1] = (uint8_t)((index / 2U) >> 4U);

        result = BMI270_WriteRegisters(
            BMI270_REG_INIT_ADDR_0, configAddress, sizeof(configAddress));
        if (result != BMI270_OK) {
            return result;
        }

        result = BMI270_WriteRegisters(
            BMI270_REG_INIT_DATA, &bmi270_config_file[index], chunkLength);
        if (result != BMI270_OK) {
            return result;
        }
    }

    result = BMI270_SetRegisterBit(BMI270_REG_INIT_CTRL, 0x01U, true);
    if (result != BMI270_OK) {
        return result;
    }

    result = BMI270_SetRegisterBit(BMI270_REG_PWR_CONF, 0x01U, true);
    if (result != BMI270_OK) {
        return result;
    }

    BMI270_PortDelayUs(20000U, NULL);

    result = BMI270_ReadRegisters(BMI270_REG_INTERNAL_STATUS,
                                  &internalStatus,
                                  1U);
    if (result != BMI270_OK) {
        return result;
    }

    if ((internalStatus & BMI270_CONFIG_LOADED_MASK) !=
        BMI270_CONFIG_LOADED_VALUE) {
        return BMI270_E_CONFIG_LOAD;
    }

    return BMI270_OK;
}

static int8_t BMI270_ConfigureSensors(void)
{
    const uint8_t accelConfig[2] = {
        BMI270_ACC_CONF_200HZ_AVG4,
        BMI270_ACC_RANGE_8G
    };
    const uint8_t gyroConfig[2] = {
        BMI270_GYR_CONF_200HZ_NORMAL,
        BMI270_GYR_RANGE_2000DPS
    };
    uint8_t powerControl;
    int8_t result;

    result = BMI270_SetRegisterBit(BMI270_REG_PWR_CONF, 0x01U, false);
    if (result != BMI270_OK) {
        return result;
    }
    BMI270_PortDelayUs(450U, NULL);

    result = BMI270_WriteRegisters(BMI270_REG_ACC_CONF,
                                   accelConfig,
                                   sizeof(accelConfig));
    if (result != BMI270_OK) {
        return result;
    }

    result = BMI270_WriteRegisters(BMI270_REG_GYR_CONF,
                                   gyroConfig,
                                   sizeof(gyroConfig));
    if (result != BMI270_OK) {
        return result;
    }

    result = BMI270_SetRegisterBit(BMI270_REG_PWR_CONF, 0x01U, true);
    if (result != BMI270_OK) {
        return result;
    }

    result = BMI270_ReadRegisters(BMI270_REG_PWR_CTRL, &powerControl, 1U);
    if (result != BMI270_OK) {
        return result;
    }

    powerControl |= BMI270_ACCEL_GYRO_ENABLE_MASK;
    result = BMI270_WriteRegisters(BMI270_REG_PWR_CTRL, &powerControl, 1U);
    if (result == BMI270_OK) {
        BMI270_PortDelayUs(450U, NULL);
    }

    return result;
}

int8_t BMI270_ReadChipId(uint8_t *chipId)
{
    if (chipId == NULL) {
        return BMI270_E_NULL_POINTER;
    }

    return BMI270_ReadRegisters(BMI270_REG_CHIP_ID, chipId, 1U);
}

int8_t BMI270_InitDevice(void)
{
    uint8_t chipId;
    const uint8_t resetCommand = BMI270_COMMAND_SOFT_RESET;
    int8_t result;

    result = BMI270_ReadChipId(&chipId);
    if (result != BMI270_OK) {
        return result;
    }
    BMI270_PortDelayUs(450U, NULL);

    result = BMI270_ReadChipId(&chipId);
    if (result != BMI270_OK) {
        return result;
    }
    BMI270_PortDelayUs(450U, NULL);

    if (chipId != BMI270_CHIP_ID) {
        return BMI270_E_DEVICE_NOT_FOUND;
    }

    result = BMI270_WriteRegisters(BMI270_REG_COMMAND,
                                   &resetCommand,
                                   1U);
    if (result != BMI270_OK) {
        return result;
    }

    BMI270_PortDelayUs(2000U, NULL);

    result = BMI270_ReadChipId(&chipId);
    if (result != BMI270_OK) {
        return result;
    }
    BMI270_PortDelayUs(450U, NULL);

    result = BMI270_UploadConfigFile();
    if (result != BMI270_OK) {
        return result;
    }

    return BMI270_ConfigureSensors();
}

int8_t BMI270_ReadRawData(BMI270_RawData *rawData)
{
    uint8_t data[12];
    int8_t result;

    if (rawData == NULL) {
        return BMI270_E_NULL_POINTER;
    }

    result = BMI270_ReadRegisters(BMI270_REG_ACC_DATA, data, sizeof(data));
    if (result != BMI270_OK) {
        return result;
    }

    rawData->accX = (int16_t)(((uint16_t)data[1] << 8U) | data[0]);
    rawData->accY = (int16_t)(((uint16_t)data[3] << 8U) | data[2]);
    rawData->accZ = (int16_t)(((uint16_t)data[5] << 8U) | data[4]);
    rawData->gyrX = (int16_t)(((uint16_t)data[7] << 8U) | data[6]);
    rawData->gyrY = (int16_t)(((uint16_t)data[9] << 8U) | data[8]);
    rawData->gyrZ = (int16_t)(((uint16_t)data[11] << 8U) | data[10]);
    return BMI270_OK;
}

void BMI270_DelayUs(uint32_t delayUs)
{
    BMI270_PortDelayUs(delayUs, NULL);
}
