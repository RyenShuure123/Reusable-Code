#include "bmi270_port.h"

#include "ti_msp_dl_config.h"

#define BMI270_PORT_CPU_CLOCK_MHZ 32U
#define BMI270_PORT_DELAY_CHUNK_US 100000U

static uint8_t BMI270_PortTransferByte(uint8_t transmitData)
{
    DL_SPI_transmitDataBlocking8(SPI_BMI270_INST, transmitData);
    return DL_SPI_receiveDataBlocking8(SPI_BMI270_INST);
}

static void BMI270_PortSelect(void)
{
    DL_GPIO_clearPins(GPIO_BMI270_PORT, GPIO_BMI270_PIN_BMI270_CS_PIN);
}

static void BMI270_PortDeselect(void)
{
    DL_GPIO_setPins(GPIO_BMI270_PORT, GPIO_BMI270_PIN_BMI270_CS_PIN);
}

int8_t BMI270_PortSpiRead(uint8_t regAddr,
                          uint8_t *regData,
                          uint32_t length,
                          void *interfacePointer)
{
    uint32_t index;

    (void)interfacePointer;

    if ((regData == NULL) || (length == 0U)) {
        return -1;
    }

    BMI270_PortSelect();
    (void)BMI270_PortTransferByte(regAddr);

    for (index = 0U; index < length; index++) {
        regData[index] = BMI270_PortTransferByte(0U);
    }

    BMI270_PortDeselect();
    return 0;
}

int8_t BMI270_PortSpiWrite(uint8_t regAddr,
                           const uint8_t *regData,
                           uint32_t length,
                           void *interfacePointer)
{
    uint32_t index;

    (void)interfacePointer;

    if ((regData == NULL) || (length == 0U)) {
        return -1;
    }

    BMI270_PortSelect();
    (void)BMI270_PortTransferByte(regAddr);

    for (index = 0U; index < length; index++) {
        (void)BMI270_PortTransferByte(regData[index]);
    }

    BMI270_PortDeselect();
    return 0;
}

void BMI270_PortDelayUs(uint32_t periodUs, void *interfacePointer)
{
    uint32_t delayUs;

    (void)interfacePointer;

    while (periodUs > 0U) {
        delayUs = (periodUs > BMI270_PORT_DELAY_CHUNK_US) ?
                      BMI270_PORT_DELAY_CHUNK_US :
                      periodUs;
        DL_Common_delayCycles(delayUs * BMI270_PORT_CPU_CLOCK_MHZ);
        periodUs -= delayUs;
    }
}
