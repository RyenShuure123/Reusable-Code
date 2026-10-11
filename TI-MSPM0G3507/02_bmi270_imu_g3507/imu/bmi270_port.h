#ifndef BMI270_PORT_H
#define BMI270_PORT_H

#include <stdint.h>

int8_t BMI270_PortSpiRead(uint8_t regAddr,
                          uint8_t *regData,
                          uint32_t length,
                          void *interfacePointer);
int8_t BMI270_PortSpiWrite(uint8_t regAddr,
                           const uint8_t *regData,
                           uint32_t length,
                           void *interfacePointer);
void BMI270_PortDelayUs(uint32_t periodUs, void *interfacePointer);

#endif
