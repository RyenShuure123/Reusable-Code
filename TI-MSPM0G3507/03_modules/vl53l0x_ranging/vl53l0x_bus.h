#ifndef VL53L0X_BUS_H
#define VL53L0X_BUS_H

#include <stdbool.h>
#include <stdint.h>

bool VL53L0X_Bus_Write(uint8_t address, const uint8_t *data, uint8_t length);
bool VL53L0X_Bus_Read(uint8_t address, uint8_t *data, uint8_t length);

#endif
