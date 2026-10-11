#include "vl53l0x_bus.h"

#include "ti_msp_dl_config.h"
#include "timebase.h"

#define VL53L0X_BUS_TIMEOUT_MS 20U

static bool VL53L0X_Bus_TimedOut(uint32_t startMs)
{
    return (Timebase_GetMilliseconds() - startMs) >= VL53L0X_BUS_TIMEOUT_MS;
}

static bool VL53L0X_Bus_WaitForIdle(void)
{
    uint32_t startMs = Timebase_GetMilliseconds();

    while ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0U) {
        if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
             DL_I2C_CONTROLLER_STATUS_ERROR) != 0U ||
            VL53L0X_Bus_TimedOut(startMs)) {
            return false;
        }
    }

    return true;
}

bool VL53L0X_Bus_Write(uint8_t address, const uint8_t *data, uint8_t length)
{
    uint16_t sent;
    uint32_t startMs;

    if ((data == NULL) || (length == 0U) || !VL53L0X_Bus_WaitForIdle()) {
        return false;
    }

    DL_I2C_flushControllerTXFIFO(I2C_OLED_INST);
    sent = DL_I2C_fillControllerTXFIFO(I2C_OLED_INST, data, length);
    DL_I2C_startControllerTransfer(
        I2C_OLED_INST, address, DL_I2C_CONTROLLER_DIRECTION_TX, length);

    startMs = Timebase_GetMilliseconds();
    while (sent < length) {
        while (DL_I2C_isControllerTXFIFOFull(I2C_OLED_INST)) {
            if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
                 DL_I2C_CONTROLLER_STATUS_ERROR) != 0U ||
                VL53L0X_Bus_TimedOut(startMs)) {
                return false;
            }
        }

        DL_I2C_transmitControllerData(I2C_OLED_INST, data[sent]);
        sent++;
    }

    return VL53L0X_Bus_WaitForIdle();
}

bool VL53L0X_Bus_Read(uint8_t address, uint8_t *data, uint8_t length)
{
    uint16_t received = 0U;
    uint32_t startMs;

    if ((data == NULL) || (length == 0U) || !VL53L0X_Bus_WaitForIdle()) {
        return false;
    }

    DL_I2C_flushControllerRXFIFO(I2C_OLED_INST);
    DL_I2C_startControllerTransfer(
        I2C_OLED_INST, address, DL_I2C_CONTROLLER_DIRECTION_RX, length);

    startMs = Timebase_GetMilliseconds();
    while (received < length) {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_OLED_INST)) {
            if ((DL_I2C_getControllerStatus(I2C_OLED_INST) &
                 DL_I2C_CONTROLLER_STATUS_ERROR) != 0U ||
                VL53L0X_Bus_TimedOut(startMs)) {
                return false;
            }
        }

        data[received] = DL_I2C_receiveControllerData(I2C_OLED_INST);
        received++;
    }

    return VL53L0X_Bus_WaitForIdle();
}
