/**
  * @file    vl53l0x.h
  * @brief   VL53L0X ToF sensor driver for STM32 (HAL)
  * @note    Based on Pololu Arduino library, adapted for STM32.
  */

#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>

#define VL53L0X_DEFAULT_ADDRESS  0x29U

/* 超时设置（ms） */
#define VL53L0X_DEFAULT_TIMEOUT  500

/* 测距返回值错误（超时或无效） */
#define VL53L0X_INVALID_DISTANCE 65535

/* 寄存器地址枚举 (来自API) */
typedef enum {
    SYSRANGE_START                                   = 0x00,
    SYSTEM_THRESH_HIGH                               = 0x0C,
    SYSTEM_THRESH_LOW                                = 0x0E,
    SYSTEM_SEQUENCE_CONFIG                           = 0x01,
    SYSTEM_RANGE_CONFIG                              = 0x09,
    SYSTEM_INTERMEASUREMENT_PERIOD                   = 0x04,
    SYSTEM_INTERRUPT_CONFIG_GPIO                     = 0x0A,
    GPIO_HV_MUX_ACTIVE_HIGH                          = 0x84,
    SYSTEM_INTERRUPT_CLEAR                           = 0x0B,
    RESULT_INTERRUPT_STATUS                          = 0x13,
    RESULT_RANGE_STATUS                              = 0x14,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN            = 0xBC,
    RESULT_CORE_RANGING_TOTAL_EVENTS_RTN             = 0xC0,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF            = 0xD0,
    RESULT_CORE_RANGING_TOTAL_EVENTS_REF             = 0xD4,
    RESULT_PEAK_SIGNAL_RATE_REF                      = 0xB6,
    ALGO_PART_TO_PART_RANGE_OFFSET_MM                = 0x28,
    I2C_SLAVE_DEVICE_ADDRESS                         = 0x8A,
    MSRC_CONFIG_CONTROL                              = 0x60,
    PRE_RANGE_CONFIG_MIN_SNR                         = 0x27,
    PRE_RANGE_CONFIG_VALID_PHASE_LOW                 = 0x56,
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH                = 0x57,
    PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT               = 0x64,
    FINAL_RANGE_CONFIG_MIN_SNR                       = 0x67,
    FINAL_RANGE_CONFIG_VALID_PHASE_LOW               = 0x47,
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH              = 0x48,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT      = 0x44,
    PRE_RANGE_CONFIG_SIGMA_THRESH_HI                 = 0x61,
    PRE_RANGE_CONFIG_SIGMA_THRESH_LO                 = 0x62,
    PRE_RANGE_CONFIG_VCSEL_PERIOD                    = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI               = 0x51,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO               = 0x52,
    SYSTEM_HISTOGRAM_BIN                             = 0x81,
    HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT            = 0x33,
    HISTOGRAM_CONFIG_READOUT_CTRL                    = 0x55,
    FINAL_RANGE_CONFIG_VCSEL_PERIOD                  = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI             = 0x71,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO             = 0x72,
    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS            = 0x20,
    MSRC_CONFIG_TIMEOUT_MACROP                       = 0x46,
    SOFT_RESET_GO2_SOFT_RESET_N                      = 0xBF,
    IDENTIFICATION_MODEL_ID                          = 0xC0,
    IDENTIFICATION_REVISION_ID                       = 0xC2,
    OSC_CALIBRATE_VAL                                = 0xF8,
    GLOBAL_CONFIG_VCSEL_WIDTH                        = 0x32,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0                 = 0xB0,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_1                 = 0xB1,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_2                 = 0xB2,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_3                 = 0xB3,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_4                 = 0xB4,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_5                 = 0xB5,
    GLOBAL_CONFIG_REF_EN_START_SELECT                = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD              = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET                 = 0x4F,
    POWER_MANAGEMENT_GO1_POWER_FORCE                 = 0x80,
    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV                = 0x89,
    ALGO_PHASECAL_LIM                                = 0x30,
    ALGO_PHASECAL_CONFIG_TIMEOUT                     = 0x30,
} vl53l0x_reg_t;

/* VCSEL 周期类型 */
typedef enum {
    VcselPeriodPreRange,
    VcselPeriodFinalRange
} vcselPeriodType;

/* 设备结构体 */
typedef struct {
    uint8_t  address;            // I2C 地址
    uint16_t timeout_ms;         // 超时毫秒
    bool     did_timeout;        // 超时标志
    uint32_t timeout_start_ms;   // 内部使用
    uint8_t  stop_variable;      // 从传感器读取的停止变量
    uint32_t measurement_timing_budget_us; // 当前时序预算
} VL53L0X_Dev_t;

/* 初始化函数 */
bool VL53L0X_Init(VL53L0X_Dev_t *dev, bool io_2v8);
/* 设置新 I2C 地址（7位） */
void VL53L0X_SetAddress(VL53L0X_Dev_t *dev, uint8_t new_addr);
/* 获取当前地址 */
uint8_t VL53L0X_GetAddress(VL53L0X_Dev_t *dev);

/* 配置函数 */
bool VL53L0X_SetSignalRateLimit(VL53L0X_Dev_t *dev, float limit_Mcps);
float VL53L0X_GetSignalRateLimit(VL53L0X_Dev_t *dev);
bool VL53L0X_SetMeasurementTimingBudget(VL53L0X_Dev_t *dev, uint32_t budget_us);
uint32_t VL53L0X_GetMeasurementTimingBudget(VL53L0X_Dev_t *dev);
bool VL53L0X_SetVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type, uint8_t period_pclks);
uint8_t VL53L0X_GetVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type);

/* 测距模式 */
void VL53L0X_StartContinuous(VL53L0X_Dev_t *dev, uint32_t period_ms);
void VL53L0X_StopContinuous(VL53L0X_Dev_t *dev);
uint16_t VL53L0X_ReadRangeContinuousMillimeters(VL53L0X_Dev_t *dev);
uint16_t VL53L0X_ReadRangeSingleMillimeters(VL53L0X_Dev_t *dev);

/* 超时处理 */
void VL53L0X_SetTimeout(VL53L0X_Dev_t *dev, uint16_t timeout_ms);
uint16_t VL53L0X_GetTimeout(VL53L0X_Dev_t *dev);
bool VL53L0X_TimeoutOccurred(VL53L0X_Dev_t *dev);

/* 低层读写（用户一般不直接使用） */
bool VL53L0X_WriteReg(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t value);
bool VL53L0X_WriteReg16(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t value);
bool VL53L0X_WriteReg32(VL53L0X_Dev_t *dev, uint8_t reg, uint32_t value);
uint8_t VL53L0X_ReadReg(VL53L0X_Dev_t *dev, uint8_t reg);
uint16_t VL53L0X_ReadReg16(VL53L0X_Dev_t *dev, uint8_t reg);
uint32_t VL53L0X_ReadReg32(VL53L0X_Dev_t *dev, uint8_t reg);
bool VL53L0X_WriteMulti(VL53L0X_Dev_t *dev, uint8_t reg, const uint8_t *src, uint8_t count);
bool VL53L0X_ReadMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t *dst, uint8_t count);

/* 硬件控制（XSHUT） */
bool VL53L0X_Probe(uint8_t *modelId);
void VL53L0X_HardwareReset(void);

#endif /* VL53L0X_H */
