/**
  * @file    vl53l0x.c
  * @brief   VL53L0X driver implementation.
  */

#include "vl53l0x.h"
#include "vl53l0x_bus.h"
#include "ti_msp_dl_config.h"
#include "timebase.h"
#include <stddef.h>
#include <math.h>

/* 私有宏定义 */
#define START_TIMEOUT(dev)   ((dev)->timeout_start_ms = Timebase_GetMilliseconds())
#define CHECK_TIMEOUT(dev)   ((dev)->timeout_ms > 0U && (Timebase_GetMilliseconds() - (dev)->timeout_start_ms) > (dev)->timeout_ms)

/* 解码/编码 VCSEL 周期 */
#define DECODE_VCSEL_PERIOD(reg_val)      (((reg_val) + 1) << 1)
#define ENCODE_VCSEL_PERIOD(period_pclks) (((period_pclks) >> 1) - 1)

/* 计算宏周期（纳秒） */
#define CALC_MACRO_PERIOD(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

/* 私有函数声明 */
static bool getSpadInfo(VL53L0X_Dev_t *dev, uint8_t *count, bool *type_is_aperture);
static void getSequenceStepEnables(VL53L0X_Dev_t *dev, bool *tcc, bool *dss, bool *msrc, bool *pre_range, bool *final_range);
static void getSequenceStepTimeouts(VL53L0X_Dev_t *dev, bool pre_range_en, bool final_range_en,
                                    uint16_t *pre_range_vcsel_period_pclks, uint16_t *final_range_vcsel_period_pclks,
                                    uint16_t *msrc_dss_tcc_mclks, uint32_t *msrc_dss_tcc_us,
                                    uint16_t *pre_range_mclks, uint32_t *pre_range_us,
                                    uint16_t *final_range_mclks, uint32_t *final_range_us);
static uint16_t decodeTimeout(uint16_t reg_val);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
static bool performSingleRefCalibration(VL53L0X_Dev_t *dev, uint8_t vhv_init_byte);

/* ========== 底层 I2C 读写实现 ========== */
bool VL53L0X_WriteReg(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};

    return (dev != NULL) && VL53L0X_Bus_Write(dev->address, data, sizeof(data));
}

bool VL53L0X_WriteReg16(VL53L0X_Dev_t *dev, uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {reg, (uint8_t)(value >> 8), (uint8_t)value};

    return (dev != NULL) && VL53L0X_Bus_Write(dev->address, data, sizeof(data));
}

bool VL53L0X_WriteReg32(VL53L0X_Dev_t *dev, uint8_t reg, uint32_t value)
{
    uint8_t data[5] = {reg, (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                       (uint8_t)(value >> 8), (uint8_t)value};

    return (dev != NULL) && VL53L0X_Bus_Write(dev->address, data, sizeof(data));
}

uint8_t VL53L0X_ReadReg(VL53L0X_Dev_t *dev, uint8_t reg)
{
    uint8_t value = 0U;

    (void)VL53L0X_ReadMulti(dev, reg, &value, 1U);
    return value;
}

uint16_t VL53L0X_ReadReg16(VL53L0X_Dev_t *dev, uint8_t reg)
{
    uint8_t data[2] = {0U, 0U};

    (void)VL53L0X_ReadMulti(dev, reg, data, sizeof(data));
    return ((uint16_t)data[0] << 8) | data[1];
}

uint32_t VL53L0X_ReadReg32(VL53L0X_Dev_t *dev, uint8_t reg)
{
    uint8_t data[4] = {0U, 0U, 0U, 0U};

    (void)VL53L0X_ReadMulti(dev, reg, data, sizeof(data));
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) | data[3];
}

bool VL53L0X_WriteMulti(VL53L0X_Dev_t *dev, uint8_t reg, const uint8_t *src, uint8_t count)
{
    uint8_t data[33];
    uint8_t index;

    if ((dev == NULL) || (src == NULL) || (count > (sizeof(data) - 1U))) {
        return false;
    }

    data[0] = reg;
    for (index = 0U; index < count; index++) {
        data[index + 1U] = src[index];
    }

    return VL53L0X_Bus_Write(dev->address, data, count + 1U);
}

bool VL53L0X_ReadMulti(VL53L0X_Dev_t *dev, uint8_t reg, uint8_t *dst, uint8_t count)
{
    if ((dev == NULL) || (dst == NULL) || (count == 0U)) {
        return false;
    }

    return VL53L0X_Bus_Write(dev->address, &reg, 1U) &&
           VL53L0X_Bus_Read(dev->address, dst, count);
}

void VL53L0X_HardwareReset(void)
{
    uint32_t startMs;

    DL_GPIO_clearPins(
        GPIO_VL53L0X_PORT,
        GPIO_VL53L0X_PIN_VL53L0X_XSHUT_PIN);
    startMs = Timebase_GetMilliseconds();
    while ((Timebase_GetMilliseconds() - startMs) < 10U) {
    }

    DL_GPIO_setPins(
        GPIO_VL53L0X_PORT,
        GPIO_VL53L0X_PIN_VL53L0X_XSHUT_PIN);
    startMs = Timebase_GetMilliseconds();
    while ((Timebase_GetMilliseconds() - startMs) < 50U) {
    }
}

bool VL53L0X_Probe(uint8_t *modelId)
{
    uint8_t registerAddress = IDENTIFICATION_MODEL_ID;

    if (modelId == NULL) {
        return false;
    }

    return VL53L0X_Bus_Write(
               VL53L0X_DEFAULT_ADDRESS,
               &registerAddress,
               1U) &&
           VL53L0X_Bus_Read(
               VL53L0X_DEFAULT_ADDRESS,
               modelId,
               1U);
}
bool VL53L0X_Init(VL53L0X_Dev_t *dev, bool io_2v8)
{
    dev->address = VL53L0X_DEFAULT_ADDRESS;
    dev->timeout_ms = VL53L0X_DEFAULT_TIMEOUT;
    dev->did_timeout = false;

    // 1. 如果 io_2v8 为真，切换到 2V8 模式
    if (io_2v8) {
        uint8_t val = VL53L0X_ReadReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV);
        val |= 0x01;
        VL53L0X_WriteReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, val);
    }

    // 2. 设置 I2C 标准模式
    VL53L0X_WriteReg(dev, 0x88, 0x00);

    // 3. 读取 stop_variable
    VL53L0X_WriteReg(dev, 0x80, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x00, 0x00);
    dev->stop_variable = VL53L0X_ReadReg(dev, 0x91);
    VL53L0X_WriteReg(dev, 0x00, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x80, 0x00);

    // 4. 禁用 MSRC 和 PRE_RANGE 信号速率检查
    uint8_t msrc_config = VL53L0X_ReadReg(dev, MSRC_CONFIG_CONTROL);
    msrc_config |= 0x12;
    VL53L0X_WriteReg(dev, MSRC_CONFIG_CONTROL, msrc_config);

    // 5. 设置信号速率限制为 0.25 MCPS
    VL53L0X_SetSignalRateLimit(dev, 0.25f);

    // 6. 设置系统序列配置
    VL53L0X_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xFF);

    // 7. 获取 SPAD 信息
    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!getSpadInfo(dev, &spad_count, &spad_type_is_aperture))
        return false;

    // 8. 读取参考 SPAD 映射
    uint8_t ref_spad_map[6];
    if (!VL53L0X_ReadMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6))
        return false;

    // 9. 设置参考 SPAD (与 API 一致)
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    VL53L0X_WriteReg(dev, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x01) {
            spads_enabled++;
        }
    }
    VL53L0X_WriteMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    // 10. 加载 tuning settings (从 API 复制)
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x00, 0x00);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x09, 0x00);
    VL53L0X_WriteReg(dev, 0x10, 0x00);
    VL53L0X_WriteReg(dev, 0x11, 0x00);
    VL53L0X_WriteReg(dev, 0x24, 0x01);
    VL53L0X_WriteReg(dev, 0x25, 0xFF);
    VL53L0X_WriteReg(dev, 0x75, 0x00);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x4E, 0x2C);
    VL53L0X_WriteReg(dev, 0x48, 0x00);
    VL53L0X_WriteReg(dev, 0x30, 0x20);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x30, 0x09);
    VL53L0X_WriteReg(dev, 0x54, 0x00);
    VL53L0X_WriteReg(dev, 0x31, 0x04);
    VL53L0X_WriteReg(dev, 0x32, 0x03);
    VL53L0X_WriteReg(dev, 0x40, 0x83);
    VL53L0X_WriteReg(dev, 0x46, 0x25);
    VL53L0X_WriteReg(dev, 0x60, 0x00);
    VL53L0X_WriteReg(dev, 0x27, 0x00);
    VL53L0X_WriteReg(dev, 0x50, 0x06);
    VL53L0X_WriteReg(dev, 0x51, 0x00);
    VL53L0X_WriteReg(dev, 0x52, 0x96);
    VL53L0X_WriteReg(dev, 0x56, 0x08);
    VL53L0X_WriteReg(dev, 0x57, 0x30);
    VL53L0X_WriteReg(dev, 0x61, 0x00);
    VL53L0X_WriteReg(dev, 0x62, 0x00);
    VL53L0X_WriteReg(dev, 0x64, 0x00);
    VL53L0X_WriteReg(dev, 0x65, 0x00);
    VL53L0X_WriteReg(dev, 0x66, 0xA0);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x22, 0x32);
    VL53L0X_WriteReg(dev, 0x47, 0x14);
    VL53L0X_WriteReg(dev, 0x49, 0xFF);
    VL53L0X_WriteReg(dev, 0x4A, 0x00);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x7A, 0x0A);
    VL53L0X_WriteReg(dev, 0x7B, 0x00);
    VL53L0X_WriteReg(dev, 0x78, 0x21);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x23, 0x34);
    VL53L0X_WriteReg(dev, 0x42, 0x00);
    VL53L0X_WriteReg(dev, 0x44, 0xFF);
    VL53L0X_WriteReg(dev, 0x45, 0x26);
    VL53L0X_WriteReg(dev, 0x46, 0x05);
    VL53L0X_WriteReg(dev, 0x40, 0x40);
    VL53L0X_WriteReg(dev, 0x0E, 0x06);
    VL53L0X_WriteReg(dev, 0x20, 0x1A);
    VL53L0X_WriteReg(dev, 0x43, 0x40);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x34, 0x03);
    VL53L0X_WriteReg(dev, 0x35, 0x44);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x31, 0x04);
    VL53L0X_WriteReg(dev, 0x4B, 0x09);
    VL53L0X_WriteReg(dev, 0x4C, 0x05);
    VL53L0X_WriteReg(dev, 0x4D, 0x04);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x44, 0x00);
    VL53L0X_WriteReg(dev, 0x45, 0x20);
    VL53L0X_WriteReg(dev, 0x47, 0x08);
    VL53L0X_WriteReg(dev, 0x48, 0x28);
    VL53L0X_WriteReg(dev, 0x67, 0x00);
    VL53L0X_WriteReg(dev, 0x70, 0x04);
    VL53L0X_WriteReg(dev, 0x71, 0x01);
    VL53L0X_WriteReg(dev, 0x72, 0xFE);
    VL53L0X_WriteReg(dev, 0x76, 0x00);
    VL53L0X_WriteReg(dev, 0x77, 0x00);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x0D, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x80, 0x01);
    VL53L0X_WriteReg(dev, 0x01, 0xF8);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x8E, 0x01);
    VL53L0X_WriteReg(dev, 0x00, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x80, 0x00);

    // 11. 配置 GPIO 中断 (新数据准备)
    VL53L0X_WriteReg(dev, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    uint8_t gpio_hv = VL53L0X_ReadReg(dev, GPIO_HV_MUX_ACTIVE_HIGH);
    gpio_hv &= ~0x10;   // 低电平有效
    VL53L0X_WriteReg(dev, GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv);
    VL53L0X_WriteReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

    // 12. 获取当前时序预算
    dev->measurement_timing_budget_us = VL53L0X_GetMeasurementTimingBudget(dev);

    // 13. 禁用 MSRC 和 TCC
    VL53L0X_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    // 14. 重新应用时序预算
    VL53L0X_SetMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    // 15. 执行参考校准 (VHV 和 Phase)
    VL53L0X_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(dev, 0x40))
        return false;
    VL53L0X_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(dev, 0x00))
        return false;

    // 16. 恢复序列配置
    VL53L0X_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return true;
}

void VL53L0X_SetAddress(VL53L0X_Dev_t *dev, uint8_t new_addr)
{
    VL53L0X_WriteReg(dev, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F);
    dev->address = new_addr;
}

uint8_t VL53L0X_GetAddress(VL53L0X_Dev_t *dev)
{
    return dev->address;
}

bool VL53L0X_SetSignalRateLimit(VL53L0X_Dev_t *dev, float limit_Mcps)
{
    if (limit_Mcps < 0 || limit_Mcps > 511.99f)
        return false;
    uint16_t value = (uint16_t)(limit_Mcps * (1 << 7));
    return VL53L0X_WriteReg16(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, value);
}

float VL53L0X_GetSignalRateLimit(VL53L0X_Dev_t *dev)
{
    uint16_t value = VL53L0X_ReadReg16(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT);
    return (float)value / (1 << 7);
}

bool VL53L0X_SetMeasurementTimingBudget(VL53L0X_Dev_t *dev, uint32_t budget_us)
{
    const uint16_t StartOverhead = 1320;
    const uint16_t EndOverhead = 960;
    const uint16_t MsrcOverhead = 660;
    const uint16_t TccOverhead = 590;
    const uint16_t DssOverhead = 690;
    const uint16_t PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;
    const uint32_t MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget)
        return false;

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    bool tcc, dss, msrc, pre_range, final_range;
    getSequenceStepEnables(dev, &tcc, &dss, &msrc, &pre_range, &final_range);

    uint16_t pre_range_vcsel_pclks, final_range_vcsel_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint32_t msrc_dss_tcc_us;
    uint16_t pre_range_mclks;
    uint32_t pre_range_us;
    uint16_t final_range_mclks;
    uint32_t final_range_us;

    getSequenceStepTimeouts(dev, pre_range, final_range,
                            &pre_range_vcsel_pclks, &final_range_vcsel_pclks,
                            &msrc_dss_tcc_mclks, &msrc_dss_tcc_us,
                            &pre_range_mclks, &pre_range_us,
                            &final_range_mclks, &final_range_us);

    if (tcc) {
        used_budget_us += (msrc_dss_tcc_us + TccOverhead);
    }
    if (dss) {
        used_budget_us += 2 * (msrc_dss_tcc_us + DssOverhead);
    } else if (msrc) {
        used_budget_us += (msrc_dss_tcc_us + MsrcOverhead);
    }
    if (pre_range) {
        used_budget_us += (pre_range_us + PreRangeOverhead);
    }
    if (final_range) {
        used_budget_us += FinalRangeOverhead;
        if (used_budget_us > budget_us) {
            return false;
        }
        uint32_t final_range_timeout_us = budget_us - used_budget_us;
        uint16_t final_range_timeout_mclks = (uint16_t)timeoutMicrosecondsToMclks(final_range_timeout_us, final_range_vcsel_pclks);
        if (pre_range) {
            final_range_timeout_mclks += pre_range_mclks;
        }
        VL53L0X_WriteReg16(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));
        dev->measurement_timing_budget_us = budget_us;
    }
    return true;
}

uint32_t VL53L0X_GetMeasurementTimingBudget(VL53L0X_Dev_t *dev)
{
    const uint16_t StartOverhead = 1910;
    const uint16_t EndOverhead = 960;
    const uint16_t MsrcOverhead = 660;
    const uint16_t TccOverhead = 590;
    const uint16_t DssOverhead = 690;
    const uint16_t PreRangeOverhead = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t budget_us = StartOverhead + EndOverhead;

    bool tcc, dss, msrc, pre_range, final_range;
    getSequenceStepEnables(dev, &tcc, &dss, &msrc, &pre_range, &final_range);

    uint16_t pre_range_vcsel_pclks, final_range_vcsel_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint32_t msrc_dss_tcc_us;
    uint16_t pre_range_mclks;
    uint32_t pre_range_us;
    uint16_t final_range_mclks;
    uint32_t final_range_us;

    getSequenceStepTimeouts(dev, pre_range, final_range,
                            &pre_range_vcsel_pclks, &final_range_vcsel_pclks,
                            &msrc_dss_tcc_mclks, &msrc_dss_tcc_us,
                            &pre_range_mclks, &pre_range_us,
                            &final_range_mclks, &final_range_us);

    if (tcc) {
        budget_us += (msrc_dss_tcc_us + TccOverhead);
    }
    if (dss) {
        budget_us += 2 * (msrc_dss_tcc_us + DssOverhead);
    } else if (msrc) {
        budget_us += (msrc_dss_tcc_us + MsrcOverhead);
    }
    if (pre_range) {
        budget_us += (pre_range_us + PreRangeOverhead);
    }
    if (final_range) {
        budget_us += (final_range_us + FinalRangeOverhead);
    }

    dev->measurement_timing_budget_us = budget_us;
    return budget_us;
}

bool VL53L0X_SetVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type, uint8_t period_pclks)
{
    uint8_t vcsel_period_reg = ENCODE_VCSEL_PERIOD(period_pclks);

    bool tcc, dss, msrc, pre_range, final_range;
    getSequenceStepEnables(dev, &tcc, &dss, &msrc, &pre_range, &final_range);

    uint16_t pre_range_vcsel_pclks, final_range_vcsel_pclks;
    uint16_t msrc_dss_tcc_mclks;
    uint32_t msrc_dss_tcc_us;
    uint16_t pre_range_mclks;
    uint32_t pre_range_us;
    uint16_t final_range_mclks;
    uint32_t final_range_us;
    getSequenceStepTimeouts(dev, pre_range, final_range,
                            &pre_range_vcsel_pclks, &final_range_vcsel_pclks,
                            &msrc_dss_tcc_mclks, &msrc_dss_tcc_us,
                            &pre_range_mclks, &pre_range_us,
                            &final_range_mclks, &final_range_us);

    if (type == VcselPeriodPreRange) {
        switch (period_pclks) {
            case 12: VL53L0X_WriteReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18); break;
            case 14: VL53L0X_WriteReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30); break;
            case 16: VL53L0X_WriteReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40); break;
            case 18: VL53L0X_WriteReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50); break;
            default: return false;
        }
        VL53L0X_WriteReg(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        VL53L0X_WriteReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        uint16_t new_pre_range_timeout_mclks = (uint16_t)timeoutMicrosecondsToMclks(pre_range_us, period_pclks);
        VL53L0X_WriteReg16(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));

        uint16_t new_msrc_timeout_mclks = (uint16_t)timeoutMicrosecondsToMclks(msrc_dss_tcc_us, period_pclks);
        uint8_t msrc_timeout = (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1);
        VL53L0X_WriteReg(dev, MSRC_CONFIG_TIMEOUT_MACROP, msrc_timeout);
    } else if (type == VcselPeriodFinalRange) {
        switch (period_pclks) {
            case 8:
                VL53L0X_WriteReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
                VL53L0X_WriteReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_WriteReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
                VL53L0X_WriteReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
                VL53L0X_WriteReg(dev, 0xFF, 0x01);
                VL53L0X_WriteReg(dev, ALGO_PHASECAL_LIM, 0x30);
                VL53L0X_WriteReg(dev, 0xFF, 0x00);
                break;
            case 10:
                VL53L0X_WriteReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
                VL53L0X_WriteReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_WriteReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_WriteReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
                VL53L0X_WriteReg(dev, 0xFF, 0x01);
                VL53L0X_WriteReg(dev, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_WriteReg(dev, 0xFF, 0x00);
                break;
            case 12:
                VL53L0X_WriteReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
                VL53L0X_WriteReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_WriteReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_WriteReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
                VL53L0X_WriteReg(dev, 0xFF, 0x01);
                VL53L0X_WriteReg(dev, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_WriteReg(dev, 0xFF, 0x00);
                break;
            case 14:
                VL53L0X_WriteReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
                VL53L0X_WriteReg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
                VL53L0X_WriteReg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
                VL53L0X_WriteReg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
                VL53L0X_WriteReg(dev, 0xFF, 0x01);
                VL53L0X_WriteReg(dev, ALGO_PHASECAL_LIM, 0x20);
                VL53L0X_WriteReg(dev, 0xFF, 0x00);
                break;
            default: return false;
        }
        VL53L0X_WriteReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

        uint16_t new_final_range_timeout_mclks = (uint16_t)timeoutMicrosecondsToMclks(final_range_us, period_pclks);
        if (pre_range) {
            new_final_range_timeout_mclks += pre_range_mclks;
        }
        VL53L0X_WriteReg16(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks));
    } else {
        return false;
    }

    // 重新应用时序预算
    VL53L0X_SetMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    // 重新执行相位校准
    uint8_t seq_config = VL53L0X_ReadReg(dev, SYSTEM_SEQUENCE_CONFIG);
    VL53L0X_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    performSingleRefCalibration(dev, 0x00);
    VL53L0X_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, seq_config);

    return true;
}

uint8_t VL53L0X_GetVcselPulsePeriod(VL53L0X_Dev_t *dev, vcselPeriodType type)
{
    if (type == VcselPeriodPreRange) {
        return DECODE_VCSEL_PERIOD(VL53L0X_ReadReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD));
    } else {
        return DECODE_VCSEL_PERIOD(VL53L0X_ReadReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    }
}

void VL53L0X_StartContinuous(VL53L0X_Dev_t *dev, uint32_t period_ms)
{
    VL53L0X_WriteReg(dev, 0x80, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x00, 0x00);
    VL53L0X_WriteReg(dev, 0x91, dev->stop_variable);
    VL53L0X_WriteReg(dev, 0x00, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x80, 0x00);

    if (period_ms != 0) {
        // 定时连续模式
        uint16_t osc_cal_val = VL53L0X_ReadReg16(dev, OSC_CALIBRATE_VAL);
        if (osc_cal_val != 0) {
            period_ms = (uint32_t)period_ms * osc_cal_val;
        }
        VL53L0X_WriteReg32(dev, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);
        VL53L0X_WriteReg(dev, SYSRANGE_START, 0x04);
    } else {
        // 背对背连续模式
        VL53L0X_WriteReg(dev, SYSRANGE_START, 0x02);
    }
}

void VL53L0X_StopContinuous(VL53L0X_Dev_t *dev)
{
    VL53L0X_WriteReg(dev, SYSRANGE_START, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x00, 0x00);
    VL53L0X_WriteReg(dev, 0x91, 0x00);
    VL53L0X_WriteReg(dev, 0x00, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
}

uint16_t VL53L0X_ReadRangeContinuousMillimeters(VL53L0X_Dev_t *dev)
{
    START_TIMEOUT(dev);
    while ((VL53L0X_ReadReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (CHECK_TIMEOUT(dev)) {
            dev->did_timeout = true;
            return VL53L0X_INVALID_DISTANCE;
        }
    }

    uint16_t range = VL53L0X_ReadReg16(dev, RESULT_RANGE_STATUS + 10);
    VL53L0X_WriteReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
    return range;
}
uint16_t VL53L0X_ReadRangeSingleMillimeters(VL53L0X_Dev_t *dev)
{
    VL53L0X_WriteReg(dev, 0x80, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x00, 0x00);
    VL53L0X_WriteReg(dev, 0x91, dev->stop_variable);
    VL53L0X_WriteReg(dev, 0x00, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x80, 0x00);
    VL53L0X_WriteReg(dev, SYSRANGE_START, 0x01);

    START_TIMEOUT(dev);
    while (VL53L0X_ReadReg(dev, SYSRANGE_START) & 0x01) {
        if (CHECK_TIMEOUT(dev)) {
            dev->did_timeout = true;
            return VL53L0X_INVALID_DISTANCE;
        }
    }
    return VL53L0X_ReadRangeContinuousMillimeters(dev);
}

void VL53L0X_SetTimeout(VL53L0X_Dev_t *dev, uint16_t timeout_ms)
{
    dev->timeout_ms = timeout_ms;
}

uint16_t VL53L0X_GetTimeout(VL53L0X_Dev_t *dev)
{
    return dev->timeout_ms;
}

bool VL53L0X_TimeoutOccurred(VL53L0X_Dev_t *dev)
{
    bool tmp = dev->did_timeout;
    dev->did_timeout = false;
    return tmp;
}

/* ========== 私有函数实现 ========== */
static bool getSpadInfo(VL53L0X_Dev_t *dev, uint8_t *count, bool *type_is_aperture)
{
    VL53L0X_WriteReg(dev, 0x80, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x00, 0x00);
    VL53L0X_WriteReg(dev, 0xFF, 0x06);
    uint8_t reg83 = VL53L0X_ReadReg(dev, 0x83);
    reg83 |= 0x04;
    VL53L0X_WriteReg(dev, 0x83, reg83);
    VL53L0X_WriteReg(dev, 0xFF, 0x07);
    VL53L0X_WriteReg(dev, 0x81, 0x01);
    VL53L0X_WriteReg(dev, 0x80, 0x01);
    VL53L0X_WriteReg(dev, 0x94, 0x6b);
    VL53L0X_WriteReg(dev, 0x83, 0x00);

    START_TIMEOUT(dev);
    while (VL53L0X_ReadReg(dev, 0x83) == 0x00) {
        if (CHECK_TIMEOUT(dev))
            return false;
    }
    VL53L0X_WriteReg(dev, 0x83, 0x01);
    uint8_t tmp = VL53L0X_ReadReg(dev, 0x92);
    *count = tmp & 0x7F;
    *type_is_aperture = (tmp >> 7) & 0x01;

    VL53L0X_WriteReg(dev, 0x81, 0x00);
    VL53L0X_WriteReg(dev, 0xFF, 0x06);
    reg83 = VL53L0X_ReadReg(dev, 0x83);
    reg83 &= ~0x04;
    VL53L0X_WriteReg(dev, 0x83, reg83);
    VL53L0X_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_WriteReg(dev, 0x00, 0x01);
    VL53L0X_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_WriteReg(dev, 0x80, 0x00);
    return true;
}

static void getSequenceStepEnables(VL53L0X_Dev_t *dev, bool *tcc, bool *dss, bool *msrc, bool *pre_range, bool *final_range)
{
    uint8_t seq_config = VL53L0X_ReadReg(dev, SYSTEM_SEQUENCE_CONFIG);
    *tcc         = (seq_config >> 4) & 0x01;
    *dss         = (seq_config >> 3) & 0x01;
    *msrc        = (seq_config >> 2) & 0x01;
    *pre_range   = (seq_config >> 6) & 0x01;
    *final_range = (seq_config >> 7) & 0x01;
}

static void getSequenceStepTimeouts(VL53L0X_Dev_t *dev, bool pre_range_en, bool final_range_en,
                                    uint16_t *pre_range_vcsel_pclks, uint16_t *final_range_vcsel_pclks,
                                    uint16_t *msrc_dss_tcc_mclks, uint32_t *msrc_dss_tcc_us,
                                    uint16_t *pre_range_mclks, uint32_t *pre_range_us,
                                    uint16_t *final_range_mclks, uint32_t *final_range_us)
{
    *pre_range_vcsel_pclks = VL53L0X_GetVcselPulsePeriod(dev, VcselPeriodPreRange);
    *msrc_dss_tcc_mclks = VL53L0X_ReadReg(dev, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    *msrc_dss_tcc_us = timeoutMclksToMicroseconds(*msrc_dss_tcc_mclks, *pre_range_vcsel_pclks);

    uint16_t pre_raw = VL53L0X_ReadReg16(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI);
    *pre_range_mclks = decodeTimeout(pre_raw);
    *pre_range_us = timeoutMclksToMicroseconds(*pre_range_mclks, *pre_range_vcsel_pclks);

    *final_range_vcsel_pclks = VL53L0X_GetVcselPulsePeriod(dev, VcselPeriodFinalRange);
    uint16_t final_raw = VL53L0X_ReadReg16(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI);
    *final_range_mclks = decodeTimeout(final_raw);

    if (pre_range_en) {
        *final_range_mclks -= *pre_range_mclks;
    }
    *final_range_us = timeoutMclksToMicroseconds(*final_range_mclks, *final_range_vcsel_pclks);
}

static uint16_t decodeTimeout(uint16_t reg_val)
{
    return (uint16_t)((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encodeTimeout(uint16_t timeout_mclks)
{
    if (timeout_mclks == 0) return 0;
    uint32_t ls_byte = timeout_mclks - 1;
    uint16_t ms_byte = 0;
    while ((ls_byte & 0xFFFFFF00) > 0) {
        ls_byte >>= 1;
        ms_byte++;
    }
    return (ms_byte << 8) | (ls_byte & 0xFF);
}

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = CALC_MACRO_PERIOD(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = CALC_MACRO_PERIOD(vcsel_period_pclks);
    return ((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns;
}

static bool performSingleRefCalibration(VL53L0X_Dev_t *dev, uint8_t vhv_init_byte)
{
    VL53L0X_WriteReg(dev, SYSRANGE_START, 0x01 | vhv_init_byte);
    START_TIMEOUT(dev);
    while ((VL53L0X_ReadReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (CHECK_TIMEOUT(dev))
            return false;
    }
    VL53L0X_WriteReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
    VL53L0X_WriteReg(dev, SYSRANGE_START, 0x00);
    return true;
}
