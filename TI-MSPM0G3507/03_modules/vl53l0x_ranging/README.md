# VL53L0X 激光测距模块（I2C）

来源：`mspm0g3507_hcsr04_vl53l0` 工程。寄存器级实现，不依赖 ST 官方 API 库，
带独立的 I2C 总线抽象层，换主控只需重写 `vl53l0x_bus.c`。

- `vl53l0x.c/.h`：探测、初始化、连续测距启动/读取、XSHUT 硬件复位、
  GPIO 中断极性配置；无效量程返回 `VL53L0X_INVALID_DISTANCE`
- `vl53l0x_bus.c/.h`：I2C 读写抽象（DL_I2C controller 模式，处理 FIFO 满/空）
- `example_main.c`：初始化顺序 + 100ms 周期读距 + OLED 显示 + VOFA 输出的完整演示

## 依赖的 SysConfig 配置

- I2C 控制器，`$name = I2C_OLED`（宏 `I2C_OLED_INST`；VL53L0X 与 OLED 共总线也可）
- GPIO 输出脚，`$name = GPIO_VL53L0X`，引脚名 `VL53L0X_XSHUT`
  （宏 `GPIO_VL53L0X_PORT` / `GPIO_VL53L0X_PIN_VL53L0X_XSHUT_PIN`）

## 用法

```c
VL53L0X_Dev_t vl53l0x;
VL53L0X_HardwareReset();
uint8_t modelId;
if (VL53L0X_Probe(&modelId) && modelId == 0xEE) {   // Model ID 应为 0xEE
    VL53L0X_Init(&vl53l0x, true);
    VL53L0X_StartContinuous(&vl53l0x, 0U);
    uint16_t mm = VL53L0X_ReadRangeContinuousMillimeters(&vl53l0x);
}
```

注意：原工程名带 "hcsr04"，但 HC-SR04 超声部分从未实现，本模块只有 VL53L0X。
