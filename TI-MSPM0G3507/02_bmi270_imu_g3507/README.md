# BMI270 姿态解算工程：MSPM0G3507 + 硬件 SPI + Mahony

不依赖 Bosch 官方庞大驱动包的 BMI270 寄存器级驱动，配 Mahony 互补滤波姿态解算，
可直接作为 IMU 节点固件底座。

## 代码导览

```
imu/bmi270_minimal.c/.h   BMI270 最小驱动：软复位、等 config file 加载完成、
                          配置 ODR/量程/滤波器、读 FIFO/原始数据（寄存器级实现）
imu/bmi270_port.c/.h      SPI 读写移植层（所有硬件相关调用集中在这里，换主控只改这一个文件）
imu/bmi270_config.c/.h    BMI270 初始化配置文件数据
imu/imu.c/.h              应用层：IMU_Init / IMU_CalibrateGyro(600 样本静止零偏校准) /
                          IMU_Update(dt) / IMU_GetAngles(roll, pitch, yaw)
imu/mahonyRK4.c/.h        Mahony 姿态融合（四元数，纯 IMU 无磁力计；另有仅陀螺积分模式）
src/app_bmi270.c          演示：5ms 周期解算，串口文本输出 R=/P=/Y=
src/timebase.c            毫秒时基（SysTick），调度用
src/oled.c, servo.c       顺带包含的显示与舵机驱动（同 03_modules）
syscfg/                   SysConfig 配置（SPI、I2C OLED、UART、定时器等）
```

## 关键细节（踩过坑的）

- BMI270 上电后必须**等内部 config file 加载完成**（轮询内部状态寄存器）才能写
  配置，`bmi270_minimal.c` 里已处理；
- 陀螺仪零偏校准：上电静置采 600 个样本（3 秒）求平均，作为后续补偿，
  `IMU_CalibrateGyro()` 实现；
- Mahony 的 Kp/Ki 可通过 `Mahony_SetPI()` 调整，yaw 单独用陀螺积分（无磁力计
  时加速度计修正不了 yaw）。

## 构建

先改 `CMakeLists.txt` 顶部 `SDK_ROOT` / `SYSCONFIG_CLI` / OpenOCD 路径，然后

```powershell
cmake --preset g3507
cmake --build --preset g3507
```

产物 `build/g3507/bmi270_spi_basic.hex`，`tools/flash_openocd.bat` 烧录
（`flash_fast_20mhz.bat` 为 20MHz SWD 快速烧录版本）。

## 注意

本工程把解算结果以**文本**打印到串口，并未实现发往小车的 yaw 二进制协议
要做 IMU 节点，在本工程基础上按协议组帧发送即可。
