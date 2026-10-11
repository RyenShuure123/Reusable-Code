# Car3519_smartcar：MSPM0G3519 电赛四轮小车（完整工程）

串级 PID 四轮小车参考实现：速度内环 + 循迹 PD / 角度环外环，10ms 定时器节拍，
蓝牙在线调参，VOFA+ 上位机曲线，OLED 诊断台，IMU 走自定义 UART 协议。
构建链路：`SysConfig CLI -> CMake -> ARM GCC -> OpenOCD`，不依赖 Code Composer Studio。

## 目录结构

```
src/           应用代码（main、pid、drive_control、encoder、motor、loop 循迹、
               control_timer 10ms 调度、button、LED、uart_bluetooth、vofa_justfloat）
include/       对应头文件
imu/           外置 IMU 节点协议：yaw_uart_protocol.h（帧格式）、
               yaw_uart_receiver.c（状态机解析 + 丢帧/校验诊断）、imu_uart.c（DMA 收 + 反馈管理）
tasks/         赛题任务：task1 = 1m×1m 正方形走位（task2/task3 为空占位）
Module/        SSD1306 OLED + 字库
syscfg/        SysConfig 源文件（唯一应手工编辑的硬件配置）
generated/     SysConfig 生成物，构建时自动重新生成，不要手改（本仓库未收录）
cmake/         arm-none-eabi 工具链文件
openocd/       DAPLink / XDS110 烧录配置
tools/         flash、工厂复位、生成 syscfg 的脚本
```

## 硬件外设（syscfg/car3519_minimal.syscfg）

| 外设 | 用途 |
|---|---|
| QEI_LEFT / QEI_RIGHT | 双路硬件正交编码器（TimerG） |
| PWM_LEFT / PWM_RIGHT | 左右电机 PWM + GPIO_MOTOR 方向脚（PH/EN 驱动板） |
| TIMER_PID | 10ms 控制节拍中断（Button/Drive/Track/Angle 全在这里 Tick） |
| UART_IMU | 外置 IMU 节点 yaw 反馈 |
| UART_BLUETOOTH + DMA_CH0/CH1 | HC-05 蓝牙：收调参命令，发 VOFA JustFloat |
| I2C_OLED | SSD1306 诊断显示 |
| GPIO_LOOP (AD0/AD1/AD2 + INPUT) | 8 路灰度传感器多路复用采集 |
| GPIO_LED (RGB) / GPIO_Key (Key0-2) | 状态灯与按键（Key0=task1，Key1=循迹，Key2=停止） |

## 控制要点（代码对应）

- `src/pid.c`：速度 PI（条件积分抗饱和）、循迹 PD、角度 PID（D 项 = 陀螺 yaw 角速度，
  积分独立限幅）。标定常数 `PULSES_PER_METER 268209`，控制周期 `dt 0.01f`。
- `src/drive_control.c`：直线定距（脉冲计程）、角度环（断链保护/到位判定/原地转
  最小速度）、循迹（丢线方向记忆 + 偏差分段降速）、蓝牙调参命令解析
  （`KP:0.002 KD:0.001 AP:0.008 AI:0.002 AD:0.001`，空格/逗号分隔）。
- `imu/yaw_uart_receiver.c`：16 字节帧 `A5 5A | ver | len | seq | status | 航向/角速度/
  横滚/俯仰(0.01°) | err | checksum(累加和)`，逐字节状态机，含丢帧与错误统计。
  **协议发送端不在本仓库**，需自行在 IMU 节点按 `imu/yaw_uart_protocol.h` 组帧。

## 构建（PowerShell，工程根目录）

CMakeLists 顶部几处绝对路径需先改成自己机器的安装位置：
`SDK_ROOT`、`SYSCONFIG_CLI`、`OPENOCD_EXECUTABLE/SCRIPTS`。

```powershell
cmake --preset g3519-debug
cmake --build --preset g3519-debug
```

构建自动先跑 SysConfig，产物为 `build/g3519-debug/app.hex`。

## 烧录

接好 DAPLink 的 SWDIO/SWCLK/GND 与目标板供电后：

```powershell
.\tools\flash_openocd.bat        # 或双击；XDS110 用 flash_xds110.bat
```

`tools\factory_reset_mspm0.bat` 仅在芯片锁死（如 NONMAIN 配置错误）时使用，会擦除全片。

## CLion

打开工程目录后，右上角 CMake 配置选 `g3519-debug`（读 `CMakePresets.json`），
锤子构建即可。新增外设时先在 SysConfig GUI 里编辑 `syscfg/car3519_minimal.syscfg`
再构建，代码只能使用 `generated/ti_msp_dl_config.h` 中实际存在的宏。
