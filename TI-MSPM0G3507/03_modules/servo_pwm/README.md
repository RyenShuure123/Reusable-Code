# 舵机 PWM 模块

来源：`mspm0g3507_servo_learning` 工程。标准 50Hz 舵机控制。

- `servo.c/.h`：`Servo_Init()` / `Servo_SetPulseUs(pulseUs)` /
  `Servo_SetAngle(angle)`；周期 20ms（50Hz），中位 1500us，
  比较值直接由"周期 - 脉宽"推出
- `example_main.c`：初始化后定位到 45° 的最小示例

## 依赖的 SysConfig 配置

- PWM（TimerA），`$name = PWM_SERVO`（宏 `PWM_SERVO_INST`），周期 20ms（50Hz）

## 说明

`Servo_SetPulseUs` 直接以微秒设比较值，比"角度查表"更透明；
换舵机行程时只需要改限幅常量。
