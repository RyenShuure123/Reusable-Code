# G3507 电机驱动 + 10ms 控制定时器

来源：`mspm0g3507_timer_pid` 工程（`control/`）。

- `motor.c/.h`：双电机 IN/IN 驱动（两路 PWM 控一路电机的速度与方向），
  接口为**千分比占空比**：`Motor_SetOutputs(left, right)`，范围 ±1000，
  内部限幅
- `timer.c/.h`：10ms 周期控制定时器，中断里维护 tick 计数并按固定周期
  调 `IMU_Update()`（IMU 不在此模块时删掉该调用即可）
- `接线记录.txt`：原作者的引脚分配记录（PA0/PA1=左电机 TIMA0，
  PA17/PA24=右电机 TIMG7，编码器左路 TIMG8 硬件 QEI、右路 GPIO 中断软解码）
- `example_main.c`：按键启停电机 + BMI270 校准的演示

## 依赖的 SysConfig 配置

- 两路 PWM：`$name = PWM_MOTOR_LEFT`（TimerA）、`$name = PWM_MOTOR_RIGHT`（TimerG）
- 定时器：`$name = TIMER_CONTROL`（宏 `TIMER_CONTROL_INST`，10ms 周期中断）

## 说明

- `control/encoder.c` 在原工程里是 0 字节占位（软解码方案未完成），
  故未收录；G3519 整车上已经用双路硬件 QEI 解决，见 01 工程 `src/encoder.c`。
- 01 工程 `src/motor.c` 是这套驱动的"进化版"：PH/EN 接口、
  换向前先归零占空比，配合速度环输出的是 ±PWM tick。
