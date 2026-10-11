# VOFA+ JustFloat 遥测 + HC-05 蓝牙通道

来源：`Car3519new`（即 01 工程）。两者配合：蓝牙串口发 JustFloat 帧，
电脑端 VOFA+ 直接看波形调参。

- `vofa_justfloat.c/.h`：JustFloat 帧打包（N 个 float + 帧尾 `00 00 80 7F`），
  提供 Send3/4/6 三个重载；发送前检查通道忙，忙则丢弃本帧（不阻塞控制环）
- `uart_bluetooth.c/.h`：蓝牙串口驱动 —— RX：DMA 单字节→环形缓冲
  （供调参命令逐字节读取）；TX：DMA 整块发送 + 忙标志；另有 SendByte/
  SendString/SendHex8 小工具

## 依赖的 SysConfig 配置

- UART，`$name = UART_BLUETOOTH`（宏 `UART_BLUETOOTH_INST`），挂 DMA
  CHANNEL_RX=CH1 / CHANNEL_TX=CH0（宏 `DMA_CH1_CHAN_ID` / `DMA_CH0_CHAN_ID`）

## 用法（在 10ms 控制中断或主循环里）

```c
VOFA_JustFloat_Send4(targetSpeed, measuredSpeed, (float)trackError, pwmOut);
```

接收调参命令一侧见 01 工程 `drive_control.c` 的 `TrackControl_ProcessTuning()`
（命令行协议 `KP:0.002 KD:0.001 ...`）。
