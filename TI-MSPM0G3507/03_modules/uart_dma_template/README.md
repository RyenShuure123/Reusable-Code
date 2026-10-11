# UART DMA 收发模板（MSPM0 DriverLib）

来源：`mspm0g3507_uart_dma` 工程（`src/uart_pc.c`）。MSPM0 上少见的一套
完整"DMA 收 + DMA 发"串口方案，主循环完全无阻塞。

## 设计

- **RX**：DMA 通道设为"每次搬 1 字节"从 `UART->RXDATA` 到内存，
  `DMA_DONE_RX` 中断里把字节压入 128 字节**环形缓冲**并重新挂载 DMA。
  主循环用 `UART_PC_TryReceiveByte()` 逐字节取，不丢数据、不阻塞。
- **TX**：`UART_PC_StartDmaTransmit(buf, len)` 整块 DMA 发送，
  `IsDmaTransmitBusy()` 查询（含 EOT）——发 VOFA 帧前先判断，避免撕裂。
- 溢出置 `g_rxOverflowed` 标志，可查询。

## 文件

- `uart_pc.c/.h`：完整实现
- `example_main.c`：收发回显演示（收到即 DMA 发回，并翻转 LED）

## 依赖的 SysConfig 配置

- UART，`$name = UART_PC`（宏 `UART_PC_INST`、`UART_PC_INST_INT_IRQN`），
  开启 DMA RX/TX 通道（宏 `DMA_CH1_CHAN_ID` / `DMA_CH0_CHAN_ID`，
  对应 UART 的 DMA_CHANNEL_RX/TX）

移植到第二条串口时，把 `uart_pc` 前缀、实例宏和 DMA 通道号换掉即可
（01 工程里的 `uart_bluetooth.c` 就是同一模板的蓝牙版实例）。
