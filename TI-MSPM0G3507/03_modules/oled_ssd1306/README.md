# SSD1306 OLED 驱动（I2C，双缓冲帧）

来源：`mspm0g3507_oled_learning` 工程（与 01 工程的 `Module/oled.c` 同源）。

- `oled.c/.h`：画点/线/圆、ASCII 字符串、数字、中文取模字符串、
  `OLED_NewFrame()` + `OLED_ShowFrame()` 双缓冲（先在 RAM 里画完整帧再一次
  刷出，避免逐行刷新撕裂）
- `font.c/.h`：`afont8x6` 等 ASCII 字库 + 中文取模
- `example_main.c`：上电显示 "MSPM0G3507 / OLED OK" 的最小文本演示

## 依赖的 SysConfig 配置

- I2C 控制器，`$name = I2C_OLED`（宏 `I2C_OLED_INST`），0.96 寸 SSD1306，
  从机地址在 `oled.h`（`OLED_ADDRESS`，默认 0x78）

## 用法

```c
OLED_Init();
OLED_NewFrame();
OLED_ShowString(1, 1, "MSPM0G3507");
OLED_PrintASCIIString(0, 16, "hello", &afont8x6, OLED_COLOR_NORMAL);
OLED_ShowFrame();
```

01 工程的 `main.c` 里还有一个"诊断台"实例：只在数据变化超过阈值时重绘，
避免 I2C 刷屏占用主循环。
