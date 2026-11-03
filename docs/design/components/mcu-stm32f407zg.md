# 主控 STM32F407ZGT6（核心板 V2.0）

## 型号与来源

- 板卡：**STM32F407ZG-P1 核心板 V2.0**，厂商资料包 `docs/references/vendor/STM32F407-CoreBoard/`（原理图 `STM32F407ZG-P1核心板原理图V2.0.pdf`、`详细尺寸图.png`、正反面丝印图）。
- 芯片：STM32F407ZGT6；固件 PlatformIO 板级 `board = black_f407zg`（`software/firmware/platformio.ini`）。
- 资料日期：2026-11-03 归档；板卡版本以原理图 V2.0 为准。

## 关键参数

| 项目 | 值 | 来源 |
| --- | --- | --- |
| 内核/主频 | Cortex-M4F，168 MHz（F407 系列标称） | 厂商数据手册 |
| 供电路线 1 | 外部 5 V 或 USB → 板上 3V3 稳压 → STM32 | `【8】供电说明/供电说明.png` |
| 供电路线 2 | 外部 3V3 直接供 STM32（二选一，不可同时） | 同上 |
| 3V3 对外输出 | **最大 500 mA** | 同上（厂商供电说明图标注） |
| 调试/下载 | ST-Link（`upload_protocol = stlink`）、DAP、串口 ISP | `platformio.ini`、资料包驱动目录 |

## 接线与引脚

本项目的实际引脚分配见 [`../../system-overview.md`](../../system-overview.md)，摘要：

- Motor1：PA6 STEP（TIM3_CH1）/ PA7 DIR
- Motor2：PA8 STEP（TIM1_CH1）/ PB0 DIR
- BMI270：PB8 SCL / PB9 SDA
- 编码器 1：PA0/PA1；编码器 2：PB6/PB7
- 蓝牙 USART3：PB10 TX / PB11 RX
- TMC UART：后轮 PA9（USART1）、前轮 PA2（USART2）

## 实测记录

- 2026-09-28：PB0 在后轮 TMC 载板插上时被拉到约 1 V，不插载板正常——最终定位为载板缺陷，不是 MCU（详见 `history/`）。
- 2026-10-22：`firmware.elf` 编译产物 152 772 B；`firmware.bin` 77 852 B（2026-10-15）。

## 已知问题

- **3V3 只有 500 mA 余量**：IMU、蓝牙、TMC 逻辑、编码器都挂在这路，后续加模块（OLED、无线、第二路传感器）前要先算电流，电机电源必须走 12 V 独立回路。
- 资料包里的例程以 F103/F407 通用教学为主，不能直接当本项目的板级定义。

## 待验证

- 板上 5 V 实际来自 USB 还是外部 DC，未记录。
- 3V3 现有负载实测值（mA）未测；加模块前应补测。
