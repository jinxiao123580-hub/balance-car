# 参考资料索引

| 属性 | 内容 |
| --- | --- |
| 文档编号 | PH-REFERENCES |
| 更新日期 | 2026-11-03，Asia/Shanghai |

## 本地资料与公开链接

离线厂商资料位于 `vendor/`，**默认不入 Git**（见仓库 `.gitignore` 的 `docs/references/vendor/`）。
厂商 PDF、示例工程、工具软件和视频的版权归原厂，公开仓库只保留链接与本项目自己的结论。

| 目录 | 内容 | 版本/日期 | 体积 |
| --- | --- | --- | --- |
| `vendor/STM32F407-CoreBoard/` | STM32F407ZG-P1 核心板：原理图 V2.0、详细尺寸图、正反面丝印、HAL/标准库例程、参考手册与数据手册、DAP/ST-Link/CH340 驱动、烧录教程、供电说明 | 板卡 V2.0 | 798 MB |
| `vendor/BMI270/` | Bosch BMI270 数据手册（BST-BMI270-DS000-05 Rev 1.3，2020-11）+ 商品页 | Rev 1.3 | 4.6 MB |
| `vendor/MKS-TMC2209/` | MKS TMC2209 商品页截图（含官方 GitHub 与 B 站教程链接） | 卖家页 | 5.0 MB |
| `vendor/WHEELTEC-MS42DC/` | 驱控一体电机手册、驱控一体底盘资料 V2.2、步进电机原理与驱动教程 V1.0。**本项目只采购了该厂商的 42 电机与编码器**，驱控一体与底盘部分未采用 | V2.2 / V1.0 | 1.9 GB |
| `vendor/*.pdf` | MKS TMC2209 V2.0 原理图、TMC2209 数据手册、TI SLVA321A | — | 2.6 MB |

公开链接：

- [MKS 官方仓库](https://github.com/makerbase-mks/MKS-StepStick-Driver)
- [MKS TMC2209 仓库](https://github.com/makerbase-mks/MKS-TMC2209)
- [ADI TMC2209 资料](https://www.analog.com/en/products/tmc2209.html)
- [TI SLVA321A](https://www.ti.com/lit/an/slva321a/slva321a.pdf)

## 使用边界

- 厂商资料用于理解器件与连接方式，**不能单独证明手中载板的版本、焊位或实测电平**；结论以本项目实测记录为准（见 `docs/validation/`、`history/`）。
- `WHEELTEC-MS42DC` 里的驱控一体电机与底盘资料**本项目未采用**（只买了电机 + 编码器）。资料中的 MS42DC 参数（10–25 V、最大 1.2 A、固定 32 细分、串口/CAN）不能套到本项目的 MS42C + 外置驱动器上，详见 `docs/design/components/motor-ms42c.md`。
- 转载、二次分发前需分别核对厂商许可与自有设计来源。

## 版权与署名

- 本仓库**不包含**厂商的 PDF、图片、示例工程、工具软件和视频；`vendor/` 仅在本机使用且不入 Git。
- 文中出现的型号、参数、引脚定义为**事实性引用**，用于说明本项目硬件；商标与文档版权归原厂（WHEELTEC、Bosch Sensortec、Makerbase/MKS、BigTreeTech、TI、STMicroelectronics 等）。
- 需要原文请通过厂商官方渠道获取；本仓库只保留参数结论与本项目自己的实测记录。
