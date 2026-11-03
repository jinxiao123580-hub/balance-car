# 驱动器：MKS TMC2209（Makerbase）

## 型号与来源

- **MKS TMC2209** 步进驱动模块（Makerbase），商品页截图：`docs/references/vendor/MKS-TMC2209/商品页详情（图片里有github库）.png`。
- 官方仓库：<https://github.com/makerbase-mks/MKS-TMC2209>（卖家页给的是 `makerbase-mks/MKS-TMC2209`）。
- 本机另有 MKS TMC2209 **V2.0** 原理图与标注图：`docs/references/vendor/MKS_TMC2209_V2.0_SCH.pdf`、`mks2209_docs/`（工作区侧）。

## 关键参数（卖家页口径）

| 项目 | 值 |
| --- | --- |
| 默认电流 | 1.25 A |
| 最大电流 | 2.5 A（峰值）；稳定打印电流最高 1.8 A |
| 逻辑电压 | 3.3 V / 5 V |
| 输入电压 | 5.5–28 V |
| 细分 | 最高 1/256（芯片内部 16 拓展 256） |
| 模式 | STEP/DIR、UART |
| 跳线细分 | M0/M1：GND GND=8；VCC GND=32；GND VCC=64；VCC VCC=16（卖家页表，顺序与 BTT 板不同） |

> 以上是**模块上限**，不是本项目设定值。本项目在 MKS 独立模式下用板载电位器定电流，实际值未记录。

## 接线与引脚（本项目）

- 后轮：STEP=PA8、DIR=PB0、EN=GND；前轮：STEP=PA6、DIR=PA7。
- 独立模式（`TMC_STANDALONE_STEP_DIR=1`），**不接 PDN_UART**；`motor2Invert=true`（MKS 后轮与前轮方向相反）。
- 若要改 UART：PDN_UART 接 PA9（后轮）/ PA2（前轮），115200 半双工；固件里已有 write-only 分支可切回。

## 斩波模式

- MKS V2.0 板上 **R9 双焊位 0 Ω**：接 +5V = SpreadCycle；拆掉 = StealthChop（SPREAD 悬空靠内部下拉）。
- 2026-10-10 用户万用表实测：本板 R9 已在 +5V 位，**出厂即 SpreadCycle**，无需改焊。
- 排针上的 INDEX 是输出脚，不是 SPREAD；R5 是外部 CLK 选择，与模式无关。

## 已知问题

- 卖家页的 M0/M1 细分表与 BTT 板顺序不同，照抄会得到错误细分——以板上丝印为准。
- MKS 原理图把 SPREAD 标成第 7 脚，芯片手册是第 6 脚（符号整组错位），别按图数引脚。

## 待验证

- 独立模式下实际相电流（VREF 实测值）未记录。
- 两块 MKS 是否同批次、前轮是否也是 MKS，未确认。
