# 别人的步进/两轮自平衡方案 vs 我们（2026-11-20）

> 目的：回答"其他人做的和我们的差在哪里"，并指出我们"只能点动不能巡航"的结构性差距。
> 证据级别：`[一手代码]` > `[教程/文档]` > `[推断]`。外部链接见文末。

---

## 1. 别人普遍怎么做（共性）

| 环节 | 通行做法 | 出处 |
|---|---|---|
| 结构 | **内环直立 PD + 外环速度 PI**，速度环输出 = **角度目标**（串级） | [自平衡小车串级PID（miaowlabs）](https://c.miaowlabs.com/A51.html)、[ROCKO 调参文档](https://rose-bic.atlassian.net/wiki/spaces/R/pages/420348043)、[ProtoTwin 教程](https://prototwin.com/docs/tutorials/self-balancing-robot) |
| 速度环 | **PI**（明确要积分的理由：消稳态误差） | miaowlabs：「因为编码器噪声…速度控制使用 PI 控制」；ROCKO：速度环 I 可到 P 的 **80%** |
| 死区补偿 | **电机输出加固定死区**，抗静摩擦 | miaowlabs 代码：`if(out>0) out += MOTOR_OUT_DEAD_VAL; else if(out<0) out -= MOTOR_OUT_DEAD_VAL;` |
| 符号 | 速度环对直立环是"负反馈变正反馈"的陷阱，极性必须靠实验定 | miaowlabs 专门用"屏蔽直立环、只留速度环"实验验证 |
| 步进驱动 | **每步之后重算下一步延迟**（定时中断 40kHz、尽量整数运算），而不是固定频率定时器 | [DFRobot：用步进电机做自平衡机器人](https://mc.dfrobot.com.cn/article-375358.html)（作者：zjor/balancing-robot） |
| 步进驱动（进阶） | 用 **TMC2209 UART 速度模式**每轮写一次速度，省掉 STEP 翻转 | [CuriousScientist TMC2209 PID](https://curiousscientist.tech/blog/stepper-motor-developing-platform-pid)、[janelia-arduino/TMC2209](https://github.com/janelia-arduino/tmc2209) |
| 速度估计 | 轮速 + 车身倾角速度一起进速度估计 | [Bulduper/BTW-Robot](https://github.com/Bulduper/BTW-Robot)（步进电机 + MPU6050）：「absolute motor speed and relative head's speed」 |
| 方法论 | **先建模/仿真再上实车** | DFRobot 文：作者给出 Kaggle 仿真链接并建议先仿真 |

---

## 2. 我们 vs 他们（差异表）

| 项 | 别人 | 我们（当前固件） | 影响 |
|---|---|---|---|
| 速度环 | **PI**（带积分） | **纯 P**，`I=0` | 无积分 ⇒ **无法维持巡航所需的稳态倾角**；零误差时输出为 0 |
| 目标角权限 | 限幅通常放在**电机输出**，角度目标可到几度~十几度 | **`N=0.8°` 硬限幅**在"目标角修正"上 | 速度环输出被削平 ⇒ `|err|≥8 count/50ms` 就饱和（≈14.8mm/s），**整个工作区是 bang-bang** |
| 死区补偿 | 电机输出加 `MOTOR_OUT_DEAD_VAL` | **没有**（只有 PID 误差项） | 静摩擦 0.5~0.7° 门槛必须靠"误差攒出来"，小指令必然不动 |
| 前馈 | 部分实现有速度前馈；标配是 PI 的 I 承担 | SP 通路**无前馈**（旧手动通路里有 `feedForwardGain=0.06`） | SP=8 需要 0.48° 才够破粘，我们只能靠误差 |
| 步进输出 | 每步重算延迟（40kHz 中断） | 固定频率定时器，**400Hz 更新频率命令** | 频率更新带宽低，但 400Hz 对平衡够用（次要） |
| 速度估计 | 轮速 + 倾角速度 | 编码器 + IMU 前向加速度 Kalman（等价思路） | 差异不大 |
| 调试方法 | 先仿真再实车 | 纯实车试凑 | 反复"翻车/极限环"，参数之间互相牵制 |

---

## 3. 关键结论：我们差的不是"算法玄学"，是三个工程细节

1. **`N=0.8°` 不是安全限幅，而是把速度环压瘫了**
   别人限的是电机输出或较大的角度目标；我们把"速度修正角"限到 0.8°，于是速度环只有 8 count/50ms 的线性区，
   而我们的指令本身就是 8~30 count/50ms ⇒ **一给指令就饱和**，等效开关控制（bang-bang）。这直接解释了：
   - 稳态速度≈0（饱和来回推，净位移小）
   - SP≥16 原地抖（bang-bang 极限环）
   - SP=20 反向前冲（饱和 + 粘滑 + 符号非线性）

2. **没有积分 ⇒ 物理上没有巡航平衡点**
   匀速前进需要**持续**的少量前倾（克服摩擦）；P 只在有误差时给倾角，误差→0 倾角→0 ⇒ 车停下。
   别人用 PI，靠 I 维持这个恒定倾角。我们实测"加 I 变极限环、稳态反向"——
   但那次实验是在 **`|err|≥8 已饱和** 的条件下做的，I 被饱和吞掉，结论未必成立（[推断]，需在非饱和区重测）。

3. **没有死区补偿 ⇒ 小指令根本不动**
   步进电机 + 静摩擦，输出低于门槛就是不动；别人在电机输出上加固定死区偏置。
   我们有"破粘 0.5~0.7°"的实测，却没有对应的输出偏置。

> 附带一个被推翻的既有结论：`N=2.0 反向猛拽翻车` 是在 `K=-0.12`（一给就饱和）下测的，
> **N 与 K 必须成对选**，"N 不能大"这个结论不成立于小 K 的情形（[推断]，需实验）。

---

## 4. 别人的做法里可直接抄的三条

1. **速度环改 PI**：`speedAngleUnclamped = dir*(K*err + I*∫err + ...)`，I 的初值从**很小的值**起（例如 P 的 10~20%，ROCKO 说最多到 80%），
   带条件积分 + 抗饱和（我们代码里本来就有这套结构，只是 `speedKi` 默认 0）。
2. **输出死区补偿**：在 STEP Hz 合成后再加 `±DEAD_HZ`（初值取"能带动但明显不抖"的频率，实测逐步加），
   位置放在 `control::mixMotors()` 之后、`setMotor*` 之前。
3. **把 N 从"工作限幅"改回"安全限幅"**：先按 `K·err ≤ 0.8°` 选 K（例如期望满速 30 count/50ms 时仍线性 ⇒ `|K|≤0.027`），
   再把 N 放宽到 1.5~2°，用"安全"角色而不是"工作"角色。

---

## 5. 外部出处

- 串级 PID + 速度环 PI + 电机死区补偿（中文，代码级）：https://c.miaowlabs.com/A51.html
- ROCKO 速度/位置 PID 调参（速度环 I 可达 P 的 80%）：https://rose-bic.atlassian.net/wiki/spaces/R/pages/420348043
- ProtoTwin 自平衡教程（外环位置 → 目标角，内环角速度）：https://prototwin.com/docs/tutorials/self-balancing-robot
- 步进电机版自平衡（DFRobot 转载，A4988/NEMA17，40kHz 步进中断 + 每步重算）：https://mc.dfrobot.com.cn/article-375358.html
- 步进版开源工程（轮速 + 倾角速度估计）：https://github.com/Bulduper/BTW-Robot
- TMC2209 UART 速度模式 + 磁编码器 PID：https://curiousscientist.tech/blog/stepper-motor-developing-platform-pid
- TMC2209 速度模式公式（microsteps/s = v·fclk/2^24）：https://github.com/janelia-arduino/tmc2209

> 注：以上"别人"多为直流电机平衡车；**步进电机版（DFRobot/BTW-Robot）的结构与我们最接近**，同样用串级 PD+速度环，
> 差异主要在"速度环带积分 + 电机死区补偿 + 步进时序实现"。
