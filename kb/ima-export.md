# balance-car-showcase 项目知识库

> 导出时间 2026-11-19T17:02:50+08:00，共 9 节

## 项目概览

两轮自平衡车，**用双步进电机（STEP/DIR）而不是直流电机**，所以控制量最终必须落到
"STEP 频率"这一层：直立 PD 的输出不是 PWM 占空比，而是频率增量，再积分成脉冲频率。

## 硬件一句话

| 部件 | 型号 | 接口 |
|---|---|---|
| 主控 | STM32F407ZG（PlatformIO `black_f407zg`） | — |
| 姿态 | BMI270 | I2C（PB8/PB9） |
| 电机 | 双步进 + MKS TMC2209（独立 STEP/DIR） | PA6/PA7、PA8/PB0 |
| 反馈 | 双路 AB 相编码器 | PA0/PA1、PB6/PB7 |
| 无线 | JDY-23 BLE | USART3（PB10/PB11） |

## 控制链（2026-11-19 起是单一串级）

```
SP 速度指令（BLE）
   → 经典速度 PI（K/I）→ 倾角修正，上限 N=0.8°
   → targetBalanceAngle（= O + 修正）
   → 直立 PD（P/D，BMI270 姿态）
   → 频率增量与限幅（R/L/S/B/H）→ 积分 → STEP 边沿（TIM2/TIM5）
```

> 11-19 之前是"多套补偿器互相抢目标角"（直投积分、TF/TB 配平、learned 自学习、kick 状态机）——
> 已全部删除，见 `fb` 提交 `faed8a7`。现在**只有这一条通路**。

## 当前参数（已 SAVE 进闪存）

`O=2.99  N=0.80  K=-0.10  I=0.00  SP=±8` —— 完整表与依据见 [参数基准](30-决策/参数基准-2026-11-19.md)。
**O 每次上电都会漂，先 `?` 看 O= 再动。**

## 怎么跑

编译、烧录、上电查 O、在线改参数并 `SAVE` 的完整步骤（含前提与验证）见
[构建、烧录与调参](20-操作/构建、烧录与调参.md)。本节只讲"为什么这样设计"，不重复步骤。

## 当前状态（读代码前先知道）

| 项 | 状态 |
|---|---|
| 前后点动 | ✅ 12/12 腿方向正确，四步回原点 ≤10mm，幅度比 39:1→1.3:1 |
| 巡航 | ❌ 被 `MAX_TARGET_FORWARD_SPEED` 编译期常量锁在静摩擦区 |
| 单腿幅度可预测性 | ❌ 离散 6 倍（2.8~20.5mm），疑静摩擦破粘随机 |
| 编码器 pos = 车体位移 | ❓ 机械紧固后未做相机对照复测 |

> 即：**"能点动"已被实车验证，"能巡航"没有**。别把前者推广成后者。


（人工补充区：脚本永不覆盖这里）

## 架构与模块边界

## 四块地，各自独立可跑

```
software/firmware/     STM32 固件（PlatformIO + Arduino framework）
   └─ src/             main.cpp（143KB，单文件大循环）+ 6 个纯逻辑头文件
software/ble-panel/    蓝牙调参面板（PyQt5 + bleak，单文件 64KB）
software/analysis/     一次性分析脚本（ble_*.py 一批扫频/脉冲实验）
software/ros/          ROS2 接入雏形（cmd/state 节点 + backends + gframe）
```

## 固件的关键分界：头文件 = 可在主机上测的纯逻辑

`main.cpp` 只做"接线 + 调度"，真正的算法被抽进 6 个 header，每个都有对应的
`software/firmware/test/run_*_regressions.py` 在主机上跑：

| 头文件 | 管什么 | 对应回归 |
|---|---|---|
| `motion_drive.h` | **只剩两个纯工具**：`MotionSpeedProfile`（速度剖面限加速）、`NetSpeedEstimator`（200ms 净速度 + 3 点中值）。原来的 `MotionLeanControl`（前馈/增益/trim/自学习）已在 11-19 删除 | `run_motion_drive_regressions.py` |
| `position_jog.h` | 位置里程、点动 | `run_jog_*`、`run_position_mode_regressions.py` |
| `manual_motion.h` | 固定相对倾角（按住 ±2°，带速率/加速度限制） | `run_motion_regressions.py` |
| `manual_turn.h` | 转向的 STEP 频率差 | `run_turn_regressions.py` |
| `param_store.h` | 参数持久化的纯逻辑（追加写、槽位、校验） | `run_param_store_regressions.py` |
| `param_flash_f4.h` | 上面那层的 F4 flash 端口实现 | 同上（主机上打桩） |

**这条边界不能破**：算法一旦写回 `main.cpp`，主机回归就失效，等于失去唯一的自动化防线。

## 数据流（谁给谁）

- 面板 → BLE → 固件：单字母参数命令（P/D/O/...）+ SAVE/LOAD + 遥测流
- 固件 → 面板：回显（`PAR`）+ 遥测（角度/速度/频率）
- 面板 → CSV：写入 `docs/validation/recordings/`
- `analysis/*.py` 读 CSV 出结论 → 落到 `docs/validation/analysis/<日期>-*/report.md`

## 不能动的地方

1. **`param_flash_f4.h` 的扇区选择**：用最后两个 128KB 扇区（10/11），
   前提是应用本体 < 768KB。改分区前先看 `pio run` 的体积。
2. **中断里不许写 flash**：擦除 1.1s、编程 0.5ms，会直接毁掉 2.5ms 的控制周期。
3. **STEP 频率域参数与细分强耦合**：改细分必须重标 L/S/B/H/R，不能沿用旧值。


（人工补充区：脚本永不覆盖这里）

## 构建、烧录与在线调参

## 前提

| 项 | 要求 |
|---|---|
| PlatformIO | 装在 `~/.platformio/penv/Scripts/pio.exe`；**`pio` 通常不在 PATH** |
| 下载器 | ST-Link 在线（`VID_0483&PID_3748`），否则 `Error: open failed` |
| 车 | 有电；BLE 地址 `E1:6F:42:A7:FF:36`（扫不到多半是没电） |
| 构建目录 | `~/.platformio/build/pinghengche`，**不是** `.pio` |

## 步骤

1. 进固件目录：

```powershell
cd <项目>\software\firmware
```

2. 编译（先用绝对路径的 pio，避免 PATH 问题）：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e black_f407zg
```

3. 需要时才烧录（用户口径：**如无必要不要烧录**，参数优先在线改）：

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -e black_f407zg -t upload
```

4. 上电后先查当前值：

```
?          → 看 O= / N= / K= / SP= / SAVED=
```

5. 在线改参数（走 BLE，不烧录）：

```
O2.99     N0.8      K-0.10     I0      SP8
SAVE      → 写进闪存
```

6. 开面板（要曲线/录制才需要）：

```powershell
.\open-panel.bat
```

## 验证

| 检查 | 通过标准 |
|---|---|
| 编译 | 末尾出现 `SUCCESS`，Flash ≈ 88908 B / RAM ≈ 8384 B |
| 烧录 | `Programming` + `Verified` 都 OK |
| 参数写入 | `?` 回显里 `SAVED=1`，且值等于刚发的 |
| 平衡点 | 上电后 `?` 看 O，必要时用 `ble_find_o.py` 二分重测 |
| 能不能动 | 用 `ble_sptest.py`，带 `--max-travel 60` 防撞边界 |

## 常见失败

- `pio: The term ... is not recognized` → PATH 里没有 pio，用上面的绝对路径。
- 烧录报 `open failed` → ST-Link 掉线，重插后重新 upload。
- 扫不到 BLE → 先确认车有电，再看面板日志。
- 改完参数没生效 → 忘了 `SAVE`，或改的是编译期常量（那必须烧录）。


（人工补充区：脚本永不覆盖这里）

## 固件主循环

## 职责划分

`main.cpp`（143KB / 约 4000 行）**只做三件事**：初始化外设、跑 2.5ms 控制节拍、解析 BLE 命令。
算法不在这里，在 6 个头文件（见 [10-架构](../10-架构.md)）。
改算法时优先改头文件——那里有主机回归兜底。

## 关键外设与调度

| 用途 | 实现 | 备注 |
|---|---|---|
| STEP 脉冲 | TIM2（前）/ TIM5（后）中断里翻 GPIO | 不再用旧 TIM3/TIM1 PWM |
| 方向 | `motor1Invert=false`、`motor2Invert=true` | 前后轮 DIR 极性相反：`main.cpp:448-450` |
| 姿态 | BMI270 over I2C，Kalman 融合 | `main.cpp:44` 注释 "Kalman：Angle + Gyro Bias" |
| 编码器 | PA0/PA1、PB6/PB7 中断计数 | 双路 + 可信通道选路 |
| BLE | USART3，单字母命令 | 面板发 `P1000` 这类 |

## 周期

- 主循环 **2.5ms**
- 速度/位置目标约 **100Hz**
- 直立输出每 **10ms** 形成一个端点，再拆 4 个子步下发

## 三条互相排斥的运动通路

| 通路 | 入口 | 说明 |
|---|---|---|
| **速度串级（主）** | BLE `SP<x>` | 设定目标速度 → 经典速度 PI（K/I）→ 倾角修正（上限 N）→ 直立 PD。`SP>0`=后退，`SP<0`=前进 |
| 手动倾角 | BLE `^` | 另一条，**与 SP 互斥**；`^` 活动时 `speedCmd` 被强制清零 |
| 转向 | `manual_turn.h` | 直接做 **STEP 频率差**，注释明确"不是标定过的偏航角速度" |
| 按住 ±2° | `manual_motion.h` | maxRate 2°/s、maxAcceleration 4°/s²，松开中途可平滑回收 |

> 11-19 删掉的机制（别再找）：直投积分 `speedDirectIntegral`、方向配平 `TF/TB`、
> 自学习 `learnedForward/Backward`、`kick` 状态机。`grep` 确认零残留。
> `motion_drive.h` 现在只剩速度剖面和净速度估计两个纯工具。

## 速度环的两个旋钮

- `K` = `speedKp`（`main.cpp` 命令解析里一直是它）；
- `SV` 是 15:20 加的同义旋钮，**后证实多余**——调速度环直接用 `K`。

## TMC2209：两条路径并存

`main.cpp` 里还留着 UART 配置的代码（`HalfDuplexSerial` 类、`configureTmcDriver`、
`findAndConfigureTmc`），但当前运行的是**独立 STEP/DIR 模式**（`main.cpp:26` 注释）。
不要因为看到 TMCStepper 依赖就以为 UART 在主链上——它只在诊断路径。


（人工补充区：脚本永不覆盖这里）

## 蓝牙调参面板

## 结构（单文件 64KB）

| 块 | 位置 | 职责 |
|---|---|---|
| `BleClient(QThread)` | `ble_tune_panel.py:138` | 扫描/连接/重连、收数据回调 |
| `Bridge(QObject)` | `:302` | 跨线程信号转发 |
| `TunePanel(QWidget)` | `:307` | 全部 UI 与参数逻辑 |
| `PARAMS` 表 | 模块级 | 每个参数一行：字母、字段名、默认值、上下限、小数位、单位、中文名 |

**表驱动**是这块的核心设计：加一个参数只改 `PARAMS`，UI 的滑块、输入框、回显、
CSV 表头全部自动跟着变。不要在 `build_ui` 里硬编码控件。

## 参数行格式

```python
("O", "balanceAngleOffset", 4.0, -10000.0, 10000.0, 2, "°", "平衡目标角")
#   ↑字母  ↑固件字段        ↑默认   ↑下限      ↑上限    ↑小数 ↑单位 ↑标签
```

> ⚠️ 上表 O 的默认值 **4.0 与固件 `main.cpp:471` 的 2.0 不一致**，见 [90-坑](../90-坑/参数与配置.md) 第 1 条。

## 主要能力

- 设参数：`send_code()` 发单字母命令；`commit_box/commit_slider` 处理两种输入控件
- 回显：`apply_param_echo()` 解析固件回的 `PAR` 行
- 曲线：`push_plot/refresh_plot`，窗口长度可切（`on_win_change`）
- 录制：`start_new_recording/record_line` → `docs/validation/recordings/*.csv`
- 持久化：`do_save/do_load/do_def` 对应固件的 SAVE/LOAD/DEF

## 测试

`test_jog_config.py`、`test_param_persist.py`、`test_turn_controls.py` 是纯逻辑测试，
不需要连车。改 UI 逻辑后先跑这三个。


（人工补充区：脚本永不覆盖这里）

## 决策：TMC2209 为什么用独立 STEP/DIR 模式

## 状态

已采纳（2026-11-19 复核，无取代者）

## 背景

TMC2209 有两种用法：

- **UART 模式**：软件设置电流、细分、斩波、StallGuard，可读回诊断；
- **独立 STEP/DIR 模式**：只吃脉冲和方向，参数由硬件（拨码/采样电阻）决定。

这台车会倒。上电失败 = 立不住，不是"功能少一点"。

## 选项

| 方案 | 优点 | 缺点 |
|---|---|---|
| A. UART 配置 | 软件可调电流/细分、能读故障 | 初始化失败整机不动；依赖 PDN_UART 接线与地址；多一条会挂的路径 |
| B. 独立 STEP/DIR | 上电就能跑，UART 断了也不影响运动 | 电流/细分改不了软件；没有 StallGuard 等诊断 |

## 决定

**选 B。** 证据：`main.cpp:26` "MKS TMC2209 V2.0 以独立 STEP/DIR 模式运行；不依赖 PDN_UART 启动"；
`docs/system-overview.md:13` 同口径。

否掉 A 的理由只有一条：**少一条能挂的初始化路径，换来确定性**。
代码里 UART 诊断路径仍保留（`HalfDuplexSerial`、`configureTmcDriver`、`findAndConfigureTmc`），
但不在主链——说明作者是"保留诊断、切断依赖"，不是不会写。

## 后果

必须接受：

- 改电流/细分要动硬件拨码，不是改参数；
- 面板上的 L/S/B/H/R 都是 STEP 频率域参数，换细分后必须重标；
- 没有 StallGuard，堵转只能靠外部现象判断。

**什么情况下该推翻**：需要软件限流保护、需要 StallGuard 做无感堵转检测、
或需要 StealthChop 静音（注意静音与力矩响应在平衡车上是冲突的）。
推翻前先确认 PDN_UART 接线、地址、以及"初始化失败时如何降级"。


（人工补充区：脚本永不覆盖这里）

## 决策：参数基准以 2026-11-19 实车测试为准

## 状态

已采纳（2026-11-19）。取代了"以文档为准"和"以固件声明为准"两条旧口径。

## 背景

同三个参数（O、N、K）在四个地方有值：文档、面板 `PARAMS`、固件声明、以及实车实际跑的值。
谁也没错——它们是不同时间点的快照：文档 10-28 定稿，固件 11-12 微调，实测 11-19 推翻了结论。

## 选项

| 方案 | 结果 |
|---|---|
| A. 以文档为准 | N=2、O=4 —— **正是导致翻车的配置** |
| B. 以固件声明为准 | N=2.0、K=−0.02 —— 11-12 的状态，11-19 已证伪 |
| C. 以 11-19 实测为准 | O=2.99、N=0.80、K=−0.10、I=0 |

## 决定

**选 C。** 依据 `docs/validation/analysis/20261119-MOTION-CONTROL.md` 与交接件
`~/.handoff/1119-1551-pinghengche-motion-closed-loop/INDEX.md`，参数已 `SAVE` 到闪存（`SAVED=1`）。

理由：只有 C 是**在这台车上跑出来的**。A、B 都是"写下来但没被实车确认"的值，
而 B 的 N=2.0 在 15:32 被测为翻车的直接原因。

## 后果

已改动：

- `main.cpp`：`balanceAngleOffset` 2.0→2.99、`speedKp` −0.02→−0.10、`MAX_SPEED_ANGLE` 2.0→0.80
- `ble_tune_panel.py` 的 `PARAMS`：O 4.0→2.99、K −0.02→−0.10、N 2.0→0.8
- `docs/parameters.md`：重写为"今日基准 / 编译期默认 / 待定项"三层
- 编译验证：`pio run -e black_f407zg` → SUCCESS，Flash 88908 B / RAM 8384 B（与改前一致）

要接受的代价：

- **N=0.8 是稳定边界，不是保守值**。N=2.0 时速度误差一大，外环反向往死里拽 2°，直接翻。
- X / Y / M（位置环）今日未覆盖，仍是代码与文档不一致，留在 `docs/parameters.md` 的"待定"表。

**什么情况下该推翻**：换电机/细分/载板（L/S/B/H/R 连同 N 一起重标）；
机械结构改动（O 必须重新二分——它每次上电都会漂）；要巡航而非点动（先解决 `MAX_TARGET_FORWARD_SPEED` 这个编译期瓶颈）。


（人工补充区：脚本永不覆盖这里）

## 坑：参数与配置

## 1. 参数基线三方漂移

**症状**：照文档填参数，车往单侧倒，怎么调都调不平。

**根因**：同一个参数在文档、面板 `PARAMS`、固件声明里是三个时代的值（文档 10-28、固件 11-12、实测 11-19）。

| 参数 | 今日基准 | 固件旧默认 | 面板旧默认 | 旧文档 |
|---|---:|---:|---:|---:|
| O | 2.99 | 2.0 | 4.0 | 4 |
| N | 0.80 | 2.0 | 2.0 | 2 |
| K | −0.10 | −0.02 | −0.02 | −0.02 |

**处置**：以 `docs/validation/analysis/20261119-MOTION-CONTROL.md` 实测值为准，已同步到 `main.cpp` 三处默认值与面板 `PARAMS`，编译通过。

**验证**：`?` 回显 `O=2.99 N=0.8 K=-0.10 I=0 SAVED=1`；依据见 [参数基准](../30-决策/参数基准-2026-11-19.md)。

**不回归**：`docs/parameters.md` 保留"三方对照"表；改任一处必须同时改表。

## 2. 源码 ≠ 实车

**症状**：改了常量、编译通过，以为车上就是新值。

**根因**：源码、闪存 `SAVED`、已烧录固件是三层状态。`MAX_TARGET_FORWARD_SPEED` 源码已是 30，实车仍跑 8。

**处置**：查实车值一律用 `?` 回显，不读源码下结论。

**验证**：`?` 里的 `SP` 上限与实际点动幅度一致。

**不回归**：`docs/parameters.md` 固定保留"源码 vs 实车"一节。

## 3. 烧录重启后 O 掉回闪存值

**症状**：刚调好的平衡点，重新烧录后车又漂。

**根因**：上电从 flash 的 `SAVED` 记录恢复 O，而不是上次在线改的内存值。

**处置**：每次上电先 `?` 看 `O=`，用 `ble_find_o.py` 重测二分。

**验证**：重测后漂移 < 1 mm/s（11-19 实测 −0.11 mm/s）。

**不回归**：已写进 [构建、烧录与调参](../20-操作/构建、烧录与调参.md) 步骤 4。


（人工补充区：脚本永不覆盖这里）

## 坑：硬件与运动

## 4. 编码器 2 方向是反的

**症状**：两路编码器读数符号相反，位置环一开就发散。

**根因**：V10 实机线序所致，`encoder2Direction = -1`（`main.cpp`）。

**处置**：保持 `-1`，不要"顺手改成正的"。

**验证**：架空推车，两路计数同号。

**不回归**：换编码器或换线序后必须重测符号。

## 5. 50ms 速度窗口的"振荡"是极限环

**症状**：50ms 窗口内速度摆 ±29 counts，66% 相邻样本换向，像噪声。

**根因**：位置环极限环的真实摆动，Kalman 抹不掉（`motion_drive.h` 头注释）。

**处置**：运动环改用 200ms 净速度 + 3 点中值，而不是加大滤波。

**验证**：同一工况下相邻样本换向比例明显下降。

**不回归**：别再往速度窗上加低通。

## 6. flash 擦除不能进控制环

**症状**：保存参数时车抖一下甚至失控。

**根因**：128KB 扇区擦除典型 1.1s、最长 2s+，而控制环 2.5ms。

**处置**：扇区内追加写（一条 104B），写满才换扇区，擦除前要求车静止。

**验证**：`SAVE` 时车不动、控制周期不丢拍。

**不回归**：任何"在中断里写 flash"的想法直接否掉。

## 7. 点动能跑 ≠ 能巡航

**症状**：拿点动结论解释巡航行为，结论全错。

**根因**：11-19 只验证了点动前后（12/12 腿、回原点 ≤10mm、幅度比 1.3:1）；巡航被 `MAX_TARGET_FORWARD_SPEED` 锁死，未验证。

**处置**：说"这版能跑"必须限定范围。

**验证**：巡航要等改常量并烧录后，用 `SP±20` 测稳态速度。

**不回归**：`kb/00-概览.md` 的状态表按"已验证/未验证"分列。


（人工补充区：脚本永不覆盖这里）
