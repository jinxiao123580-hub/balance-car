# 01 · 代码介绍：这堆代码在干嘛，该从哪读起

> 范围：`software/firmware/src`（`main.cpp` 5838 行 + 6 个头文件）。
> 前置：`00-STRUCTURE.md`（分层图、函数索引）、`P1-setup-loop.md`（setup/loop 细讲）。
> 本文只建立全局印象：一句话定位、文件分工、主流程、四个环、常用单位与缩写。
> 逐行分析、参数总表、问题清单留给后续章节，这里不做。

---

## 1. 这是什么固件

一句话：**一台双轮自平衡车的固件**——BMI270 测俯仰角，两路编码器测轮速/位移，
直立 PD 把"要倒"变成目标 STEP 频率，速度外环再微调"该往哪边倾"，
最后经 TIM2/TIM5 定时器边沿给两个步进驱动器发 STEP/DIR 脉冲。

运行平台：

- MCU：STM32F407ZGT6；PlatformIO 环境 `black_f407zg`，Arduino framework（ststm32 19.7.0）。
- 传感器：BMI270（I2C，PB8/PB9，400kHz）；编码器×2（PA0/PA1、PB6/PB7，A 相边沿中断）。
- 执行器：步进电机×2，STEP/DIR 接 PA6/PA7 与 PA8/PB0；脉冲由硬件定时器中断产生，高电平 4µs。
- 通信：JDY-23 BLE 模块走 USART3（PB10/PB11），固定 9600 波特——全系统带宽瓶颈。
- 参数存储：内部 flash sector 10/11（0x080C0000 / 0x080E0000，各 128KB）。

节拍：`loop()` 不是"跑到哪算哪"，而是一个**固定 400Hz（2.5ms）的软件节拍器**；
外环 100Hz、快帧 4Hz、慢帧 1Hz。一拍里所有控制必须在 2.5ms 内跑完
（`controlMaxWorkUs` 在统计这件事），连续掉 2 拍就直接停电机。

当前这版的实际状态：直立 + 速度 + 位置三环在跑，转向环保留结构但增益为 0，
手动差速转向（`V` / `^` 命令）才是实际转向通路；OLED 完全不参与运行。

---

## 2. 代码怎么组织

`main.cpp` 是**唯一的 .cpp**，5838 行里塞了调度、自检、姿态、速度估计、串级控制、
BLE 协议、参数持久化全部逻辑；6 个头文件都是 **header-only 的纯结构体/类**，
没有自己的 .cpp、不起线程、不注册中断，只等 `main.cpp` 里的函数来驱动。

| 文件 | 管什么 | 谁在用它 |
|---|---|---|
| `motion_drive.h` | 前后运动四件套：速度目标斜坡、200ms 净速度、预测位置计划、速度误差→倾角（EG） | `updateOuterLoop()` 的 `^` 手动通路 |
| `manual_motion.h` | `^` 手动俯仰：固定相对平衡角、±2°、smoothstep 起停、350ms 超时释放 | `loop()`、`handleBtCommand()` |
| `manual_turn.h` | 差速转向：输入是 STEP 频率差（±300Hz）、350ms 超时、随倾角收权限；附带全局 `clamp()` | `updateTurnLoop()`、所有头文件 |
| `position_jog.h` | 编码器里程（含 32 位回绕）、位置 P→小倾角、目标速度斜坡、位置点动状态机 | `updatePositionHold()`、`updateOuterLoop()`、BLE `~`/`&` |
| `param_store.h` | 22 项参数的记录格式（104B/条）、CRC32、双扇区扫描与追加写 | `paramStorageBoot()`、`SAVE`/`LOAD` |
| `param_flash_f4.h` | 上者的 F4 落地：sector 10/11 擦写、按字编程、关中断保护 | `paramPort` 全局对象 |

**谁调用谁**（这是读代码时最该先记住的一张关系网）：

- `setup()` 一次性装配：电机/编码器 → BLE → TMC → 参数读回 → BMI270 三步握手 → 清状态。
- `loop()` 是节拍器：每 2.5ms 依次调 `updateAttitude()`、`predictVehicleSpeedKalman()`、
  遥测帧、倒地保护、`manualForward.update()`、`updatePositionHold()`、
  （每 4 拍）`updateOuterLoop()`、`updateOuterLoopAuthority()`、
  `updateBalanceLoop()`、`updateTurnLoop()`，最后合成两轮命令。
- `updateOuterLoop()` 是唯一集中调用 `motion_drive.h` / `position_jog.h` 那些零件的地方：
  编码器速度、Kalman 修正、位置/速度目标、速度 PI 全在这一个函数里按顺序走完。
- 头文件之间是单向依赖：`motion_drive.h`、`manual_motion.h`、`position_jog.h`
  都 `#include "manual_turn.h"`（为了共用 `clamp()`），`param_flash_f4.h` include
  `param_store.h`；没有循环依赖，也没有跨头文件的互相调用。

为什么这么拆：这些逻辑要能在 PC 上做主机回归（参数记录用 RAM 假 flash、斜坡/点动用纯
整数/浮点状态机），所以刻意写成不依赖 Arduino 外设、只依赖 `<cmath>/<cstdint>` 的头文件。
真正碰硬件的（I2C、定时器、flash 擦写）都留在 `main.cpp` 和 `param_flash_f4.h` 里。

---

## 3. 主流程怎么跑

### 3.1 `setup()`：一次性把"手脚、感觉、参数"准备到位

1. `initMotors()` + `initEncoders()`：先把 STEP/DIR/EN 置成"停"的安全态，再挂编码器中断。
2. `BTSerial.begin(9600)` + `delay(100)`：日志通道必须先通，否则后面死循环重试没法诊断。
3. TMC 配置：当前编译开关 `TMC_STANDALONE_STEP_DIR=1`，**不访问 UART**，
   细分/电流/模式由底板硬件决定，只打印一行说明；另两种 UART 配置分支还留在 `#if` 里。
4. `paramStorageBoot()`：先抄一份编译默认值，再用 flash 里 seq 最大且 CRC 正确的记录覆盖；
   若当前扇区写满，趁开机把另一块先擦好（约 1.1s，绝不能发生在控制环运行中）。
5. BMI270 三次握手，每步失败都 `stopBothMotors()` + 1s 后重试：
   能读到 chip id → 真的被配成 400Hz（setClock 提到 400kHz）→ 静止校准（提示 keep car still）。
6. 关中断快照编码器计数，设当前位置为零点，`vehiclePosition = 0`。
7. 清速度 Kalman 与控制状态；`targetBalanceAngle = balanceAngleOffset`（上电目标角就是 O），
   `outerLoopAuthority = 1`（上电外环给满权限）。
8. 最后一句 `lastControlUs = micros()`——**必须放最后**，否则 setup 里的 `delay()`/重试
   会被 loop 当成一次巨大掉拍，第一拍就触发停机保护。

### 3.2 `loop()`：每 2.5ms 的一拍

每圈进来先做两件**不受节拍限制**的事：`pollBtCommand()`（蓝牙命令随时可能改变量）、
`flushTelemetryTx()`（9600 波特下发送是碎步走的）。然后取 `nowUs`，不到 2.5ms 直接 return；
到了才跑一拍，顺序如下：

1. **掉拍检查**：距上拍 ≥5ms 判掉拍——停电机、清诊断、时间基准重设；正常则
   `lastControlUs += 2500us`（固定步长累加，不让 400Hz 随调度抖动漂移）。
2. **读 IMU**：`readBMI270()` 失败 <8 拍先保持上一拍输出直接 return；≥8 拍（约 20ms）
   判真故障，停电机并清掉会累积的积分/微分/位置目标。
3. **姿态**：`updateAttitude()` → 加速度角 + 陀螺角速度经 Kalman（动态 R）融合出
   `pitch`、`pitchRate`，并顺带算出 IMU 前向加速度。
4. **速度预测**：`predictVehicleSpeedKalman()` 用 IMU 前向加速度做 400Hz 速度预测。
5. **遥测分频**：400 拍发 `D` 帧、100 拍发 `G` 帧——特意放在倒地保护**之前**，
   这样就算倒了，手机上也还能继续看到姿态数据。
6. **倒地保护**：`|pitch - O| ≥ 75°` 连续 8 拍 → 停电机、清全部控制量与位置原点、
   目标角回 O、外环权限归零，然后 return。
7. **手动与位置**：`manualForward.update()`（`^` 俯仰）→ `updatePositionHold()`
   （400Hz 刷新里程和目标位置，位置 P 本身在 100Hz 外环里算）。
8. **100Hz 外环**：`outerLoopDivider` 每 4 拍触发一次 `updateOuterLoop()`——
   编码器增量→速度→Kalman 修正→（手动/位置）目标速度→速度 PI→`speedAngleCorrection`。
9. **权限**：`updateOuterLoopAuthority()` 按 `|pitch - O|` 在 5°~10° 之间把外环权限
   从 1 平滑压到 0，并算出本拍真正用的 `targetBalanceAngle`。
10. **直立环**：`updateBalanceLoop()`（400Hz）算 PD，再按 100Hz 端点增量累加成 STEP 频率。
11. **转向环**：`updateTurnLoop()` 算 `turnOutput`（当前 Kp/Kd=0，主要靠手动通路）。
12. **电机合成**：`motor1 = finalBalanceOutput + turnOutput`、
    `motor2 = finalBalanceOutput - turnOutput` → `setMotor1/2` → `commandStepOutput(Hz)`
    → 定时器 ISR 发脉冲；`|Hz|<1` 视为停。
13. **收尾**：累积 250ms 遥测窗口极值、`flushTelemetryTx()`、记录单拍最大耗时。

### 3.3 频率一览

| 频率 | 触发方式 | 内容 |
|---:|---|---|
| 400 Hz（2.5ms） | 每拍 | 读 IMU、姿态、速度预测、权限、直立 PD、转向、电机输出 |
| 100 Hz（10ms） | `outerLoopDivider >= 4` | 编码器速度修正、位置/速度目标、速度 PI、PD 频率端点 |
| 4 Hz | `fastTelemetryDivider >= 100` | `G,` 快帧（8 个数值字段） |
| 1 Hz | `telemetryDivider >= 400` | `D,` 慢帧（74 个数值字段） |
| 事件 | BLE 一行命令 | 改参数、`SP/MS/EG/SV`、`SAVE/LOAD/DEF`、`^`、`~`、`?`、`!` |

> 读注释时注意：源码里偶尔还留着"20Hz / 50ms 窗口"的说法，那是早期节拍的历史值；
> 现在实际是 400Hz 主拍、外环 100Hz，单位名 `counts/50ms` 只是沿用下来的习惯。

---

## 4. 四个控制环

### 4.1 直立 PD（400Hz，最内层）

输入 `angleError = targetBalanceAngle - pitch`，P 项纠偏、D 项用陀螺角速度做阻尼。
关键点：**PD 输出不是直接当 STEP 频率**，而是先在 100Hz 端点换算成一个"频率增量"，
再在随后的 4 个 400Hz 周期里分 4 步执行、累加（积分）成 `integratedMotorSpeedCommand`，
最后受总限幅 `L` 和每个 100Hz 端点的增量限幅 `S` 约束。换向瞬间还有 250ms 的软窗口把增量再压一档，
防止转子在扭矩零点附近丢步。所以"直立环"在这份代码里实际是
"PD → 加速度式频率积分器"，而不是教科书里直接输出占空比。

### 4.2 速度外环（100Hz）

`targetForwardSpeed` 减去 Kalman 融合速度得到速度误差，纯 P（`K`，积分 `I` 默认 0）算出
`speedAngleCorrection`，叠加到目标角上：想往前就多倾一点，想停就反过来拽。
修正角被 `N=0.8°` 硬限幅、每次更新最多变 0.1°（约 10°/s），还要乘外环权限——
倾角一大（5°→10°）权限就从 1 掉到 0，把控制权全交给直立环。
目标速度有三个来源：`SP` 指令、位置环、`^` 手动通路，三者互斥。

### 4.3 位置环（可选，夹在速度环外侧）

位置误差 = 目标位置 − 当前编码器里程，经 `X` 增益变成"受限目标速度"（上限 `Y`，
死区 `M`），再交给速度外环，**不直接抢 STEP 输出**。`X=0` 即整条路关闭；
本版默认 `X=0.02` 是打开的，但目标速度上限很小，本质是"驻车抗漂移"。
`^` 手动运动期间目标位置跟随实际位置，避免回位环把车拽回旧起点。

### 4.4 转向环（400Hz）

`targetTurnRate - gyroTurnRate` 走保留的 PD（`T`/`U`，当前为 0），实际转向量来自
`V <Hz>` / `^ turn,deg` 的手动差速通路：输入是左右轮 STEP 频率差，±300Hz、按 300Hz/s 斜坡，
350ms 没新命令就自动归零；倾角越大权限越小，车身歪到 10° 直接归零。
最终以 `motor1 = 平衡输出 + 转向`、`motor2 = 平衡输出 − 转向` 的形式差速。

---

## 5. 关键概念与单位

### 5.1 counts 与 counts/50ms

- **count**：编码器计数。A 相每次边沿 ±1（B 相判方向），32 位有符号、在 ISR 里累加，
  所以是"边沿数"，不是毫米也不是圈。
- **counts/50ms**：这份代码里所有"速度"的单位。名字来自早期 50ms 采样窗口；
  现在在 100Hz 外环里用"10ms 内的 count 增量 × 5"等效得到。
  换算关系：**1 counts/50ms = 20 counts/s**。
- 速度 Kalman、`SP`/`MS`、位置环输出上限、手动通路参考速度，全部统一用这个单位；
  直立环那边才是 Hz，两者之间靠 `WHEEL_TO_HZ_SCALE` 之类的标定量衔接。

### 5.2 counts/mm

固件内部**只认 count，不做毫米换算**；`10.8 counts/mm` 是外部（相机对表）标出来的比例，
上位机/分析脚本把遥测里的 `pos/100` 再除以 10.8 得到毫米。
按这个比例：1 counts/50ms ≈ 1.85 mm/s，`SP=8` ≈ 15 mm/s、约 100 counts/s。
位置死区 `M=12 counts` 约合 1.1mm——调参时心里要有这个尺子。

### 5.3 字母参数与两字母指令

BLE 命令分两层，这是最容易混的地方：

- **单字母**：22 项面板参数（`P D O Z L S R B H K I N W J E X Y M F G T U`），
  改完可以用 `SAVE` 写 flash，`?` 回显全部当前值，`LOAD`/`DEF` 分别读回/恢复默认。
- **两字母**：只改 RAM、不落 flash 的运行时指令，避开单字母命名空间。

必须记住的几个：

- `O`：`balanceAngleOffset`，机械平衡角/目标俯仰角。车"立住"的目标不是 0°，而是 O
  （默认 2.99°），所有"大角度"判断都是相对 O 算的。它随装配和上电状态变化，换车/重装要重测。
- `N`：速度修正角上限（默认 0.8°）。它是稳定边界而不是"越大越有力"——这是实测结论。
- `K` / `I`：速度环 P / I。当前 `K=-0.10`（负号是正确方向），`I=0`（积分停用）。
- `SP <v>`：目标速度，单位 counts/50ms，是**唯一的速度设定入口**，钳在 `±MS` 内；
  与 `^` 手动通路互斥（`^` 动作时 SP 被清零）。
- `MS <v>`：目标速度上限（0.5~60，默认 30），只改 RAM。
- `EG <v>`：`MotionLeanControl::errorGain`（0~0.15），只影响 `^` 手动前后通路的
  "速度误差→倾角"增益。
- `SV <v>`：写的就是 `speedKp`——**和 `K` 是同一个变量**，改 `SV` 等于改 `K`。

### 5.4 D 帧与 G 帧

两帧都是 ASCII 一行、逗号分隔、数值用定点整数（×100 / ×1000）避免浮点打印：

- **`D,` 慢帧（1Hz，74 个数值字段）**：millis + 完整控制链（俯仰、目标角、P/D 项、
  外环权限、速度/位置误差、融合速度、编码器计数、电机 Hz、窗口极值、点动状态、诊断量）。
  它是对外最大的"契约"，字段位置和顺序被面板/CSV 脚本依赖，动之前先看 `sendTelemetry()` 的注释。
- **`G,` 快帧（4Hz，8 个数值字段）**：`millis, pos, speed, pitch, target, output, manual, btRx`，
  用于手机端实时画曲线和确认命令是否真的到达车端（`btRx` 是接收字节计数）。

两帧和命令回显共用同一条 9600 波特 TX：总预算约 800B/s，`D` 帧一行就几百字节，
所以遥测和回显都先**整行进环形缓冲**，再按预算分片发出；每 2.5ms 只给约 2 字节预算，
缓冲不够就整行丢弃，绝不阻塞控制环。

---

## 6. 从哪读起（给接手的同事）

1. 先看 `00-STRUCTURE.md` 的分层图和 `P1-setup-loop.md` 的时序表，把"一拍"记住。
2. 按函数名（不是行号）跳：`setup()` → `loop()` → `updateOuterLoop()` →
   `updateBalanceLoop()` → `updateAttitude()`。行号会漂，函数名是稳定锚点。
3. 想理解"为什么车能动"：先 P6（外环串级）再 P7（直立/转向），最后回头看 P5（速度估计）。
4. 想理解"为什么电机这么响/这么抖"：P3（步进输出与中断）和 P7 的限幅/斜坡最相关。
5. 想调参或接面板：P8（遥测与 BLE 协议）+ P9（参数持久化 + 6 个头文件）。
6. 读 `main.cpp` 时按逻辑块读：大量语句被拆成 `if (` / 空行 / 条件 / 空行 / `)` 的形式，
   同一逻辑块可能横跨几十个物理行，别被空行带跑。
