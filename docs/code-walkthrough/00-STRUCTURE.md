# 平衡车固件 · 代码结构总览（00）

> 范围：`software/firmware/`（PlatformIO / STM32F407ZG / Arduino framework）
> 目标：先建立"它在跑什么、按什么顺序跑、数据从哪来到哪去"的地图，再按章节逐行展开。
> 生成时间：2026-11-20

---

## 1. 先说源码的物理状况（影响"逐行"怎么讲）

| 文件 | 物理行 | 说明 |
|---|---:|---|
| `src/main.cpp` | 5838 | 非空 3283（代码 ≈2524 + 注释 759） |
| `src/motion_drive.h` | 142 | 运动驱动四件套（速度斜坡/净速度/位置计划/倾角控制） |
| `src/position_jog.h` | 126 | 位置点动状态机 |
| `src/param_store.h` | 184 | 22 项参数记录格式 + 双扇区扫描 |
| `src/manual_motion.h` | 82 | `^` 手动俯仰通路（三次 smoothstep 规划） |
| `src/param_flash_f4.h` | 66 | STM32F4 sector 10/11 读写端口 |
| `src/manual_turn.h` | 44 | 转向手动通路 + `clamp()` 工具 |

**重要**：`main.cpp` 有 **387 处连续空行块（最长 7 行）**，很多语句被拆成
`if (` / 空行 / 条件 / 空行 / `)` 这种形式。所以：

- **不要按物理行号理解代码**；本文档里的行号只用于定位，讲解按"逻辑语句块"。
- 后续每章会给出 `L起-L止` 的物理行范围 + 该块的逻辑说明。

---

## 2. 分层结构

```
┌──────────────────────────────────────────────────────────────────────┐
│ L0 调度            loop() 400Hz 主节拍          setup() 一次性自检     │
├──────────────────────────────────────────────────────────────────────┤
│ L1 接口/契约       BLE 命令解析(pollBtCommand)   遥测帧(D 1Hz / G 4Hz) │
│                   参数持久化(param_store+flash)  面板/PAR 回显         │
├──────────────────────────────────────────────────────────────────────┤
│ L2 控制            外环 updateOuterLoop 100Hz                         │
│                    ├ 位置环 updatePositionHold（X=0 时禁用）           │
│                    ├ 速度 PI → speedAngleCorrection                    │
│                    └ 运动驱动 MotionLeanControl.errorGain(EG)          │
│                    权限 updateOuterLoopAuthority 400Hz                 │
│                    直立 updateBalanceLoop 400Hz（PD → finalBalanceOutput）│
│                    转向 updateTurnLoop 400Hz                           │
├──────────────────────────────────────────────────────────────────────┤
│ L3 估计            姿态 updateAttitude（Kalman/互补 → pitch）          │
│                    速度 predictVehicleSpeedKalman 400Hz                │
│                         correctVehicleSpeedKalman 100Hz（编码器修正）  │
├──────────────────────────────────────────────────────────────────────┤
│ L4 驱动/中断       stepEdge ISR(STEP 计数)  commandStepOutput(频率)     │
│                    编码器 ISR ×2            BMI270 读写/配置            │
│                    TMC2209 UART 配置                                   │
├──────────────────────────────────────────────────────────────────────┤
│ L5 硬件            BMI270 IMU  编码器×2  TMC2209×2  步进电机×2         │
│                    JDY-23 BLE  flash sector10/11                       │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 3. 运行时调度（`loop()` 的真实顺序）

```c
loop()  每圈：
  pollBtCommand()            // 蓝牙命令：随时可能改变量/参数
  flushTelemetryTx()         // 非阻塞发送缓冲
  if (now - lastControlUs < 2500us) return;        // 未到 400Hz 直接返回
  controlGapUs 统计 / 超时(>=2拍)→ stopBothMotors() 并重基准
  readBMI270()               // 失败累计，达阈值→停机清环返回
  updateAttitude()           // Kalman + 互补 → pitch/pitchRate
  predictVehicleSpeedKalman()// IMU 前向加速度 → 速度预测量
  telemetryDivider++  ; ==400 → sendTelemetry()     // 1Hz 慢帧 D
  fastTelemetryDivider++ ; ==100 → sendFastTelemetry() // 4Hz 快帧 G
  倒地保护：|pitch - O| 连续确认 / 紧急阈值 → 停机清全部环 并 return
  manualForward.update(...)  // ^ 手动俯仰通路
  updatePositionHold()       // 位置环（X=0 时内部早退）
  outerLoopDivider++ ; ==4 → updateOuterLoop()      // 100Hz
  updateOuterLoopAuthority() // 400Hz
  updateBalanceLoop()        // 直立 PD 400Hz
  updateTurnLoop()           // 转向 400Hz
  合成：motor1 = finalBalanceOutput + turnOutput
        motor2 = finalBalanceOutput - turnOutput
  setMotor1/setMotor2()      // → commandStepOutput(Hz) → STEP ISR → TMC
  250ms 遥测窗口极值累积
  flushTelemetryTx()
```

节拍表：

| 频率 | 触发 | 跑什么 |
|---:|---|---|
| 400 Hz | 每拍（2.5ms） | IMU 读、姿态、速度预测、权限、直立 PD、转向、电机合成 |
| 100 Hz | `outerLoopDivider==4` | 速度环 PI、编码器速度修正、运动驱动 |
| 4 Hz | `FAST_TELEMETRY_DIVIDER=100` | `G,` 快帧（8 字段） |
| 1 Hz | `TELEMETRY_DIVIDER=400` | `D,` 慢帧（78 字段） |
| 事件 | BLE 命令 | 改参数、`SP`/`MS`/`EG`/`SV`、`SAVE`/`LOAD`/`DEF`、`^`、`?`、`!` |

---

## 4. 三条闭环通路

**① 直立（最快、最内层，400Hz）**
```
BMI270 → readBMI270() → updateKalman/updateComplementary → updateAttitude()
   → pitch, pitchRate
   → updateBalanceLoop():  P·(pitch-O) + D·pitchRate (+前馈) → balanceOutput
   → 限幅/速率限制 → finalBalanceOutput(Hz)
   → motor1/2 合成 → setMotor1/2 → commandStepOutput → STEP 定时器/ISR → TMC2209 → 电机
```

**② 速度（外环，100Hz）**
```
编码器 ISR 累计 count → correctVehicleSpeedKalman(修正) ┐
IMU 前向加速度 → predictVehicleSpeedKalman(预测) ──────┤
                                                      ▼
                                            vehicleSpeed (counts/50ms)
targetForwardSpeed ← SP(speedCmd) / 位置环 / 手动通路
                                                      │
                              speedError = target - vehicleSpeed
                                                      ▼
                    speedP = K·speedError (+ I 已停用, D 可选)
                                                      ▼
                    speedAngleCorrection → 叠加到 targetBalanceAngle
```
**注意**：`targetBalanceAngle = balanceAngleOffset + 权限 · 修正`，
修正被 `N=MAX_SPEED_ANGLE(0.8° )` 硬限幅 —— 这是"只能点动、不能巡航"的结构原因。

**③ 位置（可选，默认关）**
```
vehiclePosition ← 编码器累计 − positionZeroCount
updatePositionHold():  positionError → positionSpeedTarget（X=0 时直接 0）
                                  ▼
                     targetForwardSpeed = positionSpeedTarget
```

---

## 5. 主要函数索引（物理行号为锚点；按调用顺序分组）

### L0 调度与自检
| 函数 | 行 | 职责 |
|---|---:|---|
| `setup()` | 4336 | 一次性初始化：电机→编码器→BLE→TMC→参数读回→IMU→校准→清状态 |
| `loop()` | 5163 | 400Hz 主循环（见 §3） |

### L1 接口 / 契约
| 函数 | 行 | 职责 |
|---|---:|---|
| `pollBtCommand()` | 4891 | 从串口/BLE 取出完整行命令 |
| `handleBtCommand()` | 4909 | 命令分发：单字母参数 / `PAR` / `?` / `!` / `SAVE` / `LOAD` / `DEF` / `^` |
| `handleTunableCommand()` | 4822 | 两字母运行参数：`EG` / `SP` / `SV` / `MS` |
| `paramStorageBoot()` | 4764 | 开机：抄编译默认值 → 尝试用 flash 记录覆盖 |
| `paramCollect/Apply/WriteAllowed` | 4748/4752/4759 | 参数↔数组、写盘前置条件（必须静止） |
| `txPushLine/flushTelemetryTx` | 1522/1545 | 遥测发送缓冲（9600 波特，非阻塞） |
| `sendTelemetry()` | 4225 | `D,` 慢帧 78 字段（1Hz） |
| `sendFastTelemetry()` | 4320 | `G,` 快帧 8 字段（4Hz） |

### L2 控制
| 函数 | 行 | 职责 |
|---|---:|---|
| `updateOuterLoop()` | 3337 | 编码器速度修正→运动驱动→速度 PI→目标角（100Hz） |
| `updateOuterLoopAuthority()` | 3828 | 大角度时外环平滑退出，直立优先（400Hz） |
| `updatePositionHold()` | 3972 | 位置环（400Hz 调用，内部按条件使能） |
| `captureCurrentPositionTarget()` | 4000 | 锁定当前位置为目标 |
| `stopPositionJog()` | 4015 | 停止点动并复位状态 |
| `updateBalanceLoop()` | 4029 | 直立 PD → `finalBalanceOutput`（400Hz） |
| `updateTurnLoop()` | 4199 | 转向环 → `turnOutput`（400Hz） |

### L3 估计
| 函数 | 行 | 职责 |
|---|---:|---|
| `updateAttitude()` | 2364 | 姿态总入口：加速度/陀螺融合、互补备用 |
| `updateKalman()` | 2035 | 俯仰角 Kalman |
| `updateComplementary()` | 2321 | 互补滤波（备用/对照） |
| `calculateAccelPitch()` | 1937 | 重力向量 → 倾角 |
| `predictVehicleSpeedKalman()` | 3027 | 400Hz 速度预测（IMU 加速度驱动） |
| `correctVehicleSpeedKalman()` | 3193 | 100Hz 编码器速度修正 |

### L4 驱动 / 中断
| 函数 | 行 | 职责 |
|---|---:|---|
| `initMotors()` / `stopBothMotors()` | 2873 / 2962 | 电机引脚/定时器与紧急停止 |
| `commandStepOutput()` | 2894 | 设定 STEP 频率/方向（带反向软斜坡） |
| `stepEdge()` + `motor1/2EdgeISR` | 2845 / 2860 | STEP 上升沿计数（用于自检/诊断） |
| `setMotor1/2()` | 2983 / 2999 | 单轮输出（含零保持计数） |
| `encoder1/2ISR()` | 1603 / 1636 | 四倍频编码器计数 |
| `initEncoders()` | 2740 | 编码器引脚中断配置 |
| `initBMI270/configureBMI270_400Hz/readBMI270` | 1881 / 1821 / 1854 | IMU 初始化与 400Hz 采样 |
| `calibrateIMU()` | 1974 | 开机静止校准（零偏/安装偏置） |
| `initTmcDrivers()/…WriteOnly()` | 337 / 386 | TMC2209 UART 配置（三种编译模式） |

### 工具类（头文件）
| 结构 | 文件 | 职责 |
|---|---|---|
| `MotionSpeedProfile` | motion_drive.h | 速度目标斜坡（`accel=20 count/50ms per s`） |
| `NetSpeedEstimator` | motion_drive.h | 200ms 净速度 + 3 点中值 |
| `MotionPositionPlan` | motion_drive.h | 位置计划 + 前视补偿 |
| `MotionLeanControl` | motion_drive.h | 速度误差 → 倾角（`EG` 增益，限 0.9°） |
| `ManualPitchControl` | manual_motion.h | `^` 手动俯仰（smoothstep/刹车） |
| `ManualTurnControl::clamp` | manual_turn.h | 通用限幅 |
| `PositionJog` | position_jog.h | 位置点动状态机 |
| `ParamStore` | param_store.h | 22 项参数记录/扫描/写入 |
| `Stm32F4FlashPort` | param_flash_f4.h | sector 10/11 擦写 |

---

## 6. 全局量分区（讲解时按这四类走，避免"变量海洋"）

1. **参数区（22 项，可 SAVE）**：`balanceKp/Kd`、`balanceAngleOffset(O)`、`BALANCE_ANGLE_DEADBAND_DEG(Z)`、
   `BALANCE_OUTPUT_LIMIT_HZ(L)`、`BALANCE_OUTPUT_SLEW_HZ(S)`、`BALANCE_ACCEL_SCALE(R)`、
   `HIGH_SPEED_ACCEL_START/LIMIT(B/H)`、`speedKp(K)`、`speedKi(I)`、`MAX_SPEED_ANGLE(N)`、
   `SPEED_INTEGRAL_LIMIT(W)`、`SPEED_ERROR_DEADBAND(J)`、`speedKd(E)`、`positionKp(X)`、
   `MAX_POSITION_RETURN_SPEED(Y)`、`POSITION_DEADBAND_COUNT(M)`、`OUTER_FULL/DISABLE_ANGLE(F/G)`、
   `turnKp/turnKd(T/U)`。
2. **运行指令（RAM-only）**：`speedCmd(SP)`、`MAX_TARGET_FORWARD_SPEED(MS)`、
   `motionLean.errorGain(EG)`、`speedKp(SV，与 K 同一变量)`。
3. **控制状态**：`pitch/pitchRate`、`targetBalanceAngle`、`balanceOutput/finalBalanceOutput`、
   `speedError/speedIntegral/speedAngleCorrection`、`vehicleSpeed/vehiclePosition`、
   `outerLoopAuthority`、`targetForwardSpeed`、`turnOutput`。
4. **诊断/保护**：`imuFailCount`、`fallConfirmCount`、`controlOverrunCount`、
   `controlMaxGapUs`、`controlMaxWorkUs`、`telem*Max/Min` 窗口极值。

---

## 7. 对外契约（讲解时必须逐字段核对的"协议面"）

- **`D,` 慢帧**：78 个逗号字段（1Hz），含姿态/速度/外环/电流/诊断；
- **`G,` 快帧**：`G,millis,pos,speed,pitch,target,output,manual,btRx`（4Hz）；
- **`PAR:` 回显**：22 项 + `EG/SAVED/SP/SV/MS` 追加字段；
- **`JOGCFG`**：位置点动的 kp/limit；
- **命令**：单字母参数写入、`?`、`!`、`SAVE`、`LOAD`、`DEF`、`^ turn,pitchDeg`、`SP/SV/EG/MS`。

---

## 8. 逐行讲解章节计划（P1–P9）

| 章 | 主题 | 物理行范围 | 要回答的问题 |
|---|---|---|---|
| **P1** | 调度骨架 | 4336–4672、5163–5825 | 一拍 2.5ms 里到底按什么顺序做了什么？超时/掉帧/IMU 失败怎么兜底？ |
| **P2** | 启动自检链 | 310–395、4340–4460 | 为什么 TMC/BMI270 初始化失败要死循环重试？失败时电机为什么必须停？ |
| **P3** | 步进输出与中断 | 2845–3020、1595–1676 | Hz 怎么变成 STEP 脉冲？零保持计数、反向软斜坡是干什么的？ |
| **P4** | IMU 与姿态 | 1677–2035、2035–2360、2364–2740 | 俯仰角怎么来的？加速度/陀螺怎么融合？校准做了什么、没做什么？ |
| **P5** | 速度估计 | 3027–3193、3193–3336 | 400Hz 预测 + 100Hz 修正的双层 Kalman 怎么保证不打架？单位是什么？ |
| **P6** | 外环串级 | 3337–3827、3828–3971、3972–4027 | 位置→速度→倾角三级怎么串？`N=0.8` 限幅为什么导致"只能点动"？ |
| **P7** | 直立环 / 转向环 | 4029–4220 | PD 的实际算式、限幅/速率限制顺序、与速度修正怎么叠加？ |
| **P8** | 遥测与 BLE 协议 | 4225–4334、1455–1600、4687–5162 | 78 字段契约、9600 波特预算、命令解析的优先级与陷阱 |
| **P9** | 参数持久化 + 头文件 | 4700–4812 + 6 个头文件 | 三层参数（源码默认/闪存/在跑固件）、22 项表、`FORMAT_VERSION`、四件套工具类 |

**建议顺序**：P1 → P2/P3 → P4/P5 → P6 → P7 → P8/P9
（先看调度和"手脚"，再看"感觉"，最后看"大脑"与"协议"）

---

## 9. 讲课时要特别点出的历史坑（避免读者踩同一坑）

1. `MAX_TARGET_FORWARD_SPEED` 曾是 `const`（编译期锁死 8），2026-11-20 才改成可写 + `MS` 命令。
2. `N=2.0` 会反向猛拽 2° 翻车；`N=0.8` 是实测稳定边界，不是"越大越有力"。
3. 速度环 `I≠0` 在静摩擦下变极限环、稳态速度反向；当前 `I=0`。
4. `SV` 与 `K` 是同一个变量（`speedKp`），改 `SV` 等于改 `K`。
5. 参数有**三层**：源码默认值 → 闪存 `SAVED` 记录 → 正在跑的固件；`?` 才是当前值。
6. 遥测发送与命令回显共用 9600 波特 TX，命令后 400ms 暂停遥测。
