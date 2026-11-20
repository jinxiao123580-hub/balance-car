# P1 · 调度骨架：`setup()` 与 `loop()`

> 本章范围：`main.cpp` `setup()` L4336–4672、`loop()` L5163–5825
> 前置：先看 `00-STRUCTURE.md` 的分层图与时序表。
> 约定：行号是物理行（文件有大量空行，行号会漂），**函数名才是稳定锚点**。

---

## 0. 一句话总结

`setup()` 是**一次性安全检查 + 外设上电 + 参数就位**；
`loop()` 是一个**每 2.5ms 拍一次的硬实时调度器**，把"读传感器 → 估计 → 控制 → 输出 → 上报"串成固定顺序，
并在三个地方兜底：**掉拍、IMU 失效、倒地**。

两者共同的设计原则：**任何异常都先停电机，再清控制状态，最后才 return**。

---

## 1. `setup()`：为什么是"这个顺序"

### 1.1 先动"手脚"，再动"感觉"

```c
void setup()
{
    initMotors();      // L4340
    initEncoders();    // L4344
```

- **为什么 `initMotors()` 排第一**：上电瞬间 STEP/DIR/EN 引脚是浮空/默认电平，电机可能被噪声驱动。
  先把电机输出初始化到"停"的安全态，后面所有耗时初始化期间车才不会自己动。
- **为什么紧接着 `initEncoders()`**：编码器是**中断计数**，越早挂上中断，后面的位置比较才连续；
  而且它只依赖中断，不依赖传感器，属于"不会失败的初始化"。
- 这一行也定下了全文的失败策略：**能自己兜的立刻兜，不能兜的（IMU/TMC）才进重试循环**。

### 1.2 蓝牙必须最先能用

```c
    BTSerial.begin(BT_BAUD);   // L4356, BT_BAUD = 9600
    delay(100);                // L4364
```

- `BT_BAUD = 9600`：JDY-23 透传模块的固定速率。**这个 9600 是全系统带宽瓶颈**——
  后面 `D`/`G` 帧的发送频率、命令回显的 400ms 静默，全都是为它让路。
- `delay(100)`：不是"随手等"，是等 JDY-23 模块自己启动完成。早了会丢开头几字节，
  表现为"上电后第一条命令没反应"。
- **为什么在 TMC/IMU 之前**：后面每个失败分支都要 `BTSerial.println("BOOT RETRY...")` 报错。
  日志通道必须先通，否则黑屏死循环无法诊断。

### 1.3 TMC2209：三种编译模式，同一个原则

```c
#if TMC_STANDALONE_STEP_DIR
    // 纯硬件模式：不访问 UART，模式/电流/细分由拨码/电阻决定
#elif TMC_GPIO_WRITE_ONLY
    while (!initTmcDriversWriteOnly(...)) { stopBothMotors(); ... delay(1000); }
#else
    while (!initTmcDrivers()) { stopBothMotors(); delay(1000); }
    BTSerial.println("TMC UART OK: both spreadCycle, 1/16 step");
#endif
```

- `initTmcDrivers()` 走 UART **读回**：确认 2209 真的在线、真的被写成 spreadCycle/1-16 细分。
- `WriteOnly` 分支只写不读（硬件不支持读回），所以打 `UNVERIFIED` 字样——**不谎报成功**。
- **为什么失败要 `while` 死循环并每轮 `stopBothMotors()`**：驱动器配置错误时电流/细分不可知，
  宁可不开机也不能带着错误配置输出。每轮停电机是防止引脚复位过程里意外转动。
- 三种模式靠宏切换，意味着**同一份代码要能适配三块不同底板**，这是历史兼容留下的分支。

### 1.4 参数必须在"控制开始之前"就位

```c
    paramStorageBoot();          // L4403
    while (!initBMI270()) {...}  // L4405
```

- `paramStorageBoot()` 做两件事：抄一份编译默认值，再尝试用 flash 记录覆盖；
  如果当前扇区写满，还会**顺便擦好另一块扇区（约 1.1s）**。
- **为什么放在 IMU 之前而不是之后**：① 这 1.1s 绝不能发生在控制环运行中（会卡住取指）；
  ② 后面的校准/控制全都依赖 `O/N/K` 等参数，晚读一秒就有一秒用错参数。
- 注意：**这一步读取的是"上次 SAVE 的值"，不是当前实车最优点**。
  这也是为什么每次上电要 `?` 复核 `O=`——闪存值会覆盖源码默认值。

### 1.5 IMU：三次握手，任一步失败就重试（并要求车静止）

```c
    while (!initBMI270()) { ... "BOOT RETRY: BMI270 init failed"; }
    BMIWire.setClock(400000);   // L4413
    delay(20);
    while (!configureBMI270_400Hz()) { ... }
    while (!calibrateIMU()) { ... "keep car still"; }
```

顺序是 **能通信 → 能跑到 400Hz → 能校准**，三步递进、每步独立重试：

- `initBMI270()`：SPI/I2C 能读到 chip id。
- `configureBMI270_400Hz()`：量程/带宽/ODR 真的被写成 400Hz。**这一步失败最隐蔽**——
  能读到数据 ≠ 数据是按 400Hz 更新的，配错会让 Kalman 的时间常数全错。
- `BMIWire.setClock(400000)`：初始化成功后再提 I2C 速率到 400kHz。
  理由：400Hz × 6 字节/次 ≈ 需要 ≈24kHz 以上净吞吐，100kHz 下会挤占控制时间。
  放在 init 之后是因为**低速更容易初始化成功**，先用保险的速率把设备唤起来。
- `calibrateIMU()`：**要求车静止**，采零偏/安装偏置。失败会一直重试并提示 `keep car still`。
  它采的是"当前姿态作为零参考"，所以**开机姿态不会改写平衡点**（`pitchZero` 是固定安装偏置）——
  这条曾被误判过（`20261119-FINDINGS.json` 里已否证）。

### 1.6 编码器初值：必须用临界区快照

```c
    noInterrupts();
        lastEncoder1Count  = encoder1Count;      // L4471
        lastEncoder2Count  = encoder2Count;
        positionZeroCount1 = encoder1Count;      // L4483
        positionZeroCount2 = encoder2Count;
    interrupts();
    vehiclePosition = 0.0f;  targetVehiclePosition = 0.0f;
```

- `encoder1Count` 是 32 位，但**读它和"另一处写它"之间可能被中断打断**，
  于是 `last` 与 `zero` 可能来自不同时刻 → 第一拍速度出现尖峰。
- 关中断只包住这 4 条赋值（微秒级），是"临界区最小化"的标准写法。
- `positionZeroCount*` 是位置原点的基准：`vehiclePosition = (count - zero)/(counts/mm)`。
  把原点设在开机处，意味**每次上电位置都从 0 开始**。

### 1.7 控制状态清零：把"上一次的残影"抹掉

```c
    vehicleSpeed = rawVehicleSpeed = imuForwardAccelG = 0;
    imuSpeedPredictionTrust = 1.0f;   // 预测器满信任启动
    speedKalmanP = 1.0f;              // 协方差从"不确定"开始
    speedKalmanQ = SPEED_KALMAN_Q_BASE;
    speedKalmanR = SPEED_KALMAN_R_LOW_SPEED;   // 低速用低 R（更信编码器）
    speedError = speedDirectOutput = speedDirectAuthority = 0;
    finalBalanceOutput = targetForwardSpeed = 0;
    targetBalanceAngle = balanceAngleOffset;   // ★ 目标角上电即等于平衡点
    outerLoopDivider = telemetryDivider = 0;
    imuFailCount = fallConfirmCount = 0;
    motor1ZeroHoldCount = motor2ZeroHoldCount = 0;
    outerLoopAuthority = 1.0f;                 // ★ 上电给满权限
    BTSerial.println("time_ms,fused_speed,speed_direct_hz");  // 网页忽略的表头
    lastControlUs = micros();                  // ★ 必须最后一句
```

三处值得单独记的：

| 语句 | 为什么这么写 |
|---|---|
| `targetBalanceAngle = balanceAngleOffset` | 上电第一拍的目标角**直接是 O**，不是 0。否则 `pitch≈O` 时会朝 0 猛拽一次。 |
| `outerLoopAuthority = 1.0f` | 权限是"外环能改多少目标角"，上电给满；后面 `updateOuterLoopAuthority()` 会在大角度时把它压下去。 |
| `lastControlUs = micros()` **放最后** | 若放在开头，setup 里那些 `delay(1000)`/重试会被算成"巨大掉拍"，`loop()` 第一拍直接触发超时保护。放最后 = 第一拍的时间基准从"现在"开始。 |

`BTSerial.println("time_ms,...")` 这行表头是给网页/上位机画图用的，固件自己不解析。

---

## 2. `loop()`：一拍 2.5ms 的完整顺序

### 2.1 命令与发送：**不受 400Hz 节拍限制**

```c
void loop()
{
    uint32_t nowUs = micros();     // L5167 ← 这个值随后被覆盖（见下）
    pollBtCommand();               // L5175
    flushTelemetryTx();            // L5176
    nowUs = micros();              // L5177 ← 真正的计时起点
```

- `pollBtCommand()` 放在节拍闸门 **之前**：命令处理不能等 2.5ms 的窗口，
  否则 `SP`/`O` 这类在线调参会累积到几十毫秒的抖动。
- `flushTelemetryTx()` 同理：9600 波特下发送是"攒够一点就发"，不能占满整拍。
- **代码异味**：L5167 的 `nowUs` 被 L5177 覆盖，第一次 `micros()` 是死代码。
  影响微乎其微（多一次读），但说明这段经过多次改动。讲解时如实指出，不要美化。

### 2.2 节拍闸门与掉拍自救

```c
    if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;  // 2500us
    uint32_t controlGapUs = nowUs - lastControlUs;
    if (controlGapUs > controlMaxGapUs) controlMaxGapUs = controlGapUs;
    if (controlGapUs >= CONTROL_PERIOD_US * 2UL) {
        controlOverrunCount++;
        lastControlUs = nowUs;          // 重基准
        stopBothMotors();
        clearControlOutputDiagnostics();
        outerLoopDivider = 0;
    } else {
        lastControlUs += CONTROL_PERIOD_US;   // 固定步长累加
    }
```

- `(uint32_t)(a-b)` 的**无符号回绕写法**：`micros()` 约 71.6 分钟回绕一次，
  必须用无符号差而不是 `a<b`，否则回绕瞬间会当成"负时间"而死循环。
- **正常情况用 `lastControlUs += PERIOD` 而不是 `= nowUs`**：这叫"固定时间基准"，
  让 400Hz 不随调度抖动漂移。抖动量被吸收到下一拍，长期平均频率精确。
- **掉一拍以上（≥2 拍）就停电机**：这拍的控制量已经过时（IMU 采样间隔不对），
  继续用力等于用旧姿态做决策 → 宁可停一拍也不给错误电流。
- 统计量：`controlOverrunCount`（掉拍次数）、`controlMaxGapUs`（最大间隔）、
  `controlMaxWorkUs`（单拍最长耗时），都在 `!` 诊断命令里回显。

### 2.3 读 IMU：分级降级，不是一刀切

```c
    if (!readBMI270()) {
        imuFailCount++;
        if (imuFailCount < IMU_FAIL_CONFIRM_COUNT) return;   // 8 次 ≈ 20ms
        stopBothMotors(); clearControlOutputDiagnostics();
        balanceOutput = finalBalanceOutput = speedDirectOutput = speedDirectAuthority = 0;
        speedIntegral = speedAngleCorrection = speedErrorPrev = speedErrorDotFilt = 0;
        positionSpeedTarget = targetForwardSpeed = 0;
        return;
    }
    imuFailCount = 0;
```

- **< 8 次失败：直接 return，保持上一拍的 STEP 输出**。
  为什么敢"不上报"？因为步进输出是**定时器频率**，停在这里并不会立刻断电，
  电机保持上一拍的速度——一次 I2C 抖动不该造成车身一顿。
- **≥ 8 次（≈20ms）：判定真故障**，停电机 + 清掉**所有会累积的状态**
  （积分、微分历史、位置目标），防止故障恢复瞬间用旧积分猛冲。
- `IMU_FAIL_CONFIRM_COUNT = 8` 的依据：400Hz × 8 = 20ms，比人的反应快、比总线抖动长。

### 2.4 估计：姿态 + 速度预测

```c
    updateAttitude();              // L5338  → pitch / pitchRate（Kalman，失败可退互补）
    predictVehicleSpeedKalman();   // L5357  → 用 IMU 前向加速度预测速度（400Hz）
```

- `updateAttitude()` 是**姿态唯一入口**，详见 P4。
- 400Hz 只做**预测**；编码器的**绝对速度修正**在 100Hz 外环里做（`correctVehicleSpeedKalman`）——
  "快预测 + 慢修正"是这套 Kalman 的核心分工，详见 P5。

### 2.5 遥测：先发数据，再做保护

```c
    telemetryDivider++;      if (telemetryDivider >= 400) { sendTelemetry(); }      // 1Hz  D 帧
    fastTelemetryDivider++;  if (fastTelemetryDivider >= 100) { sendFastTelemetry(); } // 4Hz  G 帧
```

- 位置在**倒地保护之前**，注释写得很清楚：*"这样即使触发倒地保护，手机上仍能继续看到 IMU/Kalman 数据"*。
  这是**可观测性优先于保护顺序**的典型取舍。
- 1Hz 的 `D` 帧有 78 个字段，4Hz 的 `G` 帧只有 8 个。9600 波特 ≈ 960 B/s 的总预算里，
  两个帧加起来约占一半，其余留给命令回显。

### 2.6 倒地保护：两道阈值 → 停机 + 全状态清零

```c
    float fallAngleError = fabsf(pitch - balanceAngleOffset);
    if (fallAngleError >= FALL_ANGLE_DEG) fallConfirmCount++; else fallConfirmCount = 0;
    bool confirmedFall = fallConfirmCount >= FALL_CONFIRM_COUNT;   // 8 帧
    bool emergencyFall = fallAngleError >= EMERGENCY_FALL_ANGLE_DEG;
    if (confirmedFall || emergencyFall) { ... 清空一切 ... return; }
```

- `FALL_ANGLE_DEG = 75°`、`FALL_CONFIRM_COUNT = 8`（≈20ms 去抖）。
- **`EMERGENCY_FALL_ANGLE_DEG` 也是 75°** —— 这是个值得指出的细节：
  两道阈值相等时，`emergencyFall` 在**第一帧**就成立，把"连续 8 帧确认"短路掉了。
  实际影响不大（真到 75° 已经倒了），但**若想让去抖真正生效，应把 EMERGENCY 设得明显更大（如 120°）**。
- 倒地时清的东西比 IMU 故障更多，关键是这四项：
  - `noInterrupts()` 快照编码器并重置 `positionZeroCount*`、`vehiclePosition = 0`
    （车被拿起/挪动过，累计位置作废）；
  - `targetBalanceAngle = balanceAngleOffset`（目标角回平衡点）；
  - `outerLoopAuthority = 0`（外环权限归零，防止站起来瞬间被旧速度目标拖走）；
  - `fallStopCount++` 便于统计"今天倒了几次"。
- **为什么"清空这么多"**：倒地后所有历史量都失去物理意义（积分、微分、位置）。
  不清，扶起来的第一秒就会用旧账算出大输出 → 二次翻车。

### 2.7 正常路径：外环 → 权限 → 直立 → 转向 → 电机

```c
    manualForward.update(millis(), CONTROL_DT, pitch - balanceAngleOffset);  // ^ 手动俯仰
    updatePositionHold();                                   // 位置环（默认 X=0 → 内部早退）

    if (++outerLoopDivider >= 4) { outerLoopDivider = 0; updateOuterLoop(); }  // 100Hz
    updateOuterLoopAuthority();                             // 400Hz 权限平滑
    updateBalanceLoop();                                    // 400Hz 直立 PD
    updateTurnLoop();                                       // 400Hz 转向

    float motor1Command = finalBalanceOutput + turnOutput;  // 差速转向
    float motor2Command = finalBalanceOutput - turnOutput;
    setMotor1(motor1Command);  setMotor2(motor2Command);
```

- **为什么外环是 ÷4**：外环（速度 PI）用 400Hz 会放大编码器量化噪声；
  100Hz 既够快又能平均。分频器是软件计数，不额外占定时器。
- **顺序不能换**：先算外环（产生 `targetBalanceAngle`）→ 再算权限（决定外环有多少发言权）
  → 再算直立（消费目标角）→ 转向独立。
- **`m1 = final + turn`、`m2 = final - turn`**：两轮差速，`turnOutput` 为 0 时就是纯前后。
- 注意这里**没有限幅**：限幅发生在 `updateBalanceLoop`/`updateTurnLoop` 内部
  （`finalBalanceOutput` 出来时已经是限好的 Hz），`setMotor1/2` 再做最后的零保持/换向处理（P3 详解）。

### 2.8 结尾：遥测窗口极值 + 单拍耗时统计

```c
    { // 250ms 窗口极值：陀螺最大、加速度-积分偏差、平衡输出最大、Kalman 偏差范围
      if (aGyro > telemGyroAbsMax) ...
    }
    flushTelemetryTx();
    uint32_t workUs = micros() - controlWorkStartUs;
    if (workUs > controlMaxWorkUs) controlMaxWorkUs = workUs;
```

- 这些极值只在 1Hz 的 `D` 帧里上报，是**远程诊断"抖不抖/饱和没饱和"**的核心指标。
- `controlMaxWorkUs` 必须放在**所有控制量算完之后**，否则统计不到电机输出那一段。
- 若 `controlMaxWorkUs` 接近 2500us，说明这一拍已经跑不动 400Hz 了 —— 这是实时性红线。

---

## 3. 本章的"时间预算"总表

| 项 | 值 | 依据 |
|---|---:|---|
| 控制周期 | 2500us | `CONTROL_PERIOD_US` |
| 掉拍判据 | ≥ 5000us（2 拍） | `CONTROL_PERIOD_US*2` |
| IMU 故障确认 | 8 拍 ≈ 20ms | `IMU_FAIL_CONFIRM_COUNT` |
| 倒地确认 | 8 拍 ≈ 20ms | `FALL_CONFIRM_COUNT` |
| 倒地阈值 | 75° | `FALL_ANGLE_DEG` / `EMERGENCY_FALL_ANGLE_DEG` |
| 外环 | 100Hz | `outerLoopDivider >= 4` |
| 快帧 | 4Hz | `FAST_TELEMETRY_DIVIDER = 100` |
| 慢帧 | 1Hz | `TELEMETRY_DIVIDER = 400` |
| 串口 | 9600 baud ≈ 960B/s | `BT_BAUD` |

---

## 4. 本章要记住的 5 个坑

1. **`lastControlUs = micros()` 必须在 `setup()` 最后**，否则第一拍被当掉拍保护。
2. **IMU 失败 < 8 次是"保持输出"，不是"停"**；≥ 8 次才清状态——这个分级容易看反。
3. **`EMERGENCY_FALL_ANGLE_DEG == FALL_ANGLE_DEG == 75`**，去抖被紧急通道短路（见 §2.6）。
4. **掉拍后 `lastControlUs = nowUs`（重基准），正常时 `+= PERIOD`（固定步长）**，两处理由不同。
5. **倒地与 IMU 故障都会清状态，但倒地会额外重置位置原点**——因为车被人挪过。

---

## 5. 下一章

- **P2 启动自检链**：`initMotors` / `initEncoders` / `initTmcDrivers*` 的内部实现与失败模式
- **P3 步进输出与中断**：`commandStepOutput` 如何把 Hz 变成脉冲、零保持计数、反向软斜坡
