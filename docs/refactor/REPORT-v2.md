# 固件三层拆分交付报告（SPEC-modules-v2 执行结果）

- 分支：`refactor/lean-split`
- 回滚点（拆前）：`99eadae`（Flash 89136B / RAM 8388B）
- 本次提交链：
  - `c0b4e38` 格式修复（token 等价，5838 → 3103 行，Flash/RAM 与基线逐字节一致）
  - `ce03367` core/（cfg.h + state.h + util.h）
  - `2184f89` control/motor
  - `206bc19` control/imu + attitude + speed_est
  - `a168421` control/cascade + remote/remote_control 骨架（同时按 SPEC §5 删死代码）
  - `50f2304` remote/ 五个子模块拆分，删旧 4 个头文件
  - `a493f75` panel/ 四个子模块 + 薄 main.cpp + 死全局量清理
- 目标树与 SPEC §1 一致；`main.cpp` **344 行**（≤350）。

---

## 验收 1：`pio run` 成功 + Flash/RAM

```
Building in release mode
Checking size C:\Users\10905\.platformio\build\pinghengche\black_f407zg\firmware.elf
RAM:   [=         ]   6.4% (used 8360 bytes from 131072 bytes)
Flash: [=         ]   8.5% (used 89472 bytes from 1048576 bytes)
========================= [SUCCESS] Took 18.81 seconds =========================
```

| 项 | 基线 99eadae | 拆分后 a493f75 | 差异 |
|---|---|---|---|
| Flash | 89136 B | **89472 B** | +336 B |
| RAM | 8388 B | **8360 B** | −28 B |

差异解释（无功能差异）：
- **RAM −28B**：删掉死全局量（`COMP_ALPHA`/`R_measure`/`telemetryMuteUntilMs`/`btCmdCount`/`verticalLoopGain` 等）。
- **Flash +336B**：物理拆成 17 个编译单元后，原本同 TU 内的内联（`clamp`/小内联函数/常量折叠）跨 TU 无法内联，且 `core/cfg.h` 中 `const` 在多个 TU 各生成一份 .rodata。属于隔离的固定成本，不是代码增加。
- 交叉验证：把 `TMC_STANDALONE_STEP_DIR` 临时改为 `0`（即启用原被预处理器关掉的 TMC2209 UART 分支）也能编译通过（Flash 93324B / RAM 9248B），说明被搬走的死分支没有悬空引用；随后已改回 `1`。

---

## 验收 2：依赖方向

扫描 `software/firmware/src` 全部 `#include "..."`：

```
control/ -> remote|panel includes: 0        => PASS
remote/  -> panel includes:       0        => PASS
```

`control` 需要手动倾角/转向量时只走**数据接口**：
- `remote::leanOffset()` / `remote::turnOffset()` / `remote::isActive()` / `remote::normalAuthority()` /
  `remote::manualPitchBusy()` / `remote::jogLeanOffset()` / `remote::motionWasBusy()`（`remote/remote_control.h`）
- `main.cpp` 在每拍按原顺序把这些量作为**实参**传给 `control::outerLoop(...)` / `control::updateAuthority(...)`；
  `control/` 内没有任何 `#include "remote/..."`。

---

## 验收 3：死代码检查（SPEC §5 的 4 处）

```
$ grep -rn '\b(updateComplementary|speedDirect|speedPredicted)\b' software/firmware/src
hits: 0        => PASS
```

具体删除内容：

| # | 内容 | 处理 |
|---|---|---|
| 1 | `updateComplementary()` | 函数整体删除（`control/attitude.cpp`） |
| 2 | `speedDirectOutput / speedDirectUnclampedOutput / speedDirectAuthority / speedDirectAppliedOutput / speedDirectKp` | 变量与全部赋值删除；D 帧第 9/10/21 列保留序号、填 `0` |
| 3 | `speedPredicted / speedPredictionDelta / speedAccelerationFiltered / speedPredictPrevSpeed / speedPredictHorizon` | `speedPredictionDelta`/`speedAccelerationFiltered`/`speedPredictPrevSpeed`/`speedPredictHorizon` 删除，D 帧第 18/20 列填 `0`；**`speedPredicted` 按 Lead 指示不填 0**，改名为 `telemetrySpeedSnapshot100Hz`，仍在原位置赋值 `= vehicleSpeed`，D 帧第 19 列数值与今天逐字段一致 |
| 4 | `PAR` 回显里的 `Q=` / `A=` | 两对回显字段删除；`case 'Q'`/`'a'` 两个死旋钮 setter 一并删除（其余 48 个 case 全部保留，已比对：removed = `A/a/Q/q`，added = 无） |

D 帧格式串**逐字符未变**（75 字段，`"D,%lu,..."` 与拆分前 `==`），G 帧格式串同样 `==`。

---

## 验收 4：入口未丢

`panel/cmd.cpp` + `panel/params.cpp` 里逐条确认（正则证据在 `~/.dsh-analysis/refactor-pinghengche/acceptance.py`）：

| 入口 | 语义 | 结果 |
|---|---|---|
| `^ turnHz,pitchDeg` / `@ 0,0` | 手动俯仰+转向（`remote::parseManualMotion` → `manualTurn.command` / `manualForward.command`） | FOUND |
| `~ id,delta` / `& id` | 位置点动/心跳（`positionJog.start/heartbeat`） | FOUND |
| `!` / `! kp,limit` | 驱动诊断 / jog 配置（`step1.risingEdges`、`controlOverrunCount`、`positionLean.kp/limit`） | FOUND |
| `?` | 参数回显（22 项 + EG/SAVED/SP/SV/MS + JOGCFG） | FOUND |
| `SP` / `MS` / `SV` / `EG` | 速度指令 / 速度上限 / 速度环 P / 运动误差增益（`handleTunableCommand`） | FOUND |
| `SAVE` / `LOAD` / `DEF` | 参数持久化（`panel::handleParamWord`） | FOUND |
| `P D O Z L S R B H K I N W J E X Y M F G T U` | 22 项单字母参数 | FOUND（表顺序逐项比对一致） |
| `V` / `C` | 转向命令 / 当前位置捕获 | FOUND |

参数持久化契约：`panel/param_store.h` 的 `FORMAT_VERSION = 3`（未改），参数表 `paramSlots` 与拆分前**逐项相同**（22 项，键与指针地址一致），闪存记录继续有效（`SAVED=1`、`O` 不丢）。

---

## 验收 5：`loop()` 400Hz 顺序（与原代码逐条对齐）

```c
void loop() {
  uint32_t nowUs = micros();
  panel::poll();                    // 1 收行分发（原 pollBtCommand）
  panel::flushTelemetryTx();        // 2 TX flush
  nowUs = micros();
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) return;   // 3 节拍闸门
  ... controlGapUs / controlMaxGapUs ...                              // 4 掉拍处理
  if (controlGapUs >= CONTROL_PERIOD_US * 2UL) { controlOverrunCount++; lastControlUs = nowUs;
      control::stopBothMotors(); clearControlOutputDiagnostics(); outerLoopDivider = 0; }
  else lastControlUs += CONTROL_PERIOD_US;
  uint32_t controlWorkStartUs = micros();
  if (!control::readBMI270()) { imuFailCount++; ... }                 // 5 IMU
  imuFailCount = 0;
  control::tick400();              // 6 姿态 + 7 速度预测（原 updateAttitude/predictVehicleSpeedKalman）
  telemetryDivider++;  if (...) panel::sendSlow();                   // 8 遥测分频（D 帧 1Hz）
  fastTelemetryDivider++; if (...) panel::sendFast();               //   （G 帧 4Hz）
  ... 倒地保护 FALL_ANGLE/FALL_CONFIRM/EMERGENCY ...                  // 9 倒地保护（连续确认）
  remote::tick(CONTROL_DT);        // 10 远控 tick（原 manualForward.update）
  remote::positionHold();          // 11 位置保持（原 updatePositionHold）
  outerLoopDivider++;
  if (outerLoopDivider >= 4) {     // 12 ÷4 外环（100Hz）
    outerLoopDivider = 0;
    control::tick100();            //    编码器增量+信任门+Kalman 修正（原 updateOuterLoop 第 1~3 步）
    bool jogPos = false; bool jogAngle = false;
    if (!remote::tick100(jogPos, jogAngle)) {                      //    运动驱动/点动调度
      control::outerLoop(jogPos, jogAngle, remote::motionWasBusy(),
                         remote::manualPitchBusy(), remote::jogLeanOffset());  // 速度PI→目标角
    }
    if (leanResetRequested) { remote::resetLean(); leanResetRequested = false; }
  }
  control::updateAuthority(remote::normalAuthority(), remote::leanOffset());  // 13 权限
  control::updateBalance();        // 14 直立环
  remote::updateTurn();            // 15 转向环
  control::mixMotors(remote::turnOffset());                          // 16 电机合成
  panel::diagAccumulate();         // 17 250ms 遥测窗口累积
  panel::flushTelemetryTx();       // 18 flush
  uint32_t workUs = micros() - controlWorkStartUs;
  if (workUs > controlMaxWorkUs) controlMaxWorkUs = workUs;
}
```

与原代码对照：`poll→flush→闸门→掉拍→IMU→姿态→速度预测→遥测分频→倒地保护→远控tick→位置保持→÷4外环→权限→直立→转向→电机合成→遥测窗口→flush`，**顺序逐条一致**。

`setup()` 初始化顺序也已逐条比对：`initMotors → initEncoders → panel::begin()（原 BTSerial.begin+delay(100)）→ TMC 分支 → panel::paramsBoot() → initBMI270 → BMIWire.setClock(400000) → configureBMI270_400Hz → calibrateIMU → 编码器/位置/Kalman/速度环初值 → lastControlUs=micros()`；差异仅限函数移到命名空间后的限定符与已删死变量（`speedDirectOutput/Authority` 两行）。

> 关于 SPEC §3 的 `control::tick400()/tick100()`：Lead 已批准「loop 顺序优先」，两函数作为**段落入口**存在——`tick400()` = IMU→姿态→速度预测；`tick100()` = 编码器估计+Kalman 修正。速度 PI/目标角另为 `control::outerLoop()`，因为中间必须插入远控调度（原代码结构如此）。

---

## 验收 6：最终文件树 + 每文件行数

```
software/firmware/src/
├─ main.cpp                        344   （仅 setup 顺序 + loop 节拍 + 清故障编排）
├─ core/
│  ├─ cfg.h                        223   （引脚/周期/阈值/限幅常量，#define 与 const）
│  ├─ state.h                      160   （151 条全局 extern 声明，唯一跨模块状态通道）
│  └─ util.h                        15   （clampf / isFiniteValue）
├─ control/
│  ├─ motor.h / .cpp                51 / 494  （STEP 频率/零保持/换向软斜坡/编码器 ISR/TMC UART 初始化）
│  ├─ imu.h / .cpp                  25 / 175  （BMI270 读写/400Hz 配置/开机校准）
│  ├─ attitude.h / .cpp             23 / 189  （Kalman 姿态 → pitch/pitchRate）
│  ├─ speed_est.h / .cpp            27 / 185  （编码器累计 + 速度 Kalman 预测(400Hz)/修正(100Hz)）
│  ├─ balance.h / .cpp              21 / 182  （直立 PD + 限幅/速率/换向整形）
│  └─ cascade.h / .cpp              28 / 245  （速度 PI、外环权限、目标角）
├─ remote/
│  ├─ manual_pitch.h / .cpp         94 / 18   （ManualPitchControl + parseManualMotion）
│  ├─ manual_turn.h / .cpp          66 / 28   （转向状态机 + 转向环 PD）
│  ├─ motion_drive.h / .cpp        173 / 40   （斜坡/净速度/位置计划/倾角 + 100Hz 运动驱动）
│  ├─ position_jog.h / .cpp        171 / 129  （位置点动/位置保持/位置 P 外环）
│  └─ remote_control.h / .cpp       55 / 105  （调度 + 数据接口 + 清故障）
└─ panel/
   ├─ params.h / .cpp               29 / 200  （22 项参数表 + SAVE/LOAD/DEF + 三层值语义）
   ├─ cmd.h / .cpp                  30 / 456  （BLE 命令解析；转发 remote/params）
   ├─ telemetry.h / .cpp            55 / 165  （G 帧 4Hz / D 帧 1Hz + 非阻塞 TX）
   ├─ diag.h / .cpp                 18 / 51   （计数器 + 250ms 窗口极值）
   ├─ param_store.h                203        （闪存记录格式引擎，原样保留 FORMAT_VERSION=3）
   └─ param_flash_f4.h              73        （F4 扇区读写端口，原样保留）
```

`main.cpp` = **344 行 ≤ 350**。总代码行数：拆前 5838 物理行（2741 真实代码行 + 3097 空白/伪空行），拆后 36 个文件合计 4552 行（含空行/注释）。

**已删除的旧文件**：`src/manual_motion.h`、`src/manual_turn.h`、`src/motion_drive.h`、`src/position_jog.h`（内容原样搬入 `remote/`）。
**去留说明**：`param_store.h`/`param_flash_f4.h` 未删除，物理移入 `panel/`（它们是闪存记录格式引擎，删了会破坏 `SAVED=1` 兼容；`FORMAT_VERSION=3` 未动）。

---

## 附加证据 A：格式修复（387 处空行 / 折行语句）

原文件是格式化工具系统性打散的：

| 指标 | 修复前 | 修复后 |
|---|---|---|
| 物理行 | 5838 | 3103（格式修复时）→ 344（拆分后 main.cpp） |
| 纯空行 | 2556 | — |
| 只含空格的"伪空行" | 541 | — |
| ≥2 行连续空行段 | **392 处** | 0 |
| 被拆开的语句（`if (` / 空行 / 条件 / 空行 / `)`） | 72 处（括号未配平行 72 对） | 0 |

修复方式：C++ 词法器 → token 流重排 → **回读再 token 化，与原文件逐 token 比对**：
```
input tokens : 13957
output tokens: 13957
TOKEN-EQUIVALENT: YES
```
（Lead 用另一套分词口径独立复核为 15025 tokens 完全一致、注释 883 条一致，结论相同。）
该步单独提交 `c0b4e38`，构建产物大小与基线完全一致（89136/8388），证明**只动了空白**。

## 附加证据 B：语句级覆盖审计（防止搬漏/改算式）

把拆前 `c0b4e38` 的 main.cpp 与拆后全树都按 token 切成语句（忽略注释/空白，折叠已批准的改名），做多重集差：
- 原文件语句 1795 条，**77 条**未在树中逐字出现；
- 逐条人工分类（脚本 `verify_stmts.py`）：全部属于以下已批准变更，**没有一条是算式/阈值/时序被改**：
  - 死代码删除（`speedDirect*`/`speedPredicted` 系列/`updateComplementary`/已核实的死常量与死全局量）
  - `PAR` 的 `Q=`/`A=` 与 `case 'Q'/'A'`
  - 已批准的结构改名（`updateOuterLoop→outerLoop` 等）、`static` 去链接化（函数改到 .h 声明必须去掉 static）
  - 数据接口替代（`manualForward.normalAuthority()`→`normalAuthority` 参数、`motionLean.applied`→`leanOffset` 参数、`ManualTurnControl::clamp`→`core/util.h` 的 `clampf`（函数体逐字相同））
  - 电机合成两行搬进 `control::mixMotors()`
  - D 帧格式串未变、仅死字段实参改 `0`

## 附加证据 C：全局变量初始化值核对

对拆分后每个全局定义，把 `name = <init>;` 的初始化 token 流回查拆前 main.cpp：**全部命中**（0 条失配）。
`core/state.h` 的 151 条 extern 中，变量定义都落在对应 owner .cpp，初始化值原样搬出。

## 附加证据 D：死全局量普查（Lead 要求的额外步骤）

用 `git show 99eadae` 逐条核对「声明了但全项目 0 引用」：

**已确认是历史残留并删除**（99eadae 里只有声明、或只被 SPEC §5 已删死代码使用）：

| 名字 | 位置 | 99eadae 依据 |
|---|---|---|
| `WHEEL_TO_HZ_SCALE` | core/cfg.h（删） | 仅声明 + 注释，无运算 |
| `verticalLoopGain` | control/cascade.cpp + state.h（删） | 仅声明 + 注释 |
| `COMP_ALPHA` | control/attitude.cpp + state.h（删） | 只被 `updateComplementary()` 使用（该函数按 SPEC §5 删除） |
| `R_measure` | control/attitude.cpp + state.h（删） | 仅声明（测量噪声现由 `dynamicR` 承担） |
| `telemetryMuteUntilMs` | panel/telemetry.cpp + state.h（删） | 仅声明（`prepareCommandResponse()` 原本就是空函数） |
| `btCmdCount` | panel/diag.cpp + state.h（删） | 仅声明 |
| `SPEED_ACCEL_FILTER_ALPHA`, `SPEED_DIRECT_FULL_ANGLE`, `SPEED_DIRECT_DISABLE_ANGLE`, `SPEED_DIRECT_MAX_GYRO_DPS`, `PWM_UPDATE_MIN_DELTA_HZ`, `HOLD_LAST_OUTPUT_ON_ZERO`, `ZERO_HOLD_MAX_CYCLES`, `COMP_FILTER_INIT`, `R_MID_DYNAMIC` | core/cfg.h（删） | 99eadae 中每条都只出现 1 次（即声明本身） |

**待决（未删，报 Lead 定）**：
- `motor1ZeroHoldCount` / `motor2ZeroHoldCount`：99eadae 中共 3 次出现 = 声明 + 2 次置 0（setup/倒地保护），**从未被读取**（"零保持"功能在更早重构中已移除）。属于只写不读的历史残留，但删除会动到 setup/倒地保护里的两行赋值，故保留并列入待决。

**确认仍在用、未删**：`speedLoopDirection`、`outerLoopAuthority`、`targetTurnRate`、`positionZeroCount1/2`（仅被赋值——与 original 一致）、`highSpeedAccelProtectionActive`（被赋值并在 balance 逻辑中参与判断）等。

---

## 与 SPEC 的其它一致性

- `FORMAT_VERSION` = 3；参数表 22 项、顺序 `P D O Z L S R B H K I N W J E X Y M F G T U` 与拆前**逐项相同**。
- D 帧 75 字段、G 帧 8 字段的格式串与拆前**逐字符相同**；D 帧第 19 列保留"外环时刻 `vehicleSpeed` 快照"（Lead 指示），9/10/18/20/21 列按 SPEC §5 填 0。
- 未改任何算式/限幅/阈值/时序；未烧录、未连 BLE、未碰 `software/analysis/` 与既有 docs。
- 原被预处理器关掉的 TMC2209 UART 分支（`TMC_STANDALONE_STEP_DIR=0`）同样可编译（已实测后改回 1）。

## 复现 / 证据脚本

全部只读脚本与日志在 `C:\Users\10905\.dsh-analysis\refactor-pinghengche\`：
`normalize.py`（格式修复+token 等价校验）、`cutter.py`/`assemble.py`（按函数边界搬移）、
`verify_stmts.py`（语句级覆盖审计）、`dead_globals.py`（死全局量普查）、`acceptance.py`（SPEC §6 证据生成）。

构建：
```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run -d "D:\AI项目总库\02_机器人与机械工程\PlatformIO项目\pinghengche\balance-car-showcase\software\firmware"
```
