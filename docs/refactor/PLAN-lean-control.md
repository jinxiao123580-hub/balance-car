# 精简重构方案：控制 / 面板分离

> 分支：`refactor/lean-split`（回滚点 `99eadae`，基线 Flash 89136B / RAM 8388B）
> 目标：把 `main.cpp` 5838 行拆成「控制核心（只留必需）+ 面板接口 + 薄调度」，删掉已停用/半死的能力。
> 原则：**删的是"非必需能力"，不是"看起来没用过的变量"**；每一项删除都要能说清"删了会失去什么"。

---

## 1. 现状诊断（依据：符号扫描 + 代码自述）

| 区块 | 现状 | 判断 |
|---|---|---|
| `^` 手动通路（manualForward/manualTurn/manual_motion.h/manual_turn.h） | 在 `updateOuterLoop` 里 **走 early-return 独占分支**（L3545–3574） | 非必需，占一整套状态机 |
| `motion_drive.h` 四件套（SpeedProfile/NetSpeed/PositionPlan/LeanControl） | **只在 manualForward 分支内使用**（L3548–3580），并在 L3929 叠加 `motionLean.applied` | manual 通路的附属，随通路一起走 |
| 位置点动 + 位置保持（position_jog.h / positionLean / updatePositionHold / X Y M） | `positionKp=0` 时整条链空转；`!` 命令才启用 | 非必需（waypoint 实验遗留） |
| 转弯环（updateTurnLoop / turnKp / turnKd / T U） | `turnOutput = manualTurn.update(...)`，无手动转向时恒 0 | 非必需 |
| 直投/预测（speedDirect* / speedPredicted / Q / A） | 代码自己注释：「完全退出控制，仅保留零值遥测以兼容历史 CSV」（L3749） | 死代码 |
| 互补滤波 `updateComplementary` | **定义后无人调用**（全文 1 次命中） | 死代码 |
| `SV` 命令 | 与参数 `K` 是同一个变量 `speedKp` | 冗余 |
| `EG` 命令 | 改 `motionLean.errorGain`，随 manual 通路一起废 | 冗余 |
| 参数表 22 项 | X/Y/M（位置）、T/U（转向）共 5 项随功能一起废 | 可精简到 17 项 |

---

## 2. 目标结构

```
software/firmware/src/
├─ main.cpp                 调度 + 保护（setup / loop / 倒地 / 掉拍 / IMU 失效）
├─ cfg.h                    常量、引脚、周期、阈值（替代散落在 5000 行里的 const）
├─ state.h                  全局状态 extern 声明（参数/控制量/诊断量分区）
├─ util.h                   clampf / 有限性检查（替代 ManualTurnControl::clamp）
├─ control/
│  ├─ imu.cpp/.h            BMI270 读写 + 400Hz 配置 + 开机校准
│  ├─ attitude.cpp/.h       姿态 Kalman（→ pitch / pitchRate）
│  ├─ speed_est.cpp/.h      编码器累计 + 速度 Kalman 预测/修正（→ vehicleSpeed/Position）
│  ├─ balance.cpp/.h        直立 PD（→ finalBalanceOutput）
│  ├─ cascade.cpp/.h        速度环 P（+可选 I/D）→ speedAngleCorrection；外环权限
│  └─ motor.cpp/.h          STEP 频率输出、零保持、换向软斜坡、编码器 ISR
└─ panel/
   ├─ params.cpp/.h         17 项参数表 + SAVE/LOAD/DEF + 三层值语义
   ├─ cmd.cpp/.h            BLE 命令解析（17 单字母 + SP/MS + SAVE/LOAD/DEF/?/!）
   ├─ telemetry.cpp/.h      G 帧（4Hz）+ D 帧（1Hz，字段数保持兼容）
   └─ diag.cpp/.h           计数器/窗口极值（掉拍、IMU 失败、倒停次数、耗时）
```

**依赖方向（硬约束）**：`panel → control → cfg/state/util`，**control 不许引用 panel**。
`main.cpp` 只做编排：`setup()` 初始化顺序 + `loop()` 节拍。

---

## 3. 保留清单（控制必需）

- BMI270 采样、姿态 Kalman（pitch / pitchRate）
- 编码器 ISR、速度 Kalman 预测(400Hz)/修正(100Hz)
- 直立 PD：`P D`、输出限幅 `L`、速率 `S`、加速度整形 `R B H`、死区 `Z`
- 速度环：`K`（+ `I W J` 保留但默认 0）、目标角限幅 `N`、斜率限制、`SP` 指令
- 外环权限：`F G`（大角度平滑退出，保护必需）
- 保护：IMU 失效 8 拍、倒地 75°、掉拍 ≥2 拍 → 停电机 + 清状态
- 电机输出：`setMotor1/2`、`commandStepOutput`、零保持、换向软斜坡
- 速度量程 `MS`（2026-11-20 新增的运行时旋钮，保留）

## 4. 删除清单（附"失去什么"）

| 删除项 | 失去的能力 | 影响面 |
|---|---|---|
| `^` 手动俯仰/转向通路 | 不能再手动给倾角/转向调试 | `manual_motion.h`、`manual_turn.h` 主体、`parseManualMotion` |
| `motion_drive.h` 四件套 | manual 通路的速度斜坡/净速度/位置计划/倾角控制 | 整个头文件删除 |
| 位置点动 + 位置保持 | `!` 点动、`JOGCFG`、位置回位 | `position_jog.h`、X/Y/M、`updatePositionHold` |
| 转弯环 | 差速转向（`turnOutput` 恒 0） | T/U、`updateTurnLoop` |
| 直投/预测残留 | 无（本就不参与控制） | `speedDirect*`、`Q`、`A`、相关零值字段 |
| 互补滤波 | 姿态只剩 Kalman（本来就只用 Kalman） | `updateComplementary` |
| `EG` / `SV` 命令 | `EG` 随 manual 一起废；`SV` 与 `K` 重复 | 命令解析、PAR 回显 |

## 5. 参数表 22 → 17

保留：`P D O Z L S R B H K I N W J E` + `F G` = **17 项**
删除：`X Y M`（位置）、`T U`（转向）

**副作用**：参数表变了 → `ParamStore::FORMAT_VERSION` 必须 3 → 4，
**闪存里的旧 22 项记录会作废**，下次烧录后 `O` 回落到源码默认值 → 需要重测平衡点（约 2 分钟，工具现成）。
若不想作废记录，可保留 22 槽位、把 5 项当占位（不参与控制）——但那就没达到"精简"目的。

## 6. 面板侧同步

- `PAR:` 回显改为 17 项 + `SAVED/SP/MS`；`SV`/`EG` 移除
- `D` 帧：**字段序号保持不变**，被删功能的字段填 0（避免网页/CSV 解析错位）
- `G` 帧不变（`G,millis,pos,speed,pitch,target,output,manual→0,btRx`）
- `software/ble-panel/ble_tune_panel.py` 的参数列表同步到 17 项

## 7. 验收（缺一不可）

1. `pio run` 通过，Flash/RAM 数值记录（预期 Flash 明显下降）；
2. **符号扫描**：`manualForward|manualTurn|motionLean|positionLean|speedDirect|updateComplementary|turnOutput` 在新代码中 0 命中；
3. **依赖检查**：`grep -r "panel" control/` 为 0（控制不依赖面板）；
4. **行为核对**：`SP` 路径算式与现在**逐行等价**（`speedAngleUnclamped = speedLoopDirection*(P+I+D)`，限幅 `N`，斜率限制后叠加到 `targetBalanceAngle`）；
5. 上机冒烟（用户在场时）：`?` 回显、`SP 8` 点动方向、倒电保护，全部正常。

## 8. 风险与回滚

- 风险：拆文件后 `extern` 声明漏项、ISR 与全局量的可见性、`FORMAT_VERSION` 失效导致 O 丢失。
- 回滚：`git checkout 99eadae -- software/firmware/src` 或整分支切回。
- **不烧录**：本次重构只到"编译通过 + 静态验收"，烧录要用户另行确认。
