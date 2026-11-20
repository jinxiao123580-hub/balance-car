# 规格 v2：控制 / 远控 / 面板 三模块分离

> 分支 `refactor/lean-split`，回滚点 `99eadae`（Flash 89136B / RAM 8388B 构建通过）
> **本版与 v1 的关键差别：不删功能，只做物理隔离。** 手动调试整体搬进「远控模块」；
> 参数表保持 22 项不变 → `FORMAT_VERSION` 仍为 3 → 闪存记录继续有效 → `O` 不会丢。
> 唯一删的是**已有代码自己承认的死代码**（见 §5）。

---

## 1. 目标文件树

```
src/
├─ main.cpp                      仅编排：setup 初始化顺序 + loop 节拍（目标 ≤300 行）
├─ core/
│  ├─ cfg.h                      常量集中：周期/阈值/引脚/限幅（只放 #define 与 constexpr）
│  ├─ state.h                    全局状态的 extern 声明（参数、控制量、面板量分区）
│  └─ util.h                     clampf()、有限性检查（替代 ManualTurnControl::clamp）
├─ control/                      ★ 控制核心：只含闭环必需
│  ├─ imu.{h,cpp}                BMI270 读写 / 400Hz 配置 / 开机校准
│  ├─ attitude.{h,cpp}           姿态 Kalman → pitch / pitchRate
│  ├─ speed_est.{h,cpp}          编码器累计 + 速度 Kalman 预测(400Hz)/修正(100Hz)
│  ├─ balance.{h,cpp}            直立 PD → finalBalanceOutput（含限幅/速率/加速整形）
│  ├─ cascade.{h,cpp}            速度环 P(+I/D) → speedAngleCorrection；外环权限
│  └─ motor.{h,cpp}              STEP 频率输出/零保持/换向软斜坡 + 编码器 ISR
├─ remote/                       ★ 新增：手动调试与远控（原 `^` 通路全家）
│  ├─ manual_pitch.{h,cpp}       ← manual_motion.h 的 ManualPitchControl + parseManualMotion
│  ├─ manual_turn.{h,cpp}        ← manual_turn.h 的转向状态机
│  ├─ motion_drive.{h,cpp}       ← motion_drive.h 四件套（斜坡/净速度/位置计划/倾角）
│  ├─ position_jog.{h,cpp}       ← position_jog.h
│  └─ remote_control.{h,cpp}     ^ 通路调度、jog 调度、EG 增益、→ leanOffset/turnOffset
└─ panel/                        ★ 面板与协议（参数表也在这里）
   ├─ params.{h,cpp}             22 项参数表 + SAVE/LOAD/DEF + 三层值语义
   ├─ cmd.{h,cpp}                BLE 命令解析（单字母22 + SP/MS/SV + SAVE/LOAD/DEF/?/! + ^）
   ├─ telemetry.{h,cpp}          G 帧(4Hz) / D 帧(1Hz，字段序号不变)
   └─ diag.{h,cpp}               计数器与窗口极值
```

## 2. 依赖方向（硬约束，验收会查）

```
main.cpp ──► panel ──► remote ──► control ──► core
main.cpp ──► control / remote 直接调用也允许
❌ control 不得 include remote/ 或 panel/ 的任何头文件
❌ remote  不得 include panel/
```
如果控制核心需要"手动倾角"，只能用**数据接口**：`remote::leanOffset()`、`remote::turnOffset()`、`remote::isActive()`。

## 3. 接口契约（必须按这个签名实现）

```c
// control：输入指令 + 状态查询，不含任何 BLE/字符串
namespace control {
  void begin();                       // 原 setup 中属于控制的部分
  void setSpeedCmd(float v);          // 原 speedCmd（SP）
  void setMaxSpeed(float v);          // 原 MAX_TARGET_FORWARD_SPEED（MS）
  void setLeanOffset(float deg, bool active); // 远控叠加项（替代 motionLean.applied）
  void tick400();                     // IMU→姿态→速度预测→权限→直立→电机合成
  void tick100();                     // 外环：修正→运动驱动→速度PI→目标角
  bool imuOk(); bool fallen();
  float pitch(); float pitchRate(); float speed(); float position();
  float targetAngle(); float outputHz();
}

// remote：只做手动/远控，不碰电机、不碰姿态
namespace remote {
  void begin();
  bool handleLine(const char* line, unsigned len);  // ^ / jog 相关命令
  void tick(float dt);                              // 按 100Hz/400Hz 由 main 调
  bool isActive();                                  // 等价 manualMotionWasBusy
  float leanOffset();                               // 等价 motionLean.applied
  float turnOffset();                               // 等价 turnOutput
  void setErrorGain(float v); float errorGain();
}

// panel：命令/遥测/参数
namespace panel {
  void begin();
  void poll();                        // 收行 → 分发（参数写 / SP / MS / 远控 / SAVE…）
  void sendFast(); void sendSlow();
  void flush();
}
```

`main.cpp` 的 400Hz 顺序必须与现在**逐条对齐**：
`poll→flush→节拍闸门→掉拍处理→IMU→姿态→速度预测→遥测分频→倒地保护→远控tick→位置保持→÷4 外环→权限→直立→转向→电机合成→遥测窗口→flush`。

## 4. 参数（22 项不动，放 `panel/params.*`）

`P D O Z L S R B H K I N W J E X Y M F G T U` 全部保留，`FORMAT_VERSION` **保持 3**，
闪存记录不作废（`SAVED=1` 继续有效，O 不丢）。参数被 control 与 remote 共用时，走 `state.h` 的 extern。

## 5. 只删这四处死代码（代码自己已注释为退出控制）

1. `updateComplementary()` —— 定义后无人调用（全文仅 1 次命中）。
2. `speedDirect*` 系列（`speedDirectOutput/speedDirectUnclampedOutput/speedDirectAuthority/speedDirectKp`）—— 代码注释：「完全退出控制，仅保留零值遥测以兼容历史 CSV」。
3. `speedPredicted/speedPredictionDelta/speedAccelerationFiltered/speedPredictPrevSpeed/speedPredictHorizon` —— 同上，恒零。
4. `PAR` 回显里的 `Q=`/`A=` 两个死旋钮字段（`D` 帧字段序号**不动**，只把对应位置继续填 0）。

## 6. 验收（逐条都要给硬证据）

1. `pio run` 成功，记录 Flash/RAM（基线 89136 / 8388）；
2. 依赖检查：`control/` 内 grep `include ".*(panel|remote)/` = 0；`remote/` 内 grep `include ".*panel/` = 0；
3. 死代码检查：`updateComplementary|speedDirect|speedPredicted` 在 `src/` 下 0 命中；
4. 功能未减：`^` 手动俯仰/转向、`!` 位置点动、`SP`/`MS`、`SAVE/LOAD/DEF/?` 的**入口与语义**都在（由 panel→remote→control 转发）；
5. `loop()` 400Hz 调用顺序与 §3 一致（贴出 main.cpp 的 loop 代码供比对）；
6. 给出最终文件树与每个文件行数；`main.cpp` ≤ 350 行。

## 7. 禁止事项

- 不改行为语义（算式、限幅、保护阈值、时序一个都不许动）；
- 不烧录、不连 BLE、不碰 `software/analysis/`、不改 `docs/`（除交付报告）；
- 不预先"顺手优化"任何控制参数或控制器结构。
