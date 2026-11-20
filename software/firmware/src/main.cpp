// ============================================================
// main.cpp — 固件唯一入口：setup() 一次性初始化 + loop() 400Hz 硬实时调度。
// 分层：core(常量/全局状态) → control(控制环) → remote(远控/手动通路) → panel(蓝牙命令/遥测)；
// 本文件只做"顺序编排 + 失败兜底"，控制算式全部在 control/ 与 remote/ 里，main 不写算式。
//
// 时间预算（依据 core/cfg.h，详表见 docs/code-walkthrough/P1-setup-loop.md）：
//   控制周期 2500us（CONTROL_PERIOD_US=2500）→ 400Hz；
//   掉拍判据 ≥2 拍（≥5000us）→ 停电机 + 重基准；
//   IMU 读取失败 8 拍 ≈20ms 才确认故障；倒地确认 8 拍 ≈20ms；
//   外环 ÷4 → 100Hz；G 帧 ÷100 → 4Hz；D 帧 ÷400 → 1Hz；蓝牙 9600 波特 ≈960B/s。
//
// 铁律（三条异常路径同一套路）：任何异常都先 stopBothMotors()，再清控制状态，最后 return。
// ============================================================
#include <Arduino.h>
#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"
#include "control/motor.h"
#include "control/imu.h"
#include "control/attitude.h"
#include "control/speed_est.h"
#include "control/balance.h"
#include "control/cascade.h"
#include "remote/remote_control.h"
#include "remote/manual_pitch.h"
#include "remote/manual_turn.h"
#include "remote/motion_drive.h"
#include "remote/position_jog.h"
#include "panel/params.h"
#include "panel/cmd.h"
#include "panel/telemetry.h"
#include "panel/diag.h"
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <HardwareTimer.h>
#include <math.h>

// clearControlOutputDiagnostics — 停机/故障路径的统一"清故障"入口。
// 作用：先复位远控状态机（转向/手动俯仰/点动/倾角/速度参考），再清控制侧输出与积分。
// 参数：无。返回：无。调用者：只有 main 自己的异常分支（setup 重试、loop 掉拍/IMU 故障/倒地），
//   事件级调用（每次故障一次），不是周期函数。边界：只清状态、不碰电机——调用方必须先 stopBothMotors()。
// 实际停机时的统一清故障：远控侧复位 → 控制侧复位（原 clearControlOutputDiagnostics 的
// 两段，顺序与原函数一致：先清远控状态机，再清控制输出与积分）。
static void clearControlOutputDiagnostics() {
  remote::resetFault();
  control::clearOutputDiagnostics();
}

// ============================================================
// 调度
// ============================================================
// lastControlUs：400Hz 控制节拍的时间基准 = 上一次"控制拍"的 micros() 时刻。
// 单位 us；范围 0..2^32-1（micros() 约 71.6 分钟回绕一次，判据必须写成无符号差）。
// 写：setup() 末尾赋初值；loop() 每拍 += CONTROL_PERIOD_US（固定步长，长期平均 400Hz 不漂移），
//   只有掉拍时才重基准 = nowUs。读：loop() 每圈开头的闸门判据。更新频率：400Hz。
uint32_t lastControlUs = 0;
// outerLoopDivider：100Hz 外环分频计数器，取值 0..3。
// 无单位；每个 400Hz 控制拍 +1，到 4 归零并执行一次外环（÷4 → 100Hz）。
// 写：loop() 每拍自增/归零；掉拍与倒地保护时强制清 0（重新对齐相位）。读：仅 loop() 的分频判断。
uint32_t outerLoopDivider = 0;

// ============================================================
// 非阻塞遥测 txBuffer（9600 波特很慢，直接 print 会阻塞 400Hz 控制循环）
// 方案：sendTelemetry 只把整行写入环形缓冲；loop 末尾按 800B/s 预算 flush。
// 预算不足时丢整行（不部分发送）。
// ============================================================
// ============================================================
// setup() — 一次性初始化。顺序本身就是安全策略，不是随手排的：
//   ① 执行器先进安全态（电机停、编码器中断就绪）——后面再慢也不会自己动；
//   ② 通信通道先通（蓝牙 9600）——后续每个失败分支都要靠 BTSerial 打印诊断；
//   ③ 参数先就位（flash 读回）——flash 擦除约 1.1s，绝不能落到控制环运行中；
//   ④ IMU 三步握手：能通信 → 能跑到 400Hz → 静止校准，任一步失败就停电机重试；
//   ⑤ 编码器基准快照（临界区）；
//   ⑥ 控制状态清零（位置/速度/Kalman/外环/权限，targetBalanceAngle 直接 = 平衡点 O）；
//   ⑦ 时间基准 lastControlUs 最后设定。
// 任何失败分支统一"停电机 → 清故障 → 延时重试"，绝不带着未知状态进 loop()。
// ============================================================
void setup() {
  // ① 执行器优先：先把 STEP/DIR/EN 引脚与定时器配置成"停"的安全态。
  // 上电瞬间这些引脚电平不确定，晚一步就可能被噪声驱动而自走。
  control::initMotors();
  // 编码器中断尽早挂上：它只依赖引脚中断、不会失败；早挂可让初始化期间的转动也被计数，
  // 后面的 lastEncoder/positionZero 快照只重置"基准"，不影响计数连续性。
  control::initEncoders();
  // --------------------------------------------------------
  // YahBoom_BL UART5
  // --------------------------------------------------------
  // ② 通信通道先通：panel::begin() 内含 BTSerial.begin(9600) 与上电等待。
  // 后面 TMC/IMU 每个失败分支都要 BTSerial.println 报错——日志通道必须先于它们可用，
  // 否则一旦卡在重试死循环里就完全无法诊断。
  panel::begin();

// ③ TMC2209 三种编译模式（由 core/cfg.h 的 TMC_STANDALONE_STEP_DIR 选择）：
//    STANDALONE：UART 不接，细分/电流/模式由板上拨码与 VREF 硬件决定 → 只打印一条说明；
//    WRITE_ONLY：只写不读（硬件无读回），失败重试且明确打印 UNVERIFIED，不谎报成功；
//    UART 读回：真正读回寄存器确认 spreadCycle/1-16 细分，失败就停电机死循环重试。
// 当前编译配置走 STANDALONE（cfg.h 定义 TMC_STANDALONE_STEP_DIR=1），后两个分支不参与编译。
#if TMC_STANDALONE_STEP_DIR
  // STEP/DIR 由原有 TIM1/TIM3 和 DIR 逻辑驱动，不访问 PA9/PA2。
  // MKS 的 EN(低有效)、VREF 电流、MS1/MS2 细分和 SPREAD 模式由硬件决定。
  // 本板型：模式/电流/细分全部硬件固定，固件侧不做任何 UART 配置，仅打一条开机说明。
  BTSerial.println("TMC STEP/DIR standalone: UART disabled; mode/current/microsteps hardware-selected");
#elif TMC_GPIO_WRITE_ONLY
  // WRITE_ONLY 分支：每轮重试前先停电机（配置未知时绝不带电流输出），失败 1s 后重试；
  // 打印的 rearLineHigh/frontLineHigh 是写完后检查半双工线空闲电平（应为高）。
  control::stopBothMotors();
  bool rearLineHigh = false, frontLineHigh = false;
  while (!control::initTmcDriversWriteOnly(rearLineHigh, frontLineHigh)) {
    control::stopBothMotors();
    BTSerial.print("TMC WRITE-ONLY WAIT rearLineHigh=");
    BTSerial.print(rearLineHigh);
    BTSerial.print(" frontLineHigh=");
    BTSerial.println(frontLineHigh);
    delay(1000);
  }
  // （注：旧描述"sendTelemetry … loop 末尾按 800B/s 预算 flush" 与当前实现不符，以代码为准：
  //   发送函数是 panel::sendSlow()/sendFast()；flushTelemetryTx() 在 loop 首尾各调用一次，
  //   预算按经过时间累加（0.8 B/ms、上限 40 B），不足时整行丢弃不部分发送。）
  // txCount 只代表"已写出的字节数"，不代表驱动器已接受（该模式无读回），故末尾标 UNVERIFIED。
  BTSerial.print("TMC WRITE-ONLY SENT rearBytes=");
  BTSerial.print(control::rearTmcUart.txCount);
  BTSerial.print(" frontBytes=");
  BTSerial.print(control::frontTmcUart.txCount);
  BTSerial.println(" spreadCycle=REQUESTED current=1200mA microsteps=16 UNVERIFIED");
#else
  // UART 读回分支（当前未参与编译）：每次上电重写后轮 spreadCycle + 前轮配置，失败就停电机重试。
  // TMC2209 spreadCycle 配置(后轮)+PA9, 前轮+PA2；每次上电重写
  while (!control::initTmcDrivers()) {
    control::stopBothMotors();
    delay(1000);
  }
  BTSerial.println("TMC UART OK: both spreadCycle, 1/16 step");
#endif
  // --------------------------------------------------------
  // BMI270初始化
  // --------------------------------------------------------
  // 22 项参数：先抄编译默认值，能读到 flash 记录就覆盖（2026-11-10）
  // ③ 参数先就位：paramsBoot() 先抄编译默认值，再用 flash 记录覆盖（22 项面板参数）。
  // 必须在 IMU/控制之前：① flash 满时这里会预擦另一扇区（约 1.1s），绝不能落在控制环运行中；
  // ② O/N/K 等参数是校验与全部控制算式的输入，晚读一秒就有一秒用错参数。
  panel::paramsBoot();
  // ④ IMU 三步握手之①：能通信（读到 chip id）。失败分支统一"停电机 + 清故障 + 1s 重试"。
  while (!control::initBMI270()) {
    // 兜底：重试期间电机保持停止、旧故障状态清零；1s 间隔给传感器上电稳定留时间。
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    BTSerial.println("BOOT RETRY: BMI270 init failed");
    delay(1000);
  }
  // 初始化成功后才把 I2C 提到 400kHz：先用保险的低速把设备唤起；
  // 400Hz × 6 字节/次 ≈ 需要 24kB/s 净吞吐，100kHz 会挤占实时控制时间。
  BMIWire.setClock(400000);
  // 等总线/传感器在 400kHz 下稳定 20ms 再继续配置。
  delay(20);
  // --------------------------------------------------------
  // 配置400Hz
  // --------------------------------------------------------
  // ④ IMU 三步握手之②：把 ODR/量程/滤波真正写成 400Hz。
  // "能读到数据" ≠ "数据按 400Hz 更新"：这一步失败会让 Kalman 时间常数整体错位，故单独校验 + 重试。
  // 重试间隔 500ms（比初始化短）：传感器已在线，通常只是寄存器写时序问题。
  while (!control::configureBMI270_400Hz()) {
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    BTSerial.println("BOOT RETRY: BMI270 400Hz config failed");
    delay(500);
  }
  // --------------------------------------------------------
  // 校准
  // --------------------------------------------------------
  // ④ IMU 三步握手之③：静止校准（俯仰零位 + 陀螺零偏）。
  // 必须车静止；失败会一直重试并提示 keep car still，绝不跳过——零偏错会直接变成直立环的常值输出。
  while (!control::calibrateIMU()) {
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    BTSerial.println("BOOT RETRY: IMU calibration failed; keep car still");
    delay(1000);
  }
  // 三步握手全部通过：从此刻起 IMU 的 pitch/角速度才可信，后面才允许进入控制状态清零点。
  BTSerial.println("BOOT OK: BMI270 ready");
  // --------------------------------------------------------
  // 编码器初值
  // --------------------------------------------------------
  // ⑤ 编码器基准快照：encoder1Count/2 是 32 位、由 ISR 写入，读它与"另一处写"之间可能被中断打断，
  // 导致 last 与 zero 来自不同时刻 → 首拍速度出现尖峰。关中断只包住这 4 条赋值（微秒级），临界区最小化。
  // positionZeroCount* 记录开机基准计数（当前只写不读，作为历史/诊断字段保留）；
  // 位置坐标零点由 positionOdom 首次 update 时以当时计数建立，因此 vehiclePosition 从 0 起算。
  noInterrupts();
  lastEncoder1Count = encoder1Count;
  lastEncoder2Count = encoder2Count;
  positionZeroCount1 = encoder1Count;
  positionZeroCount2 = encoder2Count;
  interrupts();
  // 位置/点动状态清零：位置坐标从 0 起、目标=当前、无位置保持历史；
  // 保持历史没有物理意义（上电前的位置不可知），X 位置环启用时会重新 capture 目标。
  vehiclePosition = 0.0f;
  targetVehiclePosition = 0.0f;
  positionError = 0.0f;
  positionSpeedTarget = 0.0f;
  positionHoldWasEnabled = false;
  // --------------------------------------------------------
  // 速度Kalman + 串级速度PI初始化
  // --------------------------------------------------------
  // ⑥ 速度估计与控制状态清零：
  //   vehicleSpeed/rawVehicleSpeed/前向加速度 = 0（没有历史速度）；
  //   imuSpeedPredictionTrust = 1（预测器满信任启动）；
  //   speedKalmanP = 1（协方差从"不确定"开始）、Q = 基础值、R = 低速值（低速更信编码器）。
  // 这一组不清，首拍就会用上电前的残影算速度、算倾角。
  vehicleSpeed = 0.0f;
  rawVehicleSpeed = 0.0f;
  imuForwardAccelG = 0.0f;
  imuForwardAccelFilteredG = 0.0f;
  imuSpeedPredictionTrust = 1.0f;
  speedKalmanP = 1.0f;
  speedKalmanQ = SPEED_KALMAN_Q_BASE;
  speedKalmanR = SPEED_KALMAN_R_LOW_SPEED;
  speedError = 0.0f;
  // 外环历史清零：速度误差/积分/微分、最终输出、目标前向速度全归零；
  // 目标角直接设成平衡点 O（不是 0°）——否则 pitch≈O 时首拍会朝 0° 猛拽一次；
  // 外环权限上电给满（1.0），之后由 updateAuthority() 按倾角动态压回。
  finalBalanceOutput = 0.0f;
  targetForwardSpeed = 0.0f;
  targetBalanceAngle = balanceAngleOffset;
  outerLoopDivider = 0;
  telemetryDivider = 0;
  imuFailCount = 0;
  fallConfirmCount = 0;
  outerLoopAuthority = 1.0f;
  // 网页会忽略这一行表头
  // 给网页/上位机画图用的 CSV 表头（固件自己不解析，网页端会忽略这一行）。
  BTSerial.println("time_ms,fused_speed,speed_direct_hz");
  // ⑦ 时间基准必须最后一句：若提前设置，setup 里的 delay(1000)/重试会被 loop 首圈当成"巨大掉拍"
  // （≥2 拍）而触发一次 overrun 停机；放这里 = 首拍从"现在"重新计时，
  // 且首圈还要等满一个 2.5ms 周期才进入控制，不会在初始化未完成时抢跑。
  lastControlUs = micros();

}

// ============================================================
// loop() — 400Hz 硬实时调度器（每拍 2500us），顺序固定，不能随手换：
//   ① 命令/发送（不受节拍限制）—— 低延迟，且不占控制拍
//   ② 节拍闸门（未到 2.5ms 直接 return）
//   ③ 掉拍自救（≥2 拍 → 停电机 + 重基准）
//   ④ 读 IMU + 8 拍分级降级（<8 保持输出，≥8 停机清状态）
//   ⑤ 400Hz 估计：姿态 Kalman + IMU 速度预测
//   ⑥ 遥测（先发数据再做保护，保证倒地时仍可观测）
//   ⑦ 倒地保护（75°，停机 + 全状态清零）
//   ⑧ 远控 400Hz：手动俯仰平滑 + 位置里程/驻车目标
//   ⑨ 外环 ÷4（100Hz）：点动 → 运动驱动 → 位置 P → 速度 PI
//   ⑩ 外环权限（400Hz）→ ⑪ 直立 PD → ⑫ 转向 → ⑬ 电机合成 → ⑭ 诊断/发送/耗时
// 关键理由都在各段注释里；顺序错一处（例如转向放到直立之前）就会用到旧的控制量。
// ============================================================
// ============================================================
// loop
// 固定400Hz
// ============================================================
// ============================================================
// 蓝牙在线调参（在 loop 每圈开头调用，不影响400Hz控制时序）
// 命令格式：一行一个，回车发送。 字母 + 空格 + 数值
//   P <kp>    改 balanceKp
//   D <kd>    改 balanceKd
//   O <offset> 改 balanceAngleOffset（平衡点）
//   Z <deg>   改 BALANCE_ANGLE_DEADBAND_DEG（死区）
//   L <hz>    改 BALANCE_OUTPUT_LIMIT_HZ（输出限幅）
//   S <hz>    改 BALANCE_OUTPUT_SLEW_HZ（每周期最大变化）
//   ?         回显当前全部参数
//   !         驱动诊断；! kp,limit 配置位置倾角点动
// ============================================================
// ============================================================
// 参数持久化（2026-11-10）：蓝牙 SAVE / LOAD / DEF
// ------------------------------------------------------------
// 记录格式与槽位扫描见 param_store.h，扇区读写见 param_flash_f4.h（sector 10/11）。
// 这里只做三件事：把 22 个全局参数登记成表、开机读回、处理三个整词命令。
// 为什么不再依赖 ST-Link：整定一次要重烧一次，现场根本跟不上调参节奏。
// ============================================================
// 顺序必须与 PAR 回显、面板 22 项一致；改顺序或语义要同步升 ParamStore::FORMAT_VERSION
// 整词命令：只有"整条命令恰好是一个词"才算命中，避免和 P/D/O… 单字母命令抢
// （注：上面「蓝牙在线调参」那段旧命令说明与当前实现不符，以代码为准：现行协议见 panel/cmd.cpp ——
//   22 项单字母参数 P/D/O/Z/L/S/R/B/H/K/I/N/W/J/E/X/Y/M/F/G/T/U（可 SAVE/LOAD/DEF），
//   两字母运行参数 SP/SV/EG/MS，整词 SAVE/LOAD/DEF，`?` 回显、`!` 诊断与 `! kp,limit` 点动配置；
//   `^ turnHz,pitchDeg` 手动通路（forward 已是"速度档位"，不再直接写 ±2° 死倾角）；
//   `~id,delta` 位置点动、`&id` 心跳。）
void loop() {
  // ① 命令与发送：放在节拍闸门之前——命令处理不能等 2.5ms 窗口，否则 SP/O 调参会累积几十毫秒抖动；
  // flushTelemetryTx 按 800B/s 预算分批发送，避免 9600 波特阻塞控制。
  // 注意：这里第一次 micros() 的取值随后被覆盖（等价于多读一次，属历史遗留，保留原样）。
  uint32_t nowUs = micros();
  // 蓝牙在线调参：每圈都轮询，独立于400Hz时序
  panel::poll();
  panel::flushTelemetryTx();
  // 真正的计时起点：命令/发送耗掉的时间不进入 400Hz 闸门判据。
  nowUs = micros();
  // --------------------------------------------------------
  // 没到2.5ms就直接返回
  // --------------------------------------------------------
  // ② 闸门：未到 2.5ms 就直接返回（loop 不是调度器，靠主循环尽快回转）。
  // 判据用 (uint32_t)(nowUs - lastControlUs) 无符号差，兼容 micros() 约 71.6 分钟回绕。
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) {
    return;
  }
  // --------------------------------------------------------
  // 保持固定时间基准
  // --------------------------------------------------------
  // ③ 掉拍自救：先记录最大间隔（诊断用），再区分"正常"与"掉拍"两条路径。
  uint32_t controlGapUs = nowUs - lastControlUs;
  if (controlGapUs > controlMaxGapUs) controlMaxGapUs = controlGapUs;
  // 掉拍判据：间隔 ≥2 拍（≥5000us）。这一拍的 IMU 采样间隔已经不对，继续用力等于用旧姿态做决策，
  // 所以宁可停一拍：停电机、清故障、外环分频器归零，并以 nowUs 重基准。
  // 注意：正常时用 lastControlUs += PERIOD（固定步长，长期平均 400Hz 不漂移），
  // 只有掉拍才重基准——两种处理理由不同，不要合并。
  if (controlGapUs >= CONTROL_PERIOD_US * 2UL) {
    controlOverrunCount++;
    lastControlUs = nowUs;
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    outerLoopDivider = 0;
  } else {
    lastControlUs += CONTROL_PERIOD_US;
  }
  // 单拍耗时起点：统计 controlMaxWorkUs（接近 2500us 说明 400Hz 已跑不动）。
  uint32_t controlWorkStartUs = micros();
  // --------------------------------------------------------
  // 读取BMI270
  // --------------------------------------------------------
  // ④ 读 IMU：分级降级，不是一刀切。
  if (!control::readBMI270()) {
    imuFailCount++;
    // 4a) 连续失败 <8 拍（≈20ms）：直接 return，保持上一拍的 STEP 输出不停。
    // 步进输出是定时器频率，return 不会断电；一次 I2C 抖动不该让车身顿一下。
    if (imuFailCount < IMU_FAIL_CONFIRM_COUNT) {
      // 保持上一有效STEP输出
      return;
    }
    // 4b) 连续失败 ≥8 拍：判定真故障。停电机 + 清故障 + 清掉所有会累积的状态
    // （积分/微分历史/位置目标/速度目标），防止故障恢复瞬间用旧积分猛冲。
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    balanceOutput = 0.0f;
    finalBalanceOutput = 0.0f;
    speedIntegral = 0.0f;
    speedAngleCorrection = 0.0f;
    speedErrorPrev = 0.0f;
    speedErrorDotFilt = 0.0f;
    positionSpeedTarget = 0.0f;
    targetForwardSpeed = 0.0f;
    return;
  }
  // 读成功即清零失败计数（计数器只在"连续失败"时累加）。
  imuFailCount = 0;
  // --------------------------------------------------------
  // 控制段首：Kalman姿态 + 多源速度Kalman预测 400Hz
  // （IMU负责快速感知“开始加速”的趋势；编码器会在外环20Hz时做绝对速度修正）
  // （注：旧描述"编码器会在外环20Hz时做绝对速度修正"与当前实现不符，以代码为准：
  //   编码器绝对速度修正在 100Hz 外环（400Hz÷4）的 tick100() 里做。）
  // --------------------------------------------------------
  // ⑤ 400Hz 估计：tick400() = 读 IMU → 姿态 Kalman → 用 IMU 前向加速度预测车速。
  // 400Hz 只做"快预测"；编码器的绝对速度修正放在 100Hz 外环的 tick100() 里做，
  // 即"快预测 + 慢修正"（详见 control/speed_est.cpp）。
  control::tick400();
  // --------------------------------------------------------
  // 蓝牙遥测20Hz
  // 放在倒地保护之前，这样即使触发倒地保护，
  // 手机上仍能继续看到IMU/Kalman数据。
  // --------------------------------------------------------
  // ⑥ 遥测：D 帧 ÷400 → 1Hz，G 帧 ÷100 → 4Hz（分频常数在 core/cfg.h）。
  // 放在倒地保护之前：这样即使刚触发倒地保护，手机上仍能继续看到 IMU/Kalman 数据。
  // （注：旧注释"蓝牙遥测20Hz"与当前实现不符，以代码为准：TELEMETRY_DIVIDER=400→D 帧 1Hz，
  //   FAST_TELEMETRY_DIVIDER=100→G 帧 4Hz。）
  telemetryDivider++;
  if (telemetryDivider >= TELEMETRY_DIVIDER) {
    telemetryDivider = 0;
    panel::sendSlow();
  }
  // G 帧（快速帧）：只发量化速度/姿态需要的 7 个量，避免挤占 9600 波特的命令通道。
  fastTelemetryDivider++;
  if (fastTelemetryDivider >= FAST_TELEMETRY_DIVIDER) {
    fastTelemetryDivider = 0;
    panel::sendFast();
  }
  // --------------------------------------------------------
  // 倒地保护（连续确认，避免单帧尖峰突然停机）
  // --------------------------------------------------------
  // ⑦ 倒地保护：角度误差 = |pitch − 平衡点 O|（deg），两道判据：
  //   确认判据：≥FALL_ANGLE_DEG 连续 8 拍（≈20ms）→ confirmedFall；
  //   紧急判据：≥EMERGENCY_FALL_ANGLE_DEG 单拍立即 → emergencyFall。
  float fallAngleError = fabsf(pitch - balanceAngleOffset);
  if (fallAngleError >= FALL_ANGLE_DEG) {
    fallConfirmCount++;
  } else {
    fallConfirmCount = 0;
  }
  bool confirmedFall = fallConfirmCount >= FALL_CONFIRM_COUNT;
  bool emergencyFall = fallAngleError >= EMERGENCY_FALL_ANGLE_DEG;
  // 触发即"停电机 + 清故障 + 全状态清零"：倒地后积分/微分/位置/速度全部失去物理意义，
  // 不清的话扶起来第一秒就会用旧账算出大输出 → 二次翻车。
  // （注：旧注释"连续确认，避免单帧尖峰突然停机"与当前实现不符，以代码为准：
  //   FALL_ANGLE_DEG 与 EMERGENCY_FALL_ANGLE_DEG 目前都是 75°，紧急通道单拍即成立，
  //   把 8 拍确认短路了；若要让去抖真正生效，应把 EMERGENCY 调到明显更大（如 120°）。）
  if (confirmedFall || emergencyFall) {
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    fallStopCount++;  // 倒地保护触发计数：250ms遥测窗无法分辨20ms确认窗内的短促误触发
    balanceOutput = 0.0f;
    turnOutput = 0.0f;
    speedError = 0.0f;
    speedIntegral = 0.0f;
    speedAngleCorrection = 0.0f;
    speedErrorPrev = 0.0f;
    speedErrorDotFilt = 0.0f;
    finalBalanceOutput = 0.0f;
    outerLoopAuthority = 0.0f;
    targetForwardSpeed = 0.0f;
    // 车被拿起/挪动过：重新快照编码器并把位置原点重设到当前，累计位置作废；
    // 同时外环权限归零、目标角回平衡点，防止站起来瞬间被旧速度目标拖走。
    noInterrupts();
    int32_t resetCount1 = encoder1Count;
    int32_t resetCount2 = encoder2Count;
    interrupts();
    lastEncoder1Count = resetCount1;
    lastEncoder2Count = resetCount2;
    positionZeroCount1 = resetCount1;
    positionZeroCount2 = resetCount2;
    vehiclePosition = 0.0f;
    positionError = 0.0f;
    positionSpeedTarget = 0.0f;
    positionHoldWasEnabled = false;
    vehicleSpeed = 0.0f;
    rawVehicleSpeed = 0.0f;
    imuForwardAccelG = 0.0f;
    imuForwardAccelFilteredG = 0.0f;
    speedKalmanP = 1.0f;
    targetBalanceAngle = balanceAngleOffset;
    return;
  }
  // --------------------------------------------------------
  // 先在400Hz读取累计位置，20Hz位置外环使用最新位置。
  // ⑧ 远控 400Hz：手动俯仰目标平滑（^ 通路）+ 位置里程/驻车目标维护。
  // 位置必须在这里、外环之前更新：100Hz 外环要用最新位置。
  // （注：旧注释"20Hz位置外环"与当前实现不符，以代码为准：外环已是 100Hz（÷4）。）
  remote::tick(CONTROL_DT);
  remote::positionHold();
  // 速度PI外环100Hz（成熟步进平衡车串级结构）
  //
  // 400Hz控制循环，每4次执行一次 = 10ms。
  // --------------------------------------------------------
  // ⑨ 外环 ÷4 → 100Hz（每 4 个 400Hz 拍执行一次 = 10ms）：
  //   先 tick100()：编码器速度估计 + Kalman 绝对速度修正；
  //   再 remote::tick100()：点动/运动驱动/位置 P 外环的调度器；
  //   若运动驱动接管本拍（^ 前后运动），本拍跳过 control::outerLoop()，
  //   —— 由运动环直接给倾角，避免速度 PI 与运动环抢同一个目标角。
  outerLoopDivider++;
  if (outerLoopDivider >= 4) {
    outerLoopDivider = 0;
    control::tick100();      // 编码器速度估计 + Kalman 修正
    bool jogPos = false;
    bool jogAngle = false;
    // 远控 100Hz 调度顺序：点动 → 运动驱动(接管则本拍返回) → 位置 P 外环 → jog 倾角 → 速度参考限速。
    // 返回 true 表示运动驱动接管：此时速度 PI 必须跳过（防两套目标角打架）。
    if (!remote::tick100(jogPos, jogAngle)) {      // 远控运动驱动优先时接管本拍
      control::outerLoop(jogPos, jogAngle, remote::motionWasBusy(),
                          remote::manualPitchBusy(), remote::jogLeanOffset());
    }
    // 速度环被整体关闭（如 K=0）时控制侧会置位 leanResetRequested：
    // loop 在这里立即转发 remote::resetLean()，把 jog 倾角清掉，避免残留倾角继续推车。
    if (leanResetRequested) {                      // 速度环整体关闭 → 立刻清 jog 倾角
      remote::resetLean();
      leanResetRequested = false;
    }
  }
  // --------------------------------------------------------
  // 外环权限管理400Hz
  // 大角度时速度/位置环平滑退出，直立PD优先
  // --------------------------------------------------------
  // ⑩ 外环权限（400Hz）：按倾角大小把速度/位置环的发言权在 0..1 间平滑压回，
  // 输入 normalAuthority()=手动俯仰释放斜坡、leanOffset()=运动/jog 倾角；
  // 大角度时外环先退出，保证直立 PD 独占目标角（"直立优先"）。
  // 必须在跑完 100Hz 外环之后：要用本拍最新算出的 leanOffset/normalAuthority。
  control::updateAuthority(remote::normalAuthority(), remote::leanOffset());
  // --------------------------------------------------------
  // 位置已在外环计算前更新
  // --------------------------------------------------------
  // remote::positionHold() 已在本周期前面执行
  // --------------------------------------------------------
  // 直立环400Hz
  // --------------------------------------------------------
  // ⑪ 直立环（400Hz）：PD 消费 updateAuthority() 刚定好的 targetBalanceAngle，
  // 输出 finalBalanceOutput（STEP Hz）；死区/限幅/换向软斜坡/加速度整形细节在 control/balance.cpp。
  control::updateBalance();
  // --------------------------------------------------------
  // 转向环400Hz
  // --------------------------------------------------------
  // ⑫ 转向环（400Hz）：legacy PD + 转向状态机 → turnOutput（差模 STEP Hz）。
  // 必须放在直立环之后：转向状态机要先看 finalBalanceOutput 给共模平衡留了多少余量，
  // 再决定转向差模上限（"先保护共模平衡输出，绝不单独钳每只轮子"）。
  remote::updateTurn();
  // --------------------------------------------------------
  // 电机合成
  // --------------------------------------------------------
  // ⑬ 电机合成：m1 = finalBalanceOutput + turnOutput，m2 = finalBalanceOutput − turnOutput。
  // 最后一步才落成两轮 STEP 频率：前面的量都只是"目标"，mixMotors 内含零保持与换向处理。
  control::mixMotors(remote::turnOffset());
  // ---- 250ms 遥测窗口极值累积（必须在所有控制量算完后）----
  // ⑭ 收尾：250ms 遥测窗口极值（必须在所有控制量算完后）+ 非阻塞发送 + 单拍耗时统计。
  // controlMaxWorkUs 是实时性红线：接近 2500us 就跑不动 400Hz 了。
  panel::diagAccumulate();
  // 非阻塞遥测 flush（每控制周期按预算发送缓冲内容）
  panel::flushTelemetryTx();
  uint32_t workUs = micros() - controlWorkStartUs;
  if (workUs > controlMaxWorkUs) controlMaxWorkUs = workUs;
}
