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

// 实际停机时的统一清故障：远控侧复位 → 控制侧复位（原 clearControlOutputDiagnostics 的
// 两段，顺序与原函数一致：先清远控状态机，再清控制输出与积分）。
static void clearControlOutputDiagnostics() {
  remote::resetFault();
  control::clearOutputDiagnostics();
}

// ============================================================
// 调度
// ============================================================
uint32_t lastControlUs = 0;
uint32_t outerLoopDivider = 0;

// ============================================================
// 非阻塞遥测 txBuffer（9600 波特很慢，直接 print 会阻塞 400Hz 控制循环）
// 方案：sendTelemetry 只把整行写入环形缓冲；loop 末尾按 800B/s 预算 flush。
// 预算不足时丢整行（不部分发送）。
// ============================================================
void setup() {
  control::initMotors();
  control::initEncoders();
  // --------------------------------------------------------
  // YahBoom_BL UART5
  // --------------------------------------------------------
  panel::begin();

#if TMC_STANDALONE_STEP_DIR
  // STEP/DIR 由原有 TIM1/TIM3 和 DIR 逻辑驱动，不访问 PA9/PA2。
  // MKS 的 EN(低有效)、VREF 电流、MS1/MS2 细分和 SPREAD 模式由硬件决定。
  BTSerial.println("TMC STEP/DIR standalone: UART disabled; mode/current/microsteps hardware-selected");
#elif TMC_GPIO_WRITE_ONLY
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
  BTSerial.print("TMC WRITE-ONLY SENT rearBytes=");
  BTSerial.print(control::rearTmcUart.txCount);
  BTSerial.print(" frontBytes=");
  BTSerial.print(control::frontTmcUart.txCount);
  BTSerial.println(" spreadCycle=REQUESTED current=1200mA microsteps=16 UNVERIFIED");
#else
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
  panel::paramsBoot();
  while (!control::initBMI270()) {
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    BTSerial.println("BOOT RETRY: BMI270 init failed");
    delay(1000);
  }
  BMIWire.setClock(400000);
  delay(20);
  // --------------------------------------------------------
  // 配置400Hz
  // --------------------------------------------------------
  while (!control::configureBMI270_400Hz()) {
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    BTSerial.println("BOOT RETRY: BMI270 400Hz config failed");
    delay(500);
  }
  // --------------------------------------------------------
  // 校准
  // --------------------------------------------------------
  while (!control::calibrateIMU()) {
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    BTSerial.println("BOOT RETRY: IMU calibration failed; keep car still");
    delay(1000);
  }
  BTSerial.println("BOOT OK: BMI270 ready");
  // --------------------------------------------------------
  // 编码器初值
  // --------------------------------------------------------
  noInterrupts();
  lastEncoder1Count = encoder1Count;
  lastEncoder2Count = encoder2Count;
  positionZeroCount1 = encoder1Count;
  positionZeroCount2 = encoder2Count;
  interrupts();
  vehiclePosition = 0.0f;
  targetVehiclePosition = 0.0f;
  positionError = 0.0f;
  positionSpeedTarget = 0.0f;
  positionHoldWasEnabled = false;
  // --------------------------------------------------------
  // 速度Kalman + 串级速度PI初始化
  // --------------------------------------------------------
  vehicleSpeed = 0.0f;
  rawVehicleSpeed = 0.0f;
  imuForwardAccelG = 0.0f;
  imuForwardAccelFilteredG = 0.0f;
  imuSpeedPredictionTrust = 1.0f;
  speedKalmanP = 1.0f;
  speedKalmanQ = SPEED_KALMAN_Q_BASE;
  speedKalmanR = SPEED_KALMAN_R_LOW_SPEED;
  speedError = 0.0f;
  finalBalanceOutput = 0.0f;
  targetForwardSpeed = 0.0f;
  targetBalanceAngle = balanceAngleOffset;
  outerLoopDivider = 0;
  telemetryDivider = 0;
  imuFailCount = 0;
  fallConfirmCount = 0;
  outerLoopAuthority = 1.0f;
  // 网页会忽略这一行表头
  BTSerial.println("time_ms,fused_speed,speed_direct_hz");
  lastControlUs = micros();

}

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
void loop() {
  uint32_t nowUs = micros();
  // 蓝牙在线调参：每圈都轮询，独立于400Hz时序
  panel::poll();
  panel::flushTelemetryTx();
  nowUs = micros();
  // --------------------------------------------------------
  // 没到2.5ms就直接返回
  // --------------------------------------------------------
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) {
    return;
  }
  // --------------------------------------------------------
  // 保持固定时间基准
  // --------------------------------------------------------
  uint32_t controlGapUs = nowUs - lastControlUs;
  if (controlGapUs > controlMaxGapUs) controlMaxGapUs = controlGapUs;
  if (controlGapUs >= CONTROL_PERIOD_US * 2UL) {
    controlOverrunCount++;
    lastControlUs = nowUs;
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    outerLoopDivider = 0;
  } else {
    lastControlUs += CONTROL_PERIOD_US;
  }
  uint32_t controlWorkStartUs = micros();
  // --------------------------------------------------------
  // 读取BMI270
  // --------------------------------------------------------
  if (!control::readBMI270()) {
    imuFailCount++;
    if (imuFailCount < IMU_FAIL_CONFIRM_COUNT) {
      // 保持上一有效STEP输出
      return;
    }
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
  imuFailCount = 0;
  // --------------------------------------------------------
  // 控制段首：Kalman姿态 + 多源速度Kalman预测 400Hz
  // （IMU负责快速感知“开始加速”的趋势；编码器会在外环20Hz时做绝对速度修正）
  // --------------------------------------------------------
  control::tick400();
  // --------------------------------------------------------
  // 蓝牙遥测20Hz
  // 放在倒地保护之前，这样即使触发倒地保护，
  // 手机上仍能继续看到IMU/Kalman数据。
  // --------------------------------------------------------
  telemetryDivider++;
  if (telemetryDivider >= TELEMETRY_DIVIDER) {
    telemetryDivider = 0;
    panel::sendSlow();
  }
  fastTelemetryDivider++;
  if (fastTelemetryDivider >= FAST_TELEMETRY_DIVIDER) {
    fastTelemetryDivider = 0;
    panel::sendFast();
  }
  // --------------------------------------------------------
  // 倒地保护（连续确认，避免单帧尖峰突然停机）
  // --------------------------------------------------------
  float fallAngleError = fabsf(pitch - balanceAngleOffset);
  if (fallAngleError >= FALL_ANGLE_DEG) {
    fallConfirmCount++;
  } else {
    fallConfirmCount = 0;
  }
  bool confirmedFall = fallConfirmCount >= FALL_CONFIRM_COUNT;
  bool emergencyFall = fallAngleError >= EMERGENCY_FALL_ANGLE_DEG;
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
  remote::tick(CONTROL_DT);
  remote::positionHold();
  // 速度PI外环100Hz（成熟步进平衡车串级结构）
  //
  // 400Hz控制循环，每4次执行一次 = 10ms。
  // --------------------------------------------------------
  outerLoopDivider++;
  if (outerLoopDivider >= 4) {
    outerLoopDivider = 0;
    control::tick100();      // 编码器速度估计 + Kalman 修正
    bool jogPos = false;
    bool jogAngle = false;
    if (!remote::tick100(jogPos, jogAngle)) {      // 远控运动驱动优先时接管本拍
      control::outerLoop(jogPos, jogAngle, remote::motionWasBusy(),
                          remote::manualPitchBusy(), remote::jogLeanOffset());
    }
    if (leanResetRequested) {                      // 速度环整体关闭 → 立刻清 jog 倾角
      remote::resetLean();
      leanResetRequested = false;
    }
  }
  // --------------------------------------------------------
  // 外环权限管理400Hz
  // 大角度时速度/位置环平滑退出，直立PD优先
  // --------------------------------------------------------
  control::updateAuthority(remote::normalAuthority(), remote::leanOffset());
  // --------------------------------------------------------
  // 位置已在外环计算前更新
  // --------------------------------------------------------
  // remote::positionHold() 已在本周期前面执行
  // --------------------------------------------------------
  // 直立环400Hz
  // --------------------------------------------------------
  control::updateBalance();
  // --------------------------------------------------------
  // 转向环400Hz
  // --------------------------------------------------------
  remote::updateTurn();
  // --------------------------------------------------------
  // 电机合成
  // --------------------------------------------------------
  control::mixMotors(remote::turnOffset());
  // ---- 250ms 遥测窗口极值累积（必须在所有控制量算完后）----
  panel::diagAccumulate();
  // 非阻塞遥测 flush（每控制周期按预算发送缓冲内容）
  panel::flushTelemetryTx();
  uint32_t workUs = micros() - controlWorkStartUs;
  if (workUs > controlMaxWorkUs) controlMaxWorkUs = workUs;
}
