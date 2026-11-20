#include "control/speed_est.h"

#include "control/attitude.h"
#include "control/imu.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// ============================================================
// 编码器速度
// encoderSpeed1/2：两路编码器速度（低通后）｜ 单位 count/50ms ｜ 写：tick100（10ms 增量×5 再 LPF α=0.4）｜
//   读：tick100 合成 rawVehicleSpeed ｜ 100Hz。
float encoderSpeed1 = 0.0f;
// encoderSpeed2：同上，编码器2 路 ｜ 单位 count/50ms ｜ 写/读：tick100 ｜ 100Hz。
float encoderSpeed2 = 0.0f;
// lastEncoder1Count/2：上一拍 tick100 的编码器快照（算 10ms 增量）｜ 单位 count ｜
//   写：tick100、main setup/倒地重同步 ｜ 读：tick100 ｜ 100Hz。
int32_t lastEncoder1Count = 0;
// lastEncoder2Count：同上，编码器2 ｜ 单位 count ｜ 写：tick100/main ｜ 读：tick100 ｜ 100Hz。
int32_t lastEncoder2Count = 0;

// 编码器信任门状态（见 control::tick100）
// encoder1Trusted/2：编码器信任标志（500ms 滑窗判定）｜ bool ｜ 写：tick100 ｜
//   读：tick100/位置保持/点动/遥测/命令守卫 ｜ 500ms 更新一次。
bool encoder1Trusted = true;
// encoder2Trusted：编码器2 信任标志 ｜ bool ｜ 写：tick100 ｜ 读：同上 ｜ 500ms 更新。
bool encoder2Trusted = true;
// encoder1MotionTicks/2：窗内该路“有脉冲”的拍数 ｜ 单位 拍（0~50）｜ 写/读：tick100 滑窗 ｜ 100Hz 累加、窗满清零。
uint16_t encoder1MotionTicks = 0;
// encoder2MotionTicks：窗内编码器2 有脉冲拍数 ｜ 单位 拍 ｜ 写/读：tick100 ｜ 100Hz。
uint16_t encoder2MotionTicks = 0;
// encoderTrustWindowTicks：信任判定窗累计拍数 ｜ 单位 拍（0~ENCODER_TRUST_WINDOW_TICKS=50）｜ 写/读：tick100 ｜ 100Hz。
uint16_t encoderTrustWindowTicks = 0;
// rawSpeedEncoderMode：当前速度来源 ｜ 0=双路平均 1=仅enc1 2=仅enc2（两路都不可信时代码仍落到 -enc2）｜
//   写：tick100 ｜ 读：位置保持/点动/遥测第40列/命令守卫 ｜ 100Hz。
int rawSpeedEncoderMode = 0;  // 0=双路平均 1=仅enc1 2=仅enc2（遥测第40项）

// 编码器直接测得的车辆速度，单位：count / 50ms
// rawVehicleSpeed：编码器直接测得车速（未融合）｜ 单位 count/50ms ｜ 符号约定：向前=正
//   （10.8 count=1mm ⇒ 1 count/50ms = 20 count/s = 1.85 mm/s）｜ 写：tick100 ｜
//   读：correctVehicleSpeedKalman（当测量）、遥测第17列、故障清零 ｜ 100Hz。
float rawVehicleSpeed = 0.0f;

// 多源Kalman最终估计速度，仍统一使用 count / 50ms
// vehicleSpeed：Kalman 融合车速（控制主量）｜ 单位 count/50ms，向前为正 ｜
//   写：predict 每 400Hz 拍、correct 每 100Hz 拍 ｜ 读：速度外环/位置点动/运动计划/遥测第16列。
float vehicleSpeed = 0.0f;
// vehicleSpeedFiltered：速度环“测量低通后”速度（历史预留）｜ 单位 count/50ms ｜
//   写：outerLoop = vehicleSpeed 直接透传 ｜ 读：当前无控制读者。
//   （注：旧描述与当前实现不符，以代码为准：τ=SPEED_MEAS_TAU_S 的低通当前未启用，见 cascade.cpp。）
float vehicleSpeedFiltered = 0.0f;  // 速度环实际使用的低通速度（τ=SPEED_MEAS_TAU_S）

// IMU 前向加速度 → 速度预测
// imuForwardAccelG：IMU 前向线加速度（去重力趋势、限幅后）｜ 单位 g，向前为正 ｜ 写：attitude 400Hz ｜
//   读：predictVehicleSpeedKalman、故障清零 ｜ 400Hz。
float imuForwardAccelG = 0.0f;
// imuForwardAccelFilteredG：前向加速度低通值（α=0.05≈τ 51ms）｜ 单位 g ｜ 写：attitude 400Hz ｜ 读：predict ｜ 400Hz。
float imuForwardAccelFilteredG = 0.0f;
// imuSpeedPredictionTrust：IMU 预测可信度 ｜ 无量纲 1.0/0.45/0.12 三档 ｜ 写：attitude 400Hz ｜ 读：predict ｜ 400Hz。
float imuSpeedPredictionTrust = 1.0f;
// IMU_ACCEL_TO_SPEED_GAIN：加速度→速度预测增益 ｜ 单位 (count/50ms)/(g·s) ｜ 3.0。
// 量纲推导：Δv[count/50ms] = a[g]·trust·GAIN·dt[s]；1g 持续 1s 只给 3 count/50ms（≈5.6mm/s），
//   远小于物理真值（1g≈9800mm/s²→≈265 count/50ms 每秒）——这一路本意只是补短期趋势，
//   绝对速度由 100Hz 编码器测量修正，故增益故意压得很低。写：无 ｜ 读：predict ｜ 400Hz。
float IMU_ACCEL_TO_SPEED_GAIN = 3.0f;
// SPEED_KALMAN_Q_BASE：过程噪声基值 Q ｜ 单位 (count/50ms)² ｜ 0.20 ｜ 写：无 ｜
//   读：predict（每拍 Q=基值+电机增益*|balanceOutput|）｜ 400Hz。
float SPEED_KALMAN_Q_BASE = 0.20f;
// SPEED_KALMAN_Q_MOTOR_GAIN：Q 随电机命令增长系数 ｜ 单位 (count/50ms)²/Hz ｜ 0.0008 ｜
//   读：predict；命令越大越允许 Kalman 状态快速变化（电机命令不被直接当速度）｜ 400Hz。
float SPEED_KALMAN_Q_MOTOR_GAIN = 0.0008f;
// SPEED_KALMAN_R_LOW_SPEED：低速测量噪声 ｜ 单位 (count/50ms)² ｜ 0.20（|编码器速度|<2 时用）｜ 读：correct ｜ 100Hz。
float SPEED_KALMAN_R_LOW_SPEED = 0.20f;
// SPEED_KALMAN_R_NORMAL：常规测量噪声 ｜ 单位 (count/50ms)² ｜ 0.08（|编码器速度|>=2 时用）｜ 读：correct ｜ 100Hz。
float SPEED_KALMAN_R_NORMAL = 0.08f;
// speedKalmanP：速度估计协方差 ｜ 单位 (count/50ms)² ｜ 初值 1.0，下限 0.02 ｜
//   写/读：predict（P+=Q·dt）、correct（K=P/(P+R)）｜ 400Hz/100Hz。
float speedKalmanP = 1.0f;
// speedKalmanQ：当前过程噪声 ｜ 单位 (count/50ms)² ｜ 每拍重算 ｜ 写/读：predict ｜ 400Hz。
float speedKalmanQ = SPEED_KALMAN_Q_BASE;
// speedKalmanR：当前测量噪声 ｜ 单位 (count/50ms)² ｜ 0.08/0.20 二选一 ｜ 写/读：correct ｜ 100Hz。
float speedKalmanR = SPEED_KALMAN_R_LOW_SPEED;

namespace control {

// ============================================================
// 多源速度Kalman
//
// 400Hz：用IMU前向线加速度做预测
// 20Hz ：用编码器速度做测量修正
//
// 电机命令不直接当“真实速度”，只用来动态调整过程噪声Q：
// 控制命令大 -> 系统更可能快速改变速度 -> Kalman更允许状态变化。
// ============================================================
// predictVehicleSpeedKalman：400Hz 预测步（IMU 加速度→速度）。无参/无返回；调用 tick400。
// 1) 取低通加速度，|a|<IMU_SPEED_ACCEL_DEADBAND_G(0.03g) 置 0（防零偏积分成假速度）；
// 2) Δv[count/50ms] = a[g]·trust·GAIN[(count/50ms)/(g·s)]·dt[s]，单拍限幅 ±0.02；
// 3) vehicleSpeed += Δv；4) Q=0.20+0.0008×|balanceOutput|，P+=Q×dt（电机命令只影响 Q）。
// 边界：trust 由 attitude 按加速度模长误差分档；P 无上限截断（靠 100Hz 修正拉回）。
void predictVehicleSpeedKalman() {
  // --------------------------------------------------------
  // 1. IMU只负责“短期趋势预测”
  //
  // 编码器负责长期速度绝对值；
  // IMU不允许长期主导vehicleSpeed。
  // --------------------------------------------------------
  float accelForSpeed = imuForwardAccelFilteredG;
  // 小加速度死区：
  // 防止0.01~0.02g级别的零偏不断积分成假速度。
  if (fabsf(accelForSpeed) < IMU_SPEED_ACCEL_DEADBAND_G) {
    accelForSpeed = 0.0f;
  }
  // 量纲：a[g] × trust[无量纲] × GAIN[(count/50ms)/(g·s)] × dt[s] → count/50ms；单拍再限幅 ±0.02。
  float imuDeltaSpeed = accelForSpeed * imuSpeedPredictionTrust * IMU_ACCEL_TO_SPEED_GAIN * CONTROL_DT;
  // 单周期预测限幅
  if (imuDeltaSpeed > MAX_IMU_SPEED_DELTA_PER_CYCLE) {
    imuDeltaSpeed = MAX_IMU_SPEED_DELTA_PER_CYCLE;
  }
  if (imuDeltaSpeed < - MAX_IMU_SPEED_DELTA_PER_CYCLE) {
    imuDeltaSpeed = - MAX_IMU_SPEED_DELTA_PER_CYCLE;
  }
  vehicleSpeed += imuDeltaSpeed;
  // --------------------------------------------------------
  // 2. 动态过程噪声Q
  //
  // 电机命令越大，系统越可能快速改变速度，
  // 因此允许Kalman状态更快变化。
  //
  // 注意：balanceOutput只调Q，不直接当作真实速度。
  // --------------------------------------------------------
  speedKalmanQ = SPEED_KALMAN_Q_BASE + SPEED_KALMAN_Q_MOTOR_GAIN * fabsf(balanceOutput);
  speedKalmanP += speedKalmanQ * CONTROL_DT;

}
// correctVehicleSpeedKalman：100Hz 测量步（编码器速度修正）。参数 encoderMeasurement 单位 count/50ms。
// 1) R=0.20（|meas|<2，低速量化明显）或 0.08（正常）；
// 2) innovation=meas-vehicleSpeed；S=P+R；K=P/S；vehicleSpeed += K·innovation；
// 3) P=(1-K)·P，下限 0.02（防 P 过小导致后续修正迟钝）。
// 调用：tick100 末尾（100Hz）｜ 边界：输入来自编码器链路（有限值）；R 越大越不信编码器。
void correctVehicleSpeedKalman(float encoderMeasurement) {
  // --------------------------------------------------------
  // 编码器作为“速度绝对值主测量”
  //
  // 低速时虽然存在0/1 count量化，
  // 但R不再设得过大，避免Kalman长期跟着IMU预测跑偏。
  // --------------------------------------------------------
  if (fabsf(encoderMeasurement) < 2.0f) {
    speedKalmanR = SPEED_KALMAN_R_LOW_SPEED;
  } else {
    speedKalmanR = SPEED_KALMAN_R_NORMAL;
  }
  float innovation = encoderMeasurement - vehicleSpeed;
  float innovationCovariance = speedKalmanP + speedKalmanR;
  float K = speedKalmanP / innovationCovariance;
  vehicleSpeed += K * innovation;
  speedKalmanP = (1.0f - K) * speedKalmanP;
  // 防止P数值过小导致后续测量修正越来越迟钝
  if (speedKalmanP < 0.02f) {
    speedKalmanP = 0.02f;
  }

}

// ============================================================
// 100Hz 段入口：编码器速度估计 + Kalman 修正
// （原 updateOuterLoop() 的函数头 + 第 1~3 步，逐行搬入）
// ============================================================
// tick100：100Hz 段入口（无参/无返回）。调用：main.cpp loop 每 4 个 400Hz 拍一次（10ms），
//   必须在 remote::tick100() 之前（远控位置/点动用 vehicleSpeed/vehiclePosition）。
// 流程：关中断快照 encoder1/2Count → Δ（10ms）→ ×5 得 count/50ms → 一阶低通 α=0.4 →
//   500ms 滑窗信任门（每拍 |Δ|>=1 count 记活跃）→ 按可信路合成 rawVehicleSpeed（向前=正）→ Kalman 修正。
// 边界：两路都不可信时仍用 -encoderSpeed2（宁可低信也不做 2 倍放大）；静止时两路都不活跃则保持旧信任结论。
void tick100() {
  int32_t count1;
  int32_t count2;
  noInterrupts();
  count1 = encoder1Count;
  count2 = encoder2Count;
  interrupts();
  // --------------------------------------------------------
  // 1. 10ms编码器增量，乘5换算为count/50ms
  // --------------------------------------------------------
  // 10ms 增量；×5 换成 count/50ms（1 count/50ms = 20 count/s = 1.85 mm/s；10.8 count=1mm）。
  int32_t delta1 = count1 - lastEncoder1Count;
  int32_t delta2 = count2 - lastEncoder2Count;
  lastEncoder1Count = count1;
  lastEncoder2Count = count2;
  encoderSpeed1 = (1.0f - ENCODER_SPEED_LPF_ALPHA) * encoderSpeed1 + ENCODER_SPEED_LPF_ALPHA * ((float) delta1 * 5.0f);
  encoderSpeed2 = (1.0f - ENCODER_SPEED_LPF_ALPHA) * encoderSpeed2 + ENCODER_SPEED_LPF_ALPHA * ((float) delta2 * 5.0f);
  // --------------------------------------------------------
  // 1b. 编码器信任门
  //
  // 整段剔除不可信的一路，而不是把下面的 -0.5f 硬改成 -1.0f。
  // 后者等价于“永远假设 enc2 = 0”，会在这路恢复正常时把车速
  // 读成真实值的 2 倍，比原来的固定偏低更危险。
  // --------------------------------------------------------
  if (delta1 >= ENCODER_TRUST_MIN_COUNTS || delta1 <= - ENCODER_TRUST_MIN_COUNTS) encoder1MotionTicks++;
  if (delta2 >= ENCODER_TRUST_MIN_COUNTS || delta2 <= - ENCODER_TRUST_MIN_COUNTS) encoder2MotionTicks++;
  encoderTrustWindowTicks++;
  if (encoderTrustWindowTicks >= ENCODER_TRUST_WINDOW_TICKS) {
    const bool encoder1Active = (encoder1MotionTicks >= ENCODER_TRUST_ACTIVE_TICKS);
    const bool encoder2Active = (encoder2MotionTicks >= ENCODER_TRUST_ACTIVE_TICKS);
    // 车本身静止时两路都不活跃，不能据此改判，保持上一次结论。
    if (encoder1Active || encoder2Active) {
      encoder1Trusted = encoder1Active;
      encoder2Trusted = encoder2Active;
    }
    encoder1MotionTicks = 0;
    encoder2MotionTicks = 0;
    encoderTrustWindowTicks = 0;
}
rawSpeedEncoderMode = (encoder1Trusted && encoder2Trusted) ? 0: (encoder1Trusted ? 1: 2);
// --------------------------------------------------------
// 2. 编码器车辆速度
//
// 当前沿用此前实测约定：
// 向前 = 正
// 向后 = 负
// --------------------------------------------------------
  // 符号：向前=正（取负号）；双路可信取平均，单路可信只取该路；单位 count/50ms。
if (encoder1Trusted && encoder2Trusted) rawVehicleSpeed = - 0.5f * (encoderSpeed1 + encoderSpeed2);
else if (encoder1Trusted) rawVehicleSpeed = - encoderSpeed1;
else rawVehicleSpeed = - encoderSpeed2;
// --------------------------------------------------------
// 3. 用编码器修正速度Kalman
//
// 此时vehicleSpeed已经由：
// 400Hz IMU预测 + 20Hz编码器修正
// 得到融合速度。
// --------------------------------------------------------
correctVehicleSpeedKalman(rawVehicleSpeed);
}

// ============================================================
// 400Hz 段入口：IMU → 姿态 → 速度预测
// ============================================================
// tick400：400Hz 段入口（无参）。返回 true=IMU 读取成功；false=失败（main 计 imuFailCount）。
// 顺序：readBMI270() → updateAttitude()（姿态/Kalman/前向加速度）→ predictVehicleSpeedKalman()。
// 调用：main.cpp loop 每 400Hz 拍 ｜ 边界：读失败直接返回 false，不跑姿态/预测（保持上一拍状态）。
bool tick400() {
  if (!readBMI270()) return false;
  updateAttitude();
  predictVehicleSpeedKalman();
  return true;
}

}  // namespace control
