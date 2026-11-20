#include "control/speed_est.h"

#include "control/attitude.h"
#include "control/imu.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// ============================================================
// 编码器速度
float encoderSpeed1 = 0.0f;
float encoderSpeed2 = 0.0f;
int32_t lastEncoder1Count = 0;
int32_t lastEncoder2Count = 0;

// 编码器信任门状态（见 control::tick100）
bool encoder1Trusted = true;
bool encoder2Trusted = true;
uint16_t encoder1MotionTicks = 0;
uint16_t encoder2MotionTicks = 0;
uint16_t encoderTrustWindowTicks = 0;
int rawSpeedEncoderMode = 0;  // 0=双路平均 1=仅enc1 2=仅enc2（遥测第40项）

// 编码器直接测得的车辆速度，单位：count / 50ms
float rawVehicleSpeed = 0.0f;

// 多源Kalman最终估计速度，仍统一使用 count / 50ms
float vehicleSpeed = 0.0f;
float vehicleSpeedFiltered = 0.0f;  // 速度环实际使用的低通速度（τ=SPEED_MEAS_TAU_S）

// IMU 前向加速度 → 速度预测
float imuForwardAccelG = 0.0f;
float imuForwardAccelFilteredG = 0.0f;
float imuSpeedPredictionTrust = 1.0f;
float IMU_ACCEL_TO_SPEED_GAIN = 3.0f;
float SPEED_KALMAN_Q_BASE = 0.20f;
float SPEED_KALMAN_Q_MOTOR_GAIN = 0.0008f;
float SPEED_KALMAN_R_LOW_SPEED = 0.20f;
float SPEED_KALMAN_R_NORMAL = 0.08f;
float speedKalmanP = 1.0f;
float speedKalmanQ = SPEED_KALMAN_Q_BASE;
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
bool tick400() {
  if (!readBMI270()) return false;
  updateAttitude();
  predictVehicleSpeedKalman();
  return true;
}

}  // namespace control
