#include "control/attitude.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// ============================================================
// 姿态
float accPitch = 0.0f;
float pitch = 0.0f;
float gyroPitchRate = 0.0f;
float gyroTurnRate = 0.0f;
float gyroIntegAngle = 0.0f;  // 真实KF输入角（去掉kalmanBias后的gyro积分），400Hz累计

// Kalman
float kalmanAngle = 0.0f;
float compAngle = 0.0f;  // 互补滤波角度状态(取代 kalmanAngle)
// ===== Pitch 毛刺抑制 =====
float pitchPrev = 0.0f;  // 上一拍有效 pitch
float kalmanBias = 0.0f;
float P00 = 1.0f;
float P01 = 0.0f;
float P10 = 0.0f;
float P11 = 1.0f;
float Q_angle = 0.01f;
float Q_bias = 0.03f;
// ===== 互补滤波(取代 Kalman 姿态融合) =====
float COMP_ALPHA = 0.99f;  // 互补滤波权重: angle = a*(angle+gyro*dt)+(1-a)*accPitch
float R_measure = 0.03f;

// 加速度模长 / 动态测量噪声
float accelMagnitude = 1.0f;
float accelMagnitudeError = 0.0f;
float dynamicR = 0.03f;

namespace control {

// ============================================================
// 加速度Pitch
// 竖装BMI270：X = 上下
// Z = 前后；Y = 俯仰转轴
// ============================================================
float calculateAccelPitch() {
  return atan2f(- az, - ax) * 180.0f / PI;

}
// ============================================================
// Kalman
// ============================================================
float updateKalman(float measuredAngle, float measuredRate, float dt) {
  // --------------------------------------------------------
  // 1. 状态预测
  // --------------------------------------------------------
  float rate = measuredRate - kalmanBias;
  kalmanAngle += dt * rate;
  // --------------------------------------------------------
  // 2. 协方差预测
  // --------------------------------------------------------
  P00 += dt * (dt * P11 - P01 - P10 + Q_angle);
  P01 -= dt * P11;
  P10 -= dt * P11;
  P11 += Q_bias * dt;
  // --------------------------------------------------------
  // 3. 测量创新
  // --------------------------------------------------------
  float innovation = measuredAngle - kalmanAngle;
  // --------------------------------------------------------
  // 动态测量噪声
  //
  // 车辆静止/低动态时 dynamicR ≈ 0.03
  // 车辆加速或振动明显时 dynamicR 增大，
  // 从而暂时减少加速度角对Kalman的修正。
  // --------------------------------------------------------
  float S = P00 + dynamicR;
  // --------------------------------------------------------
  // 4. Kalman增益
  // --------------------------------------------------------
  float K0 = P00 / S;
  float K1 = P10 / S;
  // --------------------------------------------------------
  // 5. 状态修正
  // --------------------------------------------------------
  kalmanAngle += K0 * innovation;
  kalmanBias += K1 * innovation;
  // --------------------------------------------------------
  // 6. 协方差修正
  // --------------------------------------------------------
  float oldP00 = P00;
  float oldP01 = P01;
  P00 -= K0 * oldP00;
  P01 -= K0 * oldP01;
  P10 -= K1 * oldP00;
  P11 -= K1 * oldP01;
  return kalmanAngle;

}

// ============================================================
// 姿态更新
// ============================================================
void updateAttitude() {
  // --------------------------------------------------------
  // 1. 原始加速度角
  // --------------------------------------------------------
  accPitch = calculateAccelPitch() - pitchZero;
  // --------------------------------------------------------
  // 2. Pitch轴陀螺仪角速度
  // --------------------------------------------------------
  gyroPitchRate = GY_DIRECTION * (gy - gyroYZero);
  // --------------------------------------------------------
  // 3. 计算三轴加速度模长
  //
  // SparkFun BMI270库已将加速度换算为g。
  // 静止时理论上接近1g。
  // --------------------------------------------------------
  accelMagnitude = sqrtf(ax * ax + ay * ay + az * az);
  accelMagnitudeError = fabsf(accelMagnitude - 1.0f);
  // --------------------------------------------------------
  // 4. 根据 |a|-1g 动态调整R
  //
  // 越偏离1g，说明线性加速度/振动越明显，
  // 这时accPitch可信度降低。
  // --------------------------------------------------------
  // 连续插值：加速度模长误差 0 -> ACC_ERR_HIGH 平滑映射 R
  // 静止/低速：R小(0.02)，acc权重高，抑制陀螺零偏积分漂移
  // 高速/振动：R大(1.20)，acc权重低，防止线性加速度污染角度
  // 双判据：(1) 加速度模长误差（振动/线性加速度）(2) 方向分歧 innovation（毛刺）
  // 真实运动：accPitch 与 gyroInt 同步变化 → innovRatio 小 → 不影响
  // accPitch 单拍毛刺：gyroInt 不动 → innovRatio 大 → dynamicR 拉满 → 只信陀螺
  float errRatio = accelMagnitudeError / ACC_ERR_HIGH;
  if (errRatio > 1.0f) {
    errRatio = 1.0f;
  }
  float innovRatio = fabsf(accPitch - gyroIntegAngle) / ACC_INNOV_HIGH;
  if (innovRatio > 1.0f) {
    innovRatio = 1.0f;
  }
  float combinedRatio = (errRatio > innovRatio) ? errRatio: innovRatio;
  dynamicR = R_LOW_DYNAMIC + (R_HIGH_DYNAMIC - R_LOW_DYNAMIC) * combinedRatio;
  // --------------------------------------------------------
  // 5. Kalman融合
  // --------------------------------------------------------
  pitch = updateKalman(accPitch, gyroPitchRate, CONTROL_DT);
  // ---- Pitch 毛刺抑制：单步突变>±5° 判为IMU/卡尔曼毛刺，用上一有效值替代 ----
  // 用步长差而非绝对限幅：车正常还要走大角度(±10°+)，绝对限幅会砍正常反馈；
  // 而 400Hz 下单步>5° 是倒摆+电机也到不了的物理上限，只会掐真毛刺，不动正常响应。
  {
    float dPitch = pitch - pitchPrev;
    if (dPitch > PITCH_GLITCH_DEG) pitch = pitchPrev;
    else if (dPitch < - PITCH_GLITCH_DEG) pitch = pitchPrev;
    else pitchPrev = pitch;
  }
  // 真实KF输入角 = gyroPitchRate 去掉 kalmanBias 后在 400Hz 积分。
  // 只发这个积分角，电脑端不再做 5Hz 快照积分，根治面板紫线漂移。
  gyroIntegAngle += (gyroPitchRate - kalmanBias) * CONTROL_DT;
  // --------------------------------------------------------
  // 6. 根据Pitch去除Z轴重力分量，得到前向线加速度趋势
  //
  // 静止时：
  // az ≈ -sin(pitch)
  //
  // 因此：
  // az + sin(pitch) ≈ 0
  //
  // 再乘方向参数统一为“向前加速为正”。
  // --------------------------------------------------------
  float pitchRad = pitch * DEG_TO_RAD;
  imuForwardAccelG = IMU_FORWARD_ACCEL_DIRECTION * (az + sinf(pitchRad));
  // 尖峰限幅
  if (imuForwardAccelG > IMU_ACCEL_CLAMP_G) {
    imuForwardAccelG = IMU_ACCEL_CLAMP_G;
  }
  if (imuForwardAccelG < - IMU_ACCEL_CLAMP_G) {
    imuForwardAccelG = - IMU_ACCEL_CLAMP_G;
  }
  // 轻度低通
  imuForwardAccelFilteredG = (1.0f - IMU_ACCEL_LPF_ALPHA) * imuForwardAccelFilteredG + IMU_ACCEL_LPF_ALPHA * imuForwardAccelG;
  // 根据加速度模长偏离1g的程度降低IMU预测可信度
  if (accelMagnitudeError < ACC_ERR_LOW) {
    imuSpeedPredictionTrust = 1.0f;
  } else if (accelMagnitudeError < ACC_ERR_HIGH) {
    imuSpeedPredictionTrust = 0.45f;
  } else {
    imuSpeedPredictionTrust = 0.12f;
  }
  // --------------------------------------------------------
  // 7. 转向角速度
  // --------------------------------------------------------
  gyroTurnRate = TURN_GYRO_DIRECTION * (gx - gyroXZero);

}

}  // namespace control
