#include "control/attitude.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// ============================================================
// 姿态
// accPitch：加速度计解算倾角（已减 pitchZero）｜ 单位 deg ｜ 写：updateAttitude 400Hz ｜
//   读：updateKalman 当测量、遥测第28列、diag 窗口。
float accPitch = 0.0f;
// pitch：Kalman 融合倾角（控制主量，含毛刺抑制）｜ 单位 deg ｜ 写：updateAttitude 400Hz ｜
//   读：直立环/外环/倒地判据/手动通路/遥测 ｜ 400Hz。
float pitch = 0.0f;
// gyroPitchRate：俯仰角速度（已减零偏、已按 GY_DIRECTION 定向）｜ 单位 deg/s ｜
//   写：updateAttitude 400Hz ｜ 读：直立 D 项、updateKalman、遥测/diag ｜ 400Hz。
float gyroPitchRate = 0.0f;
// gyroTurnRate：转向角速度（gx-gyroXZero，乘 TURN_GYRO_DIRECTION）｜ 单位 deg/s ｜
//   写：updateAttitude 400Hz ｜ 读：remote::updateTurn 的 legacy PD、遥测 ｜ 400Hz。
float gyroTurnRate = 0.0f;
// gyroIntegAngle：陀螺输入角（去 kalmanBias 后 400Hz 积分的独立副本）｜ 单位 deg ｜
//   写：updateAttitude 每拍 +=(gyroPitchRate-kalmanBias)*dt ｜ 读：updateAttitude 算 innovation 分歧、遥测第30列 ｜ 400Hz。
float gyroIntegAngle = 0.0f;  // 真实KF输入角（去掉kalmanBias后的gyro积分），400Hz累计

// Kalman
// kalmanAngle：Kalman 内部角度状态（updateKalman 的直接返回值）｜ 单位 deg ｜ 写：updateKalman/calibrateIMU ｜
//   读：updateKalman 自身；控制使用毛刺抑制后的 pitch ｜ 400Hz。
float kalmanAngle = 0.0f;
// compAngle：互补滤波角度残留状态 ｜ 单位 deg ｜ 写：calibrateIMU 初始化 ｜ 读：无（互补滤波已删除，仅历史保留）。
float compAngle = 0.0f;  // 互补滤波角度状态(取代 kalmanAngle)
// （注：上一条“互补滤波角度状态(取代 kalmanAngle)”与当前实现不符：姿态融合实际使用 Kalman，
//   compAngle 只在 calibrateIMU 里赋初值，控制链不再读取；保留变量仅为历史兼容。）
// ===== Pitch 毛刺抑制 =====
// pitchPrev：上一拍有效 pitch（毛刺抑制基准）｜ 单位 deg ｜ 写/读：updateAttitude 毛刺判定 ｜ 400Hz。
float pitchPrev = 0.0f;  // 上一拍有效 pitch
// kalmanBias：Kalman 估计的陀螺零偏 ｜ 单位 deg/s ｜ 写：updateKalman 每拍 +=K1*innovation ｜
//   读：updateAttitude（积分角与分歧）、遥测第29列、diag 窗口 ｜ 400Hz。
float kalmanBias = 0.0f;
// P00..P11：Kalman 协方差矩阵 ｜ P00 单位 deg²、P01/P10 单位 deg·(deg/s)、P11 单位 (deg/s)² ｜
//   初值 [[1,0],[0,1]]（calibrateIMU 重置）｜ 写/读：updateKalman 的预测+修正 ｜ 400Hz。
float P00 = 1.0f;
// P01：角度-零偏协方差 ｜ 单位 deg·(deg/s) ｜ 初值 0 ｜ 写/读：updateKalman ｜ 400Hz。
float P01 = 0.0f;
// P10：零偏-角度协方差 ｜ 单位 (deg/s)·deg ｜ 初值 0 ｜ 写/读：updateKalman ｜ 400Hz。
float P10 = 0.0f;
// P11：零偏估计方差 ｜ 单位 (deg/s)² ｜ 初值 1.0 ｜ 写/读：updateKalman ｜ 400Hz。
float P11 = 1.0f;
// Q_angle：角度过程噪声 ｜ 单位 deg²/s ｜ 0.01 ｜ 写：无（常量）｜ 读：updateKalman 协方差预测 ｜ 400Hz。
float Q_angle = 0.01f;
// Q_bias：零偏过程噪声 ｜ 单位 (deg/s)²/s ｜ 0.03 ｜ 写：无（常量）｜ 读：updateKalman ｜ 400Hz。
float Q_bias = 0.03f;
// （注：旧描述与当前实现不符，以代码为准：姿态融合一直是 Kalman（updateKalman），互补滤波函数已按 SPEC §5 删除；
//   此标题为历史残留，其下的 accelMagnitude/accelMagnitudeError/dynamicR 服务于 Kalman 的动态测量噪声。）
// ===== 互补滤波(取代 Kalman 姿态融合) =====

// 加速度模长 / 动态测量噪声
// accelMagnitude：三轴加速度模长 ｜ 单位 g（静止≈1.0）｜ 写：updateAttitude ｜ 读：算 accelMagnitudeError ｜ 400Hz。
float accelMagnitude = 1.0f;
// accelMagnitudeError：||a|-1g| ｜ 单位 g ｜ 写：updateAttitude ｜ 读：dynamicR 插值、IMU 预测可信度分档、遥测第42列 ｜ 400Hz。
float accelMagnitudeError = 0.0f;
// dynamicR：加速度计角度的动态测量噪声 ｜ 单位 deg² ｜ 0.02~1.20 线性插值｜ 写：updateAttitude 400Hz ｜
//   读：updateKalman 用 S=P00+dynamicR 算增益；遥测第41列。
float dynamicR = 0.03f;

namespace control {

// ============================================================
// 加速度Pitch
// 竖装BMI270：X = 上下
// Z = 前后；Y = 俯仰转轴
// ============================================================
// calculateAccelPitch：atan2(-az,-ax) 求重力方向角 ｜ 无参 ｜ 返回 deg。
// 轴向：竖装 BMI270 → X≈上下、Z≈前后、Y=俯仰转轴；重力在 X-Z 平面的分量决定倾角。
// 调用：updateAttitude 每 400Hz 拍一次；calibrateIMU 采初始角时也调用 ｜ 边界：无除法、无 NaN（atan2(0,0)=0）。
float calculateAccelPitch() {
  return atan2f(- az, - ax) * 180.0f / PI;

}
// ============================================================
// Kalman
// ============================================================
// updateKalman：二阶 Kalman 融合一拍（角度 + 陀螺零偏两个状态）。
// 参数：measuredAngle=加速度角 deg；measuredRate=陀螺角速度 deg/s；dt=周期 s。返回 kalmanAngle（deg）。
// 步骤：①状态预测 angle+=dt*(rate-bias) ②协方差预测 P（Q_angle/Q_bias 控制发散速度）
//   ③创新 innovation=measuredAngle-angle ④S=P00+dynamicR、K=[P00/S,P10/S]
//   ⑤状态修正 ⑥协方差修正（标准 Joseph 化简版）。
// 调用：updateAttitude 每 400Hz 拍 ｜ 边界：dynamicR 必须由调用方先算好；
//   P 矩阵量纲：P00 deg²、P01/P10 deg·deg/s、P11 (deg/s)²，deg 与 deg/s 不混用。
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
  // S：创新方差 = P00[deg²] + dynamicR[deg²]（R 越大越不信加速度计）。
  float S = P00 + dynamicR;
  // --------------------------------------------------------
  // 4. Kalman增益
  // --------------------------------------------------------
  // Kalman 增益 K=[P00/S, P10/S]，单位无量纲（deg²/deg²、deg·dps/deg²）。
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
// updateAttitude：400Hz 姿态链总入口（无参/无返回）。
// ① accPitch=加速度角-pitchZero；② gyroPitchRate=GY_DIRECTION*(gy-gyroYZero)；
// ③ 模长/误差；④ dynamicR：errRatio 与 innovRatio 取大者线性插值 R_LOW~R_HIGH；
// ⑤ pitch=updateKalman(accPitch,gyroPitchRate,CONTROL_DT)；⑥ 单步 |Δ|>5° 毛刺用 pitchPrev 顶替；
// ⑦ gyroIntegAngle 独立积分（去 bias）；⑧ imuForwardAccelG=dir*(az+sin(pitch))，限幅±0.60g 后低通；
// ⑨ 按 accelMagnitudeError 分档 imuSpeedPredictionTrust（1.0/0.45/0.12）；⑩ gyroTurnRate。
// 调用：control::tick400() 每 400Hz 拍一次 ｜ 边界：所有输出为有限值；毛刺抑制只动 pitch，不改 Kalman 内部状态。
void updateAttitude() {
  // --------------------------------------------------------
  // 1. 原始加速度角
  // --------------------------------------------------------
  // 加速度角：先解算再减固定安装偏置 pitchZero（不减开机姿态，见 cfg.h 注释）。
  accPitch = calculateAccelPitch() - pitchZero;
  // --------------------------------------------------------
  // 2. Pitch轴陀螺仪角速度
  // --------------------------------------------------------
  // 俯仰角速度：减开机零偏后乘方向系数（gy 已在 readBMI270 翻转过一次）。
  gyroPitchRate = GY_DIRECTION * (gy - gyroYZero);
  // --------------------------------------------------------
  // 3. 计算三轴加速度模长
  //
  // SparkFun BMI270库已将加速度换算为g。
  // 静止时理论上接近1g。
  // --------------------------------------------------------
  // 模长：SparkFun 库输出单位已是 g，静止≈1.0g。
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
  // errRatio：|a|-1g 归一化到 [0,1]（ACC_ERR_HIGH=0.10g 对应 1.0）——振动/线性加速度判据。
  float errRatio = accelMagnitudeError / ACC_ERR_HIGH;
  if (errRatio > 1.0f) {
    errRatio = 1.0f;
  }
  // innovRatio：|accPitch-gyroIntegAngle| 归一化到 [0,1]（20° 对应 1.0）——单拍毛刺判据。
  // 依据：真实运动时两路同步 → 比值小；accPitch 单拍跳而陀螺积分不动 → 比值大。
  float innovRatio = fabsf(accPitch - gyroIntegAngle) / ACC_INNOV_HIGH;
  if (innovRatio > 1.0f) {
    innovRatio = 1.0f;
  }
  // 取两判据的较大者，插值出 dynamicR：静止 0.02（信 acc）、振动/毛刺 1.20（信陀螺）。
  float combinedRatio = (errRatio > innovRatio) ? errRatio: innovRatio;
  dynamicR = R_LOW_DYNAMIC + (R_HIGH_DYNAMIC - R_LOW_DYNAMIC) * combinedRatio;
  // --------------------------------------------------------
  // 5. Kalman融合
  // --------------------------------------------------------
  // Kalman 融合（dt=CONTROL_DT=2.5ms）；返回的是无毛刺抑制的 kalmanAngle。
  pitch = updateKalman(accPitch, gyroPitchRate, CONTROL_DT);
  // ---- Pitch 毛刺抑制：单步突变>±5° 判为IMU/卡尔曼毛刺，用上一有效值替代 ----
  // 用步长差而非绝对限幅：车正常还要走大角度(±10°+)，绝对限幅会砍正常反馈；
  // 而 400Hz 下单步>5° 是倒摆+电机也到不了的物理上限，只会掐真毛刺，不动正常响应。
  {
  // 毛刺抑制：单步变化超过 ±PITCH_GLITCH_DEG(5°) 判为毛刺，本拍沿用 pitchPrev；
  //   正常大角度运动不受影响（400Hz 下单车真实单步角位移远小于 5°）。
    float dPitch = pitch - pitchPrev;
    if (dPitch > PITCH_GLITCH_DEG) pitch = pitchPrev;
    else if (dPitch < - PITCH_GLITCH_DEG) pitch = pitchPrev;
    else pitchPrev = pitch;
  }
  // 真实KF输入角 = gyroPitchRate 去掉 kalmanBias 后在 400Hz 积分。
  // 只发这个积分角，电脑端不再做 5Hz 快照积分，根治面板紫线漂移。
  // 真实 KF 输入角的独立积分：只发这个角给上位机，避免 5Hz 快照积分造成的紫线漂移。
  // 单位：deg；每拍 + (gyroPitchRate-kalmanBias)*2.5ms。
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
  // 去重力：静止时 az≈-sin(pitch)，故 (az+sin(pitch))≈0；乘方向系数后“向前加速为正”。
  // 注意这里用融合后的 pitch（已含毛刺抑制），并把结果限幅 ±IMU_ACCEL_CLAMP_G 再低通。
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
  // 可信度分档：|a|-1g <0.04 → 1.0（满信）；<0.10 → 0.45；否则 0.12（几乎只信编码器）。
  // 该系数乘进 400Hz 速度预测增量，防止振动/冲击把速度估计带飞。
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
  // 转向角速度：gx 减零偏后定向；当前转向环 Kp/Kd=0，量只用于遥测与手动转向角度保护。
  gyroTurnRate = TURN_GYRO_DIRECTION * (gx - gyroXZero);

}

}  // namespace control
