#pragma once
// ============================================================
// control/speed_est.{h,cpp} — 编码器累计 + 速度 Kalman 预测(400Hz)/修正(100Hz)
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"

namespace control {

// 400Hz：IMU 前向线加速度预测 vehicleSpeed（过程噪声随电机命令增长）
void predictVehicleSpeedKalman();

// 100Hz：编码器速度作为绝对测量修正 vehicleSpeed
void correctVehicleSpeedKalman(float encoderMeasurement);

// 100Hz 段入口（原 updateOuterLoop 前 4 步）：
// 编码器增量 → 滤波 → 信任门 → 原始速度 → Kalman 修正。
// 必须在 remote::tick100() 之前调用（remote 位置/点动用 vehicleSpeed）。
void tick100();

// 400Hz 段入口：读 IMU → 姿态 → 速度预测。成功返回 true。
bool tick400();

}  // namespace control
