#pragma once
// ============================================================
// control/attitude.{h,cpp} — 姿态融合：Kalman → pitch / pitchRate
// （原互补滤波函数为死代码，按 SPEC §5 删除）
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"

namespace control {

// 加速度计倾角（度）
float calculateAccelPitch();

// Kalman 融合一阶（返回 kalmanAngle）
float updateKalman(float measuredAngle, float measuredRate, float dt);

// 400Hz 姿态更新：accPitch / gyroPitchRate / dynamicR / pitch / gyroIntegAngle
void updateAttitude();

}  // namespace control
