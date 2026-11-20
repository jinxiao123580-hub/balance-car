#pragma once
// ============================================================
// control/cascade.{h,cpp} — 串级速度环：速度 PI → speedAngleCorrection；
// 外环权限 → targetBalanceAngle。
//
// control 不得 include remote/：远控叠加量由 main 以参数/数据接口传入。
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"

namespace control {

// 100Hz 速度 PI（原 updateOuterLoop 的后半段，逐行搬入）。
// jogPositionControl / jogAngleControl 来自 remote::tick100() 的调度标志（原局部变量）。
void outerLoop(bool jogPositionControl, bool jogAngleControl, bool motionWasBusy,
               bool manualPitchBusy, float jogLeanOffset);

// 400Hz 外环权限 + 目标角（原 updateOuterLoopAuthority）。
// normalAuthority / leanOffset 来自 remote::normalAuthority() / remote::leanOffset()。
void updateAuthority(float normalAuthority, float leanOffset);

// ^ 运动驱动接管时清零速度环历史（原 updateOuterLoop 运动分支里的 3 行清零）。
void clearSpeedLoopForMotionBypass();

}  // namespace control
