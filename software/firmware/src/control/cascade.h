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
// outerLoop：100Hz 速度串级外环（无返回）。参数：jogPositionControl/jogAngleControl=远控点动调度标志；
//   motionWasBusy=^ 运动接管标志（真则清 SP）；manualPitchBusy=手动俯仰忙；jogLeanOffset=jog 倾角 deg。
//   调用：main.cpp 每 4 个 400Hz 拍一次（100Hz），在 remote::tick100() 之后。
//   输出：targetForwardSpeed/速度误差/各项贡献/speedAngleUnclamped/Limited/speedAngleCorrection。
void outerLoop(bool jogPositionControl, bool jogAngleControl, bool motionWasBusy,
               bool manualPitchBusy, float jogLeanOffset);

// 400Hz 外环权限 + 目标角（原 updateOuterLoopAuthority）。
// normalAuthority / leanOffset 来自 remote::normalAuthority() / remote::leanOffset()。
// updateAuthority：400Hz 外环权限与目标角合成。参数 normalAuthority=手动俯仰正常权限 0~1、leanOffset=远控倾角 deg。
//   无返回 ｜ 调用：main.cpp loop 每拍 400Hz ｜ 输出 outerLoopAuthority 与 targetBalanceAngle。
//   边界：|pitch-O|>=OUTER_DISABLE_ANGLE_DEG 时权限 0、目标角回到 O；权限<0.2 时积分按 0.95/拍释放。
void updateAuthority(float normalAuthority, float leanOffset);

// ^ 运动驱动接管时清零速度环历史（原 updateOuterLoop 运动分支里的 3 行清零）。
// clearSpeedLoopForMotionBypass：^ 运动驱动接管时清零速度环历史（无参/无返回）｜
//   调用：remote::motionDriveTick（100Hz 入口）｜ 清误差/积分/各贡献/修正角，避免接管瞬间跳变。
void clearSpeedLoopForMotionBypass();

}  // namespace control
