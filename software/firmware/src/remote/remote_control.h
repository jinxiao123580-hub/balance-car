#pragma once
// ============================================================
// remote/remote_control.{h,cpp} — ^ 手动通路 / 点动调度 / EG 增益
// 以及远控对控制核心的数据接口（leanOffset / turnOffset / isActive）。
//
// 依赖方向：remote → control/core 允许；remote 不 include panel。
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"

#include "control/balance.h"
#include "control/cascade.h"
#include "control/motor.h"

// 远控对象定义（过渡期沿用 src/ 下的旧头文件，后续拆到 manual_pitch/manual_turn/
// motion_drive/position_jog 各自头文件）
#include "manual_motion.h"
#include "manual_turn.h"
#include "motion_drive.h"
#include "position_jog.h"

// ---- 状态对象（定义在 remote/remote_control.cpp，全局作用域）----
extern PositionOdometry positionOdom;
extern PositionJog positionJog;
extern MotionSpeedProfile motionSpeedProfile;
extern MotionLeanControl motionLean;
extern MotionPositionPlan motionPlan;
extern NetSpeedEstimator motionNetSpeed;
extern PositionSpeedReference positionSpeedReference;
extern PositionLeanControl positionLean;
extern ManualTurnControl manualTurn;
extern ManualPitchControl manualForward;
extern bool manualMotionWasBusy;

namespace remote {

// ---- 400Hz -----------------------------------------------------
// 手动俯仰目标平滑（原 loop 中的 manualForward.update 一行）
void tick(float dt);
// 位置保持：400Hz 读累计位置，维护停车目标（原 updatePositionHold）
void positionHold();
// 转向环：legacy PD + 手动转向状态机 → turnOutput（原 updateTurnLoop）
void updateTurn();

// ---- 100Hz -----------------------------------------------------
// 运动驱动/点动调度（原 updateOuterLoop 第 4 步的远控部分）。
// 返回 true = 运动驱动接管本拍（原代码里的 "return"，速度 PI 必须跳过）。
// jogPos / jogAngle 为回传的调度标志，交给 control::outerLoop 用于积分清零。
bool tick100(bool & jogPos, bool & jogAngle);

// ---- 位置点动入口（panel/cmd 调用）------------------------------
void captureCurrentPositionTarget();
void stopPositionJog(int reason);

// ---- 数据接口（control 只能通过这些量看到远控，见 SPEC §2）--------
bool isActive();                        // 等价原 manualMotionIsActive()
float leanOffset();                     // 等价 motionLean.applied
float turnOffset();                     // 等价 turnOutput
float normalAuthority();                // 等价 manualForward.normalAuthority()
bool motionWasBusy();                   // 等价 manualMotionWasBusy
void resetLean();                       // 速度环整体关闭时清零 jog 倾角
bool manualPitchBusy();                 // 等价 manualForward.busy(millis())
float jogLeanOffset();                  // 等价 positionLean.applied

}  // namespace remote
