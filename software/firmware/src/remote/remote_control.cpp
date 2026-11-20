#include "remote/remote_control.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// 远控对象 + 远控专用的控制量。
// ============================================================
PositionOdometry positionOdom;
PositionJog positionJog;
// 2026-11-05 前后运动：操作量 -> 速度目标 -> 速度闭环倾角（平衡环不退出）
MotionSpeedProfile motionSpeedProfile;
MotionLeanControl motionLean;
MotionPositionPlan motionPlan;  // 预测位置环（X=0 时输出恒 0，行为不变）
NetSpeedEstimator motionNetSpeed;  // 前后运动外环专用的 200ms 净速度
bool manualMotionWasBusy = false;
PositionSpeedReference positionSpeedReference;
PositionLeanControl positionLean;
ManualTurnControl manualTurn;
ManualPitchControl manualForward;
bool positionHoldWasEnabled = false;
// Shared arbitration point for future forward/reverse manual commands.
bool positionHoldSuspended = false;

namespace remote {

bool isActive() {
  uint32_t now = millis();
  return manualTurn.busy(now) || manualForward.busy(now);
}

// ============================================================
// 400Hz：手动俯仰目标平滑（原 loop 中的 manualForward.update 一行）
// ============================================================
void tick(float dt) {
  manualForward.update(millis(), dt, pitch - balanceAngleOffset);
}

// ============================================================
// 100Hz：运动驱动 / 点动调度 / 位置 P 外环 —— 调度器
// 顺序与 updateOuterLoop() 原顺序逐条一致：
//   点动 → 运动驱动(接管则返回) → 运动释放 → 位置P外环 → jog倾角 → 速度参考
// ============================================================
bool tick100(bool & jogPos, bool & jogAngle) {
  bool jogPositionControl = false;
  positionJogTick(jogPositionControl);
  if (motionDriveTick()) {
    jogPos = true;
    jogAngle = false;
    return true;
  }
  positionHold100(jogPositionControl);
  bool jogAngleControl = jogPositionControl || positionLean.busy();
  if (jogAngleControl) positionSpeedTarget = targetForwardSpeed = 0;
  float jogDirection = speedLoopDirection * speedKp >= 0 ? 1.0f: - 1.0f;
  positionLean.update(jogPositionControl ? positionError: 0, jogPositionControl, .01f, jogDirection);
  targetForwardSpeed = positionSpeedReference.update(targetForwardSpeed, .01f);
  jogPos = jogPositionControl;
  jogAngle = jogAngleControl;
  return false;
}

// ============================================================
// 数据接口
// ============================================================
float leanOffset() {
  return motionLean.applied;
}

float turnOffset() {
  return turnOutput;
}

float normalAuthority() {
  return manualForward.normalAuthority();
}

bool motionWasBusy() {
  return manualMotionWasBusy;
}

void resetLean() {
  positionLean.reset();
}

// ============================================================
// 停机/故障统一清零：远控侧状态机（原 clearControlOutputDiagnostics 的远控部分）
// ============================================================
void resetFault() {
  manualTurn.clear();
  manualForward.clear();
  positionJog.cancel(PositionJog::FAULT);
  positionLean.reset();
  positionSpeedReference.reset();  // Actual motor-stop fault only; odometry remains continuous.
  positionHoldSuspended = false;
  positionHoldWasEnabled = false;
}

bool manualPitchBusy() {
  return manualForward.busy(millis());
}

float jogLeanOffset() {
  return positionLean.applied;
}

}  // namespace remote
