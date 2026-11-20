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
bool positionHoldWasEnabled = false;
// Shared arbitration point for future forward/reverse manual commands.
bool positionHoldSuspended = false;
float turnOutput = 0.0f;
float lastTurnError = 0.0f;
float targetTurnRate = 0.0f;

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
// 100Hz：运动驱动 / 点动调度 / 位置 P 外环
// （原 updateOuterLoop() 第 4 步的远控部分，逐行搬入；
//   原代码里的 "return;" 变成 "return true;" = 运动驱动接管本拍）
// ============================================================
bool tick100(bool & jogPos, bool & jogAngle) {
  bool jogPositionControl = false;
  if (positionJog.active) {
    int mode = (encoder1Trusted && encoder2Trusted) ? 0: (encoder1Trusted ? 1: (encoder2Trusted ? 2: 3));
    positionJog.update(vehiclePosition, vehicleSpeed, mode, millis(), .01f);
    if (!positionJog.active) {
      captureCurrentPositionTarget();
      if (positionJog.reason == PositionJog::ARRIVED) {
        targetVehiclePosition = positionJog.arrival.center;
        positionError = targetVehiclePosition - vehiclePosition;
      }
    } else {
      jogPositionControl = true;
      targetVehiclePosition = positionJog.reference;
      positionError = targetVehiclePosition - vehiclePosition;
      positionSpeedTarget = targetForwardSpeed = 0;
    }
  }
  if (manualForward.busy(millis())) {
    // 前后运动闭环（2026-11-05）：操作量只给速度目标，倾角由速度误差生成。
    // 不再把最大 2° 的死倾角直接写进目标角；平衡环全程参与，倾角受 1° 上限和速率限制。
    if (!manualMotionWasBusy) {
      // 入口无缝：用速度环当前倾角做初值，目标角不跳变；净速度从当前位置重新起算。
      motionLean.seed(speedAngleCorrection);
      motionSpeedProfile.reset();
      motionNetSpeed.reset(vehiclePosition);
      motionPlan.start(vehiclePosition);
      manualMotionWasBusy = true;
    }
    float normalized = ManualTurnControl::clamp(manualForward.planned / ManualPitchControl::maxAngle, - 1.0f, 1.0f);
    float motionSpeedReference = motionSpeedProfile.update(normalized * MotionSpeedProfile::maxSpeed, .01f);
    // 200ms 净速度：滤掉位置环绕着平衡点的高频摆动，只留真正的位移趋势。
    float netSpeed = motionNetSpeed.update(vehiclePosition);
    // 预测位置环：计划位置 = 参考速度积分，误差用 (实际 + 净速度×前视) 计算，
    // 输出只是速度增量；positionKp=0 时恒为 0，等价于纯速度环。
    float posCorrection = motionPlan.update(motionSpeedReference, vehiclePosition, netSpeed, true, .01f, positionKp, MAX_POSITION_RETURN_SPEED, POSITION_DEADBAND_COUNT);
    motionLean.update(motionSpeedReference + posCorrection, netSpeed, true, .01f);
    positionSpeedTarget = targetForwardSpeed = 0.0f;
    speedError = speedIntegral = speedErrorPrev = speedErrorDotFilt = 0.0f;
    speedPContribution = speedIContribution = 0.0f;
    speedAngleCorrection = speedAngleUnclamped = speedAngleLimited = 0.0f;
    return true;
  }
  // 运动结束后的释放段：速度目标与倾角平滑回零，再由驻车环接管，不产生角度阶跃。
  manualMotionWasBusy = false;
  motionPlan.stop(vehiclePosition);
  if (motionLean.busy() || fabsf(motionSpeedProfile.applied) > 0.0001f) {
    motionSpeedProfile.update(0.0f, .01f);
    motionLean.update(0.0f, vehicleSpeed, false, .01f);
  }
  bool positionHoldEnabled = positionKp > 0.000001f;
  if (!jogPositionControl) {
    if (positionHoldEnabled && !positionHoldWasEnabled && !positionHoldSuspended) {
      captureCurrentPositionTarget();
      positionError = 0.0f;
    }
    positionHoldWasEnabled = positionHoldEnabled;
    if (positionHoldEnabled && !positionHoldSuspended) {
      float effectivePositionError = positionError;
      if (effectivePositionError > POSITION_DEADBAND_COUNT) effectivePositionError -= POSITION_DEADBAND_COUNT;
      else if (effectivePositionError < - POSITION_DEADBAND_COUNT) effectivePositionError += POSITION_DEADBAND_COUNT;
      else effectivePositionError = 0.0f;
      positionSpeedTarget = positionKp * effectivePositionError;
      if (positionSpeedTarget > MAX_POSITION_RETURN_SPEED) positionSpeedTarget = MAX_POSITION_RETURN_SPEED;
      else if (positionSpeedTarget < - MAX_POSITION_RETURN_SPEED) positionSpeedTarget = - MAX_POSITION_RETURN_SPEED;
      targetForwardSpeed = positionSpeedTarget;
    } else {
      positionSpeedTarget = 0.0f;
      targetForwardSpeed = 0.0f;
    }
  }  // !jogPositionControl
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
// 转向环 PD（原 updateTurnLoop）
// 当前Kp/Kd = 0
// ============================================================
// ============================================================
// 转向环 PD
//
// 当前Kp/Kd = 0
// ============================================================
void updateTurn() {
  float turnError = targetTurnRate - gyroTurnRate;
  float derivative = (turnError - lastTurnError) / CONTROL_DT;
  lastTurnError = turnError;
  float legacy = turnKp * turnError + turnKd * derivative;
  turnOutput = manualTurn.update(millis(), CONTROL_DT, pitch - balanceAngleOffset, BALANCE_OUTPUT_LIMIT_HZ, finalBalanceOutput, legacy);
}

// ============================================================
// 位置快速拉回
//
// 400Hz直接读取编码器累计位置。
// 不经过50ms速度窗口，所以只要编码器出现少量位移就开始反向拉回。
// ============================================================
void positionHold() {
  int32_t a, b;
  noInterrupts();
  a = encoder1Count;
  b = encoder2Count;
  interrupts();
  int mode = (encoder1Trusted && encoder2Trusted) ? 0: (encoder1Trusted ? 1: (encoder2Trusted ? 2: 3));
  vehiclePosition = positionOdom.update(a, b, mode);
  if (positionJog.active && (fabsf(pitch - balanceAngleOffset) > 5 || fabsf(speedKp) < .000001f)) {
    stopPositionJog(PositionJog::FAULT);
    return;
  }
  if (positionJog.active && mode != positionJog.source) {
    stopPositionJog(PositionJog::SOURCE_CHANGED);
    return;
  }
  // Follow actual position throughout manual tilt, return-to-O and speed-loop handoff.
  // The parking loop must not accumulate an error against a stale pre-motion target.
  bool followPosition = isActive() || manualForward.normalAuthority() < 1.0f;
  bool resume = positionHoldSuspended && !followPosition;
  positionHoldSuspended = followPosition;
  if (followPosition) {
    if (positionJog.active) stopPositionJog(PositionJog::STOP);
    targetVehiclePosition = vehiclePosition;
    positionSpeedReference.reset();
    positionError = positionSpeedTarget = targetForwardSpeed = 0;
    return;
  }
  if (resume && !positionJog.active) targetVehiclePosition = vehiclePosition;
  if (positionJog.active) targetVehiclePosition = positionJog.reference;
  positionError = targetVehiclePosition - vehiclePosition;
}
// 把当前位置设为新的驻车位置。用于启用位置环、跌倒恢复和面板手动重设。
void captureCurrentPositionTarget() {
  if (positionJog.active) positionJog.cancel(PositionJog::STOP);
  int32_t a, b;
  noInterrupts();
  a = encoder1Count;
  b = encoder2Count;
  interrupts();
  int mode = (encoder1Trusted && encoder2Trusted) ? 0: (encoder1Trusted ? 1: (encoder2Trusted ? 2: 3));
  vehiclePosition = positionOdom.update(a, b, mode);
  // Capturing a parking target never rewrites encoder counters or the odometry coordinate.
  targetVehiclePosition = vehiclePosition;
  positionError = positionSpeedTarget = 0;
  targetForwardSpeed = positionSpeedReference.applied;
  speedIntegral = 0;
  speedErrorPrev = targetForwardSpeed - vehicleSpeed;
  speedErrorDotFilt = 0;
}
void stopPositionJog(int reason) {
  positionJog.cancel(reason);
  captureCurrentPositionTarget();
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

bool manualPitchBusy() {
  return manualForward.busy(millis());
}

float jogLeanOffset() {
  return positionLean.applied;
}

}  // namespace remote
