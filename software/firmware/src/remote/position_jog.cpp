#include "remote/position_jog.h"
#include "remote/remote_control.h"

// 车辆相对开机/重新站立位置的累计位移，单位为平均编码器count
float vehiclePosition = 0.0f;

// 位置零点对应的两个编码器累计计数
int32_t positionZeroCount1 = 0;
int32_t positionZeroCount2 = 0;

// 目标位置固定为0，即尽量回到启动位置
float targetVehiclePosition = 0.0f;

namespace remote {

// ============================================================
// 100Hz 点动调度（原 updateOuterLoop 开头的 positionJog 分支）
// ============================================================
void positionJogTick(bool & jogPositionControl) {
  jogPositionControl = false;
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
}

// ============================================================
// 100Hz 位置 P 外环（原 updateOuterLoop 的 !jogPositionControl 段）
// ============================================================
void positionHold100(bool jogPositionControl) {
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
}

// ============================================================
// 位置快速拉回（原 updatePositionHold）
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

}  // namespace remote
