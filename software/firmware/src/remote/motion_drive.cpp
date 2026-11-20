#include "remote/motion_drive.h"

#include "control/cascade.h"

namespace remote {

bool motionDriveTick() {
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
    control::clearSpeedLoopForMotionBypass();
    return true;
  }  // 运动结束后的释放段：速度目标与倾角平滑回零，再由驻车环接管，不产生角度阶跃。
  manualMotionWasBusy = false;
  motionPlan.stop(vehiclePosition);
  if (motionLean.busy() || fabsf(motionSpeedProfile.applied) > 0.0001f) {
    motionSpeedProfile.update(0.0f, .01f);
    motionLean.update(0.0f, vehicleSpeed, false, .01f);
  }
  return false;
}

}  // namespace remote
