#include "remote/motion_drive.h"

#include "control/cascade.h"

namespace remote {

// motionDriveTick — 100Hz 运动驱动：^ forward≠0 时接管倾角通路，否则只做平滑释放。
// 流程（接管）：入口对齐 → 档位归一化 → 速度斜坡 → 200ms 净速度 → 预测位置修正 → 倾角 → 清速度环目标。
// 返回 true 时调用方（remote::tick100()→main）跳过 control::outerLoop()：本拍目标角由 motionLean 决定。
bool motionDriveTick() {
  // 用 manualForward.busy() 作为"^ 前后运动是否在生效"的判据：包含 350ms 命令新鲜度与轨迹/释放状态。
  if (manualForward.busy(millis())) {
    // 前后运动闭环（2026-11-05）：操作量只给速度目标，倾角由速度误差生成。
    // 不再把最大 2° 的死倾角直接写进目标角；平衡环全程参与，倾角受 1° 上限和速率限制。
    // 入口只执行一次（靠 manualMotionWasBusy 边沿检测）：数拍之间不能重复 seed，否则状态被反复覆盖。
    if (!manualMotionWasBusy) {
      // 入口无缝：用速度环当前倾角做初值，目标角不跳变；净速度从当前位置重新起算。
      // seed 的入参是速度环当前倾角修正（speedAngleCorrection，deg），且内部再钳到 ±limit=0.9°：
      // 即使当前外环倾角更大，手动通路一进来也不会满舵。
      motionLean.seed(speedAngleCorrection);
      // 速度参考从 0 起步，按 accel=20 count/50ms/s 爬升（0→满档 5 需 0.25s）。
      motionSpeedProfile.reset();
      motionNetSpeed.reset(vehiclePosition);
      motionPlan.start(vehiclePosition);
      manualMotionWasBusy = true;
    }
    // 档位归一化：manualForward.planned 是 deg（±maxAngle=2°），除以 2° 得到 [-1,+1]；
    // × MotionSpeedProfile::maxSpeed(5) → 目标速度 count/50ms（满档 5 ≈ 100 counts/s ≈ 9.3mm/s）。
    float normalized = ManualTurnControl::clamp(manualForward.planned / ManualPitchControl::maxAngle, - 1.0f, 1.0f);
    // 速度斜坡（dt=0.01s）：把阶跃档位变成限加速度参考，防止倾角目标突变导致轮子猛冲/失步。
    float motionSpeedReference = motionSpeedProfile.update(normalized * MotionSpeedProfile::maxSpeed, .01f);
    // 200ms 净速度：滤掉位置环绕着平衡点的高频摆动，只留真正的位移趋势。
    // 本拍的 netSpeed（count/50ms）同时喂给位置计划（前视项）与倾角环（误差项）。
    float netSpeed = motionNetSpeed.update(vehiclePosition);
    // 预测位置环：计划位置 = 参考速度积分，误差用 (实际 + 净速度×前视) 计算，
    // 输出只是速度增量；positionKp=0 时恒为 0，等价于纯速度环。
    // posCorrection 是速度增量（count/50ms）：只叠加到速度参考上，不改倾角，所以不会和速度环抢目标角。
    float posCorrection = motionPlan.update(motionSpeedReference, vehiclePosition, netSpeed, true, .01f, positionKp, MAX_POSITION_RETURN_SPEED, POSITION_DEADBAND_COUNT);
    // 倾角 = 速度参考 + 位置修正 → feedForward/误差项 → 限幅/斜率；measured 用 200ms 净速度而非瞬时速度。
    motionLean.update(motionSpeedReference + posCorrection, netSpeed, true, .01f);
    // 运动接管期间把速度 PI 的目标清零：本拍 control::outerLoop() 不会执行，
    // 这两个量只可能被残留使用，清掉可防止下一拍速度 PI 用旧目标突然发力。
    positionSpeedTarget = targetForwardSpeed = 0.0f;
    // 清速度环历史（积分/微分/上一拍误差）：手动通路退出后速度 PI 要"无扰接管"，不能带着运动期间的旧账。
    control::clearSpeedLoopForMotionBypass();
    return true;
  }  // 运动结束后的释放段：速度目标与倾角平滑回零，再由驻车环接管，不产生角度阶跃。
  // 释放段：^ 已结束但可能还有残余倾角/速度参考 → 不能立刻交还速度 PI，先按 releaseRate 平滑回零。
  manualMotionWasBusy = false;
  motionPlan.stop(vehiclePosition);
  // 只有还有残余（倾角或速度参考）才继续推释放；否则直接返回 false（稳态零开销）。
  if (motionLean.busy() || fabsf(motionSpeedProfile.applied) > 0.0001f) {
    // 速度参考按 accel 斜坡回 0；倾角用 releaseRate=3.5deg/s 释放（engaged=false）。
    motionSpeedProfile.update(0.0f, .01f);
    // 释放时 measured 用 vehicleSpeed（融合速度，count/50ms）：engaged=false 时该值不参与公式，仅占位。
    motionLean.update(0.0f, vehicleSpeed, false, .01f);
  }
  // 返回 false：本拍交还常规速度 PI（control::outerLoop() 会执行）。
  return false;
}

}  // namespace remote
