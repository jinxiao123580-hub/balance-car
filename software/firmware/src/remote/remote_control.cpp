#include "remote/remote_control.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// 远控对象 + 远控专用的控制量。
// ============================================================
// positionOdom：位置里程（position 单位 count，10.8 counts=1mm）。写：positionHold/capture；读：点动/位置环。
PositionOdometry positionOdom;
// positionJog：位置点动状态机。写：panel 命令 + 100Hz update；读：positionHold、遥测、params。
PositionJog positionJog;
// 2026-11-05 前后运动：操作量 -> 速度目标 -> 速度闭环倾角（平衡环不退出）
// motionSpeedProfile：^ 前后运动的速度目标斜坡（applied 单位 count/50ms，±5）。写/读：motionDriveTick() 100Hz。
MotionSpeedProfile motionSpeedProfile;
// motionLean：^ 前后运动的速度误差→倾角（applied 单位 deg，±0.90）。写：motionDriveTick() 100Hz；
// 读：remote::leanOffset() → main 每 400Hz 送 control::updateAuthority()。
MotionLeanControl motionLean;
// motionPlan：前后运动的预测位置环（plan 单位 count）。写/读：motionDriveTick() 100Hz（仅前后运动期间）。
// （注：旧注释"X=0 时输出恒 0，行为不变"与当前实现不符，以代码为准：X(positionKp) 编译默认 0.02，
//   前后运动时位置修正项会输出；只有把 X 配成 0 才恒为 0。）
MotionPositionPlan motionPlan;  // 预测位置环（X=0 时输出恒 0，行为不变）
// motionNetSpeed：前后运动专用的 200ms 净速度（count/50ms），只喂运动外环，不参与速度内环/直立环。
NetSpeedEstimator motionNetSpeed;  // 前后运动外环专用的 200ms 净速度
// manualMotionWasBusy：上一拍运动驱动是否接管（bool）。写：motionDriveTick()；读：remote::motionWasBusy()。
bool manualMotionWasBusy = false;
// positionSpeedReference：速度参考斜率器（applied 单位 count/50ms，斜率 3.0/s）。写/读：tick100() 100Hz。
PositionSpeedReference positionSpeedReference;
// positionLean：位置点动的"位置误差→倾角"P 环（applied 单位 deg，±0.30）。写/读：tick100() 100Hz。
PositionLeanControl positionLean;
// manualTurn：转向状态机（requested/applied 单位差模 STEP Hz，±300）。写：cmd 命令+updateTurn()；读：isActive()。
ManualTurnControl manualTurn;
// manualForward：^ 手动俯仰轨迹（planned/applied 单位 deg，±2°）。写：cmd 命令 + remote::tick() 400Hz；读：motion_drive。
ManualPitchControl manualForward;
// positionHoldWasEnabled：位置环（X>0）上一拍是否启用，用于"刚启用时重新 capture"的边沿检测。写/读：positionHold100()。
bool positionHoldWasEnabled = false;
// Shared arbitration point for future forward/reverse manual commands.
// positionHoldSuspended：驻车环是否被悬挂（手动通路/权限释放期间）。写：positionHold() 400Hz；读：位置环/cmd/params。
bool positionHoldSuspended = false;

namespace remote {

// isActive — 作用：转向或手动俯仰任一仍在占用（等价原 manualMotionIsActive()）。
// 参数：无。返回 bool。调用：params 写盘前置检查、cmd 前置检查。边界：只反映"手动通路"，不含点动。
bool isActive() {
  uint32_t now = millis();
  return manualTurn.busy(now) || manualForward.busy(now);
}

// ============================================================
// 400Hz：手动俯仰目标平滑（原 loop 中的 manualForward.update 一行）
// ============================================================
// tick — 作用：400Hz 推进手动俯仰轨迹（manualForward.update）。
// 参数 dt：控制周期 s（CONTROL_DT=0.0025）。返回：无。
// 调用：main loop 远控 400Hz 段。边界：angleError=pitch−平衡点（deg）；≥10° 时 ManualPitchControl 清状态并闩锁。
void tick(float dt) {
  manualForward.update(millis(), dt, pitch - balanceAngleOffset);
}

// ============================================================
// 100Hz：运动驱动 / 点动调度 / 位置 P 外环 —— 调度器
// 顺序与 updateOuterLoop() 原顺序逐条一致：
//   点动 → 运动驱动(接管则返回) → 运动释放 → 位置P外环 → jog倾角 → 速度参考
// ============================================================
// tick100 — 远控 100Hz 调度器：顺序固定（见 remote_control.h 注释），顺序错会用到旧位置/旧标志。
// 参数 jogPos/jogAngle：回传标志。返回：true=运动驱动接管本拍。
// 调用：main loop 每 4 个 400Hz 拍一次（100Hz），必须在 control::tick100() 之后。
bool tick100(bool & jogPos, bool & jogAngle) {
  bool jogPositionControl = false;
  // ① 点动调度：推进 PositionJog 状态机；jogPositionControl=true 时本拍位置由点动参考驱动。
  positionJogTick(jogPositionControl);
  // ② 运动驱动：^ forward≠0 时接管本拍 → 直接返回 true，main 跳过速度 PI（两套目标角不打架）。
  // 注意此时 jogPos=true/jogAngle=false：位置由运动环负责，jog 倾角不参与。
  if (motionDriveTick()) {
    jogPos = true;
    jogAngle = false;
    return true;
  }
  // ③ 位置 P 外环：非点动时把位置误差变成目标速度（X/Y/M 参数在这里生效）。
  positionHold100(jogPositionControl);
  // ④ jog 倾角控制是否参与：点动位置控制中，或 jog 倾角还有残余（释放段）都要算。
  bool jogAngleControl = jogPositionControl || positionLean.busy();
  // jog 倾角参与时清掉速度环目标：本拍速度 PI 不能再叠加自己的目标（否则两套位置/速度目标叠加）。
  if (jogAngleControl) positionSpeedTarget = targetForwardSpeed = 0;
  // jog 方向符号：speedLoopDirection×speedKp 的符号决定"往哪边倾对应前进"（默认 -1×-0.10>0 → +1）。
  // 必须与速度环极性一致，否则点动会反向跑。
  float jogDirection = speedLoopDirection * speedKp >= 0 ? 1.0f: - 1.0f;
  // ⑤ jog 倾角环：仅点动位置控制时喂位置误差，否则喂 0（按 0.30deg/s 释放）；direction 见上。
  positionLean.update(jogPositionControl ? positionError: 0, jogPositionControl, .01f, jogDirection);
  // ⑤ 速度参考斜率限制：位置环/点动交接时速度请求不能阶跃（3.0 count/50ms per s），防轮子猛冲。
  targetForwardSpeed = positionSpeedReference.update(targetForwardSpeed, .01f);
  jogPos = jogPositionControl;
  jogAngle = jogAngleControl;
  return false;
}

// ============================================================
// 数据接口
// ============================================================
// leanOffset — 作用：运动/jog 叠加的目标倾角修正（motionLean.applied）。单位 deg（±0.90）。
// 调用：main 每 400Hz 传给 control::updateAuthority()。
float leanOffset() {
  return motionLean.applied;
}

// turnOffset — 作用：转向差模输出（turnOutput）。单位 STEP Hz。调用：main 每 400Hz 传给 control::mixMotors()。
float turnOffset() {
  return turnOutput;
}

// normalAuthority — 作用：手动俯仰释放斜坡（0..1），给外环权限用。调用：main 每 400Hz。
// 边界：手动通路占用时 0（外环退出），释放后 1s 内平滑回到 1。
float normalAuthority() {
  return manualForward.normalAuthority();
}

// motionWasBusy — 作用：上一拍运动驱动是否接管。调用：main 每 100Hz 传给 control::outerLoop()。
bool motionWasBusy() {
  return manualMotionWasBusy;
}

// resetLean — 作用：清零 jog 倾角。调用：main 在 leanResetRequested 时（速度环被整体关闭）。
void resetLean() {
  positionLean.reset();
}

// ============================================================
// 停机/故障统一清零：远控侧状态机（原 clearControlOutputDiagnostics 的远控部分）
// ============================================================
// resetFault — 作用：停机/故障统一清零远控侧状态机（转向/俯仰/点动/倾角/速度参考/悬挂标志）。
// 调用：main 的 clearControlOutputDiagnostics()（setup 重试、掉拍、IMU 故障、倒地，事件级）。
// 边界：只清状态机与速度参考，不重置位置里程坐标（positionOdom 的坐标保持连续）。
void resetFault() {
  manualTurn.clear();
  manualForward.clear();
  // 位置点动状态机：直接置 FAULT 取消（位置坐标不在这里重置，由上层 capture 决定）。
  positionJog.cancel(PositionJog::FAULT);
  // jog 倾角环清零：故障停机后不得残留倾角继续推车。
  positionLean.reset();
  // 速度参考清零：这是"真实停机故障"才做的动作；里程坐标保持连续（见下行注释）。
  positionSpeedReference.reset();  // Actual motor-stop fault only; odometry remains continuous.
  positionHoldSuspended = false;
  positionHoldWasEnabled = false;
}

// manualPitchBusy — 作用：手动俯仰是否占用（manualForward.busy(millis())）。
// 调用：main 每 100Hz 传给 control::outerLoop()；cmd 的 ^ 前置检查。
bool manualPitchBusy() {
  return manualForward.busy(millis());
}

// jogLeanOffset — 作用：jog 倾角当前输出（positionLean.applied）。单位 deg（±0.30）。
// 调用：main 每 100Hz 传给 control::outerLoop()。
float jogLeanOffset() {
  return positionLean.applied;
}

}  // namespace remote
