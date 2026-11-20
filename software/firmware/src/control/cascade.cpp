#include "control/cascade.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// ============================================================
// ------------------------------------------------------------
// 速度P直接STEP快速阻尼
//
// 用于解决“只改变目标角度，轮速响应滞后明显”的问题。
// 这一路直接叠加到直立环STEP频率输出，但权限受姿态限制。
// ------------------------------------------------------------
int speedLoopDirection = - 1;  // 2026-09-02 1→-1：架空实测定反。原=轮只往单方向转；翻号后=轮来回正反转（方向盘反）
// 经典速度PI外环（改目标角）：speedAngleCorrection 叠加到目标平衡角，让车倾斜以移动。
float speedKp = - 0.10f;  // 速度PI到目标角的比例增益；实车已确认该号为正确方向。
// 2026-11-19：−0.02 太小（SP=8 只产生 0.16°，低于破粘门槛 0.5~0.7°），
// 实测 −0.10 在 SP±8 下给出 +43.8/−43.0mm 的对称位移（2% 差）。
// 上限：≥0.20 会猛冲撞翻（SV 0.20/0.30 实测 −47.9/−11.9mm/s）。
float speedKi = 0.0f;  // 先用纯P验证速度方向；启用后采用条件积分/抗饱和
float speedKd = 0.0f;
float MAX_SPEED_ANGLE = 0.8f;  // 速度修正角上限（参数 N）。
float SPEED_INTEGRAL_LIMIT = 50.0f;  // 速度PI积分累积上限（误差×秒，非deg）
// ------------------------------------------------------------
// 串级速度PI外环
//
// 控制结构：
//
// targetForwardSpeed
//        ↓
// 速度误差 = target - Kalman融合速度
//        ↓
// 速度直投（P×比例 + 积分Gain）→ 直接叠加STEP Hz（该通道已退出控制，仅历史说明）
//        ↓
// 直立PD（400Hz）
//        ↓
// STEP频率
//
// 速度外环每50ms执行一次 = 20Hz。
// （旧"速度PI→目标倾角→改targetBalanceAngle"路径已移除：见 .bak_deadcode_20260902）
// ------------------------------------------------------------
// 很小的速度误差不积分，避免零附近反复积累
float SPEED_ERROR_DEADBAND = 2.0f;  // 速度误差死区(速度环积分门控) 2026-09-02 可调=J
// 目标速度变化限幅。
// 当前targetForwardSpeed通常为0，这个参数主要为后续遥控前进保留。
// 2026-11-19：8 太小——车被锁在"几 mm/s"的静摩擦主导区间，任何环都稳不住。
// 放到 30，允许进入正常巡航速度（SP 命令仍受此钳制）。
// 2026-11-20: runtime variable + `MS <v>` command (RAM-only), no reflash for range tuning.
float MAX_TARGET_FORWARD_SPEED = 30.0f;
// ============================================================
// 大角度直立优先
//
// 小角度：速度环 + 位置环全部参与
// 中等角度：外环平滑减弱
// 大角度：外环完全退出，只保留直立PD救车
// ============================================================
float OUTER_FULL_ANGLE_DEG = 5.0f;  // |pitch-平衡点| <= 此角：外环全权限
float OUTER_DISABLE_ANGLE_DEG = 10.0f;  // >=此角：外环退出，只留直立PD
// 直立环衰减的历史参数（已并入外环权限，不再单独存在）。range 0~1。
// 用户方案：削弱直立环(如0.7)给速度环让权重，速度环直接叠加到轮速而非改目标角。
// 外环当前权限：0~1
float outerLoopAuthority = 1.0f;
// ============================================================
// 目标值
// ============================================================
float targetForwardSpeed = 0.0f;
// SP 命令设定的目标速度（count/50ms）。与 ^ 手动通路互斥：^ 活动时被清零。
float speedCmd = 0.0f;
// ============================================================
// 目标角阶跃测试
//
// TEST_TARGET_ANGLE_OFFSET:
// 当前关闭阶跃测试：TEST_TARGET_ANGLE_OFFSET = 0.0f
// 速度PI同样关闭。
// 本版只用于观察：
//   1. Kalman pitch
//   2. gyroPitchRate
//
// 用于判断落地运动时：
// 是真实车体仍在持续旋转，还是Kalman角度被动态加速度拉偏。
// ============================================================
// 目标角阶跃诊断已关闭，正式外环版本不再使用固定阶跃。
// ============================================================
// 速度PI控制变量
// ============================================================
float speedError = 0.0f;
float speedPContribution = 0.0f;
float speedIContribution = 0.0f;
float speedAngleUnclamped = 0.0f;
float speedAngleLimited = 0.0f;
// 速度直投积分（治"往一边溜"）：对准 speedError 累积、直接叠到 STEP Hz，不进平衡角。
// 只加直投积分不碰速度PI目标角路，前进/转弯(targetForwardSpeed/turnKp)语义保留。
float speedIntegral = 0.0f;  // 经典速度PI积分累积（纯串级，会进目标角）
float speedAngleCorrection = 0.0f;  // 经典速度PI输出的目标倾角修正（叠加到 targetBalanceAngle）
float speedErrorPrev = 0.0f;  // 速度误差微分阻尼：上一拍误差
float speedErrorDotFilt = 0.0f;  // 速度误差微分阻尼：低通后的误差变化率
// ------------------------------------------------------------
// 位置快速拉回
//
// 作用：车只要离开开机位置一点点，就立即给出反向STEP修正。
// 这一路直接读取编码器累计位置，400Hz更新，不等待50ms速度窗口。
// ------------------------------------------------------------
// 标准串级位置外环：位置误差先生成一个受限目标速度，不直接抢占STEP输出。
float positionKp = 0.02f;  // 蓝牙X：实测安全值（原 5.0 会把静止漂移放大到 230counts/s，2026-11-12 改）
float MAX_POSITION_RETURN_SPEED = 0.30f;  // 蓝牙Y：目标回位速度限幅，count/50ms（实测安全值）
float POSITION_DEADBAND_COUNT = 12.0f;  // 蓝牙M：位置死区，平均编码器count（实测安全值）
// 位置串级外环诊断量
float positionError = 0.0f;
float positionSpeedTarget = 0.0f;

// 遥测第 19 列：外环时刻的 vehicleSpeed 快照（原预测速度列位，值不变）
float telemetrySpeedSnapshot100Hz = 0.0f;

// 速度环整体关闭时，请求远控清零 jog 倾角（由 main 立即转发 remote::resetLean()）
bool leanResetRequested = false;

namespace control {

// ============================================================
// 漂移拉回外环
// 成熟步进平衡车串级外环：100Hz速度PI -> 小目标倾角。
// 编码器增量换算为既有count/50ms量纲，保持参数和遥测单位不变。
// ============================================================
void outerLoop(bool jogPositionControl, bool jogAngleControl, bool motionWasBusy,
               bool manualPitchBusy, float jogLeanOffset) {
  // --------------------------------------------------------
  // 4. 标准串级位置P外环：位置误差 -> 受限目标速度
  // --------------------------------------------------------
  // ---- SP 速度指令：唯一的速度设定入口 ----
  // 与 ^ 手动通路互斥：^ 正在动作时清掉 SP，避免两套通路同时写目标速度。
  if (motionWasBusy) {
    speedCmd = 0.0f;
  } else if (speedCmd != 0.0f) {
    targetForwardSpeed = speedCmd;  // 直接进入下面的经典速度 PI
  }
  if (jogPositionControl) positionSpeedTarget = targetForwardSpeed;
  // --------------------------------------------------------
  // 5. 总目标速度安全限幅
  // --------------------------------------------------------
  if (targetForwardSpeed > MAX_TARGET_FORWARD_SPEED) {
    targetForwardSpeed = MAX_TARGET_FORWARD_SPEED;
  }
  if (targetForwardSpeed < - MAX_TARGET_FORWARD_SPEED) {
    targetForwardSpeed = - MAX_TARGET_FORWARD_SPEED;
  }
  // --------------------------------------------------------
  // 6. 速度误差
  // --------------------------------------------------------
  // 2026-11-12 回退：τ=0.25s 的速度低通把速度外环推过临界（pitch sd 0.17°→3.0°），
  // 回到原始瞬时速度；滤波思路保留在 SPEED_MEAS_TAU_S 注释里，要用只能小步试。
  vehicleSpeedFiltered = vehicleSpeed;
  speedError = targetForwardSpeed - vehicleSpeed;
  // --------------------------------------------------------
  // 7. 速度PI生成目标角。第一版严格采用成熟串级结构，
  //
  // 不使用速度预测、STEP直投或位置回位。
  // --------------------------------------------------------
  // 经典速度PI目标角通道：100Hz更新。
  float speedErrDot = (speedError - speedErrorPrev) * 100.0f;
  speedErrorDotFilt = 0.7f * speedErrorDotFilt + 0.3f * speedErrDot;  // 低通滤噪声
  speedErrorPrev = speedError;
  speedPContribution = speedKp * speedError;
  float speedDContribution = speedKd * speedErrorDotFilt;
  // 条件积分 + 抗饱和：禁止在外环无权限或输出继续撞向N限幅时累积。
  if (jogAngleControl || manualPitchBusy || fabsf(speedKi) < 0.000001f) {
    speedIntegral = 0.0f;
  } else if (outerLoopAuthority >= 0.2f && fabsf(speedError) > SPEED_ERROR_DEADBAND) {
    float candidateIntegral = speedIntegral + speedError * 0.01f;
    if (candidateIntegral > SPEED_INTEGRAL_LIMIT) candidateIntegral = SPEED_INTEGRAL_LIMIT;
    else if (candidateIntegral < - SPEED_INTEGRAL_LIMIT) candidateIntegral = - SPEED_INTEGRAL_LIMIT;
    float currentUnclamped = speedLoopDirection * (speedPContribution + speedKi * speedIntegral + speedDContribution);
    float candidateUnclamped = speedLoopDirection * (speedPContribution + speedKi * candidateIntegral + speedDContribution);
    if (fabsf(candidateUnclamped) <= MAX_SPEED_ANGLE || fabsf(candidateUnclamped) < fabsf(currentUnclamped)) {
      speedIntegral = candidateIntegral;
    }
  }
  speedIContribution = speedKi * speedIntegral;
  // Manual forward/reverse uses the operator target with P/D only; no hidden I trim.
  speedAngleUnclamped = speedLoopDirection * (speedPContribution + speedIContribution + speedDContribution) + jogLeanOffset;
  float desiredSpeedAngle = speedAngleUnclamped;
  // 遥测第 19 列：保留外环时刻的速度快照（原预测速度列位，数值不变）。
  telemetrySpeedSnapshot100Hz = vehicleSpeed;
  if (desiredSpeedAngle > MAX_SPEED_ANGLE) desiredSpeedAngle = MAX_SPEED_ANGLE;
  if (desiredSpeedAngle < - MAX_SPEED_ANGLE) desiredSpeedAngle = - MAX_SPEED_ANGLE;
  speedAngleLimited = desiredSpeedAngle;
  // 速度环全部关闭时立即清零历史修正，保证纯直立环测试不被残留目标角污染。
  if (fabsf(speedKp) < 0.000001f && fabsf(speedKi) < 0.000001f && fabsf(speedKd) < 0.000001f) {
    leanResetRequested = true;
    speedAngleCorrection = 0.0f;
    speedIntegral = 0.0f;
    speedPContribution = 0.0f;
    speedIContribution = 0.0f;
    speedAngleUnclamped = 0.0f;
    speedAngleLimited = 0.0f;
    speedErrorDotFilt = 0.0f;
    speedErrorPrev = speedError;
    telemetrySpeedSnapshot100Hz = vehicleSpeed;
    positionSpeedTarget = 0.0f;
    targetForwardSpeed = 0.0f;
  } else {
    float speedAngleDelta = desiredSpeedAngle - speedAngleCorrection;
    if (speedAngleDelta > SPEED_ANGLE_SLEW_DEG_PER_UPDATE) speedAngleDelta = SPEED_ANGLE_SLEW_DEG_PER_UPDATE;
    else if (speedAngleDelta < - SPEED_ANGLE_SLEW_DEG_PER_UPDATE) speedAngleDelta = - SPEED_ANGLE_SLEW_DEG_PER_UPDATE;
    speedAngleCorrection += speedAngleDelta;
  }
  // --------------------------------------------------------

}
// ============================================================
// 外环权限管理
//
// 使用相对机械平衡角，而不是相对 targetBalanceAngle。
// 这样速度PI改变目标角时，不会影响“大角度判定”。
// ============================================================
void updateAuthority(float normalAuthority, float leanOffset) {
  float uprightError = pitch - balanceAngleOffset;
  float absUprightError = fabsf(uprightError);
  if (absUprightError <= OUTER_FULL_ANGLE_DEG) {
    outerLoopAuthority = 1.0f;
  } else if (absUprightError < OUTER_DISABLE_ANGLE_DEG) {
    outerLoopAuthority = (OUTER_DISABLE_ANGLE_DEG - absUprightError) / (OUTER_DISABLE_ANGLE_DEG - OUTER_FULL_ANGLE_DEG);
  } else {
    outerLoopAuthority = 0.0f;
  }
  // 目标角必须受同一个权限控制。
  // 大角度时直接回到机械平衡角，让直立PD全力救车。
  // Normal operation uses the speed-loop angle; held motion replaces it smoothly.
  // The held angle is relative to O. 2026-11-05：前后运动改为速度闭环，不再写死角度。
  float pitchOffset = normalAuthority * speedAngleCorrection + leanOffset;
  targetBalanceAngle = balanceAngleOffset + outerLoopAuthority * pitchOffset;
  // 大角度时释放历史积分，避免恢复后旧积分继续把车推走
  if (outerLoopAuthority < 0.2f) {
    speedIntegral *= SPEED_INTEGRAL_DECAY_BIG_ANGLE;
  }

}

// ============================================================
// ^ 运动驱动接管时清零速度环历史（原 updateOuterLoop 运动分支里的 3 行清零）
// ============================================================
void clearSpeedLoopForMotionBypass() {
  speedError = speedIntegral = speedErrorPrev = speedErrorDotFilt = 0.0f;
  speedPContribution = speedIContribution = 0.0f;
  speedAngleCorrection = speedAngleUnclamped = speedAngleLimited = 0.0f;
}

}  // namespace control
