#include "remote/manual_turn.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// ============================================================
// 转向环参数（面板 T/U 可在线调）
// ============================================================
// 转向环全局量（core/state.h 的 extern 在 position_jog.cpp/本文件落地）：
//   turnKp/turnKd —— 转向 PD 增益。单位：turnKp=Hz per (deg/s)、turnKd=Hz per (deg/s²)；编译默认 0.0，
//     可在面板用 T/U 在线改（不落 flash）。读：updateTurn() 每 400Hz。
//   targetTurnRate —— 期望偏航角速度（deg/s）。当前代码里没有写入点（恒 0），转向实际由 manualTurn.requested 驱动；
//     保留字段用于将来闭环转向。读：updateTurn() 每 400Hz。
//   turnOutput —— 转向差模输出（STEP Hz）。写：updateTurn() 每 400Hz；读：main→control::mixMotors() 合成电机。
//   lastTurnError —— 上一拍转向误差（deg/s），用于 D 项差分。写/读：updateTurn() 每 400Hz。
// ============================================================
float turnKp = 0.0f;
float turnKd = 0.0f;
// 转向目标与输出
float targetTurnRate = 0.0f;
float turnOutput = 0.0f;
float lastTurnError = 0.0f;

namespace remote {

// ============================================================
// 转向环 PD（原 updateTurnLoop）
// 当前Kp/Kd = 0
// ============================================================
// updateTurn — 作用：转向环 PD 计算，再把结果交给 manualTurn 状态机做安全限幅，输出 turnOutput。
// 参数：无（全部读全局）。返回：无（写 turnOutput）。调用：main loop 每 400Hz 一次（在直立环之后）。
// 边界：turnError/derivative 可能很大，但 manualTurn.update() 会把最终输出钳在 cap 内；
//   targetTurnRate 恒 0，因此 turnError = −gyroTurnRate（车被转动时反向阻尼）。
void updateTurn() {
  // 转向误差（deg/s）= 期望偏航角速度 − 陀螺实测偏航角速度（gyroTurnRate 由姿态估计给出）。
  float turnError = targetTurnRate - gyroTurnRate;
  // D 项：误差差分 / CONTROL_DT（0.0025s）→ deg/s²；lastTurnError 每拍更新，构成高通阻尼。
  float derivative = (turnError - lastTurnError) / CONTROL_DT;
  lastTurnError = turnError;
  // legacy PD 输出（STEP Hz 差模）。T/U 编译默认 0，于是该项当前恒 0，输出完全由 manualTurn.requested 决定。
  float legacy = turnKp * turnError + turnKd * derivative;
  // 交给状态机：输入本车倾角误差（deg）、单轮平衡限幅 L（Hz）、共模平衡输出（Hz）、legacy PD（Hz）；
  // 状态机内部决定 room/cap/authority、超时与闩锁，返回值即最终差模 STEP Hz。
  turnOutput = manualTurn.update(millis(), CONTROL_DT, pitch - balanceAngleOffset, BALANCE_OUTPUT_LIMIT_HZ, finalBalanceOutput, legacy);
}

}  // namespace remote
