#include "remote/manual_turn.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// ============================================================
// 转向环参数（面板 T/U 可在线调）
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
void updateTurn() {
  float turnError = targetTurnRate - gyroTurnRate;
  float derivative = (turnError - lastTurnError) / CONTROL_DT;
  lastTurnError = turnError;
  float legacy = turnKp * turnError + turnKd * derivative;
  turnOutput = manualTurn.update(millis(), CONTROL_DT, pitch - balanceAngleOffset, BALANCE_OUTPUT_LIMIT_HZ, finalBalanceOutput, legacy);
}

}  // namespace remote
