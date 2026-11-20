#pragma once
// ============================================================
// control/balance.{h,cpp} — 直立 PD → finalBalanceOutput
// 含死区、限幅、换向软斜坡、加速度整形；不含任何 BLE/字符串。
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"
#include "control/motor.h"

namespace control {

// 原 updateBalanceLoop()：400Hz 直立环
// updateBalance：400Hz 直立环：角度 PD → 频率增量（100Hz 端点）→ 400Hz 小步积分 → 限幅输出 finalBalanceOutput。
//   无参/无返回 ｜ 调用：main.cpp loop 每拍 400Hz ｜ 写 balanceOutput/finalBalanceOutput 等全局量。
//   边界：增量先过 S/H/换向软限幅，绝对频率再过 L 限幅；不直接输出到电机（由 mixMotors 合成）。
void updateBalance();

// 停机/故障统一清零：控制侧输出与积分（原 clearControlOutputDiagnostics 的控制部分）
// clearOutputDiagnostics：停机/故障统一清零控制侧输出与积分（无参/无返回）。
//   调用：main 的倒地/掉拍/IMU 失效/欠压路径（400Hz 事件）｜ 只清控制量与积分，不清远控状态机。
void clearOutputDiagnostics();

}  // namespace control
