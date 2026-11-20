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
void updateBalance();

}  // namespace control
