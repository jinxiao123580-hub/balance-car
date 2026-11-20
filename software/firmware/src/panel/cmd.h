#pragma once
// ============================================================
// panel/cmd.{h,cpp} — BLE 命令解析
// 单字母22项 + SP/MS/SV + SAVE/LOAD/DEF/?/! + ^ / ~ & 转发远控
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "panel/params.h"
#include "panel/telemetry.h"

#include "control/balance.h"
#include "control/cascade.h"
#include "control/motor.h"
#include "control/imu.h"
#include "remote/remote_control.h"
#include "remote/manual_pitch.h"
#include "remote/manual_turn.h"
#include "remote/position_jog.h"

namespace panel {

// 收行 → 分发（原 pollBtCommand；main 每圈开头调用）
void poll();

// 处理一整行命令（原 handleBtCommand）
void handleLine(char * cmd, uint8_t len);

}  // namespace panel
