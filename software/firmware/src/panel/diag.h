#pragma once
// ============================================================
// panel/diag.{h,cpp} — 计数器与 250ms 窗口极值
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"

namespace panel {

// loop 末尾每 400Hz 控制拍调用：累积 250ms 遥测窗口极值
void diagAccumulate();

// sendSlow() 取完值后调用：窗口清零、bias 以当拍值重开
void diagResetWindow();

}  // namespace panel
