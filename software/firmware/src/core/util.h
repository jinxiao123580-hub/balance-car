#pragma once
// ============================================================
// core/util.h — 无状态小工具（替代 ManualTurnControl::clamp 的公共实现）
// 算式与 ManualTurnControl::clamp 逐字相同：x<lo?lo:(x>hi?hi:x)
// ============================================================

// 上下限钳位。NaN 会原样穿过（与 ManualTurnControl::clamp 一致）。
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// 有限性检查（封装 std::isfinite，避免各处 include <cmath>）。
static inline bool isFiniteValue(float v) {
  return isfinite(v);
}
