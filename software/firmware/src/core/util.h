#pragma once
// ============================================================
// core/util.h — 无状态小工具（替代 ManualTurnControl::clamp 的公共实现）
// 算式与 ManualTurnControl::clamp 逐字相同：x<lo?lo:(x>hi?hi:x)
// ============================================================

// 上下限钳位。NaN 会原样穿过（与 ManualTurnControl::clamp 一致）。
// 作用：把 x 钳到 [lo, hi] 区间，返回钳位后的值。
// 参数：x=待钳位值（任意 float）；lo/hi=下/上限（调用方保证 lo <= hi，否则返回 lo）。
// 返回：x<lo 返回 lo；x>hi 返回 hi；其余返回 x 本身。
// 调用者与频率：control/ 与 remote/ 各模块在命令/参数更新时调用，非周期入口。
// 边界行为：NaN 原样穿过（比较恒 false）；±Inf 会被钳到对应极限；不做类型转换、不改变量纲。
// 实现说明：static inline，每个翻译单元各自内联一份，无外部符号/无 RAM 开销；
//   与 remote/manual_turn.h 的 ManualTurnControl::clamp 算式逐字相同（x<lo?lo:(x>hi?hi:x)）。
static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// 有限性检查（封装 std::isfinite，避免各处 include <cmath>）。
// 作用：判断 float 是否为有限值（非 NaN、非 ±Inf）。
// 参数：v=任意 float。返回：true=有限值；false=NaN 或 ±Inf。
// 调用者与频率：函数级工具，供需要“先验性”检查的模块调用（本工程主要在命令解析/参数应用路径）。
// 边界行为：纯查表式判断，无副作用；封装 std::isfinite 以避免各文件重复 include <cmath>。
static inline bool isFiniteValue(float v) {
  return isfinite(v);
}
