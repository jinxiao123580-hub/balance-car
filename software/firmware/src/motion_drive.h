#pragma once
#include <cmath>
#include "manual_turn.h"

// 前后运动统一驱动（2026-11-05）
// ---------------------------------------------------------------
// 旧做法：操作量直接写进 targetBalanceAngle（最多 2° 死倾角），同时把速度环、
// 位置环全部清零。平衡环被"摘出去"，倾角只能靠人给，推起来就倒。
// 新做法：操作量只决定速度目标，倾角由速度反馈闭环生成，上限更小且带速率限制。
// 平衡环全程参与，倾角不可能跑到开环的大角度。
struct MotionSpeedProfile {
    static constexpr float maxSpeed = 5.0f;   // count/50ms，约 100 count/s
    static constexpr float accel    = 10.0f;  // count/50ms per s，0→满速约 0.5 s
    float applied = 0.0f;
    void reset() { applied = 0.0f; }
    float update(float target, float dt) {
        if (!std::isfinite(target) || !std::isfinite(dt) || dt <= 0.0f) return applied;
        float step = ManualTurnControl::clamp(target - applied, -accel * dt, accel * dt);
        applied += step;
        if (std::fabs(applied) < 0.0001f) applied = 0.0f;
        return applied;
    }
};

struct MotionLeanControl {
    static constexpr float feedForwardGain = 0.10f;  // deg per (count/50ms) 参考速度：克服静摩擦
    static constexpr float speedErrorGain  = 0.05f;  // deg per (count/50ms) 速度误差
    static constexpr float limit           = 1.00f;  // deg，绝对上限（旧开环为 2°，且无反馈）
    static constexpr float engageRate      = 2.50f;  // deg/s
    static constexpr float releaseRate     = 3.50f;  // deg/s，释放段可以快一点
    float requested = 0.0f;
    float applied = 0.0f;
    void reset() { requested = applied = 0.0f; }
    // 进入运动的无缝衔接：用当前速度环已输出的倾角作为初值，避免目标角在入口跳变。
    void seed(float offset) {
        if (!std::isfinite(offset)) offset = 0.0f;
        applied = ManualTurnControl::clamp(offset, -limit, limit);
        requested = applied;
    }
    bool busy() const { return std::fabs(applied) > 0.0001f; }
    // engaged=false：请求回 0，按 releaseRate 平滑释放，不产生角度阶跃。
    void update(float speedReference, float measuredSpeed, bool engaged, float dt) {
        if (!std::isfinite(speedReference) || !std::isfinite(measuredSpeed) ||
            !std::isfinite(dt) || dt <= 0.0f) {
            reset();
            return;
        }
        if (engaged) {
            float speedError = speedReference - measuredSpeed;
            requested = ManualTurnControl::clamp(
                feedForwardGain * speedReference + speedErrorGain * speedError,
                -limit, limit);
        } else {
            requested = 0.0f;
        }
        float rate = engaged ? engageRate : releaseRate;
        applied += ManualTurnControl::clamp(requested - applied, -rate * dt, rate * dt);
        if (std::fabs(applied) < 0.0001f) applied = 0.0f;
    }
};
