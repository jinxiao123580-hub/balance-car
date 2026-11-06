#pragma once
#include <cmath>
#include "manual_turn.h"

// 前后运动统一驱动（2026-11-05 建立，2026-11-06 按实车数据重标）
// ---------------------------------------------------------------
// 旧做法：操作量直接写进 targetBalanceAngle（最多 2° 死倾角），同时把速度环、
// 位置环全部清零。平衡环被"摘出去"，倾角只能靠人给，推起来就倒。
// 新做法：操作量只决定速度目标，倾角由速度反馈闭环生成，上限更小且带速率限制。
//
// 2026-11-06 实车记录（ble_tune_log_20261106_090136）：满档时平均有效倾角只有
// 0.64°，净位移约 20~30 counts/s，而目标 5 counts/50ms=100 counts/s；速度反馈的
// 50ms 噪声约 ±8 counts/50ms，比目标本身还大，误差项把倾角打得在 0.4~1.0° 之间跳。
// 因此把速度目标、前馈和上限一起抬高，并给速度反馈加一阶低通，降低噪声占比。
struct MotionSpeedProfile {
    static constexpr float maxSpeed = 10.0f;  // count/50ms，约 200 count/s（满档）
    static constexpr float accel    = 20.0f;  // count/50ms per s，0→满速约 0.5 s
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
    static constexpr float feedForwardGain = 0.12f;  // deg per (count/50ms) 参考速度：克服静摩擦
    static constexpr float speedErrorGain  = 0.02f;  // deg per (count/50ms) 速度误差
    static constexpr float limit           = 1.50f;  // deg，绝对上限（旧开环为 2°，且无反馈）
    static constexpr float engageRate      = 2.50f;  // deg/s
    static constexpr float releaseRate     = 3.50f;  // deg/s，释放段可以快一点
    static constexpr float speedFilterAlpha = 0.30f; // 100Hz 下一阶低通，约 30ms 时间常数
    float requested = 0.0f;
    float applied = 0.0f;
    float filteredSpeed = 0.0f;
    bool filterSeeded = false;
    void reset() { requested = applied = 0.0f; filteredSpeed = 0.0f; filterSeeded = false; }
    // 进入运动的无缝衔接：用当前速度环已输出的倾角作为初值，避免目标角在入口跳变。
    void seed(float offset) {
        if (!std::isfinite(offset)) offset = 0.0f;
        applied = ManualTurnControl::clamp(offset, -limit, limit);
        requested = applied;
        filterSeeded = false;
    }
    bool busy() const { return std::fabs(applied) > 0.0001f; }
    // engaged=false：请求回 0，按 releaseRate 平滑释放，不产生角度阶跃。
    void update(float speedReference, float measuredSpeed, bool engaged, float dt) {
        if (!std::isfinite(speedReference) || !std::isfinite(measuredSpeed) ||
            !std::isfinite(dt) || dt <= 0.0f) {
            reset();
            return;
        }
        if (!filterSeeded) { filteredSpeed = measuredSpeed; filterSeeded = true; }
        filteredSpeed += speedFilterAlpha * (measuredSpeed - filteredSpeed);
        if (engaged) {
            float speedError = speedReference - filteredSpeed;
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
