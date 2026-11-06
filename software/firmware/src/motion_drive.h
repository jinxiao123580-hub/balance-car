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
// 0.64°，净位移约 20~30 counts/s，而目标 5 counts/50ms=100 counts/s。
// 50ms 速度窗口在 0.5s 内来回摆 ±29 counts、66% 相邻样本换向 —— 这是位置环极限环
// 的真实摆动，不是传感器白噪声，Kalman 抹不掉，只能换成"更长窗口的净速度"。
// 因此：满档速度/前馈/上限一起抬高，运动环改用 200ms 净速度 + 3 点中值抑制单拍跳变。
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

// 200ms 净速度估计：位置差分 + 3 点中值抑制单拍跳变。
// 100Hz 调用；输出单位与 telemetry speed1000 一致（counts/50ms）。
// 只用于前后运动外环，不影响直立/速度内环的瞬时速度反馈。
struct NetSpeedEstimator {
    static constexpr int windowSamples = 20;      // 100Hz × 200ms
    static constexpr float windowSeconds = 0.2f;
    float history[windowSamples + 1] = {};
    int head = 0;
    bool seeded = false;
    float last1 = 0.0f, last2 = 0.0f;
    static float median3(float a, float b, float c) {
        float lo = (a < b) ? a : b;
        float hi = (a < b) ? b : a;
        if (c < lo) return lo;
        if (c > hi) return hi;
        return c;
    }
    void reset(float position = 0.0f) {
        for (int i = 0; i <= windowSamples; ++i) history[i] = position;
        head = 0;
        last1 = last2 = position;
        seeded = true;
    }
    float update(float position) {
        if (!std::isfinite(position)) return 0.0f;
        if (!seeded) { reset(position); return 0.0f; }
        float filtered = median3(last2, last1, position);
        last2 = last1;
        last1 = position;
        history[head] = filtered;
        head = (head + 1) % (windowSamples + 1);
        float oldest = history[head];
        // counts/50ms = counts/s × 0.05
        return (filtered - oldest) / windowSeconds * 0.05f;
    }
};

struct MotionLeanControl {
    // 2026-11-06 第三次修正：0.15 的误差增益对 200ms 净速度外环太高——实车出现
    // 超速（-50 counts/50ms）→ 驻车环满权限 2° 刹车 → 机身 13° → 摔倒。
    // 降增益、降上限：静止满档也只到 1.2°，速度接近目标时只剩摩擦补偿。
    static constexpr float feedForwardGain = 0.06f;  // deg per (count/50ms) 参考速度：只补静摩擦
    static constexpr float speedErrorGain  = 0.05f;  // deg per (count/50ms) 速度误差：主调节
    static constexpr float limit           = 1.20f;  // deg，绝对上限（旧开环为 2°，且无反馈）
    static constexpr float engageRate      = 2.50f;  // deg/s
    static constexpr float releaseRate     = 3.50f;  // deg/s，释放段可以快一点
    static constexpr float trimDirectionDeadband = 0.20f; // count/50ms，参考速度在此以内不配平
    // 方向配平（2026-11-06）：同一倾角下前进比后退快约 2.2 倍时的补偿量。
    // 两个值都是"正值=该方向更用力"，内部按参考速度方向取符号，只平移运动倾角，
    // 不参与速度环，也不影响驻车；回退置 0 即可。
    float forwardTrim  = 0.00f;  // deg
    float backwardTrim = 0.00f;  // deg
    // 自学习摩擦补偿（实验，2026-11-06）：按方向各学一个偏移，等价于"自动配平 + 消稳态差"。
    // 只在该方向真正滚动(|v|>learnMinSpeed)且没有顶到上限时积分；掉电清零，不写 flash。
    static constexpr float learnGain     = 0.02f;   // deg/s per (count/50ms) 速度误差
    static constexpr float learnLimit    = 0.30f;   // deg，每方向学习量上限
    static constexpr float learnMinSpeed = 0.80f;   // count/50ms，未真正滚动就不学
    float learnedForward = 0.0f;
    float learnedBackward = 0.0f;
    float lastLearned = 0.0f;                       // 遥测：最近一次参与方向的学习量
    float requested = 0.0f;
    float applied = 0.0f;
    void reset() { requested = applied = 0.0f; learnedForward = learnedBackward = lastLearned = 0.0f; }
    // 进入运动的无缝衔接：用当前速度环已输出的倾角作为初值，避免目标角在入口跳变。
    void seed(float offset) {
        if (!std::isfinite(offset)) offset = 0.0f;
        applied = ManualTurnControl::clamp(offset, -limit, limit);
        requested = applied;
    }
    bool busy() const { return std::fabs(applied) > 0.0001f; }
    // measuredSpeed 应为 NetSpeedEstimator 的 200ms 净速度；engaged=false 时按 releaseRate 平滑释放。
    void update(float speedReference, float measuredSpeed, bool engaged, float dt) {
        if (!std::isfinite(speedReference) || !std::isfinite(measuredSpeed) ||
            !std::isfinite(dt) || dt <= 0.0f) {
            reset();
            return;
        }
        if (engaged) {
            float speedError = speedReference - measuredSpeed;
            bool forward = speedReference > trimDirectionDeadband;
            bool backward = speedReference < -trimDirectionDeadband;
            float trim = forward ? forwardTrim : (backward ? -backwardTrim : 0.0f);
            float base = feedForwardGain * speedReference + speedErrorGain * speedError + trim;

            float add = 0.0f;
            if (forward || backward) {
                float &learned = forward ? learnedForward : learnedBackward;
                bool rolling = std::fabs(measuredSpeed) > learnMinSpeed;
                bool pushing = (base + learned) * speedError > 0.0f;
                bool saturated = pushing && std::fabs(base + learned) >= limit;
                if (rolling && !saturated) {
                    learned += learnGain * speedError * dt;
                    if (learned > learnLimit) learned = learnLimit;
                    else if (learned < -learnLimit) learned = -learnLimit;
                }
                add = learned;
                lastLearned = learned;
            }
            requested = ManualTurnControl::clamp(base + add, -limit, limit);
        } else {
            requested = 0.0f;
        }
        float rate = engaged ? engageRate : releaseRate;
        applied += ManualTurnControl::clamp(requested - applied, -rate * dt, rate * dt);
        if (std::fabs(applied) < 0.0001f) applied = 0.0f;
    }
};
