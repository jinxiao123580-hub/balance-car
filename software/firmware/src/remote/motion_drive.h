#pragma once
// ============================================================
// remote/motion_drive.{h,cpp} — 前后运动四件套（斜坡/净速度/位置计划/倾角）
// 代码原样搬自 src/motion_drive.h；100Hz 调度见 motionDriveTick()。
// ============================================================
#include <Arduino.h>
#include <cmath>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"
#include "remote/manual_turn.h"
#include "remote/manual_pitch.h"


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
    // 2026-11-10：满档 8.0 证伪 —— gear 1.5(vref 6) 双向不一致、2.0 冲出 12.65°。
    // 受控速度上限落在 vref 4~5 之间（gear 1.0 可重复跟踪），满档降到 5.0，
    // 于是 2.0 档的目标速度 = 5 count/50ms = 100 count/s，落在已验证的区间内。
    static constexpr float maxSpeed = 5.0f;  // count/50ms，约 100 count/s（满档）
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

// 预测位置环（2026-11-10 实验）：位置计划 = 参考速度的积分；误差用"预测位置"
// （当前位置 + 净速度 × 前视时间）来抵消 200 ms 净速度窗口的滞后；输出只是速度增量，
// 叠加到速度环参考上，不直接改倾角，所以不会和速度环抢目标角。
// 未运动时计划跟随实际位置（B 方案：不回起点，只在停车后由驻车环抗漂移）。
struct MotionPositionPlan {
    static constexpr float horizon = 0.15f;   // s，前视时间
    float plan = 0.0f;
    bool active = false;
    void start(float position) { plan = position; active = true; }
    void stop(float position) { plan = position; active = false; }
    float update(float speedRef, float position, float netSpeed, bool engaged, float dt,
                 float gain, float limit, float deadband) {
        if (!std::isfinite(position) || !std::isfinite(netSpeed) ||
            !std::isfinite(dt) || dt <= 0.0f) {
            return 0.0f;
        }
        if (engaged && active) {
            plan += speedRef * (dt / 0.05f);          // counts：1 count/50ms = 20 counts/s
        } else {
            plan = position;                          // 未运动：计划跟随实际，不产生回位
        }
        float predicted = position + netSpeed * (horizon / 0.05f);
        float error = plan - predicted;
        if (std::fabs(error) <= deadband) return 0.0f;
        error -= (error > 0.0f) ? deadband : -deadband;
        return ManualTurnControl::clamp(gain * error, -limit, limit);
    }
};

struct MotionLeanControl {
    // 2026-11-06 第四次修正：1.2° 上限处实车反而最慢（|lean|=1.2 时中位净速率 41 counts/s，\n    // 0.5~0.7° 时 178~202）——贴边会进入"摇不动"的极限环，所以上限降到 0.90°。\n    // 前一次：0.15 的误差增益对 200ms 净速度外环太高——实车出现
    // 超速（-50 counts/50ms）→ 驻车环满权限 2° 刹车 → 机身 13° → 摔倒。
    // 降增益、降上限：静止满档也只到 1.2°，速度接近目标时只剩摩擦补偿。
    static constexpr float feedForwardGain = 0.06f;  // deg per (count/50ms) 参考速度：只补静摩擦
    static constexpr float speedErrorGain  = 0.05f;  // deg per (count/50ms) 速度误差：主调节（默认值）
    // 2026-11-10：误差增益改成运行时可调（蓝牙 EG）。实测 vref 5 时稳态落在 6.7 c/50ms
    // 而不是 5 —— 前馈 0.30° 一个人就把车推到超速，误差项 0.05 太软拉不回来。
    float errorGain = speedErrorGain;
    static constexpr float limit           = 0.90f;  // deg，绝对上限（旧开环为 2°，且无反馈）
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
    // measuredSpeed 应为 NetSpeedEstimator 的 200ms 净速度；engaged=false 时按 releaseRate 平滑释放。
    void update(float speedReference, float measuredSpeed, bool engaged, float dt) {
        if (!std::isfinite(speedReference) || !std::isfinite(measuredSpeed) ||
            !std::isfinite(dt) || dt <= 0.0f) {
            reset();
            return;
        }
        if (engaged) {
            float speedError = speedReference - measuredSpeed;
            float base = feedForwardGain * speedReference + errorGain * speedError;

            requested = ManualTurnControl::clamp(base, -limit, limit);
        } else {
            requested = 0.0f;
        }
        float rate = engaged ? engageRate : releaseRate;
        applied += ManualTurnControl::clamp(requested - applied, -rate * dt, rate * dt);
        if (std::fabs(applied) < 0.0001f) applied = 0.0f;
    }
};

// 运动驱动四件套实例 + 运动状态标志（定义在 remote/remote_control.cpp）
extern MotionSpeedProfile motionSpeedProfile;
extern MotionLeanControl motionLean;
extern MotionPositionPlan motionPlan;
extern NetSpeedEstimator motionNetSpeed;
extern bool manualMotionWasBusy;

// 运动驱动四件套实例 + 运动状态标志（定义在 remote/remote_control.cpp）
extern MotionSpeedProfile motionSpeedProfile;
extern MotionLeanControl motionLean;
extern MotionPositionPlan motionPlan;
extern NetSpeedEstimator motionNetSpeed;
extern bool manualMotionWasBusy;

namespace remote {

// 100Hz 运动驱动（原 updateOuterLoop 的 manualForward.busy() 分支 + 释放段）。
// 返回 true = 运动驱动接管本拍（原代码的 "return"，速度 PI 必须跳过）。
bool motionDriveTick();

}  // namespace remote
