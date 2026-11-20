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


// ============================================================
// 前后运动统一驱动（^ 通路的 forward 分量）：四个小对象串成一条"档位→速度→倾角"链。
//   MotionSpeedProfile  目标速度斜坡：档位归一化值 × maxSpeed，按 accel 限斜率；
//   NetSpeedEstimator   200ms 净速度：位置差分 + 3 点中值，专门喂外环（抑制位置极限环摆动）；
//   MotionPositionPlan  预测位置环：参考速度积分成计划位置，用"位置 + 净速度×前视"算误差，输出速度增量；
//   MotionLeanControl   速度误差 → 倾角（前馈只补静摩擦，主调节靠误差增益），输出 motionLean.applied。
// 数据流：manualForward.planned（deg 档位）→ normalized×maxSpeed（count/50ms）→ +位置修正
//   → motionLean.applied（deg，经 remote::leanOffset() 进外环权限）→ 叠加到 targetBalanceAngle。
// 全部由 remote::motionDriveTick() 在 100Hz 外环里推进（dt=0.01s），只在 ^ forward≠0 时接管本拍。
// ============================================================
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
// MotionSpeedProfile — 速度目标斜坡：把"阶跃的档位目标"变成限加速度的速度参考。
// 输入 target 单位 count/50ms；输出 applied 同单位；dt 单位 s。
// 为什么需要它：档位由人按出来是阶跃的，直接当速度目标会让倾角目标瞬间跳变 → 轮子猛冲/失步。
struct MotionSpeedProfile {
    // 单位换算（调参必背）：10.8 编码器 count = 1 mm，故
    //   1 count/50ms = 20 counts/s = 20/10.8 ≈ 1.85 mm/s；
    //   maxSpeed=5 count/50ms = 100 counts/s ≈ 9.3 mm/s；满档 5 也写作 SP≈5（与面板 SP 同单位）。
    // 2026-11-10：满档 8.0 证伪 —— gear 1.5(vref 6) 双向不一致、2.0 冲出 12.65°。
    // 受控速度上限落在 vref 4~5 之间（gear 1.0 可重复跟踪），满档降到 5.0，
    // 于是 2.0 档的目标速度 = 5 count/50ms = 100 count/s，落在已验证的区间内。
    // maxSpeed=5.0：满档（归一化档位 1.0）对应的速度参考（count/50ms）。
    // 历史注释里的"2.0 档"指 ^ forward=2.0，即档位归一化后 2.0/2.0=1.0 → ×5.0 = 5 count/50ms
    // = 100 counts/s ≈ 9.3 mm/s；这正是实测可重复跟踪、且不冲出保护区的区间上限。
    static constexpr float maxSpeed = 5.0f;  // count/50ms，约 100 count/s（满档）
    // （注：旧注释"0→满速约 0.5 s"与当前实现不符，以代码为准：accel=20 count/50ms/s、maxSpeed=5，
    //   0→满速需要 5/20=0.25 s。）
    static constexpr float accel    = 20.0f;  // count/50ms per s，0→满速约 0.5 s
    // applied：当前速度参考（count/50ms，±maxSpeed）。写：update() 每 100Hz；读：motionDriveTick()/释放判断。
    float applied = 0.0f;
    // reset — 作用：速度参考清零。参数/返回：无。调用：每次进入 ^ 前后运动时（入口无缝衔接）。
    void reset() { applied = 0.0f; }
    // update — 作用：按 accel 限斜率逼近 target，返回新的速度参考。
    // 参数 target：目标速度（count/50ms）；dt：周期 s（100Hz→0.01）。返回：applied（count/50ms）。
    // 调用：motionDriveTick() 每 100Hz 一次。边界：target/dt 非有限或 dt≤0 → 原值返回；|applied|<1e-4 归零。
    float update(float target, float dt) {
        if (!std::isfinite(target) || !std::isfinite(dt) || dt <= 0.0f) return applied;
        float step = ManualTurnControl::clamp(target - applied, -accel * dt, accel * dt);
        applied += step;
        // 抗漂移：极小值直接吸附到 0，避免长时间停留 1e-7 量级的"假速度"。
        if (std::fabs(applied) < 0.0001f) applied = 0.0f;
        return applied;
    }
};

// 200ms 净速度估计：位置差分 + 3 点中值抑制单拍跳变。
// 100Hz 调用；输出单位与 telemetry speed1000 一致（counts/50ms）。
// 只用于前后运动外环，不影响直立/速度内环的瞬时速度反馈。
// NetSpeedEstimator — 200ms 净速度估计。
// 输入 position（编码器累计位置，count）；输出（count/50ms）：(filtered − oldest)/0.2s × 0.05。
// 量纲：count/s = count ÷ 0.2s；×0.05 把 count/s 换成 count/50ms（同一时间基准，与全工程速度单位统一）。
// 为什么不用 50ms 速度窗口：实车日志显示 50ms 窗口在 0.5s 内摆动 ±29 counts、66% 相邻样本换向，
// 那是位置环极限环而不是传感器噪声；200ms 窗口 + 3 点中值只留真正的位移趋势。
// 调用：motionDriveTick() 每 100Hz。只用于前后运动外环，不参与速度内环/直立环反馈。
struct NetSpeedEstimator {
    // windowSamples=20：100Hz × 20 = 200ms 窗口。history 长度 21（多一格用于取"最旧值"）。
    static constexpr int windowSamples = 20;      // 100Hz × 200ms
    // windowSeconds=0.2：窗口时长（s），除以它把 count 差换成 count/s。
    static constexpr float windowSeconds = 0.2f;
    // history：环形缓冲，存中值滤波后的位置（count）；长度 21 保证总能读到 200ms 前的样本。
    float history[windowSamples + 1] = {};
    // head：环形缓冲写指针（0..windowSamples）。
    int head = 0;
    // seeded：是否已建立基准。首次 update 只做 reset 并返回 0，避免拿脏历史差分出巨大速度。
    bool seeded = false;
    // last1/last2：最近两个原始位置（count），供 3 点中值滤波。
    float last1 = 0.0f, last2 = 0.0f;
    // median3 — 作用：3 点中值滤波，抑制单拍位置跳变（编码器噪声/丢脉冲）。
    // 参数 a/b/c：连续 3 个位置样本（count）。返回：三者中位数。调用：update() 每 100Hz。
    static float median3(float a, float b, float c) {
        float lo = (a < b) ? a : b;
        float hi = (a < b) ? b : a;
        if (c < lo) return lo;
        if (c > hi) return hi;
        return c;
    }
    // reset — 作用：用给定位置重建整窗基准（动车/进入运动时调用，防止历史跳变）。
    // 参数 position：当前位置（count），默认 0；返回：无。调用：motionDriveTick() 入口 / 首次 update。
    void reset(float position = 0.0f) {
        for (int i = 0; i <= windowSamples; ++i) history[i] = position;
        head = 0;
        last1 = last2 = position;
        seeded = true;
    }
    // update — 作用：推进 200ms 窗口，返回净速度（count/50ms）。
    // 参数 position：编码器累计位置（count）。返回：净速度（count/50ms，带符号）。
    // 调用：motionDriveTick() 每 100Hz。边界：position 非有限 → 返回 0（不污染窗口）；未初始化先 reset 再返回 0。
    float update(float position) {
        if (!std::isfinite(position)) return 0.0f;
        if (!seeded) { reset(position); return 0.0f; }
        float filtered = median3(last2, last1, position);
        last2 = last1;
        last1 = position;
        history[head] = filtered;
        head = (head + 1) % (windowSamples + 1);
        float oldest = history[head];
        // 量纲：两种写法等价——(filtered−oldest)/0.2s [count/s] × 0.05 [s/50ms] = count/50ms。
        // counts/50ms = counts/s × 0.05
        return (filtered - oldest) / windowSeconds * 0.05f;
    }
};

// 预测位置环（2026-11-10 实验）：位置计划 = 参考速度的积分；误差用"预测位置"
// （当前位置 + 净速度 × 前视时间）来抵消 200 ms 净速度窗口的滞后；输出只是速度增量，
// 叠加到速度环参考上，不直接改倾角，所以不会和速度环抢目标角。
// 未运动时计划跟随实际位置（B 方案：不回起点，只在停车后由驻车环抗漂移）。
// MotionPositionPlan — 预测位置环（2026-11-10 实验）：只在 ^ 前后运动期间启用。
// 原理：计划位置 plan = 参考速度的积分；误差 = plan − (position + netSpeed×horizon)。
//   · 前视项 netSpeed×horizon 补偿 200ms 净速度窗口的固有滞后（窗口平均 ≈ 重心滞后 ~0.1s，取 0.15s 折中）；
//   · 输出只是"速度增量"（count/50ms），叠加到速度参考上，不直接改倾角 → 不会和速度环抢目标角；
//   · 未运动时 plan 跟随实际位置（B 方案：不强行回起点，停车后交给驻车环抗漂移）。
// 参数 gain=positionKp（X，默认 0.02）、limit=MAX_POSITION_RETURN_SPEED（Y，0.30 count/50ms）、
//   deadband=POSITION_DEADBAND_COUNT（M，12 count ≈1.1mm）。
struct MotionPositionPlan {
    // horizon=0.15 s：前视时间。窗口 200ms，取 150ms 是"补偿滞后"与"不放大噪声"的折中（实测值）。
    static constexpr float horizon = 0.15f;   // s，前视时间
    // plan：计划位置（count，与 vehiclePosition 同坐标）。只有 engaged 且 active 时才按速度参考积分。
    float plan = 0.0f;
    // active：计划是否处于"已启动"状态；false 时 plan 跟随实际位置（不产生回位动作）。
    bool active = false;
    // start — 作用：以当前位置为起点启动计划。参数 position：当前位置（count）。返回：无。
    void start(float position) { plan = position; active = true; }
    // stop — 作用：结束计划并把 plan 对齐到当前位置（避免下次启动时突然回位）。
    void stop(float position) { plan = position; active = false; }
    // update — 作用：推进计划位置并算速度增量输出。
    // 参数：speedRef 速度参考（count/50ms）；position 实际位置（count）；netSpeed 净速度（count/50ms）；
    //   engaged 是否在运动中；dt s；gain/limit/deadband 见上。返回：速度增量（count/50ms，±limit）。
    // 调用：motionDriveTick() 每 100Hz。边界：任一输入非有限或 dt≤0 → 返回 0；|error|≤deadband → 返回 0（死区）。
    float update(float speedRef, float position, float netSpeed, bool engaged, float dt,
                 float gain, float limit, float deadband) {
        if (!std::isfinite(position) || !std::isfinite(netSpeed) ||
            !std::isfinite(dt) || dt <= 0.0f) {
            return 0.0f;
        }
        if (engaged && active) {
            // 积分量纲：speedRef[count/50ms] × (dt/0.05) = count（1 count/50ms = 20 counts/s，dt 秒内走 20×dt count）。
            plan += speedRef * (dt / 0.05f);          // counts：1 count/50ms = 20 counts/s
        } else {
            plan = position;                          // 未运动：计划跟随实际，不产生回位
        }
        // 前视量纲：netSpeed[count/50ms] × (horizon/0.05) = count，即"再过 horizon 秒会到哪"。
        float predicted = position + netSpeed * (horizon / 0.05f);
        float error = plan - predicted;
        if (std::fabs(error) <= deadband) return 0.0f;
        error -= (error > 0.0f) ? deadband : -deadband;
        return ManualTurnControl::clamp(gain * error, -limit, limit);
    }
};

// MotionLeanControl — 速度闭环 → 倾角：^ 前后运动的最终执行层（输出 deg，进外环权限）。
// 公式：base = feedForwardGain×speedRef + errorGain×(speedRef − measuredSpeed)，钳 ±limit，再按速率爬升。
// 为什么这么分：前馈只把"静摩擦死区"补掉（满档 0.30°），主调节留给误差项；
//   旧版 0.15 增益 + 2° 上限实车会超速→满权限刹车→13°→摔倒，所以降增益 + 降上限到 0.90°。
// 单位：输入 count/50ms；输出 deg；速率 deg/s。调用：motionDriveTick() 每 100Hz。
struct MotionLeanControl {
    // 2026-11-06 第四次修正：1.2° 上限处实车反而最慢（|lean|=1.2 时中位净速率 41 counts/s，\n    // 0.5~0.7° 时 178~202）——贴边会进入"摇不动"的极限环，所以上限降到 0.90°。\n    // 前一次：0.15 的误差增益对 200ms 净速度外环太高——实车出现
    // 超速（-50 counts/50ms）→ 驻车环满权限 2° 刹车 → 机身 13° → 摔倒。
    // 降增益、降上限：静止满档也只到 1.2°，速度接近目标时只剩摩擦补偿。
    // feedForwardGain=0.06 deg per (count/50ms)：参考速度的纯前馈，只补静摩擦/死区，不负责跟速。
    // 满档 5 count/50ms × 0.06 = 0.30°（实测破粘门限 0.5~0.7°，前馈只补一部分，其余靠误差项）。
    // 注意它不随误差变化：稳态若落在"前馈偏大"，车就会被一路推到超速（见 2026-11-10 记录），故上限 0.9°。
    static constexpr float feedForwardGain = 0.06f;  // deg per (count/50ms) 参考速度：只补静摩擦
    // speedErrorGain=0.05 deg per (count/50ms)：速度误差的主调节增益（默认值，可用 EG 在 0~0.15 在线改）。
    // 例：误差 2 count/50ms（≈3.7mm/s）→ 0.10° 倾角。单独前馈按 2026-11-10 实测会超速，得靠这一项拉回。
    static constexpr float speedErrorGain  = 0.05f;  // deg per (count/50ms) 速度误差：主调节（默认值）
    // 2026-11-10：误差增益改成运行时可调（蓝牙 EG）。实测 vref 5 时稳态落在 6.7 c/50ms
    // 而不是 5 —— 前馈 0.30° 一个人就把车推到超速，误差项 0.05 太软拉不回来。
    // errorGain：运行时误差增益（默认=speedErrorGain=0.05）。写：面板 EG 命令（RAM-only）；读：update() 每 100Hz。
    float errorGain = speedErrorGain;
    // limit=0.90：倾角绝对上限（deg）。旧开环做法是 2° 死倾角且无反馈；现在 0.9° 是实测稳定边界（贴边会进极限环）。
    static constexpr float limit           = 0.90f;  // deg，绝对上限（旧开环为 2°，且无反馈）
    // engageRate=2.50 deg/s：进入运动/增大倾角时的斜率上限（400Hz 内部实际按 100Hz dt=0.01 调用 → 每拍 0.025°）。
    static constexpr float engageRate      = 2.50f;  // deg/s
    // releaseRate=3.50 deg/s：释放段斜率（可以快一点，尽快把残余倾角收掉，但仍是斜率限制不是阶跃）。
    static constexpr float releaseRate     = 3.50f;  // deg/s，释放段可以快一点
    // requested：本拍期望倾角（deg，±limit），由公式算出；applied：实际输出（斜率先行，供 remote::leanOffset()）。
    float requested = 0.0f;
    float applied = 0.0f;
    // reset — 作用：清零 requested/applied。调用：resetFault()、非有限输入兜底、释放结束。
    void reset() { requested = applied = 0.0f; }
    // 进入运动的无缝衔接：用当前速度环已输出的倾角作为初值，避免目标角在入口跳变。
    // seed — 作用：进入运动时用"速度环已输出的倾角"做初值，避免目标角在入口跳变。
    // 参数 offset：当前 speedAngleCorrection（deg，|offset|≤N≈0.8°）。边界：非有限→0；钳 ±limit。
    void seed(float offset) {
        if (!std::isfinite(offset)) offset = 0.0f;
        applied = ManualTurnControl::clamp(offset, -limit, limit);
        requested = applied;
    }
    // busy — 作用：是否还有残余倾角（释放段仍在动）。返回 bool；调用：motionDriveTick()/释放判断（100Hz）。
    bool busy() const { return std::fabs(applied) > 0.0001f; }
    // measuredSpeed 应为 NetSpeedEstimator 的 200ms 净速度；engaged=false 时按 releaseRate 平滑释放。
    // update — 作用：速度闭环 → 倾角，含前馈/误差项/限幅/斜率。
    // 参数：speedReference 参考速度（count/50ms，含位置修正）；measuredSpeed 200ms 净速度（count/50ms）；
    //   engaged 是否在运动中（false=释放）；dt s（100Hz→0.01）。返回：无（写 applied，供 leanOffset() 读）。
    // 调用：motionDriveTick() 每 100Hz。边界：非有限或 dt≤0 → reset() 并 return（防 NaN 进倾角）。
    void update(float speedReference, float measuredSpeed, bool engaged, float dt) {
        if (!std::isfinite(speedReference) || !std::isfinite(measuredSpeed) ||
            !std::isfinite(dt) || dt <= 0.0f) {
            reset();
            return;
        }
        if (engaged) {
            float speedError = speedReference - measuredSpeed;
            // 前馈项 feedForwardGain×speedReference（deg）+ 误差项 errorGain×(参考−实测)（deg）；两项单位一致，直接相加。
            float base = feedForwardGain * speedReference + errorGain * speedError;

            requested = ManualTurnControl::clamp(base, -limit, limit);
        } else {
            requested = 0.0f;
        }
        // 增速用 engageRate、释放用 releaseRate；applied 按 rate×dt 逼近 requested（100Hz 每拍最多 0.025°/0.035°）。
        float rate = engaged ? engageRate : releaseRate;
        applied += ManualTurnControl::clamp(requested - applied, -rate * dt, rate * dt);
        if (std::fabs(applied) < 0.0001f) applied = 0.0f;
    }
};

// 四件套实例 + 运动状态标志：定义在 remote/remote_control.cpp，此处 extern 供 motion_drive.cpp 使用。
// 运动驱动四件套实例 + 运动状态标志（定义在 remote/remote_control.cpp）
// motionSpeedProfile：速度目标斜坡实例。applied 单位 count/50ms（±5）。写/读：motionDriveTick() 每 100Hz。
extern MotionSpeedProfile motionSpeedProfile;
// motionLean：速度误差→倾角实例。applied 单位 deg（±0.90）。写：motionDriveTick() 每 100Hz；
// 读：remote::leanOffset() → main 每 400Hz 传给 control::updateAuthority()。
extern MotionLeanControl motionLean;
// motionPlan：预测位置环实例。plan 单位 count；写/读：motionDriveTick() 每 100Hz（仅前后运动期间）。
extern MotionPositionPlan motionPlan;
// motionNetSpeed：200ms 净速度实例。输出 count/50ms；写/读：motionDriveTick() 每 100Hz。
extern NetSpeedEstimator motionNetSpeed;
// manualMotionWasBusy：上一拍运动驱动是否在接管（bool）。写：motionDriveTick()；
// 读：remote::motionWasBusy() → main 传给 control::outerLoop()（用于运动旁路清积分）。
extern bool manualMotionWasBusy;

// （注：下面第二组 extern 声明与上面 154–158 完全重复，属重构合并时的重复声明；C++ 允许，保留原样。）
// 运动驱动四件套实例 + 运动状态标志（定义在 remote/remote_control.cpp）
extern MotionSpeedProfile motionSpeedProfile;
extern MotionLeanControl motionLean;
extern MotionPositionPlan motionPlan;
extern NetSpeedEstimator motionNetSpeed;
extern bool manualMotionWasBusy;

namespace remote {

// motionDriveTick — 作用：100Hz 运动驱动入口（原 updateOuterLoop 的 manualForward.busy() 分支 + 释放段）。
// 参数：无。返回：true=运动驱动接管本拍（调用方必须跳过速度 PI，避免两套目标角打架）。
// 调用：remote::tick100() 每 100Hz，在点动调度之后、位置 P 外环之前。边界：见 .cpp。
// 100Hz 运动驱动（原 updateOuterLoop 的 manualForward.busy() 分支 + 释放段）。
// 返回 true = 运动驱动接管本拍（原代码的 "return"，速度 PI 必须跳过）。
bool motionDriveTick();

}  // namespace remote
