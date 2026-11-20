#pragma once
// ============================================================
// remote/manual_turn.{h,cpp} — 转向状态机 + 转向环 PD
// （代码原样搬自 src/manual_turn.h 与 main.cpp 的 updateTurnLoop）
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"


// ============================================================
// ManualTurnControl — 转向状态机：把 ^ 命令里的 turnHz 平滑成"差模 STEP 频率"，并做安全限幅。
// 注意：这里的 Hz 是 STEP 频率差分，不是标定过的偏航角速度；转向环 PD 才把陀螺量换算进来。
// 单位：requested/applied 与 maxHz/slew 均为 STEP Hz（差模）；receivedMs=ms；角度输入为 deg。
// 调用者/频率：update() 由 remote::updateTurn() 在 loop 400Hz 调用；command() 由 panel/cmd.cpp 的 ^ 与 V 命令调用；
//   busy()/requested 由 remote::isActive()、遥测 D 帧（1Hz）读取。
// 边界：命令 350ms 超时自动归零；|角度误差|≥10° 禁止转向并闩锁 needsZero；
//   输出永远被"共模平衡剩余空间 room"与"平衡限幅的 25%"两者中更严的一个钳住。
// ============================================================
// STEP frequency differential, not a calibrated yaw rate.
struct ManualTurnControl {
    // requested：当前转向命令（STEP Hz 差模），±maxHz=±300；command() 写入、update() 消费，命令级更新。
    // applied：实际输出（STEP Hz 差模），每拍按 slewHzPerSecond 斜率逼近 target 后返回；读方只看它。
    float requested = 0, applied = 0;
    // receivedMs：最后一条有效转向命令的 millis() 时间戳（ms）；判 350ms 超时用。
    uint32_t receivedMs = 0;
    // needsZero：安全闩锁（bool）。clear()/authority 归零时置 true，此后忽略非 0 命令，
    // 直到收到一条 0 命令才解锁（保证故障退出后必须先"明确回零"）。
    bool needsZero = false;
    // timeoutMs=350：转向命令的超时窗（ms）。^ 命令需持续重发，350ms 收不到就视为松手 → requested=0；
    // 与 ManualPitchControl 的 350ms 一致（同一套"死人开关"语义）。
    static constexpr uint32_t timeoutMs = 350;
    // maxHz=300：单侧转向差模的绝对上限（STEP Hz）。它只是上限之一，实际还会被 room 和 25% 规则压得更小。
    // 依据：BALANCE_OUTPUT_LIMIT_HZ 默认 1750Hz，×0.25 = 437.5Hz，再被 maxHz 截到 300Hz。
    static constexpr float maxHz = 300.0f;
    // slewHzPerSecond=300：applied 的斜率上限（Hz/s）。400Hz 调用时每拍最多变 300×0.0025=0.75 Hz，
    // 防止转向命令阶跃造成轮速突变/失步。
    static constexpr float slewHzPerSecond = 300.0f;
    // clamp — 作用：通用三元限幅（x 钳到 [lo,hi]）。参数 x/lo/hi 同单位；返回钳位后的值。
    // 调用：本文件与 motion_drive/position_jog 共用（static，无 NaN 检查——调用方负责先做 isfinite）。
    static float clamp(float x, float lo, float hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    }
    // clear — 作用：故障/停机时清转向命令并闩锁。参数/返回：无。
    // 调用：remote::resetFault()（掉拍/IMU 故障/倒地，事件级）。边界：requested/applied=0，needsZero=true。
    void clear() { requested = applied = 0; needsZero = true; }
    // command — 作用：登记一条转向命令（人工入口，来自 ^ 或 V）。
    // 参数 hz：STEP Hz 差模（钳到 ±300）；now：millis()。返回：无。
    // 调用：panel/cmd.cpp（人工命令级）。边界：hz==0 先解锁 needsZero 再 return；needsZero 时忽略非 0；NaN/Inf→0。
    void command(float hz, uint32_t now) {
        if (hz == 0) needsZero = false;
        if (needsZero) return;
        requested = std::isfinite(hz) ? clamp(hz, -maxHz, maxHz) : 0;
        receivedMs = now;
    }
    // busy — 作用：转向是否仍在占用。参数 now：millis()。返回 bool。
    // 调用：remote::isActive() 每 400Hz、cmd 的前置检查。
    // 边界：350ms 内有非 0 命令、或 |applied|>0.01Hz（还在释放段）都算占用。
    bool busy(uint32_t now) const {
        return (requested != 0 && uint32_t(now-receivedMs) <= timeoutMs)
            || std::fabs(applied) > 0.01f;
    }
    // update — 作用：转向状态机核心：算安全上限 → 叠加 PD → 斜率限制 → 输出差模 STEP Hz。
    // 参数：now=millis()；dt=控制周期 s（loop 400Hz 时 0.0025）；angleError=pitch−平衡点（deg）；
    //   limit=单轮平衡输出限幅 BALANCE_OUTPUT_LIMIT_HZ（STEP Hz，默认 1750）；balance=共模平衡输出 finalBalanceOutput（Hz）；
    //   legacy=旧转向 PD 输出（STEP Hz，T/U 默认 0 → 当前恒 0）。
    // 返回：applied（STEP Hz 差模，已限幅）。调用：remote::updateTurn()，loop 每 400Hz 一次。
    // 边界：非有限参数/非正 dt → clear() 并返回 0；超时→requested=0；|angleError|≥10°→禁止转向并闩锁。
    float update(uint32_t now, float dt, float angleError,
                 float limit, float balance, float legacy) {
        // 超时：350ms 没有新命令 → 视为松手，requested 归零（applied 仍按斜率平滑回零）。
        if (uint32_t(now-receivedMs) > timeoutMs) requested = 0;
        // 输入卫生：6 个输入任一非有限、或 dt≤0，直接清状态返回 0，避免 NaN 传播进电机输出。
        if (!std::isfinite(angleError) || !std::isfinite(limit) ||
            !std::isfinite(balance) || !std::isfinite(legacy) ||
            !std::isfinite(dt) || dt <= 0) { clear(); return 0; }
        // 以下是"先保护共模平衡输出"的限幅链，逐行看：
        // Reserve common-mode balance output; never clip each wheel independently.
        // room = max(0, limit − |balance|)：total 限幅 limit（Hz）先被共模平衡输出占用，剩下来的才是差模转向可用的空间。
        // 举例：limit=1750、finalBalanceOutput=1700 → room=50Hz，转向最多只能 ±50Hz，保证两轮之和仍在限幅内。
        float room = std::fmax(0.0f, limit-std::fabs(balance));
        // cap = min(maxHz, max(0,limit)×0.25)：基础差模上限 = 平衡限幅的 25%（1750×0.25=437.5），再被 maxHz=300 封顶 → 300Hz。
        float cap = std::fmin(maxHz, std::fmax(0.0f, limit)*0.25f);
        // authority = clamp((10−|angleError|)/5, 0, 1)：≤5° 满权限；5°→10° 线性衰减；≥10° 归零。
        // 物理含义：车越是偏离平衡点，转向越要让位给直立环。
        float authority = clamp((10.0f-std::fabs(angleError))/5.0f, 0, 1);
        // 最终 cap = min(room, cap×authority)：两个约束取更严者——既不超过共模剩余空间，也随倾角衰减。
        cap = std::fmin(room, cap*authority);
        // authority 已归零（|angleError|≥10°）：彻底禁止转向，requested 清零并闩锁 needsZero（要重新发 0 解锁）。
        if (authority == 0) { requested = 0; needsZero = true; }
        float target = clamp(requested+legacy, -cap, cap);
        // 斜率限幅：applied 每拍最多变化 slewHzPerSecond×dt=300×0.0025=0.75Hz（400Hz 调用），转向不会阶跃。
        applied += clamp(target-applied, -slewHzPerSecond*dt, slewHzPerSecond*dt);
        // Loss of balance authority must override the slew limiter immediately.
        // 斜率之后再硬钳一次：authority 骤降（大角度）时必须立刻削减转向，不能等斜率慢慢降下来。
        // ——"失去平衡权限"必须立即覆盖斜率限制，这是安全优先于平顺的地方。
        applied = clamp(applied, -cap, cap);
        return applied;
    }
};

// （注：下面两行 extern 声明内容完全相同，属重构时合并留下的重复声明；C++ 允许重复声明，保留原样。）
// （注：旧注释"定义在 remote/manual_turn.cpp"与当前实现不符，以代码为准：定义在 remote/remote_control.cpp。）
// 转向状态机实例（定义在 remote/manual_turn.cpp）
// manualTurn：转向状态机唯一实例（定义在 remote/remote_control.cpp）。
// 含义：requested/applied 为差模 STEP Hz（±300 量级）；写：panel/cmd 命令 + remote::updateTurn() 每 400Hz；
// 读：remote::isActive()、遥测 D 帧（1Hz）、panel 的参数写盘前置检查。
extern ManualTurnControl manualTurn;

// 转向状态机实例（定义在 remote/manual_turn.cpp）
extern ManualTurnControl manualTurn;

namespace remote {

// remote::updateTurn — 作用：转向环整体入口 = legacy PD（T/U）+ 状态机限幅 → turnOutput。
// 参数：无（输入取全局 turnKp/turnKd、targetTurnRate、gyroTurnRate、pitch、finalBalanceOutput）。
// 返回：无（写全局 turnOutput，STEP Hz 差模）。调用：main loop 每 400Hz 一次，必须在直立环之后。
// 边界：manualTurn.update() 内部做全部限幅/超时/闩锁（见上）；turnOutput 因此有界。
// 转向环 PD（原 updateTurnLoop）：legacy PD + 转向状态机 → turnOutput
void updateTurn();

}  // namespace remote
