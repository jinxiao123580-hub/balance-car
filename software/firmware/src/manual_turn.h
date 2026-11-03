#pragma once
#include <cmath>
#include <cstdint>

// STEP frequency differential, not a calibrated yaw rate.
struct ManualTurnControl {
    float requested = 0, applied = 0;
    uint32_t receivedMs = 0;
    bool needsZero = false;
    static constexpr uint32_t timeoutMs = 350;
    static constexpr float maxHz = 300.0f;
    static constexpr float slewHzPerSecond = 300.0f;
    static float clamp(float x, float lo, float hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    }
    void clear() { requested = applied = 0; needsZero = true; }
    void command(float hz, uint32_t now) {
        if (hz == 0) needsZero = false;
        if (needsZero) return;
        requested = std::isfinite(hz) ? clamp(hz, -maxHz, maxHz) : 0;
        receivedMs = now;
    }
    bool busy(uint32_t now) const {
        return (requested != 0 && uint32_t(now-receivedMs) <= timeoutMs)
            || std::fabs(applied) > 0.01f;
    }
    float update(uint32_t now, float dt, float angleError,
                 float limit, float balance, float legacy) {
        if (uint32_t(now-receivedMs) > timeoutMs) requested = 0;
        if (!std::isfinite(angleError) || !std::isfinite(limit) ||
            !std::isfinite(balance) || !std::isfinite(legacy) ||
            !std::isfinite(dt) || dt <= 0) { clear(); return 0; }
        // Reserve common-mode balance output; never clip each wheel independently.
        float room = std::fmax(0.0f, limit-std::fabs(balance));
        float cap = std::fmin(maxHz, std::fmax(0.0f, limit)*0.25f);
        float authority = clamp((10.0f-std::fabs(angleError))/5.0f, 0, 1);
        cap = std::fmin(room, cap*authority);
        if (authority == 0) { requested = 0; needsZero = true; }
        float target = clamp(requested+legacy, -cap, cap);
        applied += clamp(target-applied, -slewHzPerSecond*dt, slewHzPerSecond*dt);
        // Loss of balance authority must override the slew limiter immediately.
        applied = clamp(applied, -cap, cap);
        return applied;
    }
};
