"""前后运动闭环（motion_drive.h）主机回归。

验证：速度目标斜坡、倾角上限/速率、方向、释放无阶跃，以及在带静摩擦的
简化被控对象上速度能收敛且不过冲。真实 STEP/编码器量纲仍需上车标定。
"""
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[1]
src = root / "src"

PROGRAM = r'''
#include "motion_drive.h"
#include <cassert>
#include <cmath>
#include <cstdio>

static bool near(float a, float b, float eps) { return std::fabs(a - b) <= eps; }

int main() {
    const float dt = 0.01f;

    // 1) 速度目标按加速度斜坡，不产生阶跃
    {
        MotionSpeedProfile p;
        float first = p.update(MotionSpeedProfile::maxSpeed, dt);
        assert(near(first, MotionSpeedProfile::accel * dt, 1e-4f));
        for (int i = 0; i < 200; ++i) {
            float v = p.update(MotionSpeedProfile::maxSpeed, dt);
            assert(v <= MotionSpeedProfile::maxSpeed + 1e-4f);
            assert(v >= -1e-4f);
        }
        assert(near(p.applied, MotionSpeedProfile::maxSpeed, 1e-3f));
        // 反向指令同样受限
        float v = p.update(-MotionSpeedProfile::maxSpeed, dt);
        assert(near(v, MotionSpeedProfile::maxSpeed - MotionSpeedProfile::accel * dt, 1e-3f));
    }

    // 2) 零速起步：倾角 = 前馈 + 误差项，并受速率限制
    {
        MotionLeanControl l;
        float target = MotionLeanControl::feedForwardGain * 5.0f +
                       MotionLeanControl::speedErrorGain * (5.0f - 0.0f);
        assert(near(target, 0.90f, 1e-4f));
        for (int i = 0; i < 400; ++i) l.update(5.0f, 0.0f, true, dt);
        assert(near(l.applied, target, 1e-3f));
        assert(l.applied <= MotionLeanControl::limit + 1e-6f);
    }

    // 3) 起步第一拍不得跳变
    {
        MotionLeanControl l;
        l.update(5.0f, 0.0f, true, dt);
        assert(l.applied <= MotionLeanControl::engageRate * dt + 1e-4f);
    }

    // 4) 极端输入仍被限幅
    {
        MotionLeanControl l;
        for (int i = 0; i < 400; ++i) l.update(100.0f, -100.0f, true, dt);
        assert(near(l.applied, MotionLeanControl::limit, 1e-3f));
        MotionLeanControl m;
        for (int i = 0; i < 400; ++i) m.update(-100.0f, 100.0f, true, dt);
        assert(near(m.applied, -MotionLeanControl::limit, 1e-3f));
    }

    // 5) 方向：前进为正，后退为负
    {
        MotionLeanControl f, b;
        for (int i = 0; i < 300; ++i) { f.update(4.0f, 0.0f, true, dt); }
        for (int i = 0; i < 300; ++i) { b.update(-4.0f, 0.0f, true, dt); }
        assert(f.applied > 0.0f);
        assert(b.applied < 0.0f);
    }

    // 6) 释放：单调回零，不跳变
    {
        MotionLeanControl l;
        for (int i = 0; i < 300; ++i) l.update(5.0f, 0.0f, true, dt);
        float prev = l.applied;
        for (int i = 0; i < 400; ++i) {
            l.update(5.0f, 0.0f, false, dt);
            assert(l.applied <= prev + 1e-6f);
            assert(prev - l.applied <= MotionLeanControl::releaseRate * dt + 1e-4f);
            prev = l.applied;
        }
        assert(near(l.applied, 0.0f, 1e-5f));
        assert(!l.busy());
    }

    // 7) 被控对象闭环：静摩擦 0.15°、线性阻力、增益 3 counts/50ms per deg per s
    {
        MotionSpeedProfile p;
        MotionLeanControl l;
        const float kGain = 3.0f, friction = 0.15f, drag = 0.08f;
        float v = 0.0f, vmax = 0.0f;
        for (int i = 0; i < 1200; ++i) {
            float vref = p.update(MotionSpeedProfile::maxSpeed, dt);
            l.update(vref, v, true, dt);
            float effective = l.applied - (l.applied > friction ? friction
                                           : (l.applied < -friction ? -friction : l.applied));
            v += (effective * kGain - drag * v) * dt;
            if (v > vmax) vmax = v;
        }
        printf("closed_loop forward v=%.3f vref=%.3f vmax=%.3f lean=%.3f\n",
               v, p.applied, vmax, l.applied);
        assert(v > 0.5f * MotionSpeedProfile::maxSpeed);   // 确实走起来了
        assert(v < 3.0f * MotionSpeedProfile::maxSpeed);   // 没有失控飞车
        assert(l.applied <= MotionLeanControl::limit + 1e-6f);
    }

    // 8) 后退同样收敛且有界
    {
        MotionSpeedProfile p;
        MotionLeanControl l;
        float v = 0.0f;
        for (int i = 0; i < 1200; ++i) {
            float vref = p.update(-MotionSpeedProfile::maxSpeed, dt);
            l.update(vref, v, true, dt);
            float effective = l.applied + (l.applied < -0.15f ? 0.15f
                                          : (l.applied > 0.15f ? -0.15f : l.applied));
            v += (effective * 3.0f - 0.08f * v) * dt;
        }
        printf("closed_loop reverse v=%.3f lean=%.3f\n", v, l.applied);
        assert(v < -0.5f * MotionSpeedProfile::maxSpeed);
        assert(v > -3.0f * MotionSpeedProfile::maxSpeed);
    }

    // 9) 200ms 净速度：恒定 5 counts/50ms（100 counts/s，即 1 count/10ms）
    {
        NetSpeedEstimator e;
        e.reset(0.0f);
        float p = 0.0f, v = 0.0f;
        for (int i = 0; i < 60; ++i) { p += 1.0f; v = e.update(p); }
        printf("net_speed const v=%.3f\n", v);
        assert(near(v, 5.0f, 0.05f));
    }

    // 10) 单拍 +100 counts 跳变必须被 3 点中值吃掉
    {
        NetSpeedEstimator e;
        e.reset(0.0f);
        float p = 0.0f, v = 0.0f;
        for (int i = 0; i < 60; ++i) {
            p += 1.0f;
            float sample = (i == 30) ? p + 100.0f : p;
            v = e.update(sample);
        }
        printf("net_speed spike v=%.3f\n", v);
        assert(near(v, 5.0f, 0.1f));
    }

    // 11) 反向为负；复位后从 0 起算
    {
        NetSpeedEstimator e;
        e.reset(0.0f);
        float p = 0.0f, v = 0.0f;
        for (int i = 0; i < 60; ++i) { p -= 1.0f; v = e.update(p); }
        assert(near(v, -5.0f, 0.05f));
        e.reset(p);
        assert(near(e.update(p), 0.0f, 1e-6f));
    }

    puts("motion_drive regressions OK");
    return 0;
}
'''

def main():
    with tempfile.TemporaryDirectory() as tmp:
        cpp = Path(tmp) / "t.cpp"
        exe = Path(tmp) / ("t.exe" if sys.platform == "win32" else "t")
        cpp.write_text(PROGRAM, encoding="utf-8")
        subprocess.run(
            ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
             f"-I{src}", str(cpp), "-o", str(exe)],
            check=True,
        )
        subprocess.run([str(exe)], check=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
