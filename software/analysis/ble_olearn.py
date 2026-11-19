#!/usr/bin/env python3
"""平衡角 O 的在线学习（慢回路）：按漂移速度微调 O，使静止漂移趋于 0。

设计要点：
  * 这是**慢回路**：增益小、限幅紧、限速，目标是几十秒时间常数，不参与快速稳定。
  * 只按"漂移速度"（pos 回归）修正，不用瞬时速度，避免噪声驱动。
  * 有死区：|v| 小于 --deadband 就不动，防止把噪声积分进去。
  * 有硬限幅：O 只在 [O0-|limit|, O0+|limit|] 内，绝不跑飞。
  * 有翻车保护：|pitch−O| > --fall-deg 立即停手并回 O0。

符号约定（2026-11-19 实测）：pos 减小 = 前进；增大 O = 车身后仰 = 后退。
所以漂移为负（前进）时应增大 O。

用法：
  python ble_olearn.py --address E1:.. --o0 4.24 --secs 60 --k 0.02
"""
import argparse
import asyncio
import statistics
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
POS_PER_MM = 1080.0
I_POS, I_PITCH = 1, 3


class Rec:
    def __init__(self):
        self.frames, self.buf = [], ""

    def on_data(self, _s, data):
        now = time.time()
        self.buf += data.decode("utf-8", "replace")
        *done, self.buf = self.buf.replace("\r", "\n").split("\n")
        for line in done:
            line = line.strip()
            if not line.startswith("G,"):
                continue
            f = line[2:].split(",")
            if len(f) > I_PITCH:
                try:
                    self.frames.append((now, float(f[I_POS]) / POS_PER_MM,
                                        float(f[I_PITCH]) / 100.0))
                except ValueError:
                    pass

    def latest(self):
        return self.frames[-1] if self.frames else None

    def speed(self, window_s):
        """最近 window_s 秒的漂移速度（mm/s），线性回归"""
        if len(self.frames) < 3:
            return None
        t_end = self.frames[-1][0]
        w = [f for f in self.frames if f[0] >= t_end - window_s]
        if len(w) < 3:
            return None
        n = len(w)
        mx = sum(f[0] for f in w) / n
        my = sum(f[1] for f in w) / n
        num = sum((f[0] - mx) * (f[1] - my) for f in w)
        den = sum((f[0] - mx) ** 2 for f in w)
        return num / den if den > 0 else None


async def main(args):
    addr = args.address
    if not addr:
        for dev in await BleakScanner.discover(timeout=10.0):
            if (dev.name or "").upper().startswith("JDY"):
                addr = dev.address
                break
    if not addr:
        print("ERR 没找到 JDY-23")
        return 2

    rec = Rec()
    o = args.o0
    log = []
    async with BleakClient(addr, timeout=20.0) as cli:
        await cli.start_notify(FFE1, rec.on_data)

        async def send(cmd):
            await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)

        await asyncio.sleep(0.8)
        await send("^ 0,0")
        await asyncio.sleep(0.4)
        await send(f"O {o:.2f}")
        await asyncio.sleep(0.8)
        print("[开始] O0=%.2f  k=%.3f°/s每mm/s  死区=%.2fmm/s  限幅=±%.2f°  限速=%.3f°/s"
              % (args.o0, args.k, args.deadband, args.limit, args.max_rate))

        t0 = time.time()
        last_print = 0.0
        while time.time() - t0 < args.secs:
            await asyncio.sleep(0.25)
            cur = rec.latest()
            if cur is None:
                continue
            # 翻车保护
            if abs(cur[2] - o) > args.fall_deg:
                await send(f"O {args.o0:.2f}")
                print("[中止] 姿态偏离 %.1f°（O=%.2f）——已回 O0" % (abs(cur[2] - o), o))
                return 4
            v = rec.speed(args.window)
            if v is None:
                continue
            step = 0.0
            if abs(v) > args.deadband:
                step = -args.k * v * 0.25                     # 负漂（前进）→ 增大 O
                step = max(-args.max_rate * 0.25, min(args.max_rate * 0.25, step))
                o_new = max(args.o0 - args.limit, min(args.o0 + args.limit, o + step))
                if abs(o_new - o) > 0.005:
                    o = o_new
                    await send(f"O {o:.2f}")
            log.append((time.time() - t0, o, v, cur[2]))
            if time.time() - t0 - last_print >= 5.0:
                last_print = time.time() - t0
                print("  +%4.0fs  O=%.3f  v=%+6.2fmm/s  pitch=%.2f°" % (last_print, o, v, cur[2]))

        await send(f"O {o:.2f}")
        await asyncio.sleep(0.3)

    if log:
        vs = [abs(x[2]) for x in log[len(log) // 2:]]
        print("\n=== 结果 ===")
        print("  初始 O = %.3f   结束 O = %.3f   学习量 = %+.3f°" % (args.o0, o, o - args.o0))
        if vs:
            print("  后半程 |v| 均值 = %.2f mm/s" % statistics.mean(vs))
        print("  O 轨迹: " + " ".join("%.2f" % log[i][1] for i in range(0, len(log), max(1, len(log) // 12))))
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--o0", type=float, required=True, help="起始平衡角")
    ap.add_argument("--secs", type=float, default=60)
    ap.add_argument("--k", type=float, default=0.02, help="学习率 °/s 每 mm/s")
    ap.add_argument("--deadband", type=float, default=0.5, help="漂移死区 mm/s")
    ap.add_argument("--limit", type=float, default=3.0, help="O 允许偏离 O0 的上限（°）")
    ap.add_argument("--max-rate", type=float, default=0.10, help="O 变化限速 °/s")
    ap.add_argument("--window", type=float, default=2.0, help="测速窗口 秒")
    ap.add_argument("--fall-deg", type=float, default=8.0)
    sys.exit(asyncio.run(main(ap.parse_args())))
