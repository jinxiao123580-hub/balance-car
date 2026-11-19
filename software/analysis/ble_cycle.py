#!/usr/bin/env python3
"""前进-刹车-停 → 后退-刹车-停 的循环演示（按实测位移触发，不靠盲定时）。

每个循环：
  1 前进：Δ=-dacc，直到向前走到 --fwd-mm 或超时
  2 刹车：Δ=+dbrk，直到 |速度| < --v-stop 或超时
  3 停：  Δ=0，--hold 秒
  4 后退：Δ=+dacc，直到回到出发点 ±--back-tol
  5 刹车：Δ=-dbrk
  6 停：  Δ=0

Δ 全程 slew 限幅（阶跃会翻车）；|pitch−base| 超 --fall-deg 立刻归零。
base 写死（默认 4.00），不重估。
退出码：0 正常，4 翻车，5 异常（均写盘）。
"""
import argparse
import asyncio
import csv
import math
import os
import re
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
POS_PER_MM = 1080.0
SPEED_TO_MM_S = 20.0 / POS_PER_MM
I_POS, I_PITCH, I_SPEED = 1, 3, 2


class Abort(Exception):
    pass


class Rec:
    def __init__(self):
        self.text, self.frames, self.buf = [], [], ""

    def on_data(self, _s, data):
        now = time.time()
        self.buf += data.decode("utf-8", "replace")
        *done, self.buf = self.buf.replace("\r", "\n").split("\n")
        for line in done:
            line = line.strip()
            if not line:
                continue
            if line.startswith("G,"):
                self.frames.append((now, line[2:].split(",")))
            else:
                self.text.append((now, line))

    def _last(self, idx, scale=1.0):
        for _w, f in reversed(self.frames):
            if len(f) > idx:
                try:
                    return float(f[idx]) / scale
                except ValueError:
                    return None
        return None

    def pos(self):
        return self._last(I_POS)

    def pitch(self):
        return self._last(I_PITCH, 100.0)

    def vspeed(self):
        v = self._last(I_SPEED)
        return None if v is None else v * SPEED_TO_MM_S


async def main(args):
    d = os.path.dirname(args.out)
    if d:
        os.makedirs(d, exist_ok=True)
    addr = args.address
    if not addr:
        print("[扫描] 找 JDY-23 ...")
        for dev in await BleakScanner.discover(timeout=10.0):
            if (dev.name or "").upper().startswith("JDY"):
                addr = dev.address
                break
    if not addr:
        print("[错误] 没找到 JDY-23")
        return 2
    print(f"[连接] {addr}")

    rec = Rec()
    code = 0
    phases = []
    try:
        async with BleakClient(addr, timeout=20.0) as cli:
            await cli.start_notify(FFE1, rec.on_data)

            async def send(cmd):
                for attempt in (1, 2):
                    try:
                        await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)
                        return
                    except Exception:
                        if attempt == 2:
                            raise Abort("BLE 写入失败")
                        await asyncio.sleep(0.3)

            await asyncio.sleep(0.8)
            await send("?")
            await asyncio.sleep(1.6)
            base = args.base
            await send("^ 0,0")
            await asyncio.sleep(0.8)
            await send(f"O {base:.2f}")
            await asyncio.sleep(0.6)
            print(f"[base 写死] O={base:.2f}  Δacc={args.dacc} Δbrk={args.dbrk} "
                  f"slew={args.slew}°/s")

            applied = {"v": 0.0}

            async def ramp_to(tgt):
                """按 slew 把实际 Δ 走到 tgt 并下发。"""
                while abs(applied["v"] - tgt) > 0.02:
                    step = args.slew * args.poll
                    if tgt > applied["v"]:
                        applied["v"] = min(tgt, applied["v"] + step)
                    else:
                        applied["v"] = max(tgt, applied["v"] - step)
                    await send(f"O {base + applied['v']:.2f}")
                    await asyncio.sleep(args.poll)

            async def guard():
                pit = rec.pitch()
                if pit is not None and abs(pit - base) > args.fall_deg:
                    await send(f"O {base:.2f}")
                    raise Abort(f"翻车 pitch={pit:.1f}°")

            async def phase(name, delta, done, max_s):
                """把 Δ 缓推到 delta，然后每 poll 判一次 done()，满足即返回。"""
                await ramp_to(delta)
                t0 = time.time()
                p_start = rec.pos()
                while time.time() - t0 < max_s:
                    await guard()
                    if done(time.time() - t0):
                        dur = time.time() - t0
                        p_end = rec.pos()
                        mv = (p_end - p_start) / POS_PER_MM if (p_start is not None and p_end is not None) else float("nan")
                        phases.append((name, delta, dur, mv))
                        print(f"      {name:<6} Δ={delta:+.2f} {dur:5.2f}s  {mv:+7.2f}mm")
                        return True
                    await asyncio.sleep(args.poll)
                phases.append((name, delta, max_s, float("nan")))
                print(f"      {name:<6} Δ={delta:+.2f} 超时 {max_s}s")
                return False

            for c in range(args.cycles):
                print(f"   循环 {c+1}/{args.cycles}")
                p0 = rec.pos()
                if p0 is None:
                    raise Abort("拿不到位置")

                def fwd_done(_t, p0=p0):
                    p = rec.pos()
                    return p is not None and (p0 - p) / POS_PER_MM >= args.fwd_mm
                await phase("前进", -args.dacc, fwd_done, args.push_max_s)

                def stop_done(_t):
                    v = rec.vspeed()
                    return v is not None and abs(v) < args.v_stop
                await phase("刹车", args.dbrk, stop_done, args.brake_max_s)
                await phase("停", 0.0, lambda t: True, 0.01)
                await ramp_to(0.0)
                await asyncio.sleep(args.hold_s)

                def back_done(_t, p0=p0):
                    p = rec.pos()
                    return p is not None and (p0 - p) / POS_PER_MM <= args.back_tol
                await phase("后退", args.dacc, back_done, args.push_max_s)

                await phase("刹车", -args.dbrk, stop_done, args.brake_max_s)
                await ramp_to(0.0)
                await asyncio.sleep(args.hold_s)
                p2 = rec.pos()
                print(f"      循环末 净位移 {(p2-p0)/POS_PER_MM:+.2f}mm")

            await ramp_to(0.0)
            await send(f"O {base:.2f}")
    except Abort as e:
        print(f"[中止] {e} —— 数据仍写盘")
        code = 4
        try:
            await send(f"O {base:.2f}")
        except Exception:
            pass
    except Exception as e:
        print(f"[异常] {type(e).__name__}: {e} —— 数据仍写盘")
        code = 5

    with open(args.out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req", "rx"])
        for wall, f in rec.frames:
            if len(f) >= 8:
                w.writerow([f"{wall:.3f}"] + f[:8])
    with open(args.out + ".phases.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["phase", "delta", "secs", "moved_mm"])
        for r in phases:
            w.writerow([r[0], f"{r[1]:.2f}", f"{r[2]:.2f}",
                        "" if r[3] != r[3] else f"{r[3]:.2f}"])
    print(f"[结果] {args.out}.g.csv / {args.out}.phases.csv")
    return code


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--base", type=float, default=4.00, help="写死的平衡角（不重估）")
    ap.add_argument("--dacc", type=float, default=1.2, help="前进/后退推动档 Δ")
    ap.add_argument("--dbrk", type=float, default=1.2, help="刹车档 Δ（反号）")
    ap.add_argument("--slew", type=float, default=1.0, help="Δ 变化率 deg/s")
    ap.add_argument("--poll", type=float, default=0.2)
    ap.add_argument("--fwd-mm", type=float, default=40.0, help="前进多少毫米就刹车")
    ap.add_argument("--back-tol", type=float, default=6.0, help="后退回到出发点多少毫米内算到位")
    ap.add_argument("--push-max-s", type=float, default=5.0)
    ap.add_argument("--brake-max-s", type=float, default=2.5)
    ap.add_argument("--hold-s", type=float, default=1.5, help="停机停顿")
    ap.add_argument("--v-stop", type=float, default=6.0, help="刹车判停速度 mm/s")
    ap.add_argument("--fall-deg", type=float, default=5.0)
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
