#!/usr/bin/env python3
"""悬空/离地时的方向对称性测试：把 O 临时钉到当前姿态角，让平衡环不再饱和。

为什么需要：手里拿着车时 pitch 通常离 O 有 1~2°，平衡环会饱和到 L=1750Hz，
把 ^ 手动请求完全盖住，测出来的永远是同一个方向。

流程：
  1. 读 1s 的 pitch 均值 → O 临时设成这个值（平衡环误差≈0）
  2. X 设 0（关位置环，避免它叠加速度目标）
  3. ±gear 交替各 hold 秒，记录 G 帧
  4. 无论成败，恢复原 O 与 X

用法：
  python ble_air_test.py --address E1:.. --gear 1.0 --hold 1.5 --reps 3 --out ...\\air02
"""
import argparse
import asyncio
import csv
import os
import statistics as st
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"


def num(v, d=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


async def main(args):
    out = args.out
    for p in (out + ".g.csv", out + ".phases.csv"):
        if os.path.exists(p):
            raise SystemExit(f"输出已存在：{p}")
    addr = args.address
    if not addr:
        for d in await BleakScanner.discover(timeout=10.0):
            if d.name and ("JDY" in d.name or "BT" in d.name):
                addr = d.address
                break
    if not addr:
        raise SystemExit("扫描不到车")

    grows, par, buf = [], {"t": ""}, {"s": ""}
    async with BleakClient(addr, timeout=30.0) as cli:
        def on_data(_s, data):
            buf["s"] += data.decode("utf-8", "replace")
            *done, buf["s"] = buf["s"].replace("\r", "\n").split("\n")
            for line in done:
                line = line.strip()
                if line.startswith("PAR:"):
                    par["t"] = line
                elif line.startswith("G,"):
                    grows.append((time.time(), line[2:].split(",")))

        await cli.start_notify(FFE1, on_data)

        async def send(t):
            await cli.write_gatt_char(FFE1, (t + "\n").encode(), response=True)
            await asyncio.sleep(0.05)

        for _ in range(20):
            if par["t"]:
                break
            await send("?")
            await asyncio.sleep(0.3)
        o0 = x0 = None
        for item in par["t"].split():
            if item.startswith("O="):
                o0 = num(item[2:])
            elif item.startswith("X="):
                x0 = num(item[2:])
        await asyncio.sleep(1.2)
        pit = [num(f[3]) / 100.0 for w, f in grows if len(f) > 3]
        o_hold = round(st.median(pit[-8:]), 2) if pit else o0
        print(f"[原] O={o0} X={x0}   当前姿态 pitch={o_hold}")

        phases = []
        try:
            await send(f"O {o_hold:.2f}")
            if args.zero_x:
                await send("X 0.000")
            await asyncio.sleep(1.0)
            for i in range(args.reps):
                for g in (args.gear, -args.gear):
                    await send("^ 0,0")
                    await asyncio.sleep(0.5)
                    t0 = time.time()
                    end = time.monotonic() + args.hold
                    while time.monotonic() < end:
                        await send(f"^ 0,{g:g}")
                        await asyncio.sleep(0.12)
                    t1 = time.time()
                    await send("^ 0,0")
                    phases.append((g, t0, t1))
                    print(f"[段] gear={g:+.1f} {t0:.2f}")
                    await asyncio.sleep(args.settle)
        finally:
            await send("^ 0,0")
            if o0 is not None:
                await send(f"O {o0:.2f}")
            if args.zero_x and x0 is not None:
                await send(f"X {x0:.4f}")
            print(f"[恢复] O={o0} X={x0}")

    with open(out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req"])
        for wall, f in grows:
            w.writerow([f"{wall:.3f}"] + f[:8])
    with open(out + ".phases.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["gear", "t0", "t1"])
        w.writerows(phases)

    fw, bw = [], []
    print(f"\n{'gear':>6}{'n':>4}{'Δ编码器':>10}{'Δmm':>8}{'速度均值':>9}{'pitch':>8}")
    for g, t0, t1 in phases:
        sel = [f for w, f in grows if t0 <= w <= t1 and len(f) > 6]
        if len(sel) < 2:
            continue
        pos = [num(f[1]) for f in sel]
        spd = [num(f[3]) / 1000 for f in sel]
        pit = [num(f[4]) / 100 for f in sel]
        d = pos[-1] - pos[0]
        (fw if g > 0 else bw).append(d)
        print(f"{g:>+6.1f}{len(sel):>4}{d:>10.0f}{d/10.8:>8.0f}{st.mean(spd):>9.1f}{st.mean(pit):>8.2f}")
    if fw and bw:
        print(f"\n前进 |Δ|={st.mean([abs(x) for x in fw]):.0f} counts   "
              f"后退 |Δ|={st.mean([abs(x) for x in bw]):.0f} counts   "
              f"比值 {st.mean([abs(x) for x in bw])/max(st.mean([abs(x) for x in fw]),1):.2f}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--gear", type=float, default=1.0)
    ap.add_argument("--hold", type=float, default=1.5)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--settle", type=float, default=0.8)
    ap.add_argument("--zero-x", action="store_true", default=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args)))
