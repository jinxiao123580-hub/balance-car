#!/usr/bin/env python3
"""档位扫描：给定 gear（速度闭环），量它自己生成的前倾角和实际位移。

为什么用档位而不是直接偏 O（2026-11-11 的教训）：
  稳态倾斜只会让车「斜着站住」，不产生位移；真正决定走不走的是速度环，
  倾角是它的输出（target − O）。所以「走起来需要多大倾角」= 每档对应的 leAn。

指标全部来自 G 帧（4Hz，比 D 帧的 1Hz 密）：
  f[1]=pos  f[2]=speed  f[3]=pitch  f[4]=target  f[6]=manualReq
  pos 单位 counts，10.8 counts/mm（今天用相机对表）。

用法：
  python ble_gear_sweep.py --address E1:.. --gears 0.5,1.0,1.5,2.0 --hold 2.0 --settle 2.0 --out ...\\gs01
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
        raise SystemExit("扫描不到车（未上电？）")

    grows, drows, par, buf = [], [], {"t": ""}, {"s": ""}

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
                elif line.startswith("D,"):
                    # D 帧 1Hz，74 列；本实验只取电机指令相关列（见下方索引说明）
                    drows.append((time.time(), line[2:].split(",")))

        await cli.start_notify(FFE1, on_data)

        async def send(t):
            await cli.write_gatt_char(FFE1, (t + "\n").encode(), response=True)
            await asyncio.sleep(0.05)

        for _ in range(20):
            if par["t"]:
                break
            await send("?")
            await asyncio.sleep(0.3)
        o0 = 2.0
        for item in par["t"].split():
            if item.startswith("O="):
                o0 = num(item[2:], 2.0)
        print(f"[O0] {o0:.2f}  [PAR] {par['t'][:110]}")

        await send("^ 0,0")
        await asyncio.sleep(1.5)
        pnow = None
        for _ in range(10):
            s = [num(f[3], None) for w, f in grows if len(f) > 3]
            if s:
                pnow = s[-1] / 100
            if pnow is not None and abs(pnow - o0) < 1.0:
                break
            await asyncio.sleep(0.3)
        print(f"[就位] pitch={pnow}")
        if not args.no_seat and (pnow is None or abs(pnow - o0) > 1.0):
            print("[中止] 起始不在 O 附近")
            return 5

        plan = []
        for _ in range(args.reps):
            for g in [float(x) for x in args.gears.split(",")]:
                plan += [(g, args.hold), (-g, args.hold)]

        phases, cum = [], 0.0
        aborted = ""
        for i, (g, secs) in enumerate(plan, 1):
            if aborted:
                break
            p0 = None
            for w, f in reversed(grows):
                if len(f) > 3:
                    p0 = num(f[1], None)
                    break
            await send(f"^ 0,{g:g}")
            t0 = time.time()
            end = time.monotonic() + secs
            while time.monotonic() < end:      # 固件 350ms 超时，必须按住重发
                await send(f"^ 0,{g:g}")
                await asyncio.sleep(0.12)
                p = [num(f[3], None) for w, f in grows if w > t0 and len(f) > 3]
                if len(p) >= 2 and abs(p[-1] / 100 - o0) > 2.5 and abs(p[-2] / 100 - o0) > 2.5:
                    await send("^ 0,0")
                    aborted = f"|pitch-O|>2.5° 于 gear={g:g}（pitch={p[-1]/100:.2f}）"
                    break
                if p0 is not None and args.max_travel > 0:
                    pl = [num(f[1], None) for w, f in grows if w > t0 and len(f) > 3]
                    if pl and abs(pl[-1] - p0) / 100.0 / args.counts_per_mm > args.max_travel:
                        await send("^ 0,0")
                        aborted = (f"位移超限 {abs(pl[-1]-p0)/100/args.counts_per_mm:.0f}mm "
                                   f"> {args.max_travel:.0f}mm，于 gear={g:g}")
                        break
            t1 = time.time()
            await send("^ 0,0")
            phases.append((g, t0, t1))
            print(f"[段{i}] gear={g:+.1f} {secs:.1f}s  t0={t0:.2f}")
            await asyncio.sleep(args.settle)

    with open(out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req"])
        for wall, f in grows:
            w.writerow([f"{wall:.3f}"] + f[:8])
    with open(out + ".d.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall"] + [f"f{i}" for i in range(74)])
        for wall, f in drows:
            w.writerow([f"{wall:.3f}"] + f[:74])
    with open(out + ".phases.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["gear", "t0", "t1"])
        w.writerows(phases)

    ppm = args.counts_per_mm
    print(f"\n{'gear':>6}{'n':>4}{'req':>6}{'lean°':>7}{'pitch°':>7}{'sd':>6}"
          f"{'speed':>7}{'sd':>6}{'SEM':>6}{'位移mm':>8}{'均速':>7}{'mm/s per °':>11}")
    for g, t0, t1 in phases:
        sel = [(w, f) for w, f in grows if t0 <= w <= t1 and len(f) > 6]
        if len(sel) < 3:
            print(f"{g:>+6.1f}{len(sel):>4}  样本不足")
            continue
        pos = [num(f[1]) for _, f in sel]
        spd = [num(f[2]) / 1000 for _, f in sel]
        pit = [num(f[3]) / 100 for _, f in sel]
        tgt = [num(f[4]) / 100 for _, f in sel]
        req = [num(f[6]) / 1000 for _, f in sel]
        dt = max(t1 - t0, 1e-6)
        disp = (pos[-1] - pos[0]) / 100.0 / ppm
        sd = st.pstdev(spd) if len(spd) > 2 else 0.0
        lean = st.mean(tgt) - o0
        print(f"{g:>+6.1f}{len(spd):>4}{st.mean(req):>6.2f}{lean:>7.2f}{st.mean(pit):>7.2f}"
              f"{(st.pstdev(pit) if len(pit) > 2 else 0):>6.2f}{st.mean(spd):>7.2f}{sd:>6.2f}"
              f"{sd/max(len(spd)**0.5,1):>6.2f}{disp:>8.1f}{disp/dt:>7.1f}"
              f"{(disp/dt/lean if abs(lean) > 0.02 else 0):>11.1f}")
    print(f"\n[结果] {out}.g.csv / {out}.phases.csv")
    if aborted:
        print(f"[中止] {aborted}（已保存停止前的数据）")
        return 4

    # D 帧（1Hz）电机指令对比：f7=spdAng100 f6=balHz f13=limitedHz f14=finalHz f26/27=m1/m2Hz
    print(f"\n{'gear':>6}{'D帧':>4}{'spdAng°':>9}{'balHz':>8}{'limitedHz':>10}"
          f"{'finalHz':>9}{'m1Hz':>8}{'m2Hz':>8}")
    for g, t0, t1 in phases:
        sel = [f for w, f in drows if t0 <= w <= t1 and len(f) > 27]
        if not sel:
            print(f"{g:>+6.1f}{0:>4}  无D帧")
            continue
        mean = lambda k: st.mean([num(f[k], 0) for f in sel])
        print(f"{g:>+6.1f}{len(sel):>4}{mean(7)/100:>9.2f}{mean(6):>8.0f}"
              f"{mean(13):>10.0f}{mean(14):>9.0f}{mean(26):>8.0f}{mean(27):>8.0f}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--gears", default="0.5,1.0,1.5,2.0")
    ap.add_argument("--reps", type=int, default=1)
    ap.add_argument("--no-seat", action="store_true", help="跳过就位检查（悬空转轮实验用）")
    ap.add_argument("--max-travel", type=float, default=600, help="单段位移上限（counts，0=不限）")
    ap.add_argument("--hold", type=float, default=2.0)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--counts-per-mm", type=float, default=10.8)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args)))
