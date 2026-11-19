#!/usr/bin/env python3
"""方向配平：扫 TF/TB，找出让前进/后退位移接近的那组值。

背景（2026-11-12）：前后不对称已排除指令/软窗口/位置环/偏航/驱动/地面，判定为车体结构不对称。
固件已有 motionLean.forwardTrim / backwardTrim（命令 TF/TB，±1.0°，进 flash），当前 TRIM=0.000/0.000。

做法：对每个候选 TB 值，跑 reps 组 (+gear, -gear)，量每段 pos 位移，
      输出 前进均值 / 后退均值 / 差值（目标 |差值| < 5mm）。

用法：
  python ble_trim_tune.py --address E1:.. --tb 0,0.10,0.20 --gear 1.0 --hold 4 --reps 2 --out ...\\trim01
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
            await asyncio.sleep(0.06)

        for _ in range(20):
            if par["t"]:
                break
            await send("?")
            await asyncio.sleep(0.3)
        o0, tb0, tf0 = 2.0, 0.0, 0.0
        for item in par["t"].split():
            if item.startswith("O="):
                o0 = num(item[2:], 2.0)
            elif item.startswith("TRIM="):
                parts = item[5:].split("/")
                if len(parts) == 2:
                    tf0 = num(parts[0], 0.0)
                    tb0 = num(parts[1], 0.0)
        print(f"[原] O={o0} TRIM={tf0}/{tb0}  扫描 {args.param}")

        def last_pos():
            for w, f in reversed(grows):
                if len(f) > 3:
                    return num(f[1], None)
            return None

        await send("^ 0,0")
        await asyncio.sleep(1.5)
        pn = [num(f[3], None) for w, f in grows if len(f) > 3]
        if not pn or abs(pn[-1] / 100 - o0) > 1.5:
            print(f"[中止] 起始姿态不在 O 附近（{pn[-1]/100 if pn else '?'}）")
            return 5

        phases, results = [], []
        for tb in [float(x) for x in args.tb.split(",")]:
            await send(f"{args.param} {tb:.3f}")
            print(f"[{args.param}] {tb:+.2f}")
            await asyncio.sleep(2.0)
            fwd, bwd = [], []
            for _ in range(args.reps):
                for g in (args.gear, -args.gear):
                    await send("^ 0,0")
                    await asyncio.sleep(0.6)
                    p0 = last_pos()
                    t0 = time.time()
                    end = time.monotonic() + args.hold
                    while time.monotonic() < end:
                        await send(f"^ 0,{g:g}")
                        await asyncio.sleep(0.12)
                        pl = [num(f[1], None) for w, f in grows if w > t0 and len(f) > 3]
                        if p0 is not None and pl and abs(pl[-1] - p0) / 1080.0 > args.max_travel:
                            await send("^ 0,0")
                            print(f"[中止] 位移超 {args.max_travel:.0f}mm")
                            await send(f"{args.param} {(tf0 if args.param=='TF' else tb0):.3f}")
                            return 4
                    t1 = time.time()
                    await send("^ 0,0")
                    p1 = last_pos()
                    d = (p1 - p0) / 1080.0 if (p0 is not None and p1 is not None) else 0.0
                    (fwd if g > 0 else bwd).append(d)
                    phases.append((tb, g, t0, t1, d))
                    print(f"   gear={g:+.1f}  {d:+7.1f} mm")
                    await asyncio.sleep(args.settle)
            if fwd and bwd:
                results.append((tb, st.mean(fwd), st.mean(bwd),
                                st.pstdev(fwd) if len(fwd) > 1 else 0.0,
                                st.pstdev(bwd) if len(bwd) > 1 else 0.0))

        await send(f"{args.param} {(tf0 if args.param=='TF' else tb0):.3f}")
        await send("^ 0,0")
        print(f"[恢复] TB={tb0}")

    with open(out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req"])
        for wall, f in grows:
            w.writerow([f"{wall:.3f}"] + f[:8])
    with open(out + ".phases.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["tb", "gear", "t0", "t1", "disp_mm"])
        w.writerows(phases)

    print(f"\n{'TB':>7}{'前进均值':>10}{'后退均值':>10}{'差值':>9}{'前进sd':>8}{'后退sd':>8}")
    for tb, f, b, fs, bs in results:
        print(f"{tb:>+7.2f}{f:>10.1f}{b:>10.1f}{f-b:>9.1f}{fs:>8.1f}{bs:>8.1f}")
    print(f"\n[结果] {out}.g.csv")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--tb", default="0,0.10,0.20", help="要扫描的配平值列表")
    ap.add_argument("--param", default="TB", choices=["TB", "TF"], help="TB 后退 / TF 前进")
    ap.add_argument("--gear", type=float, default=1.0)
    ap.add_argument("--hold", type=float, default=4.0)
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--settle", type=float, default=1.5)
    ap.add_argument("--max-travel", type=float, default=120.0)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args)))

