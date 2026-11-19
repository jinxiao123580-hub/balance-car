#!/usr/bin/env python3
"""直立环抗扰测试：给 O 加一个短脉冲再收回，看车能不能自己稳住。

原理：O 是平衡点设定值，把它临时抬高 δ 等于"让车以为该往前倾 δ"，
平衡环会把车体压到 O+δ；脉冲结束后放回 O，车体必须自己回来。
这就是一次受控的"推一下"，不依赖手推、不越 0.5° 静摩擦门槛，窜车风险小。

每段输出（G 帧 4Hz）：
  脉冲峰值      |pitch − O| 最大值
  回落时间      脉冲结束后回到 |pitch−O| < 0.15° 用时
  反向超调      回落过程中越过 O 的最大幅度
  末态偏差      观测窗最后 1s 的均值偏差
  摆动次数      脉冲后 pitch 偏差过零次数（越多越振荡）

用法：
  python ble_pulse_test.py --address E1:.. --pulses "0.3:400,0.5:400,0.8:300" --out ...\\p01
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
    for p in (out + ".pulses.csv", out + ".g.csv"):
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

        for _ in range(20):
            if par["t"]:
                break
            await send("?")
            await asyncio.sleep(0.3)
        o0 = 2.0
        for item in par["t"].split():
            if item.startswith("O="):
                o0 = num(item[2:], 2.0)
        print(f"[O0] {o0:.3f}")

        await send("^ 0,0")
        await asyncio.sleep(1.5)
        pnow = None
        for _ in range(10):
            s = [num(f[3], None) for w, f in grows if len(f) > 3]
            if s:
                pnow = s[-1] / 100
            if pnow is not None and abs(pnow - o0) < 0.8:
                break
            await asyncio.sleep(0.3)
        print(f"[就位] pitch={pnow}")
        if pnow is None or abs(pnow - o0) > 0.8:
            print("[中止] 起始不在 O 附近")
            return 5

        pulses = []
        for s in args.pulses.split(","):
            a, w = s.split(":")
            pulses += [(float(a), float(w)), (-float(a), float(w))]

        marks = []
        ds_list = [float(x) for x in args.ds.split(",")] if args.ds else [None]
        d0 = None
        for item in par["t"].split():
            if item.startswith("D="):
                d0 = num(item[2:])
        for dv in ds_list:
            if dv is not None:
                await send(f"D {dv:.2f}")
                print(f"[D] {dv:.0f}")
                await asyncio.sleep(2.0)
            for amp, width in pulses:
                await asyncio.sleep(args.settle)
                t0 = time.time()
                await send(f"O {o0 + amp:.3f}")
                await asyncio.sleep(width / 1000.0)
                t1 = time.time()
                await send(f"O {o0:.3f}")
                print(f"[脉冲] D={dv} {amp:+.2f}° {width:.0f}ms  t1={t1:.2f}")
                await asyncio.sleep(args.observe)
                marks.append((dv, amp, width, t0, t1))
        if d0 is not None:
            await send(f"D {d0:.2f}")
            print(f"[恢复] D={d0}")

        await send("^ 0,0")
        await send(f"O {o0:.3f}")
        await asyncio.sleep(0.8)

    with open(out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req"])
        for wall, f in grows:
            w.writerow([f"{wall:.3f}"] + f[:8])
    with open(out + ".pulses.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["amp_deg", "width_ms", "t0", "t1"])
        w.writerows(marks)

    def pit(a, b):
        return [(w, num(f[3]) / 100) for w, f in grows if a <= w <= b and len(f) > 3]

    print(f"\n{'D':>5}{'脉冲':>8}{'峰值°':>8}{'回落ms':>9}{'反向超调°':>11}{'摆动次':>8}{'末态偏差°':>11}")
    rows = []
    for dv, amp, width, t0, t1 in marks:
        base = [v for _, v in pit(t0 - 0.6, t0)] or [o0]
        dev = [(w, v - o0) for w, v in pit(t1, t1 + args.observe)]
        if not dev:
            print(f"{str(dv):>5}{amp:>+8.2f}   缺帧")
            continue
        peak = max(abs(v) for _, v in dev)
        rec = None
        for w, v in dev:
            if abs(v) < 0.15:
                if all(abs(vv) < 0.25 for ww, vv in dev if ww >= w):
                    rec = (w - t1) * 1000
                    break
        sign = 1 if amp > 0 else -1
        over = max([-sign * v for _, v in dev], default=0.0)
        zc = sum(1 for i in range(len(dev) - 1) if dev[i][1] * dev[i + 1][1] < 0)
        tail = [v for w, v in dev if w >= t1 + args.observe - 1.0]
        rows.append((dv, amp, width, peak, rec, over, zc, st.mean(tail) if tail else 0.0))
        print(f"{('-' if dv is None else f'{dv:.0f}'):>5}{amp:>+8.2f}{peak:>8.2f}"
              f"{(f'{rec:.0f}' if rec else '>%d'%(args.observe*1000)):>9}"
              f"{over:>11.2f}{zc:>8}{st.mean(tail) if tail else 0:>11.2f}")
    print(f"\n[结果] {out}.g.csv")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--pulses", default="0.3:400,0.5:400,0.8:300")
    ap.add_argument("--settle", type=float, default=3.0, help="脉冲前静置 s")
    ap.add_argument("--observe", type=float, default=3.0, help="脉冲后观察 s")
    ap.add_argument("--out", required=True)
    ap.add_argument("--ds", default="", help="可选：按这些 D 值逐个测，如 100,150,200,260")
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args)))
