#!/usr/bin/env python3
"""相机不在线时用编码器长时间窗口标定 O（15s 一段，分辨率约 ±0.2 mm/s）。

为什么要爬山而不是按公式一次算：pos 的符号约定今天已经翻过两次，
与其猜方向，不如先给一个 +0.05 的试探步，看漂移是变小还是变大，再决定方向。

流程：^ 0,0 停车 → 测 15s 漂移 → 试探 +0.05 → 再测 → 判方向 → 继续爬山到 |drift|<0.25 mm/s → SAVE

用法：python ble_o_encoder.py --address E1:.. --o 2.00 --secs 15 --out ...\\oenc01
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
    if out and os.path.exists(out):
        raise SystemExit(f"输出已存在：{out}")
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
        o0 = args.o
        for item in par["t"].split():
            if item.startswith("O="):
                o0 = num(item[2:], args.o)
        print(f"[原] O={o0:.3f}")

        async def measure(secs):
            await send("^ 0,0")
            await asyncio.sleep(1.0)
            mark = len(grows)
            await asyncio.sleep(secs)
            sel = [(w, f) for w, f in grows[mark:] if len(f) > 3]
            if len(sel) < 10:
                return None
            pos = [num(f[1]) for _, f in sel]
            pit = [num(f[3]) / 100 for _, f in sel]
            dt = sel[-1][0] - sel[0][0]
            mm = (pos[-1] - pos[0]) / 1080.0
            return {"mm": mm, "mm_s": mm / dt, "pitch": st.mean(pit), "sd": st.pstdev(pit), "dt": dt}

        rows = []
        o = o0
        m = await measure(args.secs)
        if not m:
            print("[中止] 采样不足")
            return 3
        print(f"[基准] drift={m['mm_s']:+.2f} mm/s  pitch={m['pitch']:.2f}±{m['sd']:.2f}")
        rows.append((o, m["mm_s"], m["pitch"]))
        step = 0.05
        direction = 0
        for _ in range(args.max_iter):
            if abs(m["mm_s"]) < args.target:
                break
            if direction == 0:
                direction = -1 if m["mm_s"] > 0 else 1   # 先猜一个方向
            prev = abs(m["mm_s"])
            o = round(o + direction * step, 3)
            await send(f"O {o:.3f}")
            await asyncio.sleep(1.0)
            m2 = await measure(args.secs)
            if not m2:
                break
            print(f"[试探] O={o:.3f} drift={m2['mm_s']:+.2f} mm/s  pitch={m2['pitch']:.2f}")
            rows.append((o, m2["mm_s"], m2["pitch"]))
            if abs(m2["mm_s"]) > prev:
                direction = -direction     # 变差了 → 反向
                print("  [反向] 这步让漂移变大，翻方向")
            m = m2
            if abs(m["pitch"] - o) > 2.0:
                print("[中止] |pitch-O| 过大")
                break

        converged = abs(m["mm_s"]) < args.target
        print(f"[结果] O={o:.3f} drift={m['mm_s']:+.2f} mm/s  {'已收敛' if converged else '未收敛'}")
        if converged and args.save:
            await send("SAVE")
            await asyncio.sleep(1.0)
            print("[SAVE] 已写入 flash")
        await send("^ 0,0")

    if out:
        with open(out, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["O", "drift_mm_s", "pitch_deg"])
            w.writerows(rows)
        print(f"[记录] {out}")
    return 0 if converged else 5


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--o", type=float, default=2.0)
    ap.add_argument("--secs", type=float, default=15.0)
    ap.add_argument("--target", type=float, default=0.25)
    ap.add_argument("--max-iter", type=int, default=6)
    ap.add_argument("--save", action="store_true", default=True)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args)))
