#!/usr/bin/env python3
"""自动找平衡点 O：粗扫 2° 级 → 逐次二分 → 步长收到 min-step。

设计要点（来自 2026-11-12 的踩坑）：
  * O 偏 2° 就能让车冲出去 → 每个测点都带位移熔断（默认 25mm），超了立刻回 O
  * 漂移幅值噪声大（同一 O 两次能测出 −0.6 与 −5.5）→ 用符号做二分，不依赖幅值
  * 跑飞时"冲向哪边"本身就是有效信息：熔断后用方向作符号，标 invalid 但仍参与夹逼
  * 用 pos 对时间的线性回归斜率算漂移，比首尾差值抗尖峰

流程：
  1. 在 [o0-2, o0+2] 上按 --coarse 取点（含 o0），测符号，找过零区间
  2. 二分：每次取中点，按符号替换端点，直到区间 < --min-step 或 |drift| < --target
  3. 输出每个测点、最终 O；--save 时写 flash

用法：
  python ble_find_o.py --address E1:.. --o0 2.0 --coarse 2.0 --min-step 0.05 --out ...\\findo01
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
        o_good = args.o0
        for item in par["t"].split():
            if item.startswith("O="):
                o_good = num(item[2:], args.o0)
        print(f"[原] O={o_good:.3f}")

        async def probe(o, secs):
            """设置 O → 静置 → 采样 secs 秒 → 线性回归斜率(左负右正)。熔断返回 invalid。"""
            await send(f"O {o:.3f}")
            await send("^ 0,0")
            await asyncio.sleep(1.2)
            mark = len(grows)
            t0 = time.monotonic()
            p0 = None
            invalid = False
            while time.monotonic() - t0 < secs:
                await asyncio.sleep(0.25)
                sel = [(w, f) for w, f in grows[mark:] if len(f) > 3]
                if not sel:
                    continue
                if p0 is None:
                    p0 = num(sel[0][1])
                cur = num(sel[-1][1])
                if p0 is None or cur is None:
                    continue
                if abs(cur - p0) / 1080.0 > args.max_travel:
                    invalid = True
                    break
            sel = [(w, f) for w, f in grows[mark:] if len(f) > 3]
            if len(sel) < 6:
                return {"o": o, "drift": None, "pitch": None, "invalid": True}
            ts = [w - sel[0][0] for w, _ in sel]
            ps = [num(f[1]) / 1080.0 for _, f in sel]
            pairs = [(t, p) for t, p in zip(ts, ps) if p is not None]
            if len(pairs) < 6:
                return {"o": o, "drift": None, "pitch": None, "invalid": True}
            ts = [t for t, _ in pairs]
            ps = [p for _, p in pairs]
            mt, mp = st.mean(ts), st.mean(ps)
            den = sum((t - mt) ** 2 for t in ts)
            slope = sum((t - mt) * (p - mp) for t, p in zip(ts, ps)) / den if den > 1e-9 else 0.0
            pit = st.mean([num(f[3]) / 100 for _, f in sel])
            if invalid:
                # 跑飞了：用位移方向做符号，幅值不可信
                slope = 10.0 if ps[-1] > ps[0] else -10.0
            return {"o": o, "drift": slope, "pitch": pit, "invalid": invalid}

        results = []

        async def ev(o, secs=None):
            r = await probe(o, secs or args.secs)
            results.append(r)
            tag = " 熔断" if r["invalid"] else ""
            print(f"  [测] O={r['o']:.3f}  drift={r['drift']:+.2f} mm/s  pitch={r['pitch']:.2f}{tag}"
                  if r["drift"] is not None else f"  [测] O={o:.3f} 样本不足")
            return r

        # 1) 粗扫找过零区间
        pts = [o_good - args.coarse, o_good, o_good + args.coarse]
        got = []
        for o in pts:
            r = await ev(o)
            if r["drift"] is not None:
                got.append(r)
            await send(f"O {o_good:.3f}")
            await asyncio.sleep(0.8)
        got.sort(key=lambda r: r["o"])
        a = b = None
        for i in range(len(got) - 1):
            d1, d2 = got[i]["drift"], got[i + 1]["drift"]
            if d1 is not None and d2 is not None and (d1 <= 0 <= d2 or d2 <= 0 <= d1):
                a, b = got[i]["o"], got[i + 1]["o"]
                break
        if a is None:
            best = min((r for r in got if r["drift"] is not None), key=lambda r: abs(r["drift"]), default=None)
            if not best:
                print("[中止] 粗扫无有效点")
                await send(f"O {o_good:.3f}")
                return 3
            a, b = best["o"] - args.coarse, best["o"] + args.coarse
            print(f"[粗扫] 未找到过零点，用 |drift| 最小点 {best['o']:.2f} 作中心")

        # 2) 二分
        best = min((r for r in results if r["drift"] is not None), key=lambda r: abs(r["drift"]))
        da = None
        for r0 in got:
            if abs(r0["o"] - a) < 1e-9:
                da = r0["drift"]
        while b - a > args.min_step and len(results) < args.max_points:
            m = round((a + b) / 2, 3)
            r = await ev(m)
            d = r["drift"]
            if d is None:
                break
            if abs(d) < args.target and not r["invalid"]:
                best = r
                break
            if da is None:
                da = (await ev(a))["drift"]
                await send(f"O {o_good:.3f}")
            if (d <= 0) == (da <= 0):
                a, da = m, d
            else:
                b = m
            if abs(d) < abs(best["drift"]):
                best = r
            await send(f"O {o_good:.3f}")
            await asyncio.sleep(0.6)

        print(f"\n[结论] O≈{best['o']:.3f}  drift={best['drift']:+.2f} mm/s"
              f"{'（熔断点，仅供参考）' if best['invalid'] else ''}")
        if args.save and not best["invalid"]:
            await send(f"O {best['o']:.3f}")
            await send("SAVE")
            await asyncio.sleep(1.0)
            print("[SAVE] 已写入 flash")
        else:
            await send(f"O {o_good:.3f}")
            print("[恢复] 未保存，O 回到", o_good)

    if out:
        with open(out, "w", newline="", encoding="utf-8") as fh:
            w = csv.writer(fh)
            w.writerow(["O", "drift_mm_s", "pitch_deg", "invalid"])
            for r in results:
                w.writerow([r["o"], r["drift"], r["pitch"], int(r["invalid"])])
        print(f"[记录] {out}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--o0", type=float, default=2.0)
    ap.add_argument("--coarse", type=float, default=2.0, help="粗扫半步长（°）")
    ap.add_argument("--min-step", type=float, default=0.05, help="二分到多细停（°）")
    ap.add_argument("--target", type=float, default=0.25, help="|drift| 目标 mm/s")
    ap.add_argument("--secs", type=float, default=10.0, help="每个测点采样时长")
    ap.add_argument("--max-travel", type=float, default=25.0, help="单点位移熔断 mm")
    ap.add_argument("--max-points", type=int, default=14)
    ap.add_argument("--save", action="store_true", default=False)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args)))
