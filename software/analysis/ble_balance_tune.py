#!/usr/bin/env python3
"""直立环调参台：给定若干 (P, D) 组合，各静置一段时间，用遥测量化"稳不稳"。

判稳指标（全部相机无关，G 帧 4Hz）：
  * pitch sd —— 姿态抖动（0.01° 分辨率，主要看它）
  * pos span —— 车体前后摆动幅度（mm，10.8 counts/mm）
  * speed sd —— 速度信号（直立环摆动的导数，越小越安静）
  * 超限帧数 —— |pitch − O| > 1.0 的帧占比（大扰动的代理）

安全：任一组合出现 |pitch−O|>2.5° 连续两帧，立即 `^ 0,0` 并跳过后面的组合；
结束（无论成败）一律把 P/D 恢复到开始时读到的值。

用法：
  python ble_balance_tune.py --address E1:.. --sets "1000,100;1000,160;1000,220;800,140" --secs 15
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
            await asyncio.sleep(0.05)

        for _ in range(20):
            if par["t"]:
                break
            await send("?")
            await asyncio.sleep(0.3)
        p0 = d0 = o0 = None
        for item in par["t"].split():
            if item.startswith("P="):
                p0 = num(item[2:])
            elif item.startswith("D="):
                d0 = num(item[2:])
            elif item.startswith("O="):
                o0 = num(item[2:])
        print(f"[原参数] P={p0} D={d0} O={o0}")
        if p0 is None or d0 is None or o0 is None:
            raise SystemExit("PAR 里没读到 P/D/O")

        sets = []
        for s in args.sets.split(";"):
            p, d_ = s.split(",")
            sets.append((float(p), float(d_)))

        results = []
        for p, d_ in sets:
            await send(f"P {p:.2f}")
            await send(f"D {d_:.2f}")
            t0 = time.time()
            await asyncio.sleep(args.settle)
            mark = len(grows)
            end = time.monotonic() + args.secs
            bad = 0
            trouble = ""
            while time.monotonic() < end:
                await asyncio.sleep(0.2)
                pit = [num(f[3], None) for w, f in grows[mark:] if len(f) > 3]
                if len(pit) >= 2 and abs(pit[-1] / 100 - o0) > 2.5 and abs(pit[-2] / 100 - o0) > 2.5:
                    bad += 1
                    trouble = f"pitch={pit[-1]/100:.2f}"
                    break
            sel = [(w, f) for w, f in grows[mark:] if len(f) > 4]
            rec = {"P": p, "D": d_, "n": len(sel), "trouble": trouble}
            if sel:
                pit = [num(f[3]) / 100 for _, f in sel]
                pos = [num(f[1]) / 100 / args.counts_per_mm for _, f in sel]
                spd = [num(f[2]) / 1000 for _, f in sel]
                rec.update(pitch_mean=st.mean(pit), pitch_sd=st.pstdev(pit),
                           pos_span=max(pos) - min(pos), spd_sd=st.pstdev(spd),
                           over=sum(1 for x in pit if abs(x - o0) > 1.0) / len(pit))
            results.append(rec)
            if trouble:
                await send("^ 0,0")
                print(f"[中止] P={p} D={d_} 不稳（{trouble}），后面的组合跳过")
                break
            print(f"[测] P={p:.0f} D={d_:.0f}  {rec.get('n',0)}帧  "
                  f"pitch {rec.get('pitch_mean',0):.2f}±{rec.get('pitch_sd',0):.2f}°  "
                  f"pos 摆幅 {rec.get('pos_span',0):.1f}mm  "
                  f"speed sd {rec.get('spd_sd',0):.2f}  超限 {rec.get('over',0)*100:.0f}%")

        # 无条件恢复原值
        await send(f"P {p0:.2f}")
        await send(f"D {d0:.2f}")
        await send("^ 0,0")
        print(f"[恢复] P={p0} D={d0}")

    print(f"\n{'P':>7}{'D':>7}{'帧':>4}{'pitch均值':>10}{'pitch sd':>9}{'pos摆幅mm':>11}{'speed sd':>9}{'超限%':>7}")
    for r in results:
        print(f"{r['P']:>7.0f}{r['D']:>7.0f}{r.get('n',0):>4}{r.get('pitch_mean',0):>10.2f}"
              f"{r.get('pitch_sd',0):>9.2f}{r.get('pos_span',0):>11.1f}"
              f"{r.get('spd_sd',0):>9.2f}{r.get('over',0)*100:>7.0f}")
    if out:
        with open(out, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=["P", "D", "n", "pitch_mean", "pitch_sd",
                                               "pos_span", "spd_sd", "over", "trouble"])
            w.writeheader()
            w.writerows(results)
        print(f"[结果] {out}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--sets", default="1000,100;1000,160;1000,220", help='P,D; P,D; ...')
    ap.add_argument("--secs", type=float, default=15.0, help="每个组合观察多久")
    ap.add_argument("--settle", type=float, default=2.0, help="换参数后先等多久")
    ap.add_argument("--counts-per-mm", type=float, default=10.8)
    ap.add_argument("--out", default="")
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args)))
