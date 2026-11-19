#!/usr/bin/env python3
"""倾角-运动静态特性：直接把平衡点 O 偏 δ，看车在 δ 下 0.9s 走多少毫米。

这是「走起来到底需要多大倾角」的直接测量：
  * δ 是人为加的稳态倾角，闭环会把车体压到 O+δ，于是产生恒定加速度
  * 用 pos（10.8 counts/mm）量位移，扣掉每段前的漂移基线
  * 姿态列同时给出 pitch 实际值，可验证车体真的到了 O+δ

安全：每段只持续 --exposure 秒就回 O；|pitch−O|>1.6° 或累计位移>60mm 立即中止并复位 O。

用法：
  python ble_lean_scan.py --address E1:... --deltas 0.1,0.2,0.3,0.5,0.8 --exposure 0.9 --out ...\\lean01
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


def load_header(rec_dir):
    best = None
    for name in os.listdir(rec_dir):
        if name.endswith(".csv") and not name.endswith(".fast.csv"):
            p = os.path.join(rec_dir, name)
            if best is None or os.path.getmtime(p) > os.path.getmtime(best):
                best = p
    with open(best, encoding="utf-8-sig") as fh:
        return next(csv.reader(fh))


def num(v, d=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


async def main(args):
    here = os.path.dirname(os.path.abspath(__file__))
    project = os.path.abspath(os.path.join(here, "..", ".."))
    header = load_header(os.path.join(project, "docs", "validation", "recordings"))
    idx = {n: i for i, n in enumerate(header)}

    out = args.out
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    for p in (out + ".csv", out + ".steps.csv"):
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

    rows, grows, par, buf = [], [], {"t": ""}, {"s": ""}

    async with BleakClient(addr, timeout=30.0) as cli:
        def on_data(_s, data):
            buf["s"] += data.decode("utf-8", "replace")
            *done, buf["s"] = buf["s"].replace("\r", "\n").split("\n")
            for line in done:
                line = line.strip()
                if line.startswith("PAR:"):
                    par["t"] = line
                elif line.startswith("D,"):
                    rows.append((time.time(), line[2:].split(",")))
                elif line.startswith("G,"):
                    # G 帧 4Hz：ms,pos,speed,pitch,target,hz,req,rx —— 本实验用它取 pos/pitch
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
        o0 = 2.0
        for item in par["t"].split():
            if item.startswith("O="):
                o0 = num(item[2:], 2.0)
        print(f"[O0] {o0:.3f}   [PAR] {par['t'][:120]}")

        await send("^ 0,0")
        await asyncio.sleep(1.5)
        # 起始就位检查：车必须已经在 O 附近平衡，否则后面所有 δ 都不可比
        pnow = None
        for _ in range(12):
            sel = [num(f[3], None) for w, f in grows if len(f) > 3]
            if sel:
                pnow = sel[-1] / 100
            if pnow is not None and abs(pnow - o0) < 1.0:
                break
            await asyncio.sleep(0.3)
        print(f"[就位] pitch={pnow}°  O={o0:.2f}°")
        if pnow is None or abs(pnow - o0) > 1.0:
            print("[中止] 起始姿态不在 O 附近：车被扶着/放平/或 O 已漂，先别测")
            return 5

        deltas = []
        for x in args.deltas.split(","):
            v = float(x)
            deltas += [v, -v]

        steps = []
        cum = 0.0
        aborted = ""
        for d in deltas:
            if aborted:
                break
            await send(f"O {o0 + d:.3f}")   # O 是持久参数，发一次就够；反复发会挤掉遥测
            await asyncio.sleep(0.10)
            t0 = time.time()
            pbase = None
            for w, f in reversed(grows):
                if len(f) > 3:
                    pbase = num(f[1], None)
                    break
            endx = time.monotonic() + args.exposure
            moved = False
            while time.monotonic() < endx:
                await asyncio.sleep(0.1)
                pl = [num(f[1], None) for w, f in grows if w > t0 and len(f) > 3]
                if (pbase is not None and pl
                        and abs(pl[-1] - pbase) / 100.0 / args.counts_per_mm > args.max_travel):
                    moved = True
                    break
            t1 = time.time()
            if moved:
                await send(f"O {o0:.3f}")
                await send("^ 0,0")
                print(f"[中止] δ={d:+.2f}° 位移超 {args.max_travel:.0f}mm，先停手")
                aborted = f"δ={d:+.2f} 位移超限"
                break
            await send(f"O {o0:.3f}")
            # 回正后必须等车真的静止，再做下一档：pitch 贴 O、速度归零、位置不再移动
            t_settle0 = time.monotonic()
            min_end = t_settle0 + args.settle
            max_end = t_settle0 + args.settle + args.settle_max
            bad = 0
            still = 0
            while True:
                await asyncio.sleep(0.15)
                sel = [(w, f) for w, f in grows if w > t1 and len(f) > 4]
                now = time.monotonic()
                if sel:
                    pit = num(sel[-1][1][3]) / 100.0
                    spd = num(sel[-1][1][2]) / 1000.0
                    if abs(pit - o0) > 1.6:
                        bad += 1
                        if bad >= 3:
                            print(f"[中止] pitch 偏离过大 {pit:+.2f}°")
                            await send(f"O {o0:.3f}")
                            await send("^ 0,0")
                            aborted = "pitch 偏离过大"
                            break
                    else:
                        bad = 0
                    if abs(pit - o0) < 0.30:
                        # 用位置判静止：取最近 1s 内 pos 的变化，速度信号本身抖 ±9 不可用
                        recent = [(w, num(f[1])) for w, f in sel if w >= now - 1.0 and num(f[1]) is not None]
                        if len(recent) >= 3:
                            drift_mm = abs(recent[-1][1] - recent[0][1]) / 1080.0
                            if drift_mm < 1.5:
                                still += 1
                            else:
                                still = 0
                        else:
                            still = 0
                    else:
                        still = 0
                if now >= min_end and still >= 3:
                    break
                if now >= max_end:
                    print(f"[警告] δ={d:+.2f} 回正后 {args.settle + args.settle_max:.1f}s 未完全静止"
                          f"（pitch={pit:+.2f} spd={spd:+.1f}），仍继续")
                    break
            print(f"   回正用时 {time.monotonic() - t_settle0:.1f}s")
            steps.append((d, t0, t1))
            print(f"[δ={d:+.2f}°] t0={t0:.2f} t1={t1:.2f}")

        await send("^ 0,0")
        await send(f"O {o0:.3f}")
        await asyncio.sleep(1.0)

    with open(out + ".csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for wall, f in rows:
            w.writerow(([f"{wall:.1f}"] + f + [""] * len(header))[:len(header)])
    with open(out + ".steps.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["delta_deg", "t0", "t1"])
        w.writerows(steps)
    with open(out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req"])
        for wall, f in grows:
            w.writerow([f"{wall:.3f}"] + f[:8])

    def gpos(f):
        return num(f[1], None)

    def gpitch(f):
        return num(f[3], None)

    def med_pos(a, b):
        v = [gpos(f) for w, f in grows if a <= w <= b and gpos(f) is not None]
        return st.mean(v) if v else None      # 用窗口均值判位置，端点单帧噪声太大

    ppm = args.counts_per_mm
    print(f"\n{'δ(°)':>6}{'pitch态(°)':>11}{'段前均值':>10}{'段内均值':>10}{'Δmm':>8}"
          f"{'扣漂移mm':>10}{'均速mm/s':>10}{'加速度mm/s2':>12}")
    cum = 0.0
    for d, t0, t1 in steps:
        pre = med_pos(t0 - 0.90, t0 - 0.10)          # 动作前 0.9s 的均值
        post = med_pos(t0 + 0.15, t1)                # 动作段（跳过前 0.15s 过渡）的均值
        drift_base = med_pos(t0 - 1.80, t0 - 1.00)   # 更早 0.9s，用于估算本底漂移
        pmean = [gpitch(f) for w, f in grows if t0 <= w <= t1 and gpitch(f) is not None]
        if pre is None or post is None or not pmean:
            print(f"{d:>+6.2f}   (缺帧)")
            continue
        drift = (pre - drift_base) / 100.0 / ppm if drift_base is not None else 0.0
        disp = (post - pre) / 100.0 / ppm - drift
        cum += disp
        v = disp / args.exposure
        print(f"{d:>+6.2f}{st.mean(pmean)/100:>11.2f}{pre/100:>9.1f}{post/100:>9.1f}"
              f"{(post-pre)/100/ppm:>8.1f}{disp:>10.1f}{v:>10.1f}{2*disp/(args.exposure**2):>12.1f}")
    print(f"\n累计位移 {cum:+.1f} mm（正=+x 方向）")
    print(f"[结果] {out}.csv / {out}.steps.csv")
    if aborted:
        print(f"[中止] {aborted}（已保存中止前的数据）")
        return 4
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--deltas", default="0.1,0.2,0.3,0.5,0.8", help="倾角偏移列表，自动正负交替")
    ap.add_argument("--exposure", type=float, default=0.9, help="每个 δ 持续多久 s")
    ap.add_argument("--settle", type=float, default=2.2, help="回 O 后最少静置多久 s")
    ap.add_argument("--settle-max", type=float, default=8.0, help="等待完全静止的额外上限 s")
    ap.add_argument("--counts-per-mm", type=float, default=10.8)
    ap.add_argument("--max-travel", type=float, default=45.0, help="单段位移上限 mm")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args)))
