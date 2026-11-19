#!/usr/bin/env python3
"""相机无关的运动性能测试：按 profile 发档位，采集 D 帧全量遥测，逐段算指标。

为什么能做（2026-11-11 实测标定）：
  * 编码器位置 10.8 counts/mm（与相机对表得出），1.5s 内 ±40 counts ≈ ±4mm
  * 逐点速度 speed=9.48 counts/50ms sd（静止），单段 1.5s 只有 6 个样本 → SEM≈3.9，
    比 gear 1.0 的指令 5.0 还大，所以「瞬时速度跟踪」不可信；改用「整段位移/时长」当平均速度，
    4s 段 ≈ ±0.9 mm/s，对 9 mm/s 的指令是 10% 级
  * 姿态：pitch 分辨率 0.01°，静止 sd 0.14°，完全够用

用法：
  python ble_profile_test.py --address E1:... --profile "0:6,1.0:4,0:3,-1.0:4,0:3" --out ...\\prof01

profile 语法：gear:secs 逗号分隔，gear 可为 0（停机/静止）。每段结束都会先发 "^ 0,0" 再进下一段。
产出：<out>.csv（面板格式全列）、<out>.phases.csv（段起止墙钟）、stdout 指标表。
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
O_FALLBACK = 2.0


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

    profile = []
    for seg in args.profile.split(","):
        g, s = seg.split(":")
        profile.append((float(g), float(s)))

    out = args.out
    main_path, phase_path = out + ".csv", out + ".phases.csv"
    for p in (main_path, phase_path):
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

    rows = []          # (wall, [fields])
    par = {"text": ""}
    buf = {"s": ""}

    async with BleakClient(addr, timeout=30.0) as cli:
        def on_data(_s, data):
            buf["s"] += data.decode("utf-8", "replace")
            *done, buf["s"] = buf["s"].replace("\r", "\n").split("\n")
            for line in done:
                line = line.strip()
                if line.startswith("PAR:"):
                    par["text"] = line
                elif line.startswith("D,"):
                    rows.append((time.time(), line[2:].split(",")))

        await cli.start_notify(FFE1, on_data)

        async def send(t):
            await cli.write_gatt_char(FFE1, (t + "\n").encode(), response=True)
            await asyncio.sleep(0.05)

        for _ in range(20):
            if par["text"]:
                break
            await send("?")
            await asyncio.sleep(0.3)
        o_est = O_FALLBACK
        for item in par["text"].split():
            if item.startswith("O="):
                o_est = num(item[2:], O_FALLBACK)
        print(f"[车] {par['text'][:160]}")
        print(f"[O] {o_est:.2f}   [profile] {args.profile}")

        await send("^ 0,0")
        await asyncio.sleep(0.5)
        fall0 = None
        if rows:
            fi = header.index("fallStopCount") - 1 if "fallStopCount" in header else None
            if fi is not None and len(rows[-1][1]) > fi:
                fall0 = num(rows[-1][1][fi], 0)
        print(f"[基线] fallStopCount={fall0}")

        phases = []
        phases.append(("pre", 0.0, time.time(), time.time()))
        for idx, (gear, secs) in enumerate(profile, 1):
            t0 = time.time()
            end = time.monotonic() + secs
            while time.monotonic() < end:
                await send(f"^ 0,{gear:g}")
                await asyncio.sleep(args.send_gap)
            await send("^ 0,0")
            t1 = time.time()
            phases.append((f"P{idx} gear={gear:g}", gear, t0, t1))
            print(f"[段{idx}] gear={gear:g} {secs:.1f}s  t0={t0:.2f} t1={t1:.2f}")
        await asyncio.sleep(args.post)

    idx_of = {n: i for i, n in enumerate(header)}

    def col(row, name):
        # D 帧字段没有 time_s 列：row[0] 是 ms，所以 header 索引要减 1
        i = idx_of.get(name)
        return num(row[i - 1], None) if i is not None and 0 <= i - 1 < len(row) else None

    with open(main_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(header)
        for wall, f in rows:
            row = [f"{wall:.1f}"] + f
            w.writerow((row + [""] * len(header))[:len(header)])
    with open(phase_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["phase", "gear", "t0", "t1"])
        w.writerows(phases)

    # ---- 逐段指标 ----
    ppm = args.counts_per_mm
    print(f"\n{'段':<12}{'n':>4}{'指令':>7}{'应用':>7}{'速度mean':>9}{'sd':>6}{'SEM':>6}"
          f"{'位移mm':>9}{'均速mm/s':>9}{'pitch':>14}{'|p-O|max':>9}{'speedErr':>9}")
    for name, gear, t0, t1 in phases:
        sel = [(w, f) for w, f in rows if t0 <= w <= t1]
        if len(sel) < 3:
            print(f"{name:<12}  样本不足({len(sel)})")
            continue
        spd = [x for x in (col(f, "speed1000") for _, f in sel) if x is not None]
        pos = [x for x in (col(f, "pos100") for _, f in sel) if x is not None]
        pit = [x for x in (col(f, "pitch100") for _, f in sel) if x is not None]
        req = [x for x in (col(f, "manualReq1000") for _, f in sel) if x is not None]
        app = [x for x in (col(f, "manualApplied1000") for _, f in sel) if x is not None]
        serr = [x for x in (col(f, "speedError1000") for _, f in sel) if x is not None]
        if not spd or not pit:
            print(f"{name:<12}  缺列")
            continue
        sd = st.pstdev(spd) if len(spd) > 2 else 0.0
        disp = (pos[-1] - pos[0]) / 100.0 / ppm if len(pos) > 1 else 0.0
        dt = max(t1 - t0, 1e-6)
        print(f"{name:<12}{len(spd):>4}{(st.mean(req)/1000 if req else 0):>7.2f}"
              f"{(st.mean(app)/1000 if app else 0):>7.2f}{st.mean(spd)/1000:>9.2f}{sd/1000:>6.2f}"
              f"{sd/1000/max(len(spd)**0.5,1):>6.2f}{disp:>9.1f}{disp/dt:>9.1f}"
              f"{st.mean(pit)/100:>7.2f}±{sd/100:>5.2f}"
              f"{max(abs(x/100-o_est) for x in pit):>9.2f}{(st.mean(serr)/1000 if serr else 0):>9.2f}")
    print(f"\n[结果] {main_path}\n[结果] {phase_path}")
    print("说明：速度单位 counts/50ms（maxSpeed=5.0）；位移用 pos/100/counts_per_mm；"
          f"说明中 counts_per_mm={ppm}")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--profile", required=True, help='如 "0:6,1.0:4,0:3,-1.0:4,0:3"')
    ap.add_argument("--out", required=True, help="输出文件名（不含扩展名）")
    ap.add_argument("--send-gap", type=float, default=0.06, help="重发档位的间隔 s（固件 350ms 超时）")
    ap.add_argument("--post", type=float, default=4.0, help="跑完后继续收数据 s")
    ap.add_argument("--counts-per-mm", type=float, default=10.8)
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args)))
