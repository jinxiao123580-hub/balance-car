#!/usr/bin/env python3
"""纯编码器「倾角权限」测试：不加相机，扫 EG（速度误差→倾角增益）。

背景（2026-11-12 机制复核）：
- `^ 0,±1.0` 只给**速度**目标（main.cpp:3556），倾角由速度误差生成，
  `motionLean.errorGain`（蓝牙命令 `EG`，clamp 0~0.15，默认 0.05）就是那个增益。
- G 帧同时给出 `hz`（步进频率指令）与 `pos`（编码器）：∫hz×0.0625mm 与 Δpos 对得上
  ⇒ 电机跟指令；对不上 ⇒ 失步/卡住。
- EG 提高后位移明显变大且散差保持小 ⇒ 纯权限问题；提到 0.11~0.15 仍只有几 mm
  ⇒ 被极限环保住，问题不在增益。

用法：
  python ble_leantest.py --eg 0.05,0.08,0.11 --reps 3 --hold 2.0 --out <前缀>
退出码：0 正常，2 连不上，4 位移超限中止。
"""
import argparse
import asyncio
import csv
import os
import re
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
MM_PER_STEP = 0.0625      # 1 步 = 0.0625mm（1750Hz = 109mm/s）
POS_PER_MM = 1080.0       # pos/100/10.8 = mm
# G 帧字段（去掉 "G," 后）：0=ms 1=pos 2=speed 3=pitch 4=target 5=hz 6=req 7=btRx
I_MS, I_POS, I_PITCH, I_TARGET, I_HZ = 0, 1, 3, 4, 5
# seg_stats 内部 tuple 下标：ms,pos,pitch,target,hz
T_MS, T_POS, T_PITCH, T_TARGET, T_HZ = 0, 1, 2, 3, 4


def num(v, d=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


class Rec:
    def __init__(self):
        self.text = []
        self.frames = []          # (wall, [fields...])
        self.buf = ""

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

    def pos(self):
        for _w, f in reversed(self.frames):
            if len(f) > I_POS:
                return num(f[I_POS])
        return None


def seg_stats(frames, t0, t1):
    sel = [(w, f) for w, f in frames if t0 <= w <= t1 and len(f) >= 8]
    pts = []
    for _w, f in sel:
        p = (num(f[I_MS]), num(f[I_POS]), num(f[I_PITCH]), num(f[I_TARGET]), num(f[I_HZ]))
        if None not in p:
            pts.append(p)
    if len(pts) < 2:
        return None
    integ = 0.0
    for a, b in zip(pts, pts[1:]):
        dms = b[T_MS] - a[T_MS]
        if dms <= 0 or dms > 2000:
            continue
        integ += 0.5 * (a[T_HZ] + b[T_HZ]) * dms / 1000.0    # Σ hz·dt = 步数
    return {
        "n": len(pts),
        "disp": (pts[-1][T_POS] - pts[0][T_POS]) / POS_PER_MM,
        "pred": integ * MM_PER_STEP,
        "hz_mean": sum(p[T_HZ] for p in pts) / len(pts),
        "hz_peak": max(pts, key=lambda p: abs(p[T_HZ]))[T_HZ],
        "pitch": sum(p[T_PITCH] for p in pts) / len(pts) / 100.0,
        "target": sum(p[T_TARGET] for p in pts) / len(pts) / 100.0,
    }


async def main(args):
    out = args.out
    for p in (out + ".g.csv", out + ".phases.csv"):
        if os.path.exists(p):
            raise SystemExit(f"输出已存在：{p}")
    d = os.path.dirname(out)
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

    eg_list = [float(x) for x in args.eg.split(",")] if args.eg else [None]
    rec = Rec()
    async with BleakClient(addr, timeout=20.0) as cli:
        await cli.start_notify(FFE1, rec.on_data)

        async def send(cmd):
            await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)
            print(f"  -> {cmd}")

        await asyncio.sleep(0.8)
        await send("?")
        await asyncio.sleep(1.2)
        eg0 = None
        for _w, line in rec.text:
            m = re.search(r"EG=([-0-9.]+)", line)
            if m:
                eg0 = m.group(1)
        print(f"[原参数] EG={eg0}")

        p0 = rec.pos()
        print(f"[起始 pos] {p0}")
        phases = []
        try:
            for eg in eg_list:
                if eg is not None:
                    await send(f"EG {eg:.3f}")
                    await asyncio.sleep(0.5)
                for rep in range(args.reps):
                    for label, sign in (("FWD", +1.0), ("BWD", -1.0)):
                        for kind in ("hold", "idle"):
                            t0 = time.time()
                            if kind == "hold":
                                lean = sign * args.lean
                                # 指令 350ms 超时释放，必须持续刷新
                                while time.time() - t0 < args.hold:
                                    await cli.write_gatt_char(
                                        FFE1, f"^ 0,{lean:.2f}\n".encode(), response=True)
                                    cur = rec.pos()
                                    if (cur is not None and p0 is not None
                                            and abs(cur - p0) / POS_PER_MM > args.max_travel):
                                        print(f"[中止] 位移超 {args.max_travel}mm")
                                        await send("^ 0,0")
                                        return 4
                                    await asyncio.sleep(0.11)
                            else:
                                await asyncio.sleep(args.hold)
                            await send("^ 0,0")
                            t1 = time.time()
                            s = seg_stats(rec.frames, t0, t1)
                            if s:
                                s.update({"label": label, "kind": kind, "rep": rep,
                                          "eg": eg, "t0": t0, "t1": t1})
                                phases.append(s)
                                print(f"   EG={eg} rep{rep} {label:3s} {kind:4s} "
                                      f"disp={s['disp']:+8.2f}mm pred={s['pred']:+8.2f}mm "
                                      f"hz={s['hz_mean']:+7.1f}/{s['hz_peak']:+7.1f} "
                                      f"pitch={s['pitch']:5.2f} target={s['target']:5.2f}")
                            await asyncio.sleep(args.settle)
        finally:
            try:
                if eg_list[0] is not None:
                    back = eg0 or "0.050"
                    await cli.write_gatt_char(FFE1, f"EG {back}\n".encode(), response=True)
                    print(f"[恢复] EG={back}")
                await cli.write_gatt_char(FFE1, b"^ 0,0\n", response=True)
            except Exception:
                pass

    with open(out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req", "rx"])
        for wall, f in rec.frames:
            if len(f) >= 8:
                w.writerow([f"{wall:.3f}"] + f[:8])
    with open(out + ".phases.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["eg", "rep", "label", "kind", "t0", "t1", "disp_mm", "pred_mm",
                    "hz_mean", "hz_peak", "pitch_mean", "target_mean"])
        for s in phases:
            w.writerow([s["eg"], s["rep"], s["label"], s["kind"], f"{s['t0']:.3f}",
                        f"{s['t1']:.3f}", f"{s['disp']:.2f}", f"{s['pred']:.2f}",
                        f"{s['hz_mean']:.1f}", f"{s['hz_peak']:.1f}",
                        f"{s['pitch']:.2f}", f"{s['target']:.2f}"])

    print("\n=== 汇总（hold 段）===")
    print(f"{'EG':>6} {'方向':>4} {'n':>3} {'均值mm':>9} {'sd':>7} {'∫hz mm':>9} {'target均值':>10}")
    for eg in eg_list:
        for label in ("FWD", "BWD"):
            hs = [s for s in phases if s["kind"] == "hold" and s["eg"] == eg and s["label"] == label]
            if not hs:
                continue
            ds = [s["disp"] for s in hs]
            mu = sum(ds) / len(ds)
            sd = (sum((x - mu) ** 2 for x in ds) / len(ds)) ** 0.5
            pr = sum(s["pred"] for s in hs) / len(hs)
            tg = sum(s["target"] for s in hs) / len(hs)
            print(f"{str(eg):>6} {label:>4} {len(ds):>3} {mu:>9.2f} {sd:>7.2f} {pr:>9.2f} {tg:>10.2f}")
    ids = [s["disp"] for s in phases if s["kind"] == "idle"]
    if ids:
        print(f"空闲基线：均值 {sum(ids)/len(ids):+.2f}mm/段  n={len(ids)}")
    print(f"[结果] {out}.g.csv / {out}.phases.csv")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--eg", default="", help="逗号分隔的 EG 列表；留空则只测当前值")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--hold", type=float, default=2.0)
    ap.add_argument("--lean", type=float, default=1.0)
    ap.add_argument("--settle", type=float, default=1.0)
    ap.add_argument("--max-travel", type=float, default=120.0)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
