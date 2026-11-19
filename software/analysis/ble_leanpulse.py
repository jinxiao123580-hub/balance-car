#!/usr/bin/env python3
"""短时倾角脉冲：临时改 O（平衡角）给一个持续 ms 毫秒的直接倾角，然后立刻恢复。

为什么用 O：TF/TB 被 clamp 在 ±1.0°，kick 固定 0.5°，只有 O 能给任意角度。
O 只在运行时生效，脚本结束前一定恢复原值，全程不 SAVE（flash 里仍是原 O）。

判据：2° 偏置保持 400ms 应产生 ≈ g·tan2°·t²/2 = 27mm 的位移；
      若位移≈0 或方向反了，说明该方向走不动。
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
POS_PER_MM = 1080.0
I_MS, I_POS, I_PITCH, I_TARGET, I_HZ = 0, 1, 3, 4, 5


def num(v, d=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


class Rec:
    def __init__(self):
        self.text, self.frames, self.buf = [], [], ""

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


def window(frames, t0, t1):
    pts = []
    for w, f in frames:
        if t0 <= w <= t1 and len(f) >= 8:
            p = (num(f[I_POS]), num(f[I_PITCH]), num(f[I_TARGET]), num(f[I_HZ]))
            if None not in p:
                pts.append(p)
    if len(pts) < 2:
        return None
    return {
        "disp": (pts[-1][0] - pts[0][0]) / POS_PER_MM,
        "pitch_lo": min(p[1] for p in pts) / 100.0,
        "pitch_hi": max(p[1] for p in pts) / 100.0,
        "tgt_lo": min(p[2] for p in pts) / 100.0,
        "tgt_hi": max(p[2] for p in pts) / 100.0,
        "hz_lo": min(p[3] for p in pts),
        "hz_hi": max(p[3] for p in pts),
        "n": len(pts),
    }


async def main(args):
    out = args.out
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

    rec = Rec()
    async with BleakClient(addr, timeout=20.0) as cli:
        await cli.start_notify(FFE1, rec.on_data)

        async def send(cmd):
            await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)
            print(f"  -> {cmd}")

        await asyncio.sleep(0.8)
        await send("?")
        await asyncio.sleep(1.6)
        o0 = None
        for _w, line in rec.text:
            m = re.search(r"\bO=([-0-9.]+)", line)
            if m:
                o0 = m.group(1)
        base = float(o0) if o0 else 2.30
        print(f"[原 O] {base}")

        rows = []
        p0 = rec.pos()
        try:
            for deg in args.deg:
                for rep in range(args.reps):
                    target = base + deg
                    # O 与 SAVE 同族，受手动态门控：必须先 ^ 0,0 归零，否则命令被静默丢弃
                    await send("^ 0,0")
                    await asyncio.sleep(0.9)
                    p_before = rec.pos()
                    t0 = time.time()
                    await send(f"O {target:.2f}")
                    await asyncio.sleep(args.ms / 1000.0)
                    p_end = rec.pos()          # 位移取在保持期结束、恢复之前
                    await send(f"O {base:.2f}")
                    t1 = time.time()
                    await asyncio.sleep(args.settle)
                    p_after = rec.pos()
                    t2 = time.time()
                    s = window(rec.frames, t0, t2)
                    s2 = window(rec.frames, t1, t2)
                    # 位移=保持期内(pos_end - pos_before)，并给平均速度；恢复后的漂移另记
                    if p_before is not None and p_end is not None:
                        direct = (p_end - p_before) / POS_PER_MM
                        rate = direct / (args.ms / 1000.0)
                    else:
                        direct = float("nan")
                        rate = float("nan")
                    if p_before is not None and p_after is not None:
                        after = (p_after - p_end) / POS_PER_MM
                    else:
                        after = float("nan")
                    cur = rec.pos()
                    if cur is not None and p0 is not None and abs(cur - p0) / POS_PER_MM > args.max_travel:
                        print(f"[中止] 位移超 {args.max_travel}mm")
                        return 4
                    if s:
                        s["disp_direct"] = direct
                        s["rate"] = rate
                        rows.append((deg, rep, s, s2))
                        print(f"   Δ={deg:+.2f}° (O={target:.2f}) rep{rep}  保持期={direct:+7.2f}mm "
                              f"({rate:+6.2f}mm/s)  恢复后={after:+6.2f}mm  "
                              f"pitch[{s['pitch_lo']:.2f},{s['pitch_hi']:.2f}] n={s['n']}")
        finally:
            try:
                await cli.write_gatt_char(FFE1, f"O {base:.2f}\n".encode(), response=True)
                print(f"[恢复] O={base:.2f}")
                await cli.write_gatt_char(FFE1, b"^ 0,0\n", response=True)
            except Exception:
                pass

    with open(out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req", "rx"])
        for wall, f in rec.frames:
            if len(f) >= 8:
                w.writerow([f"{wall:.3f}"] + f[:8])
    with open(out + ".pulses.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["deg", "rep", "pulse_mm", "after_mm", "pitch_lo", "pitch_hi",
                    "tgt_lo", "tgt_hi", "hz_lo", "hz_hi"])
        for deg, rep, s, s2 in rows:
            w.writerow([deg, rep, f"{s['disp']:.2f}", f"{s2['disp']:.2f}" if s2 else "",
                        f"{s['pitch_lo']:.2f}", f"{s['pitch_hi']:.2f}",
                        f"{s['tgt_lo']:.2f}", f"{s['tgt_hi']:.2f}",
                        f"{s['hz_lo']:.0f}", f"{s['hz_hi']:.0f}"])

    print("\n=== 汇总 ===")
    for deg in args.deg:
        ds = [r[2].get("disp_direct", r[2]["disp"]) for r in rows if r[0] == deg]
        if ds:
            mu = sum(ds) / len(ds)
            sd = (sum((x - mu) ** 2 for x in ds) / len(ds)) ** 0.5
            print(f"Δ={deg:+.2f}°  n={len(ds)}  保持期均值={mu:+7.2f}mm  "
                  f"({mu/(args.ms/1000.0):+6.2f}mm/s)  sd={sd:5.2f}")
    print(f"[结果] {out}.g.csv / {out}.pulses.csv")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--deg", type=float, nargs="+", default=[-2.0, 2.0])
    ap.add_argument("--ms", type=float, default=400.0)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--settle", type=float, default=2.0)
    ap.add_argument("--max-travel", type=float, default=200.0)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
