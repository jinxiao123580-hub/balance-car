#!/usr/bin/env python3
"""成对行程：后退一段 → 前进回原点（编码器闭环），保证车不会一路走掉。

为什么这样：Delta 偏置是单向的限时脉冲，只做后退会让车每轮净走一段，最后掉桌。
本脚本每轮先按 --back-deg 后退 --back-ms，然后按 --fwd-deg 前进，
**用编码器判位，回到出发点 ±tol 内立刻收令**，净位移始终被约束住。

口径：Δ 一律是「相对 O（平衡点）的差值」，不是绝对角。
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

    def pitch(self):
        for _w, f in reversed(self.frames):
            if len(f) > I_PITCH:
                v = num(f[I_PITCH])
                return None if v is None else v / 100.0   # 帧里是百分度
        return None


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

        async def set_o(v):
            await send("^ 0,0")
            await asyncio.sleep(0.9)
            await send(f"O {v:.2f}")
            await asyncio.sleep(0.3)

        await asyncio.sleep(0.8)
        buf = ""
        await send("?")
        await asyncio.sleep(1.6)
        for _w, line in rec.text:
            buf += line + "\n"
        m = re.search(r"\bO=([-0-9.]+)", buf)
        base = float(m.group(1)) if m else 2.30
        print(f"[原 O] {base}")

        rows = []
        origin = rec.pos()
        try:
            for rep in range(args.reps):
                p0 = rec.pos()
                if p0 is None:
                    continue
                # ---- 后退段 ----
                await set_o(base + args.back_deg)
                t0 = time.time()
                while time.time() - t0 < args.back_ms / 1000.0:
                    await asyncio.sleep(0.15)
                    cur = rec.pos()
                    pit = rec.pitch()
                    if pit is not None and abs(pit - base) > args.fall_deg:
                        print(f"[中止] 翻车/失稳 pitch={pit:.1f}° (O={base:.2f})")
                        await send("^ 0,0")
                        await send(f"O {base:.2f}")
                        return 4
                    if cur is not None and abs(cur - p0) / POS_PER_MM > args.max_travel:
                        print(f"[中止] 后退段位移超 {args.max_travel}mm")
                        await send("^ 0,0")
                        await send(f"O {base:.2f}")
                        return 4
                p1 = rec.pos()
                t1 = time.time()
                back_mm = (p1 - p0) / POS_PER_MM
                back_s = t1 - t0
                await send(f"O {base:.2f}")
                await asyncio.sleep(args.hold_pause)

                # ---- 前进回原点：短脉冲逐段逼近 ----
                # 4Hz 遥测下位置每 250ms 才更新，一次长脉冲必超调，所以按脉冲+停顿逼近
                t2 = time.time()
                p2 = p1
                pulses = 0
                while time.time() - t2 < args.fwd_max_ms / 1000.0 and pulses < args.max_pulses:
                    if p2 is not None and (p0 - p2) / POS_PER_MM >= -args.tol:
                        break
                    await set_o(base + args.fwd_deg)
                    await asyncio.sleep(args.fwd_pulse_ms / 1000.0)
                    await send(f"O {base:.2f}")
                    pulses += 1
                    await asyncio.sleep(args.hold_pause)
                    p2 = rec.pos()
                    if p2 is not None and abs(p2 - p0) / POS_PER_MM > args.max_travel * 1.5:
                        print("[中止] 回程越界")
                        await send("^ 0,0")
                        await send(f"O {base:.2f}")
                        return 4
                t3 = time.time()
                await send(f"O {base:.2f}")
                await asyncio.sleep(args.hold_pause)
                p3 = rec.pos()
                fwd_mm = (p3 - p1) / POS_PER_MM
                net_mm = (p3 - p0) / POS_PER_MM
                rows.append((rep, back_mm, back_mm / back_s if back_s else 0,
                             fwd_mm, fwd_mm / (t3 - t2) if t3 > t2 else 0, net_mm, pulses))
                print(f"   rep{rep}  后退 {back_mm:+7.2f}mm ({back_mm/back_s if back_s else 0:+6.2f}mm/s)"
                      f"  前进回程 {fwd_mm:+7.2f}mm ({pulses} 脉冲, {(t3-t2):4.1f}s)"
                      f"  净位移 {net_mm:+6.2f}mm")
        finally:
            try:
                await send("^ 0,0")
                await send(f"O {base:.2f}")
                print(f"[恢复] O={base:.2f}")
            except Exception:
                pass

    with open(out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req", "rx"])
        for wall, f in rec.frames:
            if len(f) >= 8:
                w.writerow([f"{wall:.3f}"] + f[:8])
    with open(out + ".pairs.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["rep", "back_mm", "back_mm_s", "fwd_mm", "fwd_mm_s", "net_mm"])
        for r in rows:
            w.writerow([r[0]] + [f"{v:.2f}" for v in r[1:]])

    if rows:
        print("\n=== 汇总 ===")
        for i, nm in ((1, "后退位移"), (2, "后退速度"), (3, "回程位移"), (5, "净位移")):
            vs = [r[i] for r in rows]
            mu = sum(vs) / len(vs)
            sd = (sum((x - mu) ** 2 for x in vs) / len(vs)) ** 0.5
            print(f"{nm}: 均值 {mu:+8.2f}  sd {sd:6.2f}  n={len(vs)}")
    print(f"[结果] {out}.g.csv / {out}.pairs.csv")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--back-deg", type=float, default=1.3, help="后退 Δ（相对 O，正=后退）")
    ap.add_argument("--fwd-deg", type=float, default=-1.2, help="前进 Δ（负=前进）")
    ap.add_argument("--back-ms", type=float, default=8000.0)
    ap.add_argument("--fwd-max-ms", type=float, default=8000.0)
    ap.add_argument("--tol", type=float, default=8.0, help="回到原点容差 mm")
    ap.add_argument("--hold-pause", type=float, default=0.8)
    ap.add_argument("--fwd-pulse-ms", type=float, default=250.0)
    ap.add_argument("--max-pulses", type=int, default=12)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--max-travel", type=float, default=250.0)
    ap.add_argument("--fall-deg", type=float, default=6.0, help="|pitch-O| 超过此值判为翻车")
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
