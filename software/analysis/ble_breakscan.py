#!/usr/bin/env python3
"""破粘门槛扫描：对每个 Δ 重复 N 次脉冲，统计「破粘成功率」与位移分布。

为什么要成功率而不是单次位移：实测同一 Δ 单次位移跨 20~100 倍，
单次测量没有意义。这里给每个 Δ 打 N 枪，看几枪破粘（|位移| > --break-mm）。

判据：负 Δ = 前进，正 Δ = 后退（pos 为正=后退）。
退出码：0 正常，2 连不上，4 翻车中止（仍写盘）。
"""
import argparse
import asyncio
import csv
import os
import re
import statistics
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
POS_PER_MM = 1080.0
I_POS, I_PITCH = 1, 3


class Abort(Exception):
    pass


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

    def _last(self, idx, scale=1.0):
        for _w, f in reversed(self.frames):
            if len(f) > idx:
                try:
                    return float(f[idx]) / scale
                except ValueError:
                    return None
        return None

    def pos(self):
        return self._last(I_POS)

    def pitch(self):
        return self._last(I_PITCH, 100.0)


async def main(args):
    d = os.path.dirname(args.out)
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
    degs = [float(x) for x in args.deg.split(",")]
    sign = -1 if args.dir == "fwd" else +1
    rows = []
    code = 0
    try:
        async with BleakClient(addr, timeout=20.0) as cli:
            await cli.start_notify(FFE1, rec.on_data)

            async def send(cmd):
                for attempt in (1, 2):
                    try:
                        await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)
                        return
                    except Exception:
                        if attempt == 2:
                            raise Abort("BLE 写入失败")
                        await asyncio.sleep(0.3)

            await asyncio.sleep(0.8)
            await send("?")
            await asyncio.sleep(1.6)
            await send("^ 0,0")
            await asyncio.sleep(0.6)
            await send(f"O {args.base:.2f}")
            await asyncio.sleep(0.5)
            print(f"[base] O={args.base:.2f}  dir={args.dir}  角度={degs}  "
                  f"每档 {args.reps} 枪 × {args.ms}ms  间隔 {args.gap_s}s  "
                  f"破粘判据 >{args.break_mm}mm")

            for deg in degs:
                got = []
                for rep in range(args.reps):
                    pit = rec.pitch()
                    if pit is not None and abs(pit - args.base) > args.fall_deg:
                        await send(f"O {args.base:.2f}")
                        raise Abort(f"翻车 pitch={pit:.1f}°（deg={deg} rep={rep}）")
                    p0 = rec.pos()
                    await send(f"O {args.base + sign * deg:.2f}")
                    await asyncio.sleep(args.ms / 1000.0)
                    p1 = rec.pos()
                    await send(f"O {args.base:.2f}")
                    await asyncio.sleep(args.gap_s)
                    if p0 is None or p1 is None:
                        continue
                    disp = sign * (p1 - p0) / POS_PER_MM      # 正 = 朝该方向走了
                    got.append(disp)
                    print(f"   Δ={deg:.2f} #{rep+1}  {disp:+7.2f}mm")
                broke = sum(1 for x in got if x >= args.break_mm)
                rows.append((deg, len(got), broke, got))
                if got:
                    print(f"   → Δ={deg:.2f}: 破粘 {broke}/{len(got)}  "
                          f"中位 {statistics.median(got):+.1f}mm  最大 {max(got):+.1f}mm")
    except Abort as e:
        print(f"[中止] {e} —— 数据仍写盘")
        code = 4
    except Exception as e:
        print(f"[异常] {type(e).__name__}: {e} —— 数据仍写盘")
        code = 5
    finally:
        try:
            await send(f"O {args.base:.2f}")
        except Exception:
            pass

    with open(args.out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req", "rx"])
        for wall, f in rec.frames:
            if len(f) >= 8:
                w.writerow([f"{wall:.3f}"] + f[:8])
    with open(args.out + ".scan.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["deg", "n", "broke", "median_mm", "max_mm", "all_mm"])
        for deg, n, broke, got in rows:
            w.writerow([deg, n, broke,
                        f"{statistics.median(got):.2f}" if got else "",
                        f"{max(got):.2f}" if got else "",
                        " ".join(f"{x:.1f}" for x in got)])

    if rows:
        print(f"\n=== {args.dir} 汇总 ===")
        print("  Δ      枪数  破粘  中位mm  最大mm")
        for deg, n, broke, got in rows:
            med = f"{statistics.median(got):+8.1f}" if got else "     n/a"
            mx = f"{max(got):+8.1f}" if got else "     n/a"
            print(f"  {deg:5.2f} {n:5d} {broke:5d} {med} {mx}")
    print(f"[结果] {args.out}.g.csv / {args.out}.scan.csv")
    return code


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--base", type=float, default=4.00)
    ap.add_argument("--dir", choices=["fwd", "back"], default="fwd")
    ap.add_argument("--deg", default="1.15,1.30,1.45", help="要扫的 Δ 列表（正数）")
    ap.add_argument("--ms", type=float, default=1500.0)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--gap-s", type=float, default=2.5)
    ap.add_argument("--break-mm", type=float, default=10.0)
    ap.add_argument("--fall-deg", type=float, default=6.0)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
