#!/usr/bin/env python3
"""占空比调速：在一个周期内交替给 Δ 和 O，用占空比控制平均加速度。

原理：速度 = 加速度的积分。
    「通」时 a = g·tan(Δ)，速度上升；「断」时 Δ=0，速度保持。
    平均斜率 ≈ 占空比 × a_on ⇒ 占空比 = 油门开度。
好处：Δ 可以取到能稳定破粘的值（不受门槛限制），再用占空比把它按比例减弱，
      解决「小于门槛蠕爬、大于门槛暴走」的两难。

注意：不每次发 ^ 0,0（那要 ~0.9s，PWM 就跑不起来）。
      本脚本直接改 O，先验证固件跟不跟得上这个更新率。

用法：
  python ble_pwm.py --deg 1.5 --period-ms 300 --duty 0,0.25,0.5,0.75,1.0 --hold-ms 8000 --out <前缀>
退出码：0 正常，4 翻车/越界（均写盘）。
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

    duties = [float(x) for x in args.duty.split(",")]
    rec = Rec()
    rows = []
    code = 0
    try:
        async with BleakClient(addr, timeout=20.0) as cli:
            await cli.start_notify(FFE1, rec.on_data)
            writes = {"n": 0, "fail": 0}

            async def send(cmd):
                try:
                    await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)
                    writes["n"] += 1
                except Exception:
                    writes["fail"] += 1

            await asyncio.sleep(0.8)
            await send("?")
            await asyncio.sleep(1.6)
            buf = "\n".join(l for _w, l in rec.text)
            m = re.search(r"\bO=([-0-9.]+)", buf)
            base = float(m.group(1)) if m else 4.00
            print(f"[原 O] {base}   Δ={args.deg:+.2f}°  周期 {args.period_ms}ms  每档 {args.hold_ms}ms")

            for duty in duties:
                await send("^ 0,0")
                await asyncio.sleep(0.8)
                p0 = rec.pos()
                if p0 is None:
                    raise Abort("拿不到位置")
                t0 = time.time()
                on_s = args.period_ms * duty / 1000.0
                off_s = args.period_ms * (1.0 - duty) / 1000.0
                while time.time() - t0 < args.hold_ms / 1000.0:
                    if on_s > 0.02:
                        await send(f"O {base + args.deg:.2f}")
                        await asyncio.sleep(on_s)
                    pit = rec.pitch()
                    if pit is not None and abs(pit - base) > args.fall_deg:
                        await send(f"O {base:.2f}")
                        raise Abort(f"翻车 pitch={pit:.1f}° duty={duty}")
                    if abs((rec.pos() or p0) - p0) / POS_PER_MM > args.max_travel:
                        await send(f"O {base:.2f}")
                        raise Abort(f"越界 duty={duty}")
                    if off_s > 0.02:
                        await send(f"O {base:.2f}")
                        await asyncio.sleep(off_s)
                t1 = time.time()
                await send(f"O {base:.2f}")
                await asyncio.sleep(args.settle_ms / 1000.0)
                p1 = rec.pos()
                disp = (p1 - p0) / POS_PER_MM
                rate = disp / (t1 - t0)
                rows.append((duty, disp, rate, t1 - t0))
                print(f"   duty={duty*100:5.1f}%  位移 {disp:+8.2f}mm  {rate:+7.2f}mm/s  ({t1-t0:.1f}s)")
                await asyncio.sleep(args.between_s)
    except Abort as e:
        print(f"[中止] {e} —— 数据仍写盘")
        code = 4
    except Exception as e:
        print(f"[异常] {type(e).__name__}: {e} —— 数据仍写盘")
        code = 5

    with open(args.out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req", "rx"])
        for wall, f in rec.frames:
            if len(f) >= 8:
                w.writerow([f"{wall:.3f}"] + f[:8])
    with open(args.out + ".pwm.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["duty", "disp_mm", "mm_s", "secs"])
        for r in rows:
            w.writerow([r[0], f"{r[1]:.2f}", f"{r[2]:.2f}", f"{r[3]:.2f}"])

    print(f"\n写入 {writes['n']} 次，失败 {writes['fail']} 次")
    if rows:
        print("=== 占空比 → 速度 ===")
        for duty, disp, rate, secs in rows:
            print(f"  {duty*100:5.1f}%  {rate:+7.2f} mm/s")
    print(f"[结果] {args.out}.g.csv / {args.out}.pwm.csv")
    return code


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--deg", type=float, default=1.5)
    ap.add_argument("--period-ms", type=float, default=300.0)
    ap.add_argument("--duty", default="0,0.25,0.5,0.75,1.0")
    ap.add_argument("--hold-ms", type=float, default=8000.0)
    ap.add_argument("--settle-ms", type=float, default=1500.0)
    ap.add_argument("--between-s", type=float, default=2.0)
    ap.add_argument("--fall-deg", type=float, default=6.0)
    ap.add_argument("--max-travel", type=float, default=150.0)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
