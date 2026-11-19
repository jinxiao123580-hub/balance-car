#!/usr/bin/env python3
"""定角度 + 占空比调速 + 三级停车。

策略（用户方案）：
  1. 取扫出来的破粘角 θ=1.30°（前进取负、后退取正）
  2. 速度靠「θ 占用的时间比例」控制：通 on_ms / 断 off_ms，占空比 = on/(on+off)
     —— 断的时候 Δ=0，只是不加速；要真正降速还得靠最后那段反向刹车
  3. 停车三级：① Δ 回 0（停止加速）→ ② 反向角刹车 → ③ 回 0 保持

判据：负 Δ = 前进，正 Δ = 后退（pos 为正=后退）。
退出码：0 到位并刹停，3 超时，4 翻车，5 异常（均写盘）。
"""
import argparse
import asyncio
import csv
import math
import os
import re
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
POS_PER_MM = 1080.0
SPEED_TO_MM_S = 20.0 / POS_PER_MM
I_POS, I_PITCH, I_SPEED = 1, 3, 2


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

    def vspeed(self):
        v = self._last(I_SPEED)
        return None if v is None else v * SPEED_TO_MM_S


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

            async def check_fall(tag):
                pit = rec.pitch()
                if pit is not None and abs(pit - args.base) > args.fall_deg:
                    await send(f"O {args.base:.2f}")
                    raise Abort(f"翻车 pitch={pit:.1f}°（{tag}）")

            for duty in [float(x) for x in args.duty.split(",")]:
                print(f"\n[duty={duty*100:.0f}%]  θ={args.deg}°  on={args.on_ms}ms")
                p0 = rec.pos()
                if p0 is None:
                    raise Abort("拿不到位置")
                t0 = time.time()
                moved = 0.0
                pulses = 0
                on_s = args.on_ms / 1000.0
                off_s = on_s * (1.0 - duty) / duty if duty > 0.0 else 0.0
                while time.time() - t0 < args.max_s:
                    await check_fall(f"duty={duty}")
                    p = rec.pos()
                    moved = sign * (p - p0) / POS_PER_MM if p is not None else 0.0
                    if moved >= args.target_mm:
                        break
                    await send(f"O {args.base + sign * args.deg:.2f}")
                    await asyncio.sleep(on_s)
                    if off_s > 0.02:
                        await send(f"O {args.base:.2f}")
                        await asyncio.sleep(off_s)
                    pulses += 1
                t1 = time.time()
                p1 = rec.pos()

                # ---- 三级停车 ----
                await send(f"O {args.base:.2f}")          # ① 回水平，停止加速
                await asyncio.sleep(args.level_s)
                v_level = rec.vspeed()
                await send(f"O {args.base - sign * args.brake_deg:.2f}")   # ② 反向角刹车
                await asyncio.sleep(args.brake_ms / 1000.0)
                await send(f"O {args.base:.2f}")          # ③ 回水平保持
                await asyncio.sleep(args.settle_s)
                p2 = rec.pos()
                v_end = rec.vspeed()

                dist = sign * (p1 - p0) / POS_PER_MM
                over = sign * (p2 - p1) / POS_PER_MM
                dur = t1 - t0
                rows.append((duty, pulses, dist, dur, dist / dur if dur else 0.0,
                             over, v_level, v_end))
                print(f"   走 {dist:+7.1f}mm  用时 {dur:5.2f}s  "
                      f"平均 {dist/dur:+6.1f}mm/s  {pulses} 个通段")
                print(f"   刹车阶段再滑 {over:+6.1f}mm   停车瞬间速度 "
                      f"{v_level if v_level is not None else float('nan'):+5.1f} → "
                      f"{v_end if v_end is not None else float('nan'):+5.1f}mm/s")
                await asyncio.sleep(args.between_s)
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
    with open(args.out + ".duty.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["duty", "pulses", "dist_mm", "secs", "avg_mm_s", "brake_over_mm",
                    "v_level", "v_end"])
        for r in rows:
            w.writerow([r[0], r[1], f"{r[2]:.2f}", f"{r[3]:.2f}", f"{r[4]:.2f}",
                        f"{r[5]:.2f}",
                        "" if r[6] is None else f"{r[6]:.1f}",
                        "" if r[7] is None else f"{r[7]:.1f}"])

    if rows:
        print(f"\n=== {args.dir} 占空比 → 速度 ===")
        print("  duty   通段  位移mm   用时s   平均mm/s  刹车滑行mm")
        for r in rows:
            print(f"  {r[0]*100:5.0f}% {r[1]:5d} {r[2]:8.1f} {r[3]:7.2f} {r[4]:9.1f} {r[5]:10.1f}")
    print(f"[结果] {args.out}.g.csv / {args.out}.duty.csv")
    return code


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--base", type=float, default=4.00)
    ap.add_argument("--dir", choices=["fwd", "back"], default="fwd")
    ap.add_argument("--deg", type=float, default=1.30, help="破粘角（扫出来的）")
    ap.add_argument("--on-ms", type=float, default=1000.0, help="每个通段时长")
    ap.add_argument("--duty", default="1.0,0.5,0.25")
    ap.add_argument("--target-mm", type=float, default=100.0)
    ap.add_argument("--max-s", type=float, default=20.0)
    ap.add_argument("--level-s", type=float, default=0.5, help="停车①：回水平后等多久")
    ap.add_argument("--brake-deg", type=float, default=1.30, help="停车②：反向刹车角")
    ap.add_argument("--brake-ms", type=float, default=600.0)
    ap.add_argument("--settle-s", type=float, default=1.5)
    ap.add_argument("--between-s", type=float, default=2.0)
    ap.add_argument("--fall-deg", type=float, default=6.0)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
