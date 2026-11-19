#!/usr/bin/env python3
"""前后往复演示：以出发点为中心，用位置 bang-bang 做「前进一点、回来、后退一点、回来」。

安全性来自「有界」：一旦偏离中心超过 --amp-mm 就朝反方向推，所以永远跑不掉。
Δ 用 slew 斜坡给（不阶跃，阶跃会翻车），|pitch−O| 超限立刻归零。

判据：前进 = pos 变负 = 负 Δ；后退 = pos 变正 = 正 Δ。
退出码：0 正常结束，4 翻车，5 异常（均写盘）。
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
I_POS, I_PITCH = 1, 3
I_SPEED = 2
SPEED_TO_MM_S = 20.0 / POS_PER_MM


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
        """G 帧 speed 字段（c/50ms）→ mm/s"""
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
    code = 0
    log = []
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
            buf = "\n".join(l for _w, l in rec.text)
            m = re.search(r"\bO=([-0-9.]+)", buf)
            print(f"[开机 O] {m.group(1) if m else '?'}  → 按用户要求写死 {args.base:.2f}，不重估")
            base = args.base
            await send("^ 0,0")
            await asyncio.sleep(0.8)
            await send(f"O {base:.2f}")
            await asyncio.sleep(0.6)
            print(f"[原 O] {base}  幅度 ±{args.amp_mm}mm  Δacc={args.dacc} Δbrk={args.dbrk} "
                  f"slew={args.slew}°/s  时长 {args.seconds}s")

            await send("^ 0,0")
            await asyncio.sleep(0.8)
            p0 = rec.pos()
            if p0 is None:
                raise Abort("拿不到位置")
            applied = 0.0
            last_sent = None
            t0 = time.time()
            swings = 0
            last_side = 0
            state = ["FWD"]
            hold_left = 0
            pulse_t = [time.time()]
            pulse_dir = [-1]
            stable_since = [0.0]
            gate_cut = [False]
            while time.time() - t0 < args.seconds:
                p = rec.pos()
                pit = rec.pitch()
                if pit is not None and abs(pit - base) > args.fall_deg:
                    await send(f"O {base:.2f}")      # 必须回基准 O，不能留在 base+applied
                    applied = 0.0
                    raise Abort(f"翻车 pitch={pit:.1f}°")
                if p is None:
                    await asyncio.sleep(args.poll)
                    continue
                offset = (p - p0) / POS_PER_MM      # 负 = 偏前
                # 不连续点动：瞬间给 Δ、保持 pulse_s，然后回零并**等真正稳住**再发下一脚。
                # 稳定判据：|pitch−O| < settle_deg 且 |速度| < settle_v，并持续 settle_hold 秒。
                el = time.time() - pulse_t[0]
                if el < args.pulse_s and not gate_cut[0]:
                    tgt = pulse_dir[0] * args.dacc
                    stable_since[0] = 0.0
                    # 滚动门控：过了宽限期轮子还没转起来，立刻撤倾角，
                    # 否则倾角发不出去、只会把车身越顶越歪（take_F 的翻车原因）
                    if el * 1000.0 >= args.gate_ms:
                        v_g = rec.vspeed()
                        if v_g is not None and abs(v_g) < args.gate_v:
                            gate_cut[0] = True
                            print(f"      [门控] 轮子没转（{v_g:+.1f}mm/s），撤倾角")
                else:
                    tgt = 0.0
                    v_now = rec.vspeed()
                    ok = (pit is not None and abs(pit - base) < args.settle_deg
                          and v_now is not None and abs(v_now) < args.settle_v)
                    if ok:
                        if stable_since[0] == 0.0:
                            stable_since[0] = time.time()
                    else:
                        stable_since[0] = 0.0
                    ready = (stable_since[0] > 0.0
                             and time.time() - stable_since[0] >= args.settle_hold)
                    if ready or el > args.pulse_s + args.settle_max_s:
                        pulse_t[0] = time.time()
                        gate_cut[0] = False
                        if args.alternate:
                            pulse_dir[0] = -1 if (swings + 1) % 2 == 1 else +1   # 严格 前-后-前-后
                        else:
                            pulse_dir[0] = -1 if offset > -args.target_mm else +1
                        tgt = pulse_dir[0] * args.dacc
                        stable_since[0] = 0.0
                        swings += 1
                        log.append((time.time() - t0, offset, pulse_dir[0]))
                        print(f"   +{time.time()-t0:5.1f}s 点动#{swings} "
                              f"{'前进' if pulse_dir[0] < 0 else '后退'}  偏移 {offset:+7.1f}mm"
                              f"  {'已稳' if ready else '超时未稳'}")
                        if args.max_jogs and swings >= args.max_jogs:
                            break
                # 越界保护
                if abs(offset) > args.target_mm + 30.0:
                    tgt = (1 if offset > 0 else -1) * args.dacc
                step = args.slew * args.poll
                if tgt > applied:
                    applied = min(tgt, applied + step)
                elif tgt < applied:
                    applied = max(tgt, applied - step)
                if last_sent is None or abs(applied - last_sent) > 0.03:
                    await send(f"O {base + applied:.2f}")
                    last_sent = applied
                await asyncio.sleep(args.poll)

            # 收尾：Δ 斜坡回零
            while abs(applied) > 0.02:
                applied -= math.copysign(min(abs(applied), args.slew * args.poll), applied)
                await send(f"O {base + applied:.2f}")
                await asyncio.sleep(args.poll)
            await send(f"O {base:.2f}")
            await asyncio.sleep(1.5)
            p1 = rec.pos()
            total = (p1 - p0) / POS_PER_MM if p1 is not None else float("nan")
            print(f"[完成] 摆动 {swings} 次  净漂移 {total:+6.2f}mm  用时 {time.time()-t0:.1f}s")
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
    with open(args.out + ".swings.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "offset_mm", "direction"])
        for r in log:
            w.writerow([f"{r[0]:.2f}", f"{r[1]:.2f}", "back" if r[2] > 0 else "fwd"])
    print(f"[结果] {args.out}.g.csv / {args.out}.swings.csv")
    return code


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--base", type=float, default=4.00, help="写死的平衡角基准（不做重估）")
    ap.add_argument("--pulse-s", type=float, default=0.6, help="点动：Δ 保持秒数")
    ap.add_argument("--rest-s", type=float, default=1.2, help="（兼容保留，已由稳定判据取代）")
    ap.add_argument("--settle-deg", type=float, default=0.6, help="稳定判据：|pitch−O| 小于此值")
    ap.add_argument("--settle-v", type=float, default=2.0, help="稳定判据：|速度| 小于此值 mm/s")
    ap.add_argument("--settle-hold", type=float, default=0.5, help="稳定需持续的秒数")
    ap.add_argument("--settle-max-s", type=float, default=6.0, help="等稳定的最长时间")
    ap.add_argument("--max-jogs", type=int, default=4, help="总共做几脚就停（0=不限）")
    ap.add_argument("--alternate", action="store_true", help="严格交替：前-后-前-后，不看偏移")
    ap.add_argument("--gate-ms", type=float, default=250.0, help="滚动门控宽限毫秒")
    ap.add_argument("--gate-v", type=float, default=3.0, help="门控判据：|速度| 小于此值视为没转 mm/s")
    ap.add_argument("--target-mm", type=float, default=20.0, help="往复半幅（按编码器判）")
    ap.add_argument("--amp-mm", type=float, default=35.0, help="（保留）")
    ap.add_argument("--margin-mm", type=float, default=12.0, help="距目标还有这么多就停推滑行")
    ap.add_argument("--settle-polls", type=int, default=6, help="到点后滑行的 poll 数")
    ap.add_argument("--dacc", type=float, default=1.2, help="推动档 Δ 幅度")
    ap.add_argument("--dbrk", type=float, default=1.2, help="（保留）")
    ap.add_argument("--slew", type=float, default=1.0, help="Δ 变化率 deg/s")
    ap.add_argument("--poll", type=float, default=0.25)
    ap.add_argument("--seconds", type=float, default=40.0)
    ap.add_argument("--fall-deg", type=float, default=5.0)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
