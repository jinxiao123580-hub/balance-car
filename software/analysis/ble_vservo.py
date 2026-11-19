#!/usr/bin/env python3
"""速度环 + 距离环：Δ 当油门/刹车开关，Δ 本身做 slew 限幅（不阶跃）。

为什么这么设计（实测依据）：
- Δ=0 只是"不加速"，不是"刹车" ⇒ 降速必须给反向 Δ。所以 Δ 只能当开关用。
- Δ 阶跃大会翻车（O±2.2° 阶跃多次翻车）⇒ **目标 Δ 与实际 Δ 分离，每步只走 slew 限幅的量**。
- 门槛效应 ⇒ 加速用 Δacc、刹车用 Δbrk，都是固定档，不用线性增益。

控制律（每 poll 一次）：
    v_ref = min(vmax, sqrt(2*decel*剩余距离))     # 自然形成接近段的减速
    e     = v_ref − 实测速度(沿运动方向)
    e >  deadband → 目标Δ = +dir*Δacc
    e < −deadband → 目标Δ = −dir*Δbrk
    |e|<=deadband → 目标Δ = 0（滑行）
    实际Δ += clamp(目标Δ − 实际Δ, ±slew*poll)
    O = base + 实际Δ

用法：
  python ble_vservo.py --dir fwd --mm 100 --vmax 40 --decel 150 --dacc 1.2 --dbrk 1.3 --slew 1.2 --out <前缀>
退出码：0 到位，3 超时，4 翻车/越界（均写盘）。
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
SPEED_TO_MM_S = 20.0 / POS_PER_MM   # speed 字段 c/50ms → mm/s
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
            buf = "\n".join(l for _w, l in rec.text)
            m = re.search(r"\bO=([-0-9.]+)", buf)
            base = float(m.group(1)) if m else 4.00
            dirs = +1.0 if args.dir == "back" else -1.0
            print(f"[原 O] {base}  dir={args.dir} 目标 {args.mm}mm  vmax={args.vmax} "
                  f"Δacc={args.dacc} Δbrk={args.dbrk} slew={args.slew}°/s")

            for rep in range(args.reps):
                await send("^ 0,0")
                await asyncio.sleep(0.8)
                p0 = rec.pos()
                if p0 is None:
                    raise Abort("拿不到位置")
                applied = 0.0
                last_sent = None
                t0 = time.time()
                trace = []
                result = 3
                # 状态机 + 最小驻留：破粘要 0.5~6s，闪一下等于没给，所以不许被瞬态翻向
                state = "ACCEL"
                state_t = time.time()
                while time.time() - t0 < args.max_s:
                    p = rec.pos()
                    v = rec.vspeed()
                    pit = rec.pitch()
                    if pit is not None and abs(pit - base) > args.fall_deg:
                        await send(f"O {base:.2f}")
                        raise Abort(f"翻车 pitch={pit:.1f}°")
                    if p is None:
                        await asyncio.sleep(args.poll)
                        continue
                    moved = dirs * (p - p0) / POS_PER_MM
                    if moved > args.max_travel:
                        await send(f"O {base:.2f}")
                        raise Abort(f"越界 {moved:.0f}mm")
                    remaining = args.mm - moved
                    v_act = dirs * (v if v is not None else 0.0)
                    if remaining <= args.tol:
                        v_ref = 0.0
                    else:
                        v_ref = min(args.vmax, math.sqrt(max(0.0, 2.0 * args.decel * remaining)))
                    e = v_ref - v_act
                    now = time.time()
                    if now - state_t >= max(args.dwell, args.poll):
                        if e > args.db_v and state != "ACCEL":
                            state, state_t = "ACCEL", now
                        elif e < -args.db_v and state != "BRAKE":
                            state, state_t = "BRAKE", now
                        elif abs(e) <= args.db_v and state != "COAST":
                            state, state_t = "COAST", now
                    if state == "ACCEL":
                        tgt = dirs * args.dacc
                    elif state == "BRAKE":
                        tgt = -dirs * args.dbrk
                    else:
                        tgt = 0.0
                    step = args.slew * args.poll
                    if tgt > applied:
                        applied = min(tgt, applied + step)
                    else:
                        applied = max(tgt, applied - step)
                    if last_sent is None or abs(applied - last_sent) > 0.03:
                        await send(f"O {base + applied:.2f}")
                        last_sent = applied
                    trace.append((time.time() - t0, moved, v_act, v_ref, applied,
                                  (pit - base) if pit is not None else 0.0))
                    if remaining <= args.tol and abs(v_act) < args.v_stop:
                        result = 0
                        break
                    await asyncio.sleep(args.poll)
                # 收尾：Δ 斜坡回零
                while abs(applied) > 0.02:
                    applied -= math.copysign(min(abs(applied), args.slew * args.poll), applied)
                    await send(f"O {base + applied:.2f}")
                    await asyncio.sleep(args.poll)
                await send(f"O {base:.2f}")
                await asyncio.sleep(args.settle_ms / 1000.0)
                p1 = rec.pos()
                final = dirs * (p1 - p0) / POS_PER_MM if p1 is not None else float("nan")
                dur = time.time() - t0
                rows.append((rep, final, final - args.mm * dirs * dirs, dur, result, trace))
                print(f"   rep{rep} 走 {final:+7.2f}mm (目标 {args.mm})  用时 {dur:5.2f}s  "
                      f"{'到位' if result == 0 else '超时'}")
                if result != 0:
                    code = code or 3
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
    with open(args.out + ".trace.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["rep", "t", "moved_mm", "v_act", "v_ref", "delta", "pitch_err"])
        for rep, final, err, dur, result, trace in rows:
            for t, moved, va, vr, dl, pe in trace:
                w.writerow([rep, f"{t:.2f}", f"{moved:.2f}", f"{va:.2f}", f"{vr:.2f}",
                            f"{dl:.2f}", f"{pe:.2f}"])
    if rows:
        ds = [r[1] for r in rows]
        mu = sum(ds) / len(ds)
        sd = (sum((x - mu) ** 2 for x in ds) / len(ds)) ** 0.5
        ts = [r[3] for r in rows]
        print(f"\n=== {args.dir} {args.mm}mm ===")
        print(f"位移 {mu:+7.2f}mm  sd {sd:5.2f}  n={len(ds)}  各次 {[round(x,1) for x in ds]}")
        print(f"用时 {sum(ts)/len(ts):.2f}s 均")
        # pitch 摆幅
        pes = [max(abs(pe) for *_x, pe in r[5]) for r in rows if r[5]]
        if pes:
            print(f"pitch 最大偏离 O：{[round(x,2) for x in pes]}°")
    print(f"[结果] {args.out}.g.csv / {args.out}.trace.csv")
    return code


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--dir", choices=["back", "fwd"], default="fwd")
    ap.add_argument("--mm", type=float, default=100.0)
    ap.add_argument("--vmax", type=float, default=40.0, help="目标巡航速度 mm/s")
    ap.add_argument("--decel", type=float, default=150.0, help="接近段减速度 mm/s2")
    ap.add_argument("--dacc", type=float, default=1.2, help="加速档 Δ 幅度（正数）")
    ap.add_argument("--dbrk", type=float, default=1.3, help="刹车档 Δ 幅度（正数）")
    ap.add_argument("--slew", type=float, default=1.2, help="Δ 变化率上限 deg/s（防翻车）")
    ap.add_argument("--db-v", type=float, default=4.0, help="速度死区 mm/s")
    ap.add_argument("--dwell", type=float, default=1.5,
                    help="状态最小驻留秒数；破粘要 0.5~6s，短于此等于没给")
    ap.add_argument("--v-stop", type=float, default=5.0)
    ap.add_argument("--tol", type=float, default=5.0, help="到位容差 mm")
    ap.add_argument("--poll", type=float, default=0.2)
    ap.add_argument("--settle-ms", type=float, default=1500.0)
    ap.add_argument("--between-s", type=float, default=3.0)
    ap.add_argument("--max-s", type=float, default=40.0)
    ap.add_argument("--max-travel", type=float, default=200.0)
    ap.add_argument("--fall-deg", type=float, default=5.0)
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
