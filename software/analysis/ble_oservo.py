#!/usr/bin/env python3
"""O 闭环行程：把 O 偏置当油门，按位置误差实时给 Δ，做到「走 N 毫米」。

为什么必须闭环：实测 Δ 定值是双稳态（粘住几秒后突然破粘），同一 Δ 同 12 秒
可以走 +20mm 也可以走 −22mm。开环拿不到可重复位移。
本脚本每秒数次读编码器，误差大给大 Δ（加速），误差小给小 Δ（减速），到位释放。
这同时就实现了加减速剖面，不需要另设速度环。

Δ 定义：相对 O 的差值。后退 = O+Δ，前进 = O−Δ（pos 为正=后退，实测确认）。

用法：
  python ble_oservo.py --dir back --mm 50 --out <前缀>
  python ble_oservo.py --dir fwd  --mm 50 --out <前缀>
退出码：0 到位，3 超时未达，4 翻车/越界中止（均写盘）。
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
I_POS, I_PITCH, I_HZ, I_SPEED = 1, 3, 5, 2


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

    def hz(self):
        return self._last(I_HZ)


async def run_dir(cli, rec, send, base, args, direction, target_mm, label):
    """direction=+1 后退 / -1 前进。返回 (到位mm, 用时, 轨迹)"""
    await send("^ 0,0")
    await asyncio.sleep(args.guard_ms / 1000.0)
    p0 = rec.pos()
    if p0 is None:
        raise Abort("拿不到起始位置")
    t0 = time.time()
    trace = []
    applied = 0.0
    settled = 0
    while time.time() - t0 < args.max_s / 1.0:
        p = rec.pos()
        pit = rec.pitch()
        if pit is not None and abs(pit - base) > args.fall_deg:
            await send(f"O {base:.2f}")
            raise Abort(f"{label} 翻车 pitch={pit:.1f}°")
        if p is None:
            await asyncio.sleep(0.2)
            continue
        moved = direction * (p - p0) / POS_PER_MM
        remaining = target_mm - moved
        trace.append((time.time() - t0, moved, applied, rec.hz()))
        if remaining <= args.deadband:
            # 已到位（或略超），连续两次确认后收令
            settled += 1
            if settled >= 2:
                break
        else:
            settled = 0
        if moved > args.max_travel:
            await send(f"O {base:.2f}")
            raise Abort(f"{label} 越界 {moved:.0f}mm")
        delta = max(args.dmin, min(args.dmax, args.kp * remaining))
        if abs(delta - applied) > args.delta_step or applied == 0.0:
            applied = delta
            await send("^ 0,0")
            await asyncio.sleep(args.guard_ms / 1000.0)
            await send(f"O {base + direction * applied:.2f}")
        await asyncio.sleep(args.poll_s)
    t_move = time.time() - t0
    await send(f"O {base:.2f}")
    await asyncio.sleep(args.settle_ms / 1000.0)
    p1 = rec.pos()
    final = direction * (p1 - p0) / POS_PER_MM
    return final, t_move, trace, applied


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
                    except Exception as e:
                        if attempt == 2:
                            raise Abort(f"BLE 写入失败：{type(e).__name__} {e}")
                        print(f"   [重试] 写入失败：{type(e).__name__}")
                        await asyncio.sleep(0.4)

            await asyncio.sleep(0.8)
            await send("?")
            await asyncio.sleep(1.6)
            buf = "\n".join(l for _w, l in rec.text)
            m = re.search(r"\bO=([-0-9.]+)", buf)
            base = float(m.group(1)) if m else 2.30
            print(f"[原 O] {base}  dir={args.dir} 目标 {args.mm}mm  kp={args.kp} Δ[{args.dmin},{args.dmax}]")

            for rep in range(args.reps):
                direction = +1 if args.dir == "back" else -1
                try:
                    final, t_move, trace, last_delta = await run_dir(
                        cli, rec, send, base, args, direction, args.mm, args.dir)
                except Abort as e:
                    print(f"[中止] {e} —— 数据仍写盘")
                    code = 4
                    break
                err = final - args.mm
                rows.append((rep, final, err, t_move, last_delta, len(trace)))
                print(f"   rep{rep} 走 {final:+7.2f}mm (目标 {args.mm}) 误差 {err:+6.2f}mm  "
                      f"用时 {t_move:5.2f}s  末段Δ={last_delta:.2f}°")
                if abs(err) > args.max_err:
                    print(f"   [警告] 误差 {err:.1f}mm 超 {args.max_err}mm")
                    code = 3
                await asyncio.sleep(args.between_s)
    except Abort as e:
        print(f"[中止] {e} —— 数据仍写盘")
        code = code or 4
    except Exception as e:
        print(f"[异常] {type(e).__name__}: {e} —— 数据仍写盘")
        code = 5
    finally:
        pass

    with open(args.out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req", "rx"])
        for wall, f in rec.frames:
            if len(f) >= 8:
                w.writerow([f"{wall:.3f}"] + f[:8])
    with open(args.out + ".runs.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["rep", "disp_mm", "err_mm", "secs", "last_delta", "samples"])
        for r in rows:
            w.writerow([r[0], f"{r[1]:.2f}", f"{r[2]:.2f}", f"{r[3]:.2f}",
                        f"{r[4]:.2f}", r[5]])

    if rows:
        ds = [r[1] for r in rows]
        mu = sum(ds) / len(ds)
        sd = (sum((x - mu) ** 2 for x in ds) / len(ds)) ** 0.5
        ts = [r[3] for r in rows]
        print(f"\n=== {args.dir} {args.mm}mm 汇总 ===")
        print(f"位移 {mu:+7.2f}mm  sd {sd:5.2f}  n={len(ds)}  各次 {[round(x,1) for x in ds]}")
        print(f"用时 {sum(ts)/len(ts):.2f}s 均  平均速度 {mu/(sum(ts)/len(ts)):+.2f}mm/s")
    print(f"[结果] {args.out}.g.csv / {args.out}.runs.csv")
    return code


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--dir", choices=["back", "fwd"], default="back")
    ap.add_argument("--mm", type=float, default=50.0, help="目标位移（正数）")
    ap.add_argument("--kp", type=float, default=0.020, help="deg per mm 误差")
    ap.add_argument("--dmin", type=float, default=0.60, help="最小 Δ（要能蠕爬）")
    ap.add_argument("--dmax", type=float, default=1.30, help="最大 Δ")
    ap.add_argument("--delta-step", type=float, default=0.08, help="Δ 变化超过此值才重发")
    ap.add_argument("--deadband", type=float, default=4.0)
    ap.add_argument("--poll-s", type=float, default=0.25)
    ap.add_argument("--guard-ms", type=float, default=500.0)
    ap.add_argument("--settle-ms", type=float, default=1500.0)
    ap.add_argument("--between-s", type=float, default=3.0)
    ap.add_argument("--max-s", type=float, default=45.0)
    ap.add_argument("--max-err", type=float, default=12.0)
    ap.add_argument("--max-travel", type=float, default=200.0)
    ap.add_argument("--fall-deg", type=float, default=6.0)
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
