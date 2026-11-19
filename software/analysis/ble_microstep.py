#!/usr/bin/env python3
"""后退定量：保持 Δ 直到位移到位就立刻收令（带提前量），再前进回原点。

为什么要"到位即收"而不是短脉冲：
  实测 Δ=1.3 是"先粘住、几秒后突然破粘"，破粘时刻 0.5~6s 随机。
  短脉冲（300ms）常常根本破不了粘；长保持又控制不住。
  正确做法是**长保持 + 逐帧判位 + 提前量截断**：破粘后最多再走一个遥测周期（250ms）。

判位：pos/100/10.8 = mm；后退为正、前进为负。
退出码：0 正常，2 连不上，4 翻车/越界中止（仍写盘）。
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

    def _last(self, idx):
        for _w, f in reversed(self.frames):
            if len(f) > idx:
                try:
                    return float(f[idx])
                except ValueError:
                    return None
        return None

    def pos(self):
        return self._last(I_POS)

    def pitch(self):
        v = self._last(I_PITCH)
        return None if v is None else v / 100.0   # 帧里是百分度


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
    aborted = False
    try:
        async with BleakClient(addr, timeout=20.0) as cli:
            await cli.start_notify(FFE1, rec.on_data)

            async def send(cmd):
                await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)

            await asyncio.sleep(0.8)
            await send("?")
            await asyncio.sleep(1.6)
            buf = "\n".join(l for _w, l in rec.text)
            m = re.search(r"\bO=([-0-9.]+)", buf)
            base = float(m.group(1)) if m else 2.30
            print(f"[原 O] {base}  后退 Δ={args.deg:+.2f}  目标 {args.target_mm}mm  提前量 {args.allow_mm}mm")

            async def drive_to(deg, goal_mm, direction, max_ms, tag):
                """保持 Δ 直到 (位移-原点) 沿 direction 达到 goal_mm，立刻收令。"""
                await send("^ 0,0")
                await asyncio.sleep(args.guard_ms / 1000.0)
                p0 = rec.pos()
                if p0 is None:
                    raise Abort("拿不到位置")
                await send(f"O {base + deg:.2f}")
                t0 = time.time()
                cut = False
                while time.time() - t0 < max_ms / 1000.0:
                    await asyncio.sleep(0.15)
                    pit = rec.pitch()
                    if pit is not None and abs(pit - base) > args.fall_deg:
                        await send(f"O {base:.2f}")
                        raise Abort(f"{tag} 翻车/失稳 pitch={pit:.1f}°")
                    p = rec.pos()
                    if p is None:
                        continue
                    moved = direction * (p - p0) / POS_PER_MM
                    if moved >= goal_mm - args.allow_mm:
                        cut = True
                        break
                    if abs(moved) > args.max_travel:
                        await send(f"O {base:.2f}")
                        raise Abort(f"{tag} 越界 {moved:.0f}mm")
                tcut = time.time()
                await send(f"O {base:.2f}")
                await asyncio.sleep(args.pause_ms / 1000.0)
                p1 = rec.pos()
                moved = direction * (p1 - p0) / POS_PER_MM
                return p0, p1, moved, tcut - t0, cut

            origin = None
            for rep in range(args.reps):
                p0, p1, back_mm, hold_s, cut = await drive_to(
                    args.deg, args.target_mm, +1, args.max_hold_ms, "后退")
                if origin is None:
                    origin = p0
                print(f"   rep{rep} 后退 {back_mm:+7.2f}mm  保持 {hold_s:5.2f}s  "
                      f"{'到位截断' if cut else '超时未达'}")
                await asyncio.sleep(args.settle_before_return_ms / 1000.0)
                pb0, pb1, ret_mm, ret_s, _ = await drive_to(
                    args.fwd_deg, abs(p1 - origin) / POS_PER_MM, -1, args.max_hold_ms, "回程")
                net = (pb1 - origin) / POS_PER_MM
                rows.append((rep, back_mm, hold_s, cut, ret_mm, ret_s, net))
                print(f"   rep{rep} 回程 {ret_mm:+7.2f}mm ({ret_s:4.2f}s)  距原点 {net:+6.2f}mm")
                origin = origin if abs(net) < 40 else p1
    except Abort as e:
        print(f"[中止] {e}  —— 数据仍写盘")
        aborted = True
    finally:
        print(f"[收尾] 确认 O 已回 {args.out}")

    with open(args.out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req", "rx"])
        for wall, f in rec.frames:
            if len(f) >= 8:
                w.writerow([f"{wall:.3f}"] + f[:8])
    with open(args.out + ".steps.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["rep", "back_mm", "hold_s", "cut", "ret_mm", "ret_s", "net_mm"])
        for r in rows:
            w.writerow([r[0], f"{r[1]:.2f}", f"{r[2]:.2f}", int(r[3]), f"{r[4]:.2f}",
                        f"{r[5]:.2f}", f"{r[6]:.2f}"])

    if rows:
        print("\n=== 后退汇总 ===")
        bs = [r[1] for r in rows]
        hs = [r[2] for r in rows]
        mu = sum(bs) / len(bs)
        sd = (sum((x - mu) ** 2 for x in bs) / len(bs)) ** 0.5
        print(f"后退位移: 均值 {mu:+7.2f}mm  sd {sd:5.2f}  n={len(bs)}  各次 {[round(x,1) for x in bs]}")
        print(f"保持时长: {[round(x,2) for x in hs]}s   到位截断 {sum(1 for r in rows if r[3])}/{len(rows)}")
    print(f"[结果] {args.out}.g.csv / {args.out}.steps.csv")
    return 4 if aborted else 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--deg", type=float, default=1.3, help="后退 Δ（相对 O）")
    ap.add_argument("--fwd-deg", type=float, default=-1.2, help="回程 Δ")
    ap.add_argument("--target-mm", type=float, default=50.0)
    ap.add_argument("--allow-mm", type=float, default=15.0, help="提前量：离目标还有这么多就收令")
    ap.add_argument("--max-hold-ms", type=float, default=12000.0)
    ap.add_argument("--guard-ms", type=float, default=700.0)
    ap.add_argument("--pause-ms", type=float, default=800.0)
    ap.add_argument("--settle-before-return-ms", type=float, default=3000.0,
                    help="后退结束后先等这么久再反向，避免高速换向把车掀翻")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--fall-deg", type=float, default=6.0)
    ap.add_argument("--max-travel", type=float, default=200.0)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
