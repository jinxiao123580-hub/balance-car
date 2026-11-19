#!/usr/bin/env python3
"""O 偏置单枪试验（BLE 版）：每档 × 每方向 N 次，保持期内连续判倒。

与旧脚本的区别：旧脚本只在两次试验之间查 pitch，一枪之内倒了也照算；
这里在保持期内持续采样，任何一帧 |pitch−O| > fall-deg 即判该次作废。

速度一律用 pos 差分算，不依赖 G 帧的 speed 字段（该字段标度不直观）。
判据：负 Δ = 前进（pos 变负），正 Δ = 后退。
退出码：0 正常，2 连不上，5 异常；数据一律写盘。
"""
import argparse
import asyncio
import csv
import os
import statistics
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
POS_PER_MM = 1080.0
I_POS, I_PITCH = 1, 3


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
                f = line[2:].split(",")
                if len(f) > I_PITCH:
                    try:
                        self.frames.append((now, float(f[I_POS]) / POS_PER_MM,
                                            float(f[I_PITCH]) / 100.0))
                    except ValueError:
                        pass
            else:
                self.text.append((now, line))

    def latest(self):
        return self.frames[-1] if self.frames else None

    def window(self, t0, t1):
        return [(w, p, pit) for (w, p, pit) in self.frames if t0 <= w <= t1]

    def rate(self, t_from, t_to):
        w = self.window(t_from, t_to)
        if len(w) < 2:
            return None
        dt = w[-1][0] - w[0][0]
        return (w[-1][1] - w[0][1]) / dt if dt > 0 else None


async def settle(rec, base, deg_tol=0.8, v_tol=8.0, hold=0.4, max_s=4.0):
    """等稳定：|pitch−O| 小且位移速率小，持续 hold 秒"""
    t0 = time.time()
    ok_since = None
    while time.time() - t0 < max_s:
        s = rec.latest()
        if s is not None:
            v = rec.rate(time.time() - 0.4, time.time())
            ok = abs(s[2] - base) < deg_tol and (v is None or abs(v) < v_tol)
            if ok:
                if ok_since is None:
                    ok_since = time.time()
                elif time.time() - ok_since >= hold:
                    return True
            else:
                ok_since = None
        await asyncio.sleep(0.1)
    return False


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
    code = 5

    async def send(cmd):
        await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)

    try:
        async with BleakClient(addr, timeout=20.0) as cli:
            await cli.start_notify(FFE1, rec.on_data)
            await asyncio.sleep(0.8)
            await send("?")
            await asyncio.sleep(1.2)
            await send("^ 0,0")
            await asyncio.sleep(0.3)
            await send(f"O {args.base:.2f}")
            await asyncio.sleep(0.6)
            s = rec.latest()
            print("起始: pos=%.1fmm pitch=%.2f° (O=%.2f)"
                  % (s[1], s[2], args.base) if s else "起始: 无数据")
            if not await settle(rec, args.base, max_s=5.0):
                print("[警告] 起始未稳定，仍继续")

            for deg in [float(x) for x in args.degs.split(",")]:
                for name, sign in (("前进", -1), ("后退", +1)):
                    if args.dir == "fwd" and sign > 0:
                        continue
                    if args.dir == "back" and sign < 0:
                        continue
                    disps, spds, devs = [], [], []
                    valid = broke = 0
                    for rep in range(args.reps):
                        cur = rec.latest()
                        if cur is None:
                            continue
                        p0 = cur[1]
                        await send("^ 0,0")
                        await asyncio.sleep(0.2)
                        t0 = time.time()
                        await send(f"O {args.base + sign * deg:.2f}")
                        await asyncio.sleep(args.hold_ms / 1000.0)
                        wins = rec.window(t0, time.time())
                        await send(f"O {args.base:.2f}")
                        await settle(rec, args.base)
                        cur = rec.latest()
                        if cur is None or len(wins) < 2:
                            print("    %s d=%.2f #%d 跳过（窗口内帧数不足）" % (name, deg, rep + 1))
                            continue
                        disp = sign * (cur[1] - p0)
                        dev = max(abs(x[2] - args.base) for x in wins)
                        dt = wins[-1][0] - wins[0][0]
                        spd = sign * (wins[-1][1] - wins[0][1]) / dt if dt > 0 else 0.0
                        if dev > args.fall_deg or abs(disp) > args.max_travel:
                            print("    %s d=%.2f #%d 作废（倾倒 dev=%.1f° / 超程 %.0fmm）"
                                  % (name, deg, rep + 1, dev, disp))
                            continue
                        valid += 1
                        if disp >= args.break_mm:
                            broke += 1
                        disps.append(disp); spds.append(spd); devs.append(dev)
                        print("    %s d=%.2f #%d  disp=%+7.1fmm  均速=%+7.1fmm/s  姿态偏离=%.2f°"
                              % (name, deg, rep + 1, disp, spd, dev))
                    rows.append((deg, name, valid, broke, disps, spds, devs))
            code = 0
    except Exception as e:
        print(f"[异常] {type(e).__name__}: {e} —— 数据仍写盘")
    finally:
        try:
            await send(f"O {args.base:.2f}")
            await asyncio.sleep(0.3)
        except Exception:
            pass

    with open(args.out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "pos_mm", "pitch_deg"])
        for wall, p, pit in rec.frames:
            w.writerow([f"{wall:.3f}", f"{p:.2f}", f"{pit:.2f}"])
    with open(args.out + ".grid.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["deg", "dir", "valid", "broke", "median_disp", "mean_speed", "max_dev"])
        for deg, name, valid, broke, disps, spds, devs in rows:
            w.writerow([deg, name, valid, broke,
                        f"{statistics.median(disps):.1f}" if disps else "",
                        f"{statistics.mean(spds):.1f}" if spds else "",
                        f"{max(devs):.2f}" if devs else ""])

    print("\n=== 汇总（O=%.2f）===" % args.base)
    print("  Δ     方向  有效/总  破粘   位移中位mm   均速mm/s  姿态偏离max")
    for deg, name, valid, broke, disps, spds, devs in rows:
        if not disps:
            print("  %.2f  %s   %d/%d    --       --          --        --"
                  % (deg, name, valid, args.reps))
            continue
        print("  %.2f  %s   %d/%d   %d/%d  %+8.1f  %+8.1f  %.2f"
              % (deg, name, valid, args.reps, broke, valid,
                 statistics.median(disps), statistics.mean(spds), max(devs)))
    print("[结果] %s.g.csv / %s.grid.csv" % (args.out, args.out))
    return code


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--base", type=float, required=True, help="当前平衡点 O")
    ap.add_argument("--degs", default="1.0,1.5,2.0")
    ap.add_argument("--dir", choices=["both", "fwd", "back"], default="both")
    ap.add_argument("--reps", type=int, default=3)
    ap.add_argument("--hold-ms", type=float, default=1500)
    ap.add_argument("--fall-deg", type=float, default=6.0)
    ap.add_argument("--max-travel", type=float, default=250.0)
    ap.add_argument("--break-mm", type=float, default=10.0)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
