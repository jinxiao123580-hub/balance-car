#!/usr/bin/env python3
"""前后成对试验：前进 → 停稳 → 后退 → 停稳，循环 N 次。

关键指标是**每循环的净漂移**：前进走了 X、后退回来 Y，净 = X + Y。
只有净漂移接近 0，才叫"能稳定前后"。

与 ble_otest 的区别：
  * ot​est 是"同方向连着走 N 次"，位置会累积出去，越走越偏；
  * 本脚本每循环一前一后，_after 每循环都回到同一个起点附近。

判据：负 Δ = 前进（pos 变负），正 Δ = 后退。
退出码：0 正常，2 连不上，3 超出行程保护，5 异常；数据一律写盘。
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


async def settle(rec, base, deg_tol=0.25, v_tol=5.0, hold=0.5, max_s=8.0):
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

    async def leg(deg, sign, tag, hold_ms=None):
        """走一腿，返回 (位移mm, 均速, 姿态偏离, 是否作废)"""
        hold_s = (hold_ms if hold_ms is not None else args.hold_ms) / 1000.0
        cur = rec.latest()
        if cur is None:
            return None
        p0 = cur[1]
        await send("^ 0,0")
        await asyncio.sleep(0.2)
        t0 = time.time()
        await send(f"O {args.base + sign * deg:.2f}")
        await asyncio.sleep(hold_s)
        wins = rec.window(t0, time.time())
        await send(f"O {args.base:.2f}")
        await settle(rec, args.base)
        cur = rec.latest()
        if cur is None or len(wins) < 2:
            print("    %s 跳过（窗口内帧数不足）" % tag)
            return None
        disp = sign * (cur[1] - p0)
        dev = max(abs(x[2] - args.base) for x in wins)
        dt = wins[-1][0] - wins[0][0]
        spd = sign * (wins[-1][1] - wins[0][1]) / dt if dt > 0 else 0.0
        bad = dev > args.fall_deg
        print("    %s  %+7.1fmm  均速%+7.1fmm/s  姿态偏离%.2f°%s"
              % (tag, disp, spd, dev, "  ← 作废(倾倒)" if bad else ""))
        return None if bad else (disp, spd, dev)

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
            origin = s[1] if s else 0.0
            print("起点: pos=%.1fmm pitch=%.2f° (O=%.2f)" % (s[1], s[2], args.base) if s else "起点: 无数据")
            if not await settle(rec, args.base, max_s=5.0):
                print("[警告] 起点未稳定，仍继续")

            for cyc in range(1, args.cycles + 1):
                print(f"  ── 第 {cyc} 循环 ──")
                f = await leg(args.fwd, -1, "前进", args.fwd_hold_ms)
                await asyncio.sleep(args.gap_s)
                b = await leg(args.back, +1, "后退", args.back_hold_ms)
                net = (f[0] if f else 0.0) + (b[0] if b else 0.0)
                cur = rec.latest()
                drift = (cur[1] - origin) if cur else 0.0
                rows.append((cyc, f[0] if f else None, b[0] if b else None,
                             net, drift, f, b))
                print("     → 净 %+6.1fmm   相对起点漂移 %+7.1fmm" % (net, drift))
                if abs(drift) > args.max_excursion:
                    print("[停止] 相对起点已偏 %.0fmm，超过 ±%.0fmm 保护，请把车挪回中间再跑"
                          % (drift, args.max_excursion))
                    code = 3
                    break
                await asyncio.sleep(args.gap_s)
            else:
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
    with open(args.out + ".cycles.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["cycle", "fwd_mm", "back_mm", "net_mm", "drift_mm"])
        for cyc, fv, bv, net, drift, _f, _b in rows:
            w.writerow([cyc,
                        "" if fv is None else f"{fv:.1f}",
                        "" if bv is None else f"{bv:.1f}",
                        f"{net:.1f}", f"{drift:.1f}"])

    if rows:
        fw = [r[1] for r in rows if r[1] is not None]
        bk = [r[2] for r in rows if r[2] is not None]
        nets = [r[3] for r in rows]
        print("\n=== 汇总（O=%.2f，%d 循环）===" % (args.base, len(rows)))
        print("  前进: 均值 %+7.1fmm  范围 %+7.1f ~ %+7.1f"
              % (statistics.mean(fw), min(fw), max(fw)) if fw else "  前进: 无有效腿")
        print("  后退: 均值 %+7.1fmm  范围 %+7.1f ~ %+7.1f"
              % (statistics.mean(bk), min(bk), max(bk)) if bk else "  后退: 无有效腿")
        print("  每循环净漂移: 均值 %+7.1fmm  范围 %+7.1f ~ %+7.1f"
              % (statistics.mean(nets), min(nets), max(nets)))
        print("  最终相对起点: %+7.1fmm" % rows[-1][4])
    print("[结果] %s.g.csv / %s.cycles.csv" % (args.out, args.out))
    return code


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--base", type=float, required=True)
    ap.add_argument("--fwd", type=float, default=1.5, help="前进 Δ（正数，内部取负）")
    ap.add_argument("--back", type=float, default=1.5, help="后退 Δ（正数）")
    ap.add_argument("--hold-ms", type=float, default=1500)
    ap.add_argument("--fwd-hold-ms", type=float, default=None, help="前进保持毫秒（默认同 --hold-ms）")
    ap.add_argument("--back-hold-ms", type=float, default=None, help="后退保持毫秒（默认同 --hold-ms）")
    ap.add_argument("--cycles", type=int, default=4)
    ap.add_argument("--gap-s", type=float, default=0.6)
    ap.add_argument("--fall-deg", type=float, default=6.0)
    ap.add_argument("--max-excursion", type=float, default=150.0)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
