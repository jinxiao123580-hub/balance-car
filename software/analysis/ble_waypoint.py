#!/usr/bin/env python3
"""基于已学平衡点的位置闭环：走到指定位移点，再走回来。

三层串级（上位机 4Hz，直立环仍在固件）：
    位置误差 e(mm) → v_cmd(mm/s) → 倾角偏置 Δθ(°) → O = O_base + Δθ

符号约定（2026-11-19 实测）：pos 增大 = 后退；O 增大 = 后仰 = 后退。
所以位移目标为正(后退)时，Δθ 应为正。

安全：
  * Δθ 硬限幅（默认 ±2.0°），永远不给超出能站稳的角度
  * v_cmd 限幅（默认 ±40mm/s）
  * |pitch−O| 超过 --fall-deg 立即回 O_base 并中止
  * 单点超时自动放弃，继续下一个点

用法：
  python ble_waypoint.py --address E1:.. --o0 4.80 --targets "40,0,40,0" --seg-s 12
"""
import argparse
import asyncio
import csv
import os
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
POS_PER_MM = 1080.0
I_POS, I_PITCH = 1, 3


class Rec:
    def __init__(self):
        self.frames, self.buf = [], ""

    def on_data(self, _s, data):
        now = time.time()
        self.buf += data.decode("utf-8", "replace")
        *done, self.buf = self.buf.replace("\r", "\n").split("\n")
        for line in done:
            line = line.strip()
            if not line.startswith("G,"):
                continue
            f = line[2:].split(",")
            if len(f) > I_PITCH:
                try:
                    self.frames.append((now, float(f[I_POS]) / POS_PER_MM,
                                        float(f[I_PITCH]) / 100.0))
                except ValueError:
                    pass

    def latest(self):
        return self.frames[-1] if self.frames else None

    def speed(self, window_s):
        if len(self.frames) < 3:
            return None
        t_end = self.frames[-1][0]
        w = [f for f in self.frames if f[0] >= t_end - window_s]
        if len(w) < 3:
            return None
        n = len(w)
        mx = sum(f[0] for f in w) / n
        my = sum(f[1] for f in w) / n
        num = sum((f[0] - mx) * (f[1] - my) for f in w)
        den = sum((f[0] - mx) ** 2 for f in w)
        return num / den if den > 0 else None


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


async def main(args):
    addr = args.address
    if not addr:
        for dev in await BleakScanner.discover(timeout=10.0):
            if (dev.name or "").upper().startswith("JDY"):
                addr = dev.address
                break
    if not addr:
        print("ERR 没找到 JDY-23")
        return 2

    targets = [float(x) for x in args.targets.split(",")]
    rec = Rec()
    log = []
    d = os.path.dirname(args.out)
    if d:
        os.makedirs(d, exist_ok=True)

    async with BleakClient(addr, timeout=20.0) as cli:
        await cli.start_notify(FFE1, rec.on_data)

        async def send(cmd):
            await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)

        await asyncio.sleep(0.8)
        await send("^ 0,0")
        await asyncio.sleep(0.4)
        await send(f"O {args.o0:.2f}")
        await asyncio.sleep(1.0)
        cur = rec.latest()
        if cur is None:
            print("ERR 无遥测")
            return 2
        p_start = cur[1]
        print("[开始] O_base=%.2f  起点 pos=%.1fmm  pitch=%.2f°" % (args.o0, p_start, cur[2]))
        print("       目标序列 %s mm   Kp_pos=%.3f Kp_vel=%.3f  Δθ限幅=±%.1f°"
              % (args.targets, args.kp_pos, args.kp_vel, args.limit))

        for idx, tgt in enumerate(targets):
            print("  ── 第 %d 点：走到 %+.0fmm ──" % (idx + 1, tgt))
            t0 = time.time()
            last_report = 0.0
            while time.time() - t0 < args.seg_s:
                await asyncio.sleep(0.25)
                cur = rec.latest()
                if cur is None:
                    continue
                disp = cur[1] - p_start
                err = tgt - disp
                v = rec.speed(args.window)
                if v is None:
                    continue
                v_cmd = clamp(args.kp_pos * err, -args.v_max, args.v_max)
                dtheta = clamp(args.kp_vel * (v_cmd - v), -args.limit, args.limit)
                await send(f"O {args.o0 + dtheta:.2f}")
                log.append((time.time() - t0, idx + 1, disp, err, v, v_cmd, dtheta, cur[2]))
                if abs(cur[2] - (args.o0 + dtheta)) > args.fall_deg:
                    await send(f"O {args.o0:.2f}")
                    print("  [中止] 姿态偏离 %.1f° —— 已回 O_base" % abs(cur[2] - (args.o0 + dtheta)))
                    break
                if time.time() - t0 - last_report >= 3.0:
                    last_report = time.time() - t0
                    print("    +%4.1fs  位移=%+7.1fmm  误差=%+7.1f  v=%+6.1f  v_cmd=%+6.1f  Δθ=%+5.2f°"
                          % (last_report, disp, err, v, v_cmd, dtheta))
                if abs(err) < args.tol and abs(v) < args.v_tol:
                    print("    ✓ 到位：位移=%+.1fmm（目标 %+.0f）用时 %.1fs" % (disp, tgt, time.time() - t0))
                    break
            else:
                print("    ! 超时未到位，放弃这一点")
            await send(f"O {args.o0:.2f}")
            await asyncio.sleep(args.settle_s)

        cur = rec.latest()
        await send(f"O {args.o0:.2f}")
        await asyncio.sleep(0.3)

    with open(args.out + ".csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "seg", "disp_mm", "err_mm", "v_mm_s", "v_cmd", "dtheta", "pitch"])
        for r in log:
            w.writerow([f"{x:.3f}" if isinstance(x, float) else x for x in r])

    if log:
        last = [r for r in log if r[1] == len(targets)]
        print("\n=== 结果 ===")
        if last:
            print("  终点位移 = %+.1fmm（最后目标 %+.0f）" % (last[-1][2], targets[-1]))
        print("  总帧数 %d  → %s.csv" % (len(log), args.out))
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--o0", type=float, required=True, help="已学到的平衡点")
    ap.add_argument("--targets", default="40,0,40,0", help="相对起点的位移序列（正=后退），逗号分隔")
    ap.add_argument("--seg-s", type=float, default=12.0, help="每点最长耗时")
    ap.add_argument("--kp-pos", type=float, default=0.8, help="位置环增益 mm/s 每 mm")
    ap.add_argument("--kp-vel", type=float, default=0.03, help="速度环增益 °/(mm/s)")
    ap.add_argument("--v-max", type=float, default=40.0, help="期望速度限幅 mm/s")
    ap.add_argument("--limit", type=float, default=2.0, help="Δθ 硬限幅（°）")
    ap.add_argument("--window", type=float, default=1.5, help="测速窗口 秒")
    ap.add_argument("--tol", type=float, default=6.0, help="到位判据 mm")
    ap.add_argument("--v-tol", type=float, default=6.0, help="到位判据 mm/s")
    ap.add_argument("--settle-s", type=float, default=1.5)
    ap.add_argument("--fall-deg", type=float, default=8.0)
    ap.add_argument("--out", required=True)
    sys.exit(asyncio.run(main(ap.parse_args())))
