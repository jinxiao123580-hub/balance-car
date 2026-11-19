#!/usr/bin/env python3
"""漂移测量：设置若干参数 → 静置 N 秒 → 用线性回归算漂移速度 + 姿态摆动。

用来评估速度环积分 I（speedKi）和死区 J（SPEED_ERROR_DEADBAND）的效果。

用法：
  python ble_drift.py --address E1:.. --cmd "I 0.05" --cmd "J 1.0" --secs 20 --label I0.05_J1.0
  python ble_drift.py --address E1:.. --secs 20 --label baseline        # 不改参数，只测
"""
import argparse
import asyncio
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
        self.frames, self.buf, self.text = [], "", []

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


def slope(xs, ys):
    n = len(xs)
    if n < 2:
        return 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    den = sum((x - mx) ** 2 for x in xs)
    return num / den if den > 0 else 0.0


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

    rec = Rec()
    async with BleakClient(addr, timeout=20.0) as cli:
        await cli.start_notify(FFE1, rec.on_data)

        async def send(cmd):
            await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)

        await asyncio.sleep(0.8)
        for c in args.cmd:
            await send(c)
            await asyncio.sleep(0.35)
        await send("^ 0,0")
        await asyncio.sleep(0.4)
        if args.base is not None:
            await send(f"O {args.base:.2f}")
            await asyncio.sleep(0.5)
        await send("?")
        await asyncio.sleep(1.5)

        # 丢弃旧帧，只统计设定之后
        rec.frames = []
        t0 = time.time()
        while time.time() - t0 < args.secs:
            await asyncio.sleep(0.2)
        await send(f"O {args.base:.2f}" if args.base is not None else "^ 0,0")

    if len(rec.frames) < 5:
        print(f"{args.label}: 帧数不足({len(rec.frames)}) —— 可能没连上或没遥测")
        return 5

    ts = [f[0] - rec.frames[0][0] for f in rec.frames]
    ps = [f[1] for f in rec.frames]
    pi = [f[2] for f in rec.frames]
    v = slope(ts, ps)                       # mm/s
    drift_total = ps[-1] - ps[0]
    pit_std = statistics.pstdev(pi)
    pit_p2p = max(pi) - min(pi)
    fell = (max(pi) - min(pi)) > 12.0
    print("%-18s 帧=%3d  漂移=%+7.2fmm/s  总位移=%+7.1fmm  姿态std=%.3f°  p2p=%.2f°%s"
          % (args.label, len(rec.frames), v, drift_total, pit_std, pit_p2p,
             "  ← 疑似倾倒" if fell else ""))
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--cmd", action="append", default=[], help="设定命令，可重复")
    ap.add_argument("--base", type=float, default=None, help="起始先设 O（可选）")
    ap.add_argument("--secs", type=float, default=20.0)
    ap.add_argument("--label", default="run")
    sys.exit(asyncio.run(main(ap.parse_args())))
