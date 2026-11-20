#!/usr/bin/env python3
"""SP 脉冲前后前后：每腿给一次 SP，保持数秒，回 0，再给下一腿。

顺序（与固件互斥逻辑一致，全程只发一次 ^0,0）：
    ^ 0,0 → O <base> → SV <k> → [SP ±mag → 保持 → SP 0 → 间隔] × N

判据：负 SP = 前进（pos 变小），正 SP = 后退（pos 变大）。
输出：每腿位移、每循环净漂移、相对起点漂移；行程超限立即 SP 0 并停止。

用法：
  python ble_spcycle.py --address E1:.. --base 2.99 --sv -0.10 --mag 8 \
      --seg-s 3 --gap-s 2 --cycles 2 --max-travel 70
"""
import argparse
import asyncio
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
    rows = []
    async with BleakClient(addr, timeout=20.0) as cli:
        await cli.start_notify(FFE1, rec.on_data)

        async def send(cmd):
            await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)

        await asyncio.sleep(0.8)
        await send("^ 0,0"); await asyncio.sleep(0.5)
        await send(f"O {args.base:.2f}"); await asyncio.sleep(1.0)
        await send(f"SV {args.sv}"); await asyncio.sleep(0.2)
        s = rec.latest()
        if s is None:
            print("ERR 无遥测")
            return 2
        p0 = s[1]
        print("[开始] O=%.2f SV=%+.2f 起点 pos=%.1fmm pitch=%.2f°  行程保护 ±%.0fmm"
              % (args.base, args.sv, p0, s[2], args.max_travel))

        legs = [(+1, "后退"), (-1, "前进")] * args.cycles   # 后-前-后-前
        legs = [(-1, "前进"), (+1, "后退")] * args.cycles   # 前-后-前-后
        stop = False
        for i, (sign, name) in enumerate(legs, 1):
            if stop:
                break
            b = rec.latest()
            if b is None:
                continue
            pb = b[1]
            await send(f"SP {sign * args.mag}")
            t_leg = time.time()
            while time.time() - t_leg < args.seg_s:
                await asyncio.sleep(0.15)
                c = rec.latest()
                if c is None:
                    continue
                if abs(c[1] - p0) > args.max_travel:
                    print("  [保护] 相对起点 %.1fmm 超限，立即停车" % (c[1] - p0))
                    stop = True
                    break
                if abs(c[2] - args.base) > args.fall_deg:
                    print("  [保护] 姿态 %.1f° 超限，立即停车" % abs(c[2] - args.base))
                    stop = True
                    break
            await send("SP 0"); await asyncio.sleep(0.2)
            a = rec.latest()
            if a is None:
                continue
            disp = a[1] - pb
            drift = a[1] - p0
            rows.append((i, name, disp, drift))
            print("  腿%d %s  SP=%+g  位移=%+7.1fmm  相对起点=%+7.1fmm"
                  % (i, name, sign * args.mag, disp, drift))
            if not stop:
                await asyncio.sleep(args.gap_s)
        await send("SP 0"); await asyncio.sleep(0.2)
        await send("^ 0,0"); await asyncio.sleep(0.3)

    if rows:
        print("\n=== 汇总 ===")
        for i, name, disp, drift in rows:
            print("  腿%-2d %s  位移 %+7.1fmm   累计漂移 %+7.1fmm" % (i, name, disp, drift))
        fw = [d for i, n, d, _ in rows if n == "前进"]
        bk = [d for i, n, d, _ in rows if n == "后退"]
        if fw:
            print("  前进腿: %s  → 平均 %+.1fmm"
                  % (" / ".join("%+.1f" % x for x in fw), sum(-x for x in fw) / len(fw)))
        if bk:
            print("  后退腿: %s  → 平均 %+.1fmm"
                  % (" / ".join("%+.1f" % x for x in bk), sum(bk) / len(bk)))
        print("  最终相对起点: %+.1fmm" % rows[-1][3])
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--base", type=float, required=True)
    ap.add_argument("--sv", type=float, default=-0.10)
    ap.add_argument("--mag", type=float, default=8.0, help="SP 幅值")
    ap.add_argument("--seg-s", type=float, default=3.0)
    ap.add_argument("--gap-s", type=float, default=2.0)
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--max-travel", type=float, default=70.0)
    ap.add_argument("--fall-deg", type=float, default=6.0)
    sys.exit(asyncio.run(main(ap.parse_args())))
