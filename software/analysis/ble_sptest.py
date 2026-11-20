#!/usr/bin/env python3
"""SP/SV 测试：定极性、标定"速度指令 → 实际速度"。

顺序很关键（固件里 SP 与 ^ 互斥，^ 会把 speedCmd 清零）：
    ^ 0,0 → O <base> → SV <k> → SP <v> → 测量 → SP 0 → ^ 0,0

输出：位移(mm)、平均速度、姿态 p2p；姿态超限立刻 SP 0 并报出。

用法：
  python ble_sptest.py --address E1:.. --base 2.99 --sv -0.12 --sp 2 --secs 6
"""
import argparse
import asyncio
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
    async with BleakClient(addr, timeout=20.0) as cli:
        await cli.start_notify(FFE1, rec.on_data)

        async def send(cmd):
            await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)

        # 严格顺序：先归零解除门控，再设平衡点，最后才给速度指令
        await asyncio.sleep(0.8)
        await send("^ 0,0");  await asyncio.sleep(0.4)
        await send(f"O {args.base:.2f}"); await asyncio.sleep(0.8)
        await send(f"SV {args.sv}"); await asyncio.sleep(0.2)
        if args.ki is not None:
            await send(f"I {args.ki}"); await asyncio.sleep(0.2)
        await send(f"SP {args.sp}"); await asyncio.sleep(0.3)

        t0 = time.time()
        rec.frames = []
        aborted = False
        reason = ""
        await asyncio.sleep(0.3)
        _c0 = rec.latest()
        p_start = _c0[1] if _c0 else 0.0
        while time.time() - t0 < args.secs:
            await asyncio.sleep(0.2)
            cur = rec.latest()
            if cur is None:
                continue
            # 行程保护：偏离起点超过 max_travel 立刻停车，避免撞到边界
            if abs(cur[1] - p_start) > args.max_travel:
                aborted = True
                reason = "行程超限"
                await send("SP 0")
                await asyncio.sleep(0.2)
                break
            if abs(cur[2] - args.base) > args.fall_deg:
                aborted = True
                reason = "姿态超限"
                break
        await send("SP 0"); await asyncio.sleep(0.2)
        await send("^ 0,0"); await asyncio.sleep(0.3)

    if len(rec.frames) < 3:
        print("SP=%+g SV=%+g: 帧数不足(%d)" % (args.sp, args.sv, len(rec.frames)))
        return 5
    ps = [f[1] for f in rec.frames]
    pits = [f[2] for f in rec.frames]
    dur = rec.frames[-1][0] - rec.frames[0][0]
    disp = ps[-1] - ps[0]
    v = disp / dur if dur > 0 else 0.0

    # 稳态速度：只取后半段做线性回归（前段还在破粘，会拉低均值）
    half = rec.frames[len(rec.frames) // 2:]
    if len(half) >= 3:
        n = len(half)
        mx = sum(f[0] for f in half) / n
        my = sum(f[1] for f in half) / n
        num = sum((f[0] - mx) * (f[1] - my) for f in half)
        den = sum((f[0] - mx) ** 2 for f in half)
        v_ss = num / den if den > 0 else 0.0
    else:
        v_ss = 0.0

    print("SP=%+5.2f SV=%+6.3f | 位移=%+8.1fmm  全程均速=%+7.2f  稳态=%+7.2fmm/s  "
          "时长=%.1fs  姿态p2p=%.2f°%s"
          % (args.sp, args.sv, disp, v, v_ss, dur, max(pits) - min(pits),
             "   ← " + reason + " 已中止" if aborted else ""))
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--base", type=float, required=True)
    ap.add_argument("--sv", type=float, default=-0.12)
    ap.add_argument("--sp", type=float, default=2.0)
    ap.add_argument("--ki", type=float, default=None, help="speedKi（参数 I），不给则不改")
    ap.add_argument("--secs", type=float, default=6.0)
    ap.add_argument("--max-travel", type=float, default=60.0, help="行程保护：偏离起点超过此值即停车（mm）")
    ap.add_argument("--fall-deg", type=float, default=6.0)
    sys.exit(asyncio.run(main(ap.parse_args())))
