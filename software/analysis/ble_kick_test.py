#!/usr/bin/env python3
"""起步脉冲（breakaway kick）验证：不烧固件，用 O 临时抬高再收回制造短促倾角。

原理：O 是平衡点设定值。把 O 抬高 0.6° 保持 250ms 再收回，
      等于让车"先猛地前倾一下"——正好用来验证能否破掉 0.5~1.0° 的静摩擦门槛。
      这是未来烧进固件的那段逻辑的宿主端原型。

两种模式：
  --gear 0     纯脉冲：车静止，只看每一次脉冲能把车推动多少（安全，推荐先跑）
  --gear 1.0   脉冲 + 速度环：测"踢一下就走、速度环接住"是否成立（有窜车风险，带位移保护）

用法：
  python ble_kick_test.py --address E1:.. --gear 0 --kicks 3 --kick 0.6 --kick-ms 250 --out ...\\k01
"""
import argparse
import asyncio
import csv
import os
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"


def num(v, d=None):
    try:
        return float(v)
    except (TypeError, ValueError):
        return d


async def main(args):
    out = args.out
    for p in (out + ".g.csv", out + ".kicks.csv"):
        if os.path.exists(p):
            raise SystemExit(f"输出已存在：{p}")

    addr = args.address
    if not addr:
        for d in await BleakScanner.discover(timeout=10.0):
            if d.name and ("JDY" in d.name or "BT" in d.name):
                addr = d.address
                break
    if not addr:
        raise SystemExit("扫描不到车（未上电？）")

    grows, par, buf = [], {"t": ""}, {"s": ""}

    async with BleakClient(addr, timeout=30.0) as cli:
        def on_data(_s, data):
            buf["s"] += data.decode("utf-8", "replace")
            *done, buf["s"] = buf["s"].replace("\r", "\n").split("\n")
            for line in done:
                line = line.strip()
                if line.startswith("PAR:"):
                    par["t"] = line
                elif line.startswith("G,"):
                    grows.append((time.time(), line[2:].split(",")))

        await cli.start_notify(FFE1, on_data)

        async def send(t):
            await cli.write_gatt_char(FFE1, (t + "\n").encode(), response=True)

        for _ in range(20):
            if par["t"]:
                break
            await send("?")
            await asyncio.sleep(0.3)
        o0 = 2.0
        for item in par["t"].split():
            if item.startswith("O="):
                o0 = num(item[2:], 2.0)
        print(f"[O0] {o0:.2f}  [gear] {args.gear:g}  [kick] {args.kick:+.2f}° {args.kick_ms:.0f}ms")

        def last_pos():
            for w, f in reversed(grows):
                if len(f) > 3:
                    return num(f[1], None)
            return None

        await send("^ 0,0")
        await asyncio.sleep(1.5)
        pnow = None
        for _ in range(10):
            s = [num(f[3], None) for w, f in grows if len(f) > 3]
            if s:
                pnow = s[-1] / 100
            if pnow is not None and abs(pnow - o0) < 0.8:
                break
            await asyncio.sleep(0.3)
        print(f"[就位] pitch={pnow}")
        if pnow is None or abs(pnow - o0) > 0.8:
            print("[中止] 起始不在 O 附近")
            return 5

        kicks = []
        p_start = last_pos()
        for i in range(args.kicks):
            sign = 1.0 if (not args.alternate or i % 2 == 0) else -1.0
            gear_v = args.gear * sign
            kick_v = args.kick * sign
            # 保持档位
            if gear_v:
                await send(f"^ 0,{gear_v:g}")
            await asyncio.sleep(args.gap)
            p0 = last_pos()
            t0 = time.time()
            await send(f"O {o0 + kick_v:.3f}")
            await asyncio.sleep(args.kick_ms / 1000.0)
            t1 = time.time()
            await send(f"O {o0:.3f}")
            await asyncio.sleep(args.after)
            p1 = last_pos()
            dmm = (p1 - p0) / 100.0 / args.counts_per_mm if (p0 is not None and p1 is not None) else 0.0
            kicks.append((i, i + 1, kick_v, t0, t1, dmm))
            print(f"[踢{i+1}] gear={gear_v:+.1f} kick={kick_v:+.2f}°  {dmm:+.1f} mm")
            if p_start is not None and p1 is not None:
                total = (p1 - p_start) / 100.0 / args.counts_per_mm
                if abs(total) > args.max_travel:
                    print(f"[中止] 累计位移 {total:+.0f}mm 超限")
                    break
        await send("^ 0,0")
        await send(f"O {o0:.3f}")
        await asyncio.sleep(0.6)

    with open(out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req"])
        for wall, f in grows:
            w.writerow([f"{wall:.3f}"] + f[:8])
    with open(out + ".kicks.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["idx", "idx1", "kick_deg", "t0", "t1", "disp_mm"])
        w.writerows(kicks)

    if kicks:
        vals = [k[4] for k in kicks]
        print(f"\n脉冲 {len(kicks)} 次：位移 {min(vals):+.1f} ~ {max(vals):+.1f} mm，"
              f"均值 {sum(vals)/len(vals):+.1f} mm")
    print(f"[结果] {out}.g.csv / {out}.kicks.csv")
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--gear", type=float, default=0.0, help="0=纯脉冲；1.0=脉冲+速度环")
    ap.add_argument("--kicks", type=int, default=3)
    ap.add_argument("--kick", type=float, default=0.6, help="脉冲倾角 °")
    ap.add_argument("--kick-ms", type=float, default=250)
    ap.add_argument("--gap", type=float, default=0.8, help="踢之前静置 s")
    ap.add_argument("--after", type=float, default=1.6, help="踢之后观察 s")
    ap.add_argument("--alternate", action="store_true", help="每脚交替正负号，成对去回")
    ap.add_argument("--max-travel", type=float, default=50.0, help="累计位移上限 mm")
    ap.add_argument("--counts-per-mm", type=float, default=10.8)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    sys.exit(asyncio.run(main(args)))
