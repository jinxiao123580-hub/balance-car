#!/usr/bin/env python3
"""平衡点 O 的闭环标定：用下位机 50mm AprilTag 测静态漂移，按方向 ±step 收敛 O。

判据（来自 2026-11-10 交接）：
  - 前进 = 画面 −x → 漂移斜率为负 = 车在往前跑 = O 偏高 → 减 O
  - 目标 |drift| < 0.5 mm/s；每轮步长 0.05°；O 相对起点变动超过 0.9°（倾角权限）即停手

用法（software/ble-panel 的 venv）：
  .venv\\Scripts\\python.exe ..\\analysis\\calibrate_balance_point.py --o 2.00
  ... --o 2.00 --probe 8 --secs 30 --step 0.05 --target 0.5 --max-iter 8

安全：先跑 --probe 秒短测；|漂移|>15 mm/s 或净位移>60 mm 视为危险，不进 30s 长测。
产出：docs/validation/analysis/<YYYYMMDD>-balance-point-calib/calib_log.csv
退出码：0 收敛并 SAVE；2 连不上蓝牙；3 相机/标签不可用；4 危险中止；5 未收敛（已回写最佳 O）
"""
import argparse
import asyncio
import csv
import datetime as dt
import os
import re
import subprocess
import sys

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
SSH = ["ssh", "-i", os.path.expanduser(r"~\.ssh\linux_slave"),
       "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", "jx@100.72.186.31"]
REMOTE_TRK = "python3 /tmp/tags/tagtrk.py {} {}"


def measure(secs, remote_csv):
    """跑一次 tagtrk，返回 dict 或 None。"""
    cmd = SSH + [REMOTE_TRK.format(int(secs), remote_csv)]
    p = subprocess.run(cmd, capture_output=True, text=True, timeout=secs + 40)
    out = p.stdout + p.stderr
    d = re.search(r"drift slope\s+[+-][\d.]+ px/s = ([+-][\d.]+) mm/s", out)
    n = re.search(r"net\s+[+-][\d.]+ px = ([+-][\d.]+) mm", out)
    m = re.search(r"-> ([\d.]+) mm/px", out)
    f = re.search(r"frames=(\d+) detected=(\d+)", out)
    s = re.search(r"span\s+[\d.]+ px = ([\d.]+) mm", out)
    if p.returncode != 0 or not (d and n and m):
        print(out.strip()[:600])
        return None
    return {"drift": float(d.group(1)), "net": float(n.group(1)), "mpp": float(m.group(1)),
            "span": float(s.group(1)) if s else 0.0,
            "frames": int(f.group(1)) if f else 0, "detected": int(f.group(2)) if f else 0}


async def main(args):
    day = dt.datetime.now().strftime("%Y%m%d")
    outdir = os.path.join(args.outroot, f"{day}-balance-point-calib")
    os.makedirs(outdir, exist_ok=True)
    log_path = os.path.join(outdir, "calib_log.csv")
    remote = "/tmp/tags/cal_{}.csv"

    addr = args.address
    if not addr:
        for d in await BleakScanner.discover(timeout=10.0):
            if d.name and ("JDY" in d.name or "BT" in d.name):
                addr = d.address
                break
    if not addr:
        print("扫描不到车（确认上电、面板已关）", file=sys.stderr)
        return 2

    o = args.o
    rows, best = [], None
    async with BleakClient(addr, timeout=30.0) as cli:
        buf = {"s": ""}

        def on_data(_s, data):
            buf["s"] += data.decode("utf-8", "replace")

        await cli.start_notify(FFE1, on_data)

        async def send(text, wait=0.35):
            buf["s"] = ""
            await cli.write_gatt_char(FFE1, (text + "\n").encode(), response=True)
            await asyncio.sleep(wait)
            return " | ".join(l.strip() for l in buf["s"].replace("\r", "\n").split("\n") if l.strip())

        print("[车]", (await send("?", 0.6))[:300])

        for i in range(1, args.max_iter + 1):
            await send(f"O {o:.3f}", 0.5)
            await asyncio.sleep(2.0)          # 改参数后让车静下来再量
            m = measure(args.probe, remote.format(f"{i}p"))
            if not m:
                return 3
            print(f"[探测{i}] O={o:.3f} drift={m['drift']:+.2f} mm/s net={m['net']:+.1f} mm "
                  f"span={m['span']:.1f} mm detect={m['detected']}/{m['frames']}")
            if m["frames"] and m["detected"] < m["frames"] * 0.5:
                print("标签检出率 <50%，先解决视野/遮挡", file=sys.stderr)
                return 3
            if abs(m["drift"]) > 15.0 or abs(m["net"]) > 60.0 or m["span"] > 80.0:
                print(f"危险：漂移/游走过大（{m['drift']:+.1f} mm/s, span {m['span']:.0f} mm），中止",
                      file=sys.stderr)
                return 4

            m2 = measure(args.secs, remote.format(i))
            if not m2:
                return 3
            print(f"[长测{i}] O={o:.3f} drift={m2['drift']:+.2f} mm/s net={m2['net']:+.1f} mm "
                  f"span={m2['span']:.1f} mm ({m2['mpp']:.3f} mm/px)")
            rows.append((i, o, m2["drift"], m2["net"], "long"))
            if best is None or abs(m2["drift"]) < abs(best[1]):
                best = (o, m2["drift"])
            if abs(m2["drift"]) <= args.target:
                break
            if abs(m2["net"]) > 80.0:
                print(f"危险：30s 净位移 {m2['net']:+.0f} mm，快掉桌了，中止", file=sys.stderr)
                return 4

            o = round(o + (args.step if m2["drift"] > 0 else -args.step), 3)
            if abs(o - args.o) > 0.9:
                print(f"O 已动 {abs(o - args.o):.2f}° > 0.9°，停手检查机械", file=sys.stderr)
                o = round(args.o + (0.9 if o > args.o else -0.9), 3)
                break

        if best and abs(best[0] - o) > 1e-9:
            o = best[0]
            await send(f"O {o:.3f}", 0.5)
        print(f"[收敛] O={o:.3f} drift={best[1]:+.2f} mm/s" if best else "[未收敛]")
        print("[SAVE]", (await send("SAVE", 1.2))[:200])
        print("[回读]", (await send("?", 0.6))[:300])

    with open(log_path, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["iter", "O", "drift_mm_s", "net_mm", "phase"])
        w.writerows(rows)
        w.writerow(["final", o, best[1] if best else "", "", "saved"])
    print(f"[日志] {log_path}")
    return 0 if best and abs(best[1]) <= args.target else 5


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--o", type=float, required=True, help="当前 O")
    ap.add_argument("--address", default="")
    ap.add_argument("--step", type=float, default=0.05)
    ap.add_argument("--target", type=float, default=0.5)
    ap.add_argument("--probe", type=float, default=8.0)
    ap.add_argument("--secs", type=float, default=30.0)
    ap.add_argument("--max-iter", type=int, default=8)
    ap.add_argument("--outroot", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
        "docs", "validation", "analysis"))
    sys.exit(asyncio.run(main(ap.parse_args())))
