#!/usr/bin/env python3
"""无界面自动驾测：直接通过 BLE 控制平衡车做 T1/T2/T5/T6，并按面板格式落盘。

用法（在 software/ble-panel 的 venv 里跑，那里有 bleak）：
  .venv\\Scripts\\python.exe ..\\analysis\\drive_motion_test.py ^
      --address E1:6F:42:A7:FF:36 ^
      --set O=5.5 X=0.1 Y=1.0 M=5 ^
      --seq T1,T2 --gear 2.0 --repeat 1 ^
      --out ..\\..\\docs\\validation\\recordings\\auto_T1T2

安全策略：
  * 每 100 ms 检查一次 |pitch − O|，连续 2 次 > --max-dev 立即发 "^ 0,0" 并退出；
  * fallStopCount 增加立即退出；
  * 任何异常/超时都会在 finally 里发送停止命令。
"""
import argparse
import asyncio
import csv
import datetime as dt
import math
import os
import sys
import time

from bleak import BleakClient, BleakScanner

FFE0 = "0000ffe0-0000-1000-8000-00805f9b34fb"
FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"


def load_header(recordings_dir):
    """复用面板录制文件的表头，保证列名与已有分析脚本一致。"""
    best = None
    for name in os.listdir(recordings_dir):
        if name.endswith(".csv") and not name.endswith(".fast.csv"):
            path = os.path.join(recordings_dir, name)
            if best is None or os.path.getmtime(path) > os.path.getmtime(best):
                best = path
    if not best:
        raise SystemExit("找不到已有录制文件来复用表头")
    with open(best, encoding="utf-8-sig") as fh:
        return next(csv.reader(fh)), best


class Car:
    def __init__(self, client, col_index):
        self.client = client
        self.idx = col_index
        self.lines = []
        self.pitch = None
        self.target = None
        self.fall = 0
        self.par = None

    def on_data(self, _sender, data):
        try:
            text = data.decode("utf-8", "replace")
        except Exception:
            return
        for line in text.replace("\r", "\n").split("\n"):
            line = line.strip()
            if not line:
                continue
            self.lines.append(line)
            if line.startswith("PAR:"):
                self.par = line
            elif line.startswith("G,"):
                p = line.split(",")
                if len(p) >= 7:
                    self.pitch = int(p[4]) / 100.0
                    self.target = int(p[5]) / 100.0
            elif line.startswith("D,"):
                p = line.split(",")
                if len(p) > self.idx.get("fallStopCount", -1) + 1:
                    try:
                        self.fall = int(p[self.idx["fallStopCount"]])
                    except Exception:
                        pass

    async def send(self, text):
        await self.client.write_gatt_char(FFE1, (text + "\n").encode("utf-8"), response=True)
        await asyncio.sleep(0.06)


async def find_device(address, timeout=12.0):
    if address:
        return address
    devs = await BleakScanner.discover(timeout=timeout)
    for d in devs:
        if d.name and ("JDY" in d.name or "BT" in d.name):
            return d.address
    raise SystemExit("扫描不到设备，请确认车已上电且面板已关闭")


async def run(args):
    here = os.path.dirname(os.path.abspath(__file__))
    project = os.path.abspath(os.path.join(here, "..", ".."))
    rec_dir = os.path.join(project, "docs", "validation", "recordings")
    header, header_src = load_header(rec_dir)
    idx = {name: i for i, name in enumerate(header)}

    stem = os.path.join(os.path.dirname(args.out) if os.path.dirname(args.out) else rec_dir,
                        os.path.basename(args.out))
    main_path = stem + ".csv"
    fast_path = stem + ".fast.csv"
    for p in (main_path, fast_path):
        if os.path.exists(p):
            raise SystemExit(f"输出已存在：{p}")

    address = await find_device(args.address)
    print(f"[连接] {address}")
    async with BleakClient(address, timeout=30.0) as client:
        car = Car(client, idx)
        await client.start_notify(FFE1, car.on_data)
        for i in range(80):
            if car.par:
                break
            if i % 15 == 14:
                await car.send("?")
            await asyncio.sleep(0.2)
        print(f"[PAR] {car.par or '未收到'}")

        for item in args.set or []:
            letter, value = item.split("=", 1)
            await car.send(f"{letter} {float(value):.3f}")
            print(f"[参数] {letter} = {value}")
        await asyncio.sleep(0.4)
        # 固件上电后要求先收到一次零命令才接受运动请求
        await car.send("^ 0,0")
        await asyncio.sleep(0.2)
        await car.send("?")
        await asyncio.sleep(0.3)

        o_est = 4.0
        for item in args.set or []:
            if item.upper().startswith("O="):
                o_est = float(item.split("=", 1)[1])

        main_f = open(main_path, "w", newline="", encoding="utf-8")
        fast_f = open(fast_path, "w", newline="", encoding="utf-8")
        main_w = csv.writer(main_f)
        main_w.writerow(header)
        fast_f.write("ms,pos100,speed1000,pitch100,target100,finalHz,manualReq1000,rxBytes\n")
        handled = 0

        def flush(since):
            nonlocal handled
            while handled < len(car.lines):
                line = car.lines[handled]
                handled += 1
                if line.startswith("D,"):
                    # 面板 CSV = 主机时间戳 + 74 列遥测 + 22 列参数；驱动不读参数，补空
                    row = [f"{time.time():.1f}"] + line[2:].split(",")
                    if len(row) < len(header):
                        row += [""] * (len(header) - len(row))
                    main_w.writerow(row[:len(header)])
                elif line.startswith("G,"):
                    fast_f.write(line[2:] + "\n")

        stop = {"flag": False, "why": ""}

        async def guard():
            bad = 0
            while not stop["flag"]:
                await asyncio.sleep(0.1)
                if car.pitch is None:
                    continue
                dev = abs(car.pitch - o_est)
                bad = bad + 1 if dev > args.max_dev else 0
                if bad >= 2:
                    stop["flag"], stop["why"] = True, f"|pitch-O|={dev:.2f}°"
                if car.fall > 0:
                    stop["flag"], stop["why"] = True, f"fallStopCount={car.fall}"

        guard_task = asyncio.create_task(guard())

        async def hold(gear, seconds, label):
            print(f"[动作] {label} gear={gear} {seconds}s")
            await car.send(f"^ 0,{gear}")
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                if stop["flag"]:
                    break
                await asyncio.sleep(0.05)
            await car.send("^ 0,0")
            await asyncio.sleep(args.settle)

        try:
            for rep in range(args.repeat):
                for item in args.seq.split(","):
                    item = item.strip().upper()
                    if stop["flag"]:
                        break
                    if item == "T0":
                        print(f"[动作] rep{rep} 静止观察 {args.hold}s")
                        end = time.monotonic() + args.hold
                        while time.monotonic() < end and not stop["flag"]:
                            await asyncio.sleep(0.05)
                    elif item == "T1":
                        await hold(args.gear, args.hold, f"rep{rep} 前进")
                    elif item == "T2":
                        await hold(-args.gear, args.hold, f"rep{rep} 后退")
                    elif item == "T5":
                        await hold(args.gear, args.hold, f"rep{rep} 前进-停车")
                    elif item == "T6":
                        await hold(args.gear, 1.5, f"rep{rep} 前进")
                        await hold(-args.gear, 1.5, f"rep{rep} 后退")
                    else:
                        print(f"[跳过] 未知用例 {item}")
                    flush(0)
        finally:
            try:
                await car.send("^ 0,0")
            except Exception:
                pass
            stop["flag"] = True
            guard_task.cancel()
            flush(0)
            main_f.close()
            fast_f.close()

        print(f"[结果] {main_path}")
        print(f"[结果] {fast_path}")
        if stop["why"]:
            print(f"[中止] {stop['why']}")
            return 3
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="", help="BLE 地址；留空则扫描")
    ap.add_argument("--set", nargs="*", help="启动时下发的参数，如 O=5.5 X=0.1")
    ap.add_argument("--seq", default="T1,T2", help="用例序列，逗号分隔")
    ap.add_argument("--gear", type=float, default=1.0, help="档位（0.5~2.0）")
    ap.add_argument("--hold", type=float, default=3.0, help="每段按住时长 s")
    ap.add_argument("--settle", type=float, default=3.0, help="两段之间停顿 s")
    ap.add_argument("--repeat", type=int, default=1)
    ap.add_argument("--max-dev", type=float, default=3.0, help="|pitch-O| 中止阈值 °")
    ap.add_argument("--out", required=True, help="输出文件名（不含扩展名）")
    args = ap.parse_args()
    sys.exit(asyncio.run(run(args)))


if __name__ == "__main__":
    main()







