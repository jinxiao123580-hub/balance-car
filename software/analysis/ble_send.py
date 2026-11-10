#!/usr/bin/env python3
"""发蓝牙命令并打印回复 + 顺便听几秒遥测（调参用的小工具）。

用法（在 software/ble-panel 的 venv 里跑）：
  .venv\\Scripts\\python.exe ..\\analysis\\ble_send.py --cmd "O 2.000" --cmd "?" --listen 4
  .venv\\Scripts\\python.exe ..\\analysis\\ble_send.py --cmd "TF 0.2" --cmd "TB 0.3"

--listen N：发完之后再听 N 秒，打印最后几帧的 pitch/speed/位置，方便看车往哪走。
退出码：0 正常，2 连不上。
"""
import argparse
import asyncio
import time
import sys

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"


class Sink:
    def __init__(self):
        self.buf = ""
        self.lines = []
        self.stamps = []      # 每行到达的墙钟时间，用于和相机等另一台机器对表

    def on_data(self, _sender, data):
        self.buf += data.decode("utf-8", "replace")
        *done, self.buf = self.buf.replace("\r", "\n").split("\n")
        for line in done:
            line = line.strip()
            if line:
                self.lines.append(line)
                self.stamps.append(time.time())

    def take(self):
        out, self.lines = self.lines, []
        self.stamps = []
        return out

    def pairs(self):
        out = list(zip(self.stamps, self.lines))
        self.lines, self.stamps = [], []
        return out


async def run(args):
    address = args.address
    if not address:
        for d in await BleakScanner.discover(timeout=10.0):
            if d.name and ("JDY" in d.name or "BT" in d.name):
                address = d.address
                break
        if not address:
            print("扫描不到设备：确认车已上电、面板已关闭", file=sys.stderr)
            return 2

    async with BleakClient(address, timeout=30.0) as client:
        sink = Sink()
        await client.start_notify(FFE1, sink.on_data)

        async def send(text):
            await client.write_gatt_char(FFE1, (text + "\n").encode("utf-8"), response=True)
            await asyncio.sleep(args.gap)

        for cmd in args.cmd or []:
            sink.take()
            await send(cmd)
            await asyncio.sleep(0.35)
            replies = [ln for ln in sink.take() if not ln.startswith(("D,", "G,"))]
            print(f"> {cmd}")
            for line in replies:
                print(f"  {line}")

        if args.listen > 0:
            await asyncio.sleep(args.listen)
            pairs = sink.pairs()
            frames = [(w, ln) for w, ln in pairs if ln.startswith("G,")]
            print(f"--- 最近 {args.listen:.1f}s 的 {len(frames)} 帧（4Hz G 帧）---")
            for _w, ln in frames[-args.show:]:
                p = ln[2:].split(",")
                if len(p) >= 8:
                    print(f"  ms={p[0]:>8} pitch={int(p[3])/100:>6.2f}° target={int(p[4])/100:>6.2f}° "
                          f"speed={int(p[2])/1000:>8.2f} c/50ms pos={int(p[1])/100:>10.2f} "
                          f"hz={p[5]:>6} req={int(p[6])/1000:>5.1f}")
            if args.csv:
                import csv as _csv
                with open(args.csv, "w", newline="", encoding="utf-8") as fh:
                    wr = _csv.writer(fh)
                    wr.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req"])
                    for w, ln in frames:
                        p = ln[2:].split(",")
                        if len(p) < 8:
                            continue
                        wr.writerow([f"{w:.2f}", p[0], int(p[1]) / 100.0,
                                     int(p[2]) / 1000.0, int(p[3]) / 100.0, int(p[4]) / 100.0,
                                     p[5], int(p[6]) / 1000.0])
                print(f"[写出] {args.csv}  共 {len(frames)} 帧")
    return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--cmd", action="append", help="要发的命令，可重复")
    ap.add_argument("--gap", type=float, default=0.30, help="两条命令之间的间隔 s")
    ap.add_argument("--listen", type=float, default=0.0, help="命令发完后再听多少秒")
    ap.add_argument("--show", type=int, default=12, help="打印最后几帧")
    ap.add_argument("--csv", default="", help="把 G 帧写成 CSV（含墙钟时间）")
    sys.exit(asyncio.run(run(ap.parse_args())))


if __name__ == "__main__":
    main()
