#!/usr/bin/env python3
"""参数持久化的实车验收：SAVE -> 断电 -> 上电读回。

用法（在 software/ble-panel 的 venv 里跑，那里有 bleak）：
  .venv\\Scripts\\python.exe ..\\analysis\\check_param_persist.py --query
  .venv\\Scripts\\python.exe ..\\analysis\\check_param_persist.py --save
  .venv\\Scripts\\python.exe ..\\analysis\\check_param_persist.py --def

动作：
  --query   只问一次 "?"，打印 FW / SAVED / 22 项参数（默认动作）
  --save    发 SAVE，再问一次，要求 SAVED=1
  --def     发 DEF，再问一次，要求 SAVED=0（只在 RAM 恢复出厂值）
  --load    发 LOAD，再问一次，要求 SAVED=1

退出码：0 通过，1 失败，2 连不上。
"""
import argparse
import asyncio
import json
import re
import sys

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
PARAM_ORDER = "PDOZLSRBH KINWJEXYM FGTU".replace(" ", "")   # 22 项，与固件 paramSlots 同序


class Link:
    """按行累积：BLE 一次 notify 装不下整行，必须自己拼到换行为止。"""

    def __init__(self):
        self.buf = ""
        self.lines = []

    def on_data(self, _sender, data):
        self.buf += data.decode("utf-8", "replace")
        *done, self.buf = self.buf.replace("\r", "\n").split("\n")
        for line in done:
            line = line.strip()
            if line:
                self.lines.append(line)

    def take(self):
        out, self.lines = self.lines, []
        return out


def parse_par(line):
    """PAR 行长这样：PAR: TURN=2 ... TILTMAX=2.00 P=1000.00 ... M=12.00 SAVED=1"""
    caps = line.split()
    fw = next((c.split("=", 1)[1] for c in caps if c.startswith("FW=")), "")
    saved = next((c.split("=", 1)[1] for c in caps if c.startswith("SAVED=")), "")
    # 只认单字母参数，且排除被包在 SAVED= 里那种误配
    params = {}
    for m in re.finditer(r"(?:^|\s)([A-Z])=([-+]?[0-9]*\.?[0-9]+)", line):
        params[m.group(1)] = float(m.group(2))
    return {"fw": fw, "saved": saved, "params": params}


async def run(args):
    address = args.address
    if not address:
        devs = await BleakScanner.discover(timeout=10.0)
        for d in devs:
            if d.name and ("JDY" in d.name or "BT" in d.name):
                address = d.address
                break
        if not address:
            print("扫描不到设备：确认车已上电、面板已关闭", file=sys.stderr)
            return 2

    async with BleakClient(address, timeout=30.0) as client:
        link = Link()
        await client.start_notify(FFE1, link.on_data)

        async def send(text):
            await client.write_gatt_char(FFE1, (text + "\n").encode("utf-8"), response=True)
            await asyncio.sleep(0.12)

        async def ask(timeout=3.0):
            link.take()
            await send("?")
            loop = asyncio.get_event_loop()
            end = loop.time() + timeout
            while loop.time() < end:
                for line in link.take():
                    if line.startswith("PAR:"):
                        return line
                await asyncio.sleep(0.05)
            return ""

        async def wait_reply(prefix, timeout=4.0):
            loop = asyncio.get_event_loop()
            end = loop.time() + timeout
            while loop.time() < end:
                for line in link.take():
                    if line.startswith(prefix):
                        return line
                await asyncio.sleep(0.05)
            return ""

        action = args.action
        if action in ("save", "load", "def"):
            link.take()
            await send(action.upper())
            reply = await wait_reply(("SAVED" if action == "save" else action.upper()), 8.0)
            print(f"[{action.upper()}] {reply or '（无回复）'}")

        par_line = await ask()
        if not par_line:
            print("没收到 PAR 回显", file=sys.stderr)
            return 1
        info = parse_par(par_line)
        missing = [k for k in PARAM_ORDER if k not in info["params"]]
        print(f"[固件] FW={info['fw'] or '未知'}")
        print(f"[参数] SAVED={info['saved'] or '旧固件无此字段'}  读到 {len(info['params'])} 项"
              + (f"  缺 {''.join(missing)}" if missing else ""))
        print("[值] " + " ".join(f"{k}={info['params'][k]:g}" for k in PARAM_ORDER
                                 if k in info["params"]))
        if args.json:
            with open(args.json, "w", encoding="utf-8") as fh:
                json.dump(info, fh, ensure_ascii=False, indent=2)
            print(f"[写出] {args.json}")

        if args.expect_fw and info["fw"] != args.expect_fw:
            print(f"[失败] 固件版本不符：期望 {args.expect_fw}，实际 {info['fw']}", file=sys.stderr)
            return 1
        if args.expect_saved is not None and info["saved"] != args.expect_saved:
            print(f"[失败] SAVED 期望 {args.expect_saved}，实际 {info['saved']}", file=sys.stderr)
            return 1
        return 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--query", dest="action", action="store_const", const="query")
    ap.add_argument("--save", dest="action", action="store_const", const="save")
    ap.add_argument("--load", dest="action", action="store_const", const="load")
    ap.add_argument("--def", dest="action", action="store_const", const="def")
    ap.add_argument("--json")
    ap.add_argument("--expect-fw", default="")
    ap.add_argument("--expect-saved", default=None)
    ap.set_defaults(action="query")
    args = ap.parse_args()
    sys.exit(asyncio.run(run(args)))


if __name__ == "__main__":
    main()
