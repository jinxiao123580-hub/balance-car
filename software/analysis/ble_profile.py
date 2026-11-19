#!/usr/bin/env python3
"""角度曲线播放器：按「Δ:毫秒」的分段曲线改 O，做出加速-匀速-减速。

原理：两个轮子的平衡车，倾角就是加速度指令
    a = g·tan(Δ)        Δ = 相对 O 的偏置角
    前进 = pos 变负 = 负 Δ；后退 = pos 变正 = 正 Δ
所以一条梯形倾角曲线 = 一条梯形速度曲线：
    负 Δ 段（加速）→ 回 O（匀速滑行）→ 正 Δ 段（减速停车）

曲线语法：  --profile "-1.2:600,0:1200,1.2:600,0:800"
            （Δ 度 : 持续毫秒，逗号分隔）

安全性：|pitch−O|>--fall-deg 判翻车立刻归零；位移超 --max-travel 中止；**中止也写盘**。

用法：
  python ble_profile.py --profile "-1.2:600,0:1200,1.2:600,0:800" --reps 2 --out <前缀>
  python ble_profile.py --profile "..." --dry-run        # 只算预测，不连车
退出码：0 正常，2 连不上，4 翻车/越界，5 其它异常（均写盘）。
"""
import argparse
import asyncio
import csv
import math
import os
import re
import sys
import time

from bleak import BleakClient, BleakScanner

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
POS_PER_MM = 1080.0
TOP_MM_S = 109.0      # 1750Hz × 0.0625mm
G = 9.81
I_POS, I_PITCH, I_HZ, I_SPEED = 1, 3, 5, 2


class Abort(Exception):
    pass


def parse_profile(text):
    segs = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        d, _, ms = part.partition(":")
        segs.append((float(d), float(ms)))
    if not segs:
        raise SystemExit("曲线为空")
    return segs


def predict(segs):
    """按 a=g·tan(Δ) 积分，给出每段末速度与累计位移（按 109mm/s 限速）。"""
    v = 0.0        # mm/s，正=后退方向（与 pos 同向）
    x = 0.0
    out = []
    for d, ms in segs:
        t = ms / 1000.0
        a = G * math.tan(math.radians(d)) * 1000.0   # mm/s^2
        v1 = v + a * t
        if abs(v1) > TOP_MM_S:
            # 粗略处理限速：按平均速度算位移
            v1 = math.copysign(TOP_MM_S, v1)
        x1 = x + 0.5 * (v + v1) * t
        out.append((d, ms, a / 1000.0, v1, x1))
        v, x = v1, x1
    return out


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
                self.frames.append((now, line[2:].split(",")))
            else:
                self.text.append((now, line))

    def _last(self, idx, scale=1.0):
        for _w, f in reversed(self.frames):
            if len(f) > idx:
                try:
                    return float(f[idx]) / scale
                except ValueError:
                    return None
        return None

    def pos(self):
        return self._last(I_POS)

    def pitch(self):
        return self._last(I_PITCH, 100.0)


async def main(args):
    segs = parse_profile(args.profile)
    then_segs = parse_profile(args.then) if args.then else []
    plan = predict(segs)
    print("段   Δ(deg)  时长ms    a(m/s2)   末速度mm/s   累计mm")
    for (d, ms), (_d, _ms, a, v1, x1) in zip(segs, plan):
        print(f"     {d:+6.2f} {ms:8.0f} {a:+9.3f} {v1:+11.1f} {x1:+9.1f}")
    print(f"预测总位移 {plan[-1][4]:+.1f}mm   峰值 {max(abs(p[3]) for p in plan):.0f}mm/s")
    if args.dry_run:
        return 0

    d0 = os.path.dirname(args.out)
    if d0:
        os.makedirs(d0, exist_ok=True)
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
    code = 0
    try:
        async with BleakClient(addr, timeout=20.0) as cli:
            await cli.start_notify(FFE1, rec.on_data)

            async def send(cmd):
                for attempt in (1, 2):
                    try:
                        await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)
                        return
                    except Exception as e:
                        if attempt == 2:
                            raise Abort(f"BLE 写入失败 {type(e).__name__}")
                        await asyncio.sleep(0.4)

            await asyncio.sleep(0.8)
            await send("?")
            await asyncio.sleep(1.6)
            buf = "\n".join(l for _w, l in rec.text)
            m = re.search(r"\bO=([-0-9.]+)", buf)
            base = float(m.group(1)) if m else 2.30
            print(f"[原 O] {base}")

            async def play(segs2, p0, trigger_mm):
                """播放一段曲线；trigger_mm>0 时，位移到点就立刻结束本段并返回 hit=True。"""
                out = []
                hit = False
                for i, (d, ms) in enumerate(segs2):
                    await send("^ 0,0")
                    await asyncio.sleep(args.guard_ms / 1000.0)
                    await send(f"O {base + d:.2f}")
                    t0 = time.time()
                    ps = rec.pos()
                    while time.time() - t0 < ms / 1000.0:
                        await asyncio.sleep(0.15)
                        pit = rec.pitch()
                        if pit is not None and abs(pit - base) > args.fall_deg:
                            await send(f"O {base:.2f}")
                            raise Abort(f"翻车 pitch={pit:.1f}° 段{i}(Δ={d:+.2f})")
                        cur = rec.pos()
                        if cur is None:
                            continue
                        moved = abs(cur - p0) / POS_PER_MM
                        if moved > args.max_travel:
                            await send(f"O {base:.2f}")
                            raise Abort(f"越界 {moved:.0f}mm 段{i}")
                        if trigger_mm and moved >= trigger_mm:
                            hit = True
                            break
                    p1 = rec.pos()
                    out.append((i, d, ms,
                                (p1 - ps) / POS_PER_MM if (ps is not None and p1 is not None) else 0.0,
                                time.time() - t0))
                    if hit:
                        break
                return out, hit

            for rep in range(args.reps):
                p0 = rec.pos()
                if p0 is None:
                    raise Abort("拿不到位置")
                t_start = time.time()
                seg_rows, hit = await play(segs, p0, args.until_mm)
                if hit and then_segs:
                    print(f"   [触发] 位移到 {args.until_mm}mm，切刹车段")
                    more, _ = await play(then_segs, p0, 0)
                    seg_rows += [(i + 100, d, ms, dd, tt) for i, d, ms, dd, tt in more]
                await send(f"O {base:.2f}")
                await asyncio.sleep(args.settle_ms / 1000.0)
                p2 = rec.pos()
                total = (p2 - p0) / POS_PER_MM
                dur = time.time() - t_start
                rows.append((rep, total, dur, seg_rows))
                print(f"   rep{rep} 总位移 {total:+7.2f}mm  用时 {dur:5.2f}s  "
                      f"(预测 {plan[-1][4]:+.1f}mm)")
                for i, d, ms, dd, tt in seg_rows:
                    print(f"        段{i} Δ={d:+.2f} {tt*1000:5.0f}ms -> {dd:+7.2f}mm")
                await asyncio.sleep(args.between_s)
    except Abort as e:
        print(f"[中止] {e} —— 数据仍写盘")
        code = 4
    except Exception as e:
        print(f"[异常] {type(e).__name__}: {e} —— 数据仍写盘")
        code = 5

    with open(args.out + ".g.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req", "rx"])
        for wall, f in rec.frames:
            if len(f) >= 8:
                w.writerow([f"{wall:.3f}"] + f[:8])
    with open(args.out + ".segs.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["rep", "seg", "delta", "ms", "disp_mm", "secs"])
        for rep, total, dur, seg_rows in rows:
            for i, d, ms, dd, tt in seg_rows:
                w.writerow([rep, i, d, ms, f"{dd:.2f}", f"{tt:.2f}"])
    if rows:
        ts = [r[1] for r in rows]
        mu = sum(ts) / len(ts)
        sd = (sum((x - mu) ** 2 for x in ts) / len(ts)) ** 0.5
        print(f"\n=== 汇总 === 位移 {mu:+.2f}mm  sd {sd:.2f}  n={len(ts)}  各次 {[round(x,1) for x in ts]}")
    print(f"[结果] {args.out}.g.csv / {args.out}.segs.csv")
    return code


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--address", default="")
    ap.add_argument("--profile", required=True, help='"Δ:ms,Δ:ms,..." Δ 为相对 O 的度')
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--until-mm", type=float, default=0.0,
                    help="加速/巡阶段走到这么多毫米就切到 --then 刹车段（0=不触发）")
    ap.add_argument("--then", default="", help="触发后播放的刹车曲线")
    ap.add_argument("--reps", type=int, default=2)
    ap.add_argument("--between-s", type=float, default=3.0)
    ap.add_argument("--guard-ms", type=float, default=600.0)
    ap.add_argument("--settle-ms", type=float, default=1500.0)
    ap.add_argument("--fall-deg", type=float, default=6.0)
    ap.add_argument("--max-travel", type=float, default=200.0)
    ap.add_argument("--out", default="out")
    sys.exit(asyncio.run(main(ap.parse_args())))
