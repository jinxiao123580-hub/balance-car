#!/usr/bin/env python3
"""前进 / 后退 演示用例（不连续点动）。

======================================================================
用法示例
======================================================================
  # 1) 先不连车，只看会执行什么（强烈建议先跑这个）
  python ble_demo_case.py --dry-run

  # 2) 实车：2 个循环，每循环前进 3 次 + 后退 3 次，Δ=1.2°，点动 0.6s、滑行 1.2s
  python ble_demo_case.py --fwd 3 --back 3 --cycles 2 --amp 1.2 \
      --jog 0.6 --coast 1.2 --pitch-lim 4.0 \
      --out ..\\..\\docs\\validation\\analysis\\20261113-nocam\\demo_case\\run01

======================================================================
安全边界（都是实测踩出来的，别改）
======================================================================
* O 基准**写死 4.00**，不做任何重估（用户指定）。
* Δ = 相对 O 的差值：**负 Δ = 前进，正 Δ = 后退**。
* 单次 Δ 硬上限 1.4°（--amp 超过即拒绝运行）；实测 2.2° 必翻车。
* 必须**不连续点动**：瞬间给 Δ → 保持 --jog 秒 → 回零滑行 --coast 秒。
  连续斜坡（slew）实测把时间耗在摩擦门槛以下，车几乎不动。
* 翻车判据 |pitch − O| > --pitch-lim（默认 4°）：**立刻发 `O 4.00` 复位基准**
  （不是 base+applied —— 踩过的坑：留成 5.30 会让车持续后倾漂移），然后退出。
* 改 O 之前先发 `^ 0,0`；BLE 写入必须 response=True，否则命令被静默丢弃。
* 任何中止都仍然会把已录到的数据写盘。

======================================================================
!!! 重要警告 !!!
======================================================================
  帧里的 `pos` 是**编码器读数 = 轮子转角**，不是车体位移。
  实测轮子会前后晃动（±20~40mm）而车体几乎不动，二者不是一回事。
  因此本脚本对 pos 只作参考量，并加了有界保护：
  偏离起点超过 --max-offset-mm 就停止往该方向继续点动，超过 1.6 倍则硬中止。
  真实的“车走了多远”必须用相机（tagtrk）或尺子核对，不能只看 pos。
======================================================================
"""
import argparse
import asyncio
import csv
import os
import re
import sys
import time

FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"
BASE_O = 4.00          # 写死的平衡角基准，不重估
POS_PER_MM = 1080.0    # pos/100/10.8 = mm
AMP_LIMIT = 1.4        # 单次 Δ 硬上限
I_POS, I_PITCH = 1, 3

# 默认输出目录固定到项目内的验证证据目录（不落在 cwd）
_HERE = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_OUT = os.path.normpath(os.path.join(
    _HERE, "..", "..", "docs", "validation", "analysis",
    "20261113-nocam", "demo_case", "run"))


class Abort(Exception):
    pass


class Rec:
    """收 G 帧：'G,ms,pos,speed,pitch,target,hz,req,rx'"""

    def __init__(self):
        self.frames = []
        self.text = []
        self.buf = ""

    def on_data(self, _sender, data):
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
        for _t, f in reversed(self.frames):
            if len(f) > idx:
                try:
                    return float(f[idx]) / scale
                except ValueError:
                    return None
        return None

    def pos(self):
        return self._last(I_POS)

    def pitch(self):
        v = self._last(I_PITCH)
        return None if v is None else v / 100.0      # 帧里是百分度


def build_plan(args):
    """按 循环 × (前进 N 次 + 后退 M 次) 生成点动序列。"""
    plan = []
    for c in range(args.cycles):
        for i in range(args.fwd):
            plan.append({"cycle": c, "dir": -1, "idx": i + 1, "total": args.fwd,
                         "name": "前进"})
        for i in range(args.back):
            plan.append({"cycle": c, "dir": +1, "idx": i + 1, "total": args.back,
                         "name": "后退"})
    return plan


def print_plan(plan, args):
    print(f"[计划] {len(plan)} 次点动   循环 {args.cycles} × "
          f"(前进 {args.fwd} + 后退 {args.back})")
    print(f"[参数] O 基准={BASE_O:.2f}  Δ=±{args.amp:.2f}°  "
          f"点动 {args.jog:.2f}s  滑行 {args.coast:.2f}s  "
          f"pitch 保护 ±{args.pitch_lim:.1f}°  位移保护 ±{args.max_offset_mm:.0f}mm")
    print(f"{'#':>3} {'循环':>4} {'方向':>4} {'序':>3} {'Δ(deg)':>8} {'O 目标':>7} {'保持':>5} {'滑行':>5}")
    for n, p in enumerate(plan, 1):
        delta = p["dir"] * args.amp
        print(f"{n:>3} {p['cycle']+1:>4} {p['name']:>4} {p['idx']:>3} "
              f"{delta:>+8.2f} {BASE_O + delta:>7.2f} {args.jog:>5.2f} {args.coast:>5.2f}")
    print("[dry-run] 未连接任何设备，序列如上。")


async def run_live(args, plan):
    from bleak import BleakClient, BleakScanner        # 只有实车才需要 bleak

    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    addr = args.address
    if not addr:
        print("[扫描] 找 JDY-23 ...")
        for dev in await BleakScanner.discover(timeout=10.0):
            if (dev.name or "").upper().startswith("JDY"):
                addr = dev.address
                break
    if not addr:
        print("[错误] 没找到 JDY-23（车离线？）")
        return 2
    print(f"[连接] {addr}")

    rec = Rec()
    rows = []
    aborted = None
    t_start = time.time()
    pitch_dev_max = 0.0

    try:
        async with BleakClient(addr, timeout=20.0) as cli:
            await cli.start_notify(FFE1, rec.on_data)

            async def send(cmd):
                for attempt in (1, 2):
                    try:
                        await cli.write_gatt_char(FFE1, (cmd + "\n").encode(), response=True)
                        return
                    except Exception as exc:
                        if attempt == 2:
                            raise Abort(f"BLE 写入失败 {type(exc).__name__}")
                        await asyncio.sleep(0.3)

            await asyncio.sleep(0.8)
            await send("?")
            await asyncio.sleep(1.5)
            for _t, line in rec.text:
                if line.startswith("PAR"):
                    m = re.search(r"\bO=([-0-9.]+)", line)
                    print(f"[开机 O] {m.group(1) if m else '?'} → 写死 {BASE_O:.2f}")

            # 基准归位：先 ^ 0,0 再设 O
            await send("^ 0,0")
            await asyncio.sleep(0.8)
            await send(f"O {BASE_O:.2f}")
            await asyncio.sleep(0.5)

            p_start = rec.pos()

            def check_fall(tag):
                """翻车检测：超阈值必须把 O 复位成基准 4.00，不能留 base+applied。"""
                nonlocal pitch_dev_max
                pit = rec.pitch()
                if pit is None:
                    return
                dev = abs(pit - BASE_O)
                if dev > pitch_dev_max:
                    pitch_dev_max = dev
                if dev > args.pitch_lim:
                    raise Abort(f"{tag} 翻车：pitch={pit:.2f}° 偏离基准 {dev:.2f}° "
                                f"> {args.pitch_lim:.1f}°")

            for n, p in enumerate(plan, 1):
                dirn = p["dir"]
                delta = dirn * args.amp
                p0 = rec.pos()
                if p0 is None:
                    raise Abort("拿不到编码器位置")
                offset_mm = (p0 - p_start) / POS_PER_MM if p_start is not None else 0.0

                # 有界保护：越界就不再往该方向推
                would_worsen = (dirn > 0 and offset_mm > 0) or (dirn < 0 and offset_mm < 0)
                if abs(offset_mm) > args.max_offset_mm and would_worsen:
                    print(f"[跳过] #{n} {p['name']}{p['idx']}：编码器偏离起点 "
                          f"{offset_mm:+.1f}mm 已超 {args.max_offset_mm:.0f}mm，不再同向推进")
                    rows.append((n, p["cycle"] + 1, p["name"], p["idx"], delta,
                                 0.0, 0.0, rec.pitch(), "skipped"))
                    continue
                if abs(offset_mm) > args.max_offset_mm * 1.6:
                    raise Abort(f"编码器偏离起点 {offset_mm:+.1f}mm 过大（pos 只是轮子转角，"
                                f"说明已在空转或失控）")

                # 点动：^ 0,0 → O(基准+Δ) → 保持 → O(基准) → 滑行
                await send("^ 0,0")
                await asyncio.sleep(args.guard)
                check_fall(f"#{n} 设 O 前")
                await send(f"O {BASE_O + delta:.2f}")
                t_jog = time.time()
                while time.time() - t_jog < args.jog:
                    check_fall(f"#{n} 点动中")
                    await asyncio.sleep(0.1)
                await send(f"O {BASE_O:.2f}")
                t_coast = time.time()
                while time.time() - t_coast < args.coast:
                    check_fall(f"#{n} 滑行中")
                    await asyncio.sleep(0.1)

                p1 = rec.pos()
                step_mm = dirn * (p1 - p0) / POS_PER_MM if p1 is not None else 0.0
                cum_mm = (p1 - p_start) / POS_PER_MM if (p1 is not None and p_start is not None) else 0.0
                pit = rec.pitch()
                rows.append((n, p["cycle"] + 1, p["name"], p["idx"], delta,
                             step_mm, cum_mm, pit, "ok"))
                print(f"[{n:>2}/{len(plan)}] 循环{p['cycle']+1} {p['name']}{p['idx']} "
                      f"Δ={delta:+.2f}°  pos差={step_mm:+7.2f}mm  "
                      f"累计={cum_mm:+8.2f}mm  pitch={pit if pit is None else round(pit, 2)}°")

            await send("^ 0,0")
            await send(f"O {BASE_O:.2f}")
    except Abort as exc:
        # 关键：复位成基准 O（不是 base+applied）
        aborted = str(exc)
        print(f"[中止] {exc}")
        try:
            async with BleakClient(addr, timeout=10.0) as cli2:   # 重连兜底复位
                await cli2.write_gatt_char(FFE1, b"^ 0,0\n", response=True)
                await cli2.write_gatt_char(FFE1, f"O {BASE_O:.2f}\n".encode(), response=True)
                print(f"[复位] 已发 O {BASE_O:.2f}")
        except Exception:
            print("[复位] 重连失败，请手动发 `^ 0,0` 与 `O 4.00`")
    except Exception as exc:
        aborted = f"{type(exc).__name__}: {exc}"
        print(f"[异常] {aborted}")

    # ---------------- 写盘（中止也写） ----------------
    gpath = args.out + ".g.csv"
    with open(gpath, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["wall", "ms", "pos", "speed", "pitch", "target", "hz", "req", "rx"])
        for wall, f in rec.frames:
            if len(f) >= 8:
                w.writerow([f"{wall:.3f}"] + f[:8])

    jpath = args.out + ".jogs.csv"
    with open(jpath, "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["n", "cycle", "dir", "idx", "delta", "step_mm", "cum_mm", "pitch", "status"])
        for r in rows:
            w.writerow([r[0], r[1], r[2], r[3], f"{r[4]:.2f}", f"{r[5]:.2f}",
                        f"{r[6]:.2f}", "" if r[7] is None else f"{r[7]:.2f}", r[8]])

    # ---------------- 汇总 ----------------
    elapsed = time.time() - t_start
    ok = [r for r in rows if r[8] == "ok"]
    fwd = [r[5] for r in ok if r[2] == "前进"]
    bwd = [r[5] for r in ok if r[2] == "后退"]
    total = ok[-1][6] if ok else 0.0
    print("\n================ 汇总 ================")
    print(f"执行点动      : {len(ok)}/{len(plan)}"
          + (f"（跳过 {len(rows)-len(ok)}）" if len(rows) > len(ok) else ""))
    print(f"总位移(pos)   : {total:+.2f} mm   ← 轮子转角，不等于车体位移！")
    if fwd:
        print(f"前进平均单次  : {sum(fwd)/len(fwd):+.2f} mm  (n={len(fwd)})")
    if bwd:
        print(f"后退平均单次  : {sum(bwd)/len(bwd):+.2f} mm  (n={len(bwd)})")
    print(f"用时          : {elapsed:.1f} s")
    print(f"pitch 最大偏离: {pitch_dev_max:.2f}°  (保护阈值 {args.pitch_lim:.1f}°)")
    if aborted:
        print(f"中止原因      : {aborted}")
    print(f"[结果] {gpath}")
    print(f"[结果] {jpath}")
    return 4 if aborted else 0


def main(argv):
    ap = argparse.ArgumentParser(
        description="前进/后退不连续点动演示（O 写死 4.00，负 Δ 前进 / 正 Δ 后退）")
    ap.add_argument("--fwd", type=int, default=3, help="每循环前进点动次数")
    ap.add_argument("--back", type=int, default=3, help="每循环后退点动次数")
    ap.add_argument("--cycles", type=int, default=2, help="循环次数")
    ap.add_argument("--amp", type=float, default=1.2,
                    help=f"Δ 幅度（度），硬上限 {AMP_LIMIT}")
    ap.add_argument("--jog", type=float, default=0.6, help="点动保持秒数")
    ap.add_argument("--coast", type=float, default=1.2, help="回零滑行秒数")
    ap.add_argument("--guard", type=float, default=0.5, help="^ 0,0 之后等多久再设 O")
    ap.add_argument("--pitch-lim", type=float, default=4.0, help="|pitch−O| 翻车阈值（度）")
    ap.add_argument("--max-offset-mm", type=float, default=60.0,
                    help="编码器偏离起点的有界保护阈值（mm，仅参考量）")
    ap.add_argument("--address", default="", help="BLE 地址，留空则扫描 JDY-23")
    ap.add_argument("--dry-run", action="store_true", help="不连车，只打印序列")
    ap.add_argument("--out", default=_DEFAULT_OUT, help="输出 CSV 前缀（不含扩展名）")
    args = ap.parse_args(argv)

    if args.amp > AMP_LIMIT:
        print(f"[拒绝] --amp={args.amp} 超过硬上限 {AMP_LIMIT}°（实测更大必翻车）")
        return 1
    if args.amp <= 0 or args.jog <= 0 or args.coast < 0 or args.cycles <= 0:
        print("[拒绝] 参数非法：amp/jog 必须 >0，cycles 必须 >0")
        return 1

    plan = build_plan(args)
    print_plan(plan, args)
    if args.dry_run:
        return 0
    return asyncio.run(run_live(args, plan))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
