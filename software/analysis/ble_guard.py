#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ble_guard.py -- 平衡车"看车人"轮询守卫。

设计约束（来自看车人任务书，2026-11-20）：
- 轮询：连接(timeout 8s) -> 订阅采样 2.5s -> 断开 -> sleep 2s -> 重复
- 正常轮次只读遥测。仅在 CRITICAL（pitch/位移判据）时向 FFE1 写 "SP 0" 和 "^ 0,0"。
  严禁改 O/N/K/I、SAVE、烧录、发非零 SP。
- 每轮把状态合并写 <project>/docs/validation/analysis/20261120-ms-knob/guard/state.json，
  并追加一行到同目录 guard.log。跨进程保留 rounds / last_ok（脚本每 2-3 分钟会被重新
  拉起一次，state.json 就是跨进程的"记忆"）。
- 判据（|pitch-O| 单位：度；位移单位：mm）：
    WARN     : |pitch-O|>2.5 在采样窗内连续 >=1.0s；或本轮位移>8；或无遥测>60s
    CRITICAL : |pitch-O|>8（任一帧）；或本轮位移>20；或无遥测>180s
- 连接失败只记日志不报警（BLE 单客户端，lead/别的代理测试时占道是正常现象）。
- 进程自限定运行时长（--max-seconds，默认 150s），到点干净退出，不起后台孤儿进程。

退出码：0=到点正常退出；42=CRITICAL（应立即回报 lead）；1=致命错误。
stdout 约定：普通行=每轮摘要；"@@WARN@@ {json}"/"@@CRITICAL@@ {json}"=需要回报的事件。
"""

import argparse
import asyncio
import json
import os
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

from bleak import BleakClient

# ---------------------------------------------------------------- 硬环境事实
ADDR_DEFAULT = "E1:6F:42:A7:FF:36"            # JDY-23
FFE1 = "0000ffe1-0000-1000-8000-00805f9b34fb"  # 写 + notify 同特征
POS_PER_MM = 1080.0                            # pos 字段 / 1080 = mm
PITCH_PER_DEG = 100.0                          # pitch 字段 / 100 = 度
SPEED_DIV = 1000.0                             # speed 字段 / 1000

O_DEFAULT = 4.35                               # 当前平衡点（已 SAVE 到 flash，只读使用）

CONNECT_TIMEOUT = 8.0
SAMPLE_SECS = 2.5
SLEEP_SECS = 2.0

WARN_PITCH_DEG = 2.5
WARN_PITCH_SUSTAIN_S = 1.0
WARN_DISP_MM = 8.0
CRIT_PITCH_DEG = 8.0
CRIT_DISP_MM = 20.0
NO_TELEM_WARN_S = 60.0
NO_TELEM_CRIT_S = 180.0

ALERT_REPEAT_S = 300.0                         # 同一 critical 事件 5 分钟内不重复弹窗
HISTORY_KEEP = 6                               # state.json 里保留最近几轮摘要
RESUME_GAP_RESET_S = NO_TELEM_WARN_S           # 进程重启间隔 >60s：重置观测基线，不把自己下线的空档算成车失联

ROOT = Path(__file__).resolve().parents[2]
OUTDIR = ROOT / "docs" / "validation" / "analysis" / "20261120-ms-knob" / "guard"
STATE_PATH = OUTDIR / "state.json"
LOG_PATH = OUTDIR / "guard.log"


def iso_now():
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def echo(line):
    print(line, flush=True)


def atomic_write_json(path, obj):
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(json.dumps(obj, ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, path)


def append_log(line):
    with open(LOG_PATH, "a", encoding="utf-8") as fh:
        fh.write(line + "\n")


def load_state():
    try:
        data = json.loads(STATE_PATH.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def fmt_err(exc, limit=110):
    text = "%s: %s" % (type(exc).__name__, str(exc).replace("\n", " ").strip())
    return text[:limit]


# ---------------------------------------------------------------- 遥测帧
class Recorder:
    """按行切分 4Hz 文本帧；G 帧解析，其它文本（固件回执）单独留证。"""

    def __init__(self):
        self.buf = ""
        self.frames = []       # dict(t, pos_mm, speed, pitch, target, output)
        self.texts = []        # (t, 非 G 帧文本)

    def on_data(self, _sender, data):
        now = time.monotonic()
        try:
            chunk = data.decode("utf-8", "replace")
        except Exception:
            return
        self.buf += chunk
        parts = self.buf.replace("\r", "\n").split("\n")
        self.buf = parts.pop()
        for line in parts:
            line = line.strip()
            if not line:
                continue
            if line.startswith("G,"):
                f = line[2:].split(",")
                if len(f) < 8:
                    continue
                try:
                    self.frames.append({
                        "t": now,
                        "pos_mm": float(f[1]) / POS_PER_MM,
                        "speed": float(f[2]) / SPEED_DIV,
                        "pitch": float(f[3]) / PITCH_PER_DEG,
                        "target": float(f[4]) / PITCH_PER_DEG,
                        "output": float(f[5]),
                    })
                except (ValueError, IndexError):
                    continue
            else:
                self.texts.append((now, line[:80]))


def max_sustained_over(frames, o, threshold):
    """采样窗内 |pitch-O|>threshold 的最长连续时长（秒）。"""
    best = 0.0
    run_start = None
    for fr in frames:
        if abs(fr["pitch"] - o) > threshold:
            if run_start is None:
                run_start = fr["t"]
            best = max(best, fr["t"] - run_start)
        else:
            run_start = None
    return best


def round_metrics(frames, o):
    m = {
        "frames": len(frames),
        "pitch": None,
        "pitch_err": None,
        "pos_mm": None,
        "disp_mm": None,
        "drift_mm_s": None,
        "max_abs_pitch_err": None,
        "sustained_over_warn_s": 0.0,
        "window_s": None,
        "speed": None,
        "output": None,
    }
    if not frames:
        return m
    first, last = frames[0], frames[-1]
    dur = last["t"] - first["t"]
    m["pitch"] = round(last["pitch"], 3)
    m["pitch_err"] = round(last["pitch"] - o, 3)
    m["pos_mm"] = round(last["pos_mm"], 2)
    m["speed"] = round(last["speed"], 3)
    m["output"] = round(last["output"], 1)
    m["window_s"] = round(dur, 2)
    m["disp_mm"] = round(last["pos_mm"] - first["pos_mm"], 2)
    m["drift_mm_s"] = round((last["pos_mm"] - first["pos_mm"]) / dur, 2) if dur >= 0.2 else None
    m["max_abs_pitch_err"] = round(max(abs(fr["pitch"] - o) for fr in frames), 3)
    m["sustained_over_warn_s"] = round(max_sustained_over(frames, o, WARN_PITCH_DEG), 2)
    return m


# ---------------------------------------------------------------- 判定
def judge(metrics, o, no_telemetry_s, connected):
    """返回 (level, crit_reasons, warn_reasons)。连接失败本身不报警。"""
    crit, warn = [], []
    if metrics["frames"] > 0:
        worst = metrics["max_abs_pitch_err"]
        if worst > CRIT_PITCH_DEG:
            crit.append("pitch_err=%.2fdeg>%.1f" % (worst, CRIT_PITCH_DEG))
        elif worst > WARN_PITCH_DEG and metrics["sustained_over_warn_s"] >= WARN_PITCH_SUSTAIN_S:
            warn.append("pitch_err=%.2fdeg>%.1f sustained=%.2fs"
                        % (worst, WARN_PITCH_DEG, metrics["sustained_over_warn_s"]))
        if metrics["disp_mm"] is not None:
            if abs(metrics["disp_mm"]) > CRIT_DISP_MM:
                crit.append("disp=%.1fmm>%.0f" % (metrics["disp_mm"], CRIT_DISP_MM))
            elif abs(metrics["disp_mm"]) > WARN_DISP_MM:
                warn.append("disp=%.1fmm>%.0f" % (metrics["disp_mm"], WARN_DISP_MM))
    if no_telemetry_s > NO_TELEM_CRIT_S:
        crit.append("no_telemetry=%.0fs>%.0fs" % (no_telemetry_s, NO_TELEM_CRIT_S))
    elif no_telemetry_s > NO_TELEM_WARN_S:
        warn.append("no_telemetry=%.0fs>%.0fs" % (no_telemetry_s, NO_TELEM_WARN_S))
    level = "critical" if crit else ("warn" if warn else "ok")
    return level, crit, warn


# ---------------------------------------------------------------- 单轮
async def send_stop(client, rec, reasons):
    """CRITICAL 动作：SP 0 + ^ 0,0（唯一允许的两条写命令，只改 RAM）。"""
    info = {"sp0": None, "motion0": None, "texts_after": [], "reason": "; ".join(reasons)}
    for _name, payload, key in (("SP 0", b"SP 0\n", "sp0"), ("^ 0,0", b"^ 0,0\n", "motion0")):
        ok = False
        last = None
        for kwargs in ({"response": True}, {}):
            try:
                await asyncio.wait_for(client.write_gatt_char(FFE1, payload, **kwargs), 4.0)
                ok = True
                break
            except TypeError:
                continue
            except Exception as exc:
                last = exc
                break
        info[key] = "ok" if ok else ("fail:" + fmt_err(last) if last else "fail")
        await asyncio.sleep(0.35)
    await asyncio.sleep(0.5)  # 收固件回执（SP ok / MOTION OK）
    info["texts_after"] = [t for _, t in rec.texts[-6:]]
    return info


async def probe(addr, o):
    """连接 -> 订阅 -> 采样 SAMPLE_SECS -> (若 pitch/位移 critical 则发停车) -> 断开。

    返回 dict：connected / connect_err / frames / metrics / texts / stop_info
    """
    rec = Recorder()
    out = {"connected": False, "connect_err": None, "frames": rec.frames,
           "metrics": None, "texts": rec.texts, "stop_info": None}
    client = None
    try:
        try:
            client = BleakClient(addr, timeout=CONNECT_TIMEOUT)
        except TypeError:               # 老版本 bleak 无 timeout 关键字
            client = BleakClient(addr)
        await asyncio.wait_for(client.connect(), CONNECT_TIMEOUT)
        out["connected"] = True
        await asyncio.wait_for(client.start_notify(FFE1, rec.on_data), 5.0)
        t_end = time.monotonic() + SAMPLE_SECS
        while time.monotonic() < t_end:
            await asyncio.sleep(0.05)
        metrics = round_metrics(rec.frames, o)
        out["metrics"] = metrics
        # 停车动作必须在断开前完成：先按 pitch/位移判据判定（无遥测判据此处不适用）
        level, crit, _warn = judge(metrics, o, 0.0, True)
        if level == "critical":
            out["stop_info"] = await send_stop(client, rec, crit)
        return out
    except asyncio.CancelledError:
        raise
    except Exception as exc:            # 连接/订阅失败都是常态（单客户端占道）
        out["connect_err"] = fmt_err(exc)
        if out["metrics"] is None:
            out["metrics"] = round_metrics(rec.frames, o)
        out["frames"] = rec.frames
        return out
    finally:
        if client is not None:
            try:
                await asyncio.wait_for(client.disconnect(), 5.0)
            except Exception:
                pass


# ---------------------------------------------------------------- 主循环
async def run(args):
    OUTDIR.mkdir(parents=True, exist_ok=True)
    o = args.o
    prev = load_state()
    now0 = time.time()
    session_start_epoch = float(prev.get("session_start_epoch") or now0)
    rounds = int(prev.get("rounds") or 0)
    last_ok_epoch = prev.get("last_ok_epoch")
    baseline_epoch = float(prev.get("baseline_epoch") or session_start_epoch)
    last_crit_alert_epoch = float(prev.get("last_critical_alert_epoch") or 0.0)
    no_telem_alerted = bool(prev.get("no_telem_alerted"))
    history = list(prev.get("recent_rounds") or [])

    # 进程重启空档不能算"车失联"：离线超过阈值就重置观测基线并留痕
    resume_gap = 0.0
    if last_ok_epoch:
        resume_gap = now0 - float(last_ok_epoch)
    if (not last_ok_epoch) or resume_gap > RESUME_GAP_RESET_S:
        if last_ok_epoch or resume_gap > RESUME_GAP_RESET_S:
            echo("RESUME_GAP %.1fs -> telemetry baseline reset @process_start" % resume_gap)
        last_ok_epoch = None
        baseline_epoch = now0

    deadline = time.monotonic() + args.max_seconds
    exit_code = 0
    prev_level_in_process = None

    while time.monotonic() < deadline:
        if args.max_rounds and (rounds - int(prev.get("rounds") or 0)) >= args.max_rounds:
            break
        rounds += 1
        result = await probe(args.address, o)
        metrics = result["metrics"]
        connected = result["connected"]
        connect_err = result["connect_err"]
        stop_info = result["stop_info"]

        now = time.time()
        if metrics["frames"] > 0:
            last_ok_epoch = now
            baseline_epoch = now
            no_telem_alerted = False
        no_telemetry_s = now - float(last_ok_epoch or baseline_epoch)

        level, crit, warn = judge(metrics, o, no_telemetry_s, connected)
        if stop_info and level != "critical":       # 停车已发，但本轮判据仍按 critical 记
            level = "critical"
        reasons = crit if level == "critical" else (warn if level == "warn" else [])

        # ---- note
        if level == "ok":
            if metrics["frames"]:
                note = "steady"
            elif connected:
                note = "connected_but_no_frames"
            else:
                note = "connect_fail:" + str(connect_err or "no_frames")[:80]
        elif level == "warn":
            note = "; ".join(reasons)
        else:
            note = "; ".join(reasons)
            if stop_info:
                note += "; stop=" + json.dumps({k: v for k, v in stop_info.items() if k in ("sp0", "motion0")},
                                               ensure_ascii=False)
                if stop_info.get("texts_after"):
                    note += "; reply=" + "|".join(stop_info["texts_after"])
            elif connect_err:
                note += "; stop_undeliverable:" + str(connect_err)[:60]
            else:
                note += "; stop_undeliverable:no_telemetry"

        ts = iso_now()
        state = {
            "ts": ts,
            "epoch": round(now, 3),
            "connected": bool(connected),
            "pitch": metrics["pitch"],
            "O": o,
            "pitch_err": metrics["pitch_err"],
            "pos_mm": metrics["pos_mm"],
            "drift_mm_s": metrics["drift_mm_s"],
            "rounds": rounds,
            "level": level,
            "note": note,
            "disp_mm": metrics["disp_mm"],
            "max_abs_pitch_err": metrics["max_abs_pitch_err"],
            "sustained_over_warn_s": metrics["sustained_over_warn_s"],
            "window_s": metrics["window_s"],
            "frames": metrics["frames"],
            "speed": metrics["speed"],
            "output": metrics["output"],
            "no_telemetry_s": round(no_telemetry_s, 1),
            "baseline_epoch": round(baseline_epoch, 3),
            "last_ok_epoch": round(float(last_ok_epoch), 3) if last_ok_epoch else None,
            "last_ok_ts": (datetime.fromtimestamp(last_ok_epoch).astimezone().isoformat(timespec="seconds")
                           if last_ok_epoch else None),
            "session_start_epoch": round(session_start_epoch, 3),
            "session_start_ts": datetime.fromtimestamp(session_start_epoch).astimezone().isoformat(timespec="seconds"),
            "connect_err": connect_err,
            "stop_info": stop_info,
            "last_critical_alert_epoch": round(last_crit_alert_epoch, 3) if last_crit_alert_epoch else None,
            "no_telem_alerted": no_telem_alerted,
            "recent_rounds": history[-HISTORY_KEEP:],
            "pid": os.getpid(),
            "script": str(Path(__file__).resolve()),
            "addr": args.address,
        }
        # 本轮摘要进历史
        history.append({"ts": ts, "r": rounds, "lvl": level, "conn": bool(connected),
                        "pitch_err": metrics["pitch_err"], "disp": metrics["disp_mm"],
                        "drift": metrics["drift_mm_s"], "frames": metrics["frames"]})
        state["recent_rounds"] = history[-HISTORY_KEEP:]
        atomic_write_json(STATE_PATH, state)

        if connected and metrics["frames"]:
            logline = ("%s r=%d conn=1 frames=%d pitch=%.2f O=%.2f err=%+.2f pos=%+.2fmm "
                       "drift=%s disp=%s lvl=%s note=%s"
                       % (ts, rounds, metrics["frames"], metrics["pitch"], o, metrics["pitch_err"],
                          metrics["pos_mm"] if metrics["pos_mm"] is not None else 0.0,
                          ("%+.2f" % metrics["drift_mm_s"]) if metrics["drift_mm_s"] is not None else "?",
                          ("%+.2f" % metrics["disp_mm"]) if metrics["disp_mm"] is not None else "?",
                          level, note))
        else:
            logline = ("%s r=%d conn=%d frames=%d lvl=%s no_telem=%.0fs note=%s"
                       % (ts, rounds, int(bool(connected)), metrics["frames"], level,
                          no_telemetry_s, note))

        if level in ("warn", "critical"):
            append_log(logline)
            echo(logline)
            # 事件 marker：critical 立即退出让我马上回报；warn 只在状态跃迁时打标
            if level == "critical":
                repeat_gate_ok = (now - last_crit_alert_epoch) > ALERT_REPEAT_S
                first_outage = not no_telem_alerted
                need_alert = False
                if stop_info:
                    need_alert = repeat_gate_ok
                elif no_telemetry_s > NO_TELEM_CRIT_S:
                    need_alert = first_outage
                    no_telem_alerted = True
                else:
                    need_alert = repeat_gate_ok
                state["last_critical_alert_epoch"] = round(now, 3) if need_alert else state["last_critical_alert_epoch"]
                state["no_telem_alerted"] = no_telem_alerted
                atomic_write_json(STATE_PATH, state)
                if need_alert:
                    last_crit_alert_epoch = now
                    echo("@@CRITICAL@@ " + json.dumps(
                        {"ts": ts, "pitch": metrics["pitch"], "O": o, "pitch_err": metrics["pitch_err"],
                         "disp_mm": metrics["disp_mm"], "drift_mm_s": metrics["drift_mm_s"],
                         "no_telemetry_s": round(no_telemetry_s, 1), "reasons": reasons,
                         "stop": stop_info, "connected": connected, "rounds": rounds},
                        ensure_ascii=False))
                    exit_code = 42
                    break
            else:
                if prev_level_in_process != "warn":
                    echo("@@WARN@@ " + json.dumps(
                        {"ts": ts, "pitch": metrics["pitch"], "O": o, "pitch_err": metrics["pitch_err"],
                         "disp_mm": metrics["disp_mm"], "drift_mm_s": metrics["drift_mm_s"],
                         "no_telemetry_s": round(no_telemetry_s, 1), "reasons": reasons,
                         "rounds": rounds}, ensure_ascii=False))
        elif level == "ok":
            append_log(logline)
            echo(logline)
        prev_level_in_process = level

        if exit_code:
            break
        await asyncio.sleep(args.sleep)

    if not exit_code:
        echo("GUARD_DONE rounds=%d elapsed=%.0fs exit=0" % (rounds, time.monotonic() - (deadline - args.max_seconds)))
    return exit_code


def selftest():
    """合成遥测验证判据（不需要真车，只验证阈值逻辑/不假报）。"""
    o = O_DEFAULT
    ok = True

    def frames_from(pitches, pos=None, dt=0.25):
        n = len(pitches)
        pos = pos if pos is not None else [0.0] * n
        return [{"t": i * dt, "pitch": p, "pos_mm": pos[i], "speed": 0.0, "target": o, "output": 0.0}
                for i, p in enumerate(pitches)]

    import math
    # 1) 正常平衡：±2° 摆动（p2p 4°），位移 ±1mm -> ok
    pitches = [o + 2.0 * math.sin(2 * math.pi * 1.6 * i * 0.25) for i in range(10)]
    m = round_metrics(frames_from(pitches), o)
    lvl, c, w = judge(m, o, 0.0, True)
    print("[1] normal oscillation p2p=%.1fdeg -> %s %s%s" % (m["max_abs_pitch_err"] * 2, lvl, c, w))
    ok &= lvl == "ok"

    # 2) 短暂超 2.5°（0.75s 内）-> 仍 ok（"持续 1 秒"门槛，防假报警）
    pitches = [o, o, o + 3.0, o + 3.0, o + 3.0, o, o, o, o, o]
    m = round_metrics(frames_from(pitches), o)
    lvl, c, w = judge(m, o, 0.0, True)
    print("[2] brief 3.0deg for 0.75s -> %s %s%s" % (lvl, c, w))
    ok &= lvl == "ok"

    # 3) 持续 3.0°（>=4 帧，1.0s 以上）-> warn
    pitches = [o, o] + [o + 3.0] * 6 + [o]
    m = round_metrics(frames_from(pitches), o)
    lvl, c, w = judge(m, o, 0.0, True)
    print("[3] sustained 3.0deg %.2fs -> %s %s%s" % (m["sustained_over_warn_s"], lvl, c, w))
    ok &= lvl == "warn"

    # 4) 单帧 13.0°（err 8.65）-> critical
    pitches = [o] * 6 + [13.0] + [o] * 3
    m = round_metrics(frames_from(pitches), o)
    lvl, c, w = judge(m, o, 0.0, True)
    print("[4] single frame 13.0deg -> %s %s" % (lvl, c))
    ok &= lvl == "critical"

    # 5) 单轮位移 9mm -> warn；21mm -> critical
    base = [0.0, 0.0, 1.0, 2.0, 4.0, 6.0, 8.0, 9.0, 9.0, 9.0]
    m = round_metrics(frames_from([o] * 10, base), o)
    lvl, c, w = judge(m, o, 0.0, True)
    print("[5a] disp=%+.1fmm -> %s %s%s" % (m["disp_mm"], lvl, c, w))
    ok &= lvl == "warn"
    base = [0.0, 0.0, 3.0, 7.0, 12.0, 16.0, 19.0, 21.0, 21.0, 21.0]
    m = round_metrics(frames_from([o] * 10, base), o)
    lvl, c, w = judge(m, o, 0.0, True)
    print("[5b] disp=%+.1fmm -> %s %s" % (m["disp_mm"], lvl, c))
    ok &= lvl == "critical"

    # 6) 连接失败但 <60s -> ok；61s/181s -> warn/critical
    m0 = round_metrics([], o)
    for gap, expect in ((10.0, "ok"), (61.0, "warn"), (181.0, "critical")):
        lvl, c, w = judge(m0, o, gap, False)
        print("[6] no_telemetry=%.0fs connected=False -> %s %s%s" % (gap, lvl, c, w))
        ok &= lvl == expect

    # 7) 单帧读数但 2 帧（帧数不足）-> 不能凭"持续"判 warn
    m = round_metrics(frames_from([o + 4.0, o + 4.0], dt=0.25), o)
    lvl, c, w = judge(m, o, 0.0, True)
    print("[7] 2 frames at 4.0deg (sustained=%.2fs) -> %s %s%s" % (m["sustained_over_warn_s"], lvl, c, w))
    ok &= lvl == "ok"

    print("SELFTEST %s" % ("PASS" if ok else "FAIL"))
    return 0 if ok else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description="平衡车 BLE 看车人轮询守卫")
    ap.add_argument("--address", default=ADDR_DEFAULT)
    ap.add_argument("--o", type=float, default=O_DEFAULT, help="平衡点 O（默认 4.35）")
    ap.add_argument("--max-seconds", type=float, default=150.0, help="本进程最长运行时间（默认150s）")
    ap.add_argument("--max-rounds", type=int, default=0, help="最多跑几轮（0=不限制，只受 max-seconds 约束）")
    ap.add_argument("--sleep", type=float, default=SLEEP_SECS, help="轮间隔（默认2s）")
    ap.add_argument("--selftest", action="store_true", help="用合成遥测验证判据逻辑，不连 BLE")
    args = ap.parse_args(argv)
    if args.selftest:
        return selftest()
    if args.max_seconds > 300:
        args.max_seconds = 300.0
    try:
        return asyncio.run(run(args))
    except KeyboardInterrupt:
        echo("GUARD_INTERRUPTED")
        return 0
    except Exception:
        echo("GUARD_FATAL\n" + traceback.format_exc())
        return 1


if __name__ == "__main__":
    sys.exit(main())
