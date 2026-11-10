#!/usr/bin/env python3
"""前后运动/加减速/停车试验的量化分析。

用法:
    python software/analysis/analyze_motion_test.py <csv> [--max-speed 5] [--markdown]

--max-speed 必须与固件 MotionSpeedProfile::maxSpeed 一致（2026-11-10 起为 5.0，此前 8.0）：
vref = 档位/2 × max-speed，给错了稳态误差会整体偏。

对每次"按住"（manualReq1000 非零且方向不变）输出一段指标：

  速度跟踪  延迟 / 上升时间 / 稳态偏差 / 稳态波动 / 是否达到 90%
  姿态      pitch−O 均值 / RMS / 峰值 / 超阈值时间
  停车      松手后回到 10% 速度的时间 / 距离 / 速度过冲 / 姿态过冲

数据来自 2 Hz 遥测，因此速度用 0.5 s 位置差分（counts/50ms = Δpos/10），
上升时间分辨率约 0.5 s。需要更细的动力学请先启用 20 Hz 调试帧。
"""
import argparse
import json
import math
import statistics as st
import sys

try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:
    pass


def load(path):
    header = None
    idx = {}
    rows = []
    for line in open(path, encoding="utf-8-sig", errors="replace"):
        line = line.rstrip("\r\n")
        if not line or line.startswith("#"):
            continue
        parts = line.split(",")
        if header is None:
            header = parts
            idx = {n: i for i, n in enumerate(header)}
            continue
        if len(parts) != len(header):
            continue
        rows.append(parts)
    return idx, rows


def col(idx, rows, name, default=0.0):
    i = idx.get(name)
    if i is None:
        return [default] * len(rows)
    out = []
    for r in rows:
        try:
            v = float(r[i])
        except (ValueError, IndexError):
            v = default
        out.append(v if math.isfinite(v) else default)
    return out


def segments(cmd):
    """按方向切分按住段；返回 [(kind, start, end)]，kind = +1 / -1。"""
    out = []
    cur = None
    for i, c in enumerate(cmd):
        s = 1 if c > 0.05 else (-1 if c < -0.05 else 0)
        if s == 0:
            if cur:
                out.append(cur)
                cur = None
            continue
        if cur and cur[0] != s:
            out.append(cur)
            cur = None
        if cur is None:
            cur = [s, i, i]
        else:
            cur[2] = i
    if cur:
        out.append(cur)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--max-speed", type=float, default=5.0,
                    help="满档速度目标 counts/50ms（与 MotionSpeedProfile::maxSpeed 一致；"
                         "2026-11-10 起为 5.0，此前 8.0 —— 分析旧录制要显式 --max-speed 8）")
    ap.add_argument("--json")
    ap.add_argument("--markdown", action="store_true")
    ap.add_argument("--md-out", help="把 markdown 表格写入文件")
    ap.add_argument("--min-hold", type=float, default=1.0, help="忽略短于该时长的按住（秒）")
    ap.add_argument("--fast", help="10Hz 调试帧文件（*.fast.csv）；给了就用它算时域指标")
    ap.add_argument("--offset", type=float, help="平衡点 O（deg）；默认从主 CSV 读取")
    args = ap.parse_args()

    idx, rows = load(args.csv)
    n = len(rows)
    t = [x / 1000.0 for x in col(idx, rows, "ms")]
    o = col(idx, rows, "O_balanceAngleOffset")
    pitch = [x / 100.0 for x in col(idx, rows, "pitch100")]
    gyro = [abs(x) / 100.0 for x in col(idx, rows, "gyro100")]
    pos = [x / 100.0 for x in col(idx, rows, "pos100")]
    cmd = [x / 1000.0 for x in col(idx, rows, "manualReq1000")]
    fin = col(idx, rows, "finalHz")

    # 0.5 s 位置差分 -> counts/50ms
    v = [0.0] * n
    for i in range(1, n):
        dt = t[i] - t[i - 1]
        v[i] = (pos[i] - pos[i - 1]) / dt * 0.05 if dt > 0 else 0.0

    # 10Hz 快速帧优先：0.1 s 分辨率，远好于 2 Hz 主帧
    if args.fast:
        ft, fp, fv, fpi, fcm, ffi = [], [], [], [], [], []
        with open(args.fast, encoding="utf-8-sig") as fh:
            fh.readline()
            for line in fh:
                p = line.strip().split(",")
                if len(p) < 7:
                    continue
                ft.append(int(p[0]) / 1000.0)
                fp.append(int(p[1]) / 100.0)
                fv.append(int(p[2]) / 1000.0)
                fpi.append(int(p[3]) / 100.0)
                fcm.append(int(p[6]) / 1000.0)
                ffi.append(int(p[5]))
        if len(ft) >= 3:
            t, pos, v, pitch, cmd, fin = ft, fp, fv, fpi, fcm, ffi
            n = len(t)
            gyro = [0.0] * n
            o = [args.offset if args.offset is not None else (o[0] if o else 0.0)] * n
            print(f"# 使用 10Hz 快速帧 {args.fast}: {n} 采样, O={o[0]:.2f}")

    results = []
    for sign, a, b in segments(cmd):
        gear = abs(cmd[a])
        vref = gear / 2.0 * args.max_speed
        if vref <= 0:
            continue
        hold = b - a + 1
        if t[b] - t[a] < args.min_hold:
            continue
        # 稳态：按住段的最后 50%
        s0 = a + max(1, hold // 2)
        s1 = b
        steady_v = st.fmean([sign * v[i] for i in range(s0, s1 + 1)]) if s1 >= s0 else 0.0
        steady_dev = [pitch[i] - o[i] for i in range(s0, s1 + 1)]
        dev_all = [pitch[i] - o[i] for i in range(a, b + 1)]
        # 上升：按住开始 -> 首次达到 90% vref（或 10% 用于延迟）
        t10 = t90 = None
        for i in range(a, min(b + 1, n)):
            if t10 is None and sign * v[i] >= 0.1 * vref:
                t10 = t[i] - t[a]
            if t90 is None and sign * v[i] >= 0.9 * vref:
                t90 = t[i] - t[a]
                break
        # 停车：松手后
        stop_t = stop_d = overshoot = None
        dev_overshoot = None
        if b + 1 < n:
            t_rel = t[b]
            p_rel = pos[b]
            for i in range(b + 1, n):
                if abs(sign * v[i]) < 0.1 * vref:
                    stop_t = t[i] - t_rel
                    stop_d = abs(pos[i] - p_rel)
                    break
            window = [i for i in range(b + 1, min(n, b + 7))]
            if window:
                overshoot = max(abs(sign * v[i]) for i in window) - abs(steady_v)
                dev_overshoot = max(abs(pitch[i] - o[i]) for i in window)
        results.append({
            "t0": round(t[a], 1),
            "dir": "前进" if sign > 0 else "后退",
            "gear": round(gear, 2),
            "vref": round(vref, 2),
            "hold_s": round(t[b] - t[a], 1),
            "delay_s": round(t10, 1) if t10 is not None else None,
            "rise_s": round(t90, 1) if t90 is not None else None,
            "v_steady": round(steady_v, 2),
            "err": round(vref - steady_v, 2),
            "err_pct": round((vref - steady_v) / vref * 100, 0) if vref else None,
            "v_sd": round(st.pstdev([v[i] for i in range(s0, s1 + 1)]) if s1 > s0 else 0.0, 2),
            "pitch_mean": round(st.fmean(dev_all), 2),
            "pitch_rms": round(math.sqrt(st.fmean([d * d for d in dev_all])), 2),
            "pitch_peak": round(max((abs(d) for d in dev_all), default=0), 2),
            "gyro_peak": round(max((gyro[i] for i in range(a, b + 1)), default=0), 0),
            "stop_s": round(stop_t, 1) if stop_t is not None else None,
            "stop_counts": round(stop_d, 0) if stop_d is not None else None,
            "v_over": round(overshoot, 2) if overshoot is not None else None,
            "pitch_over": round(dev_overshoot, 2) if dev_overshoot is not None else None,
            "sat": sum(1 for i in range(a, b + 1) if abs(fin[i]) >= 1749.9),
        })

    rows_md = []
    if args.markdown:
        hdr = ("| t0 | 方向 | 档位 | vref | 时长 | 延迟 | 上升 | 稳态v | 误差 | 误差% | 波动 | "
               "姿态均值 | 姿态RMS | 姿态峰值 | 陀螺峰值 | 停车s | 停车counts | 速度过冲 | 姿态过冲 | 打满 |")
        sep = "|" + "---:|" * 20
        rows_md = [hdr, sep]
        for r in results:
            rows_md.append(
                "| {t0} | {dir} | {gear} | {vref} | {hold_s} | {delay_s} | {rise_s} | {v_steady} | "
                "{err} | {err_pct} | {v_sd} | {pitch_mean} | {pitch_rms} | {pitch_peak} | {gyro_peak} | "
                "{stop_s} | {stop_counts} | {v_over} | {pitch_over} | {sat} |".format(**r))
        print("\n".join(rows_md))
        if args.md_out:
            with open(args.md_out, "w", encoding="utf-8") as fh:
                fh.write("\n".join(rows_md) + "\n")
    else:
        print(json.dumps({"csv": args.csv, "max_speed": args.max_speed,
                          "tests": results}, indent=2, ensure_ascii=False))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump({"csv": args.csv, "max_speed": args.max_speed, "tests": results},
                      fh, indent=2, ensure_ascii=False)


if __name__ == "__main__":
    main()




