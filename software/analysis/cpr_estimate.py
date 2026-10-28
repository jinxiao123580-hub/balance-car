# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[2]

# -*- coding: utf-8 -*-
"""
用「每微步编码器计数」指纹反推编码器每圈计数 CPR。

原理:
    counts_per_microstep = d_enc / (m1Hz * dt)
    在电机轴上这是纯机械量, 与轮径/传动比无关:
        counts_per_microstep = CPR / MPR
其中 MPR = 每圈微步数 (1/32 -> 6400, 1/16 -> 3200 ...)

已知固件: encoder1ISR 挂在 A 相 CHANGE(双边沿) + 读 B 判方向 = 2 倍频
          -> CPR = 2 * 线数
电机铭牌: AB 相 1024 线  -> 若 1024 线 = 1024 脉冲/圈/通道, 则 CPR = 2048

失步只会让 d_enc 偏小 -> 单行比值是 CPR/MPR 的下偏估计,
所以看高分位 (p75/p90) 和取「连续滑动」窗口才接近真值。
"""
import sys, os, glob, statistics

ENC1, ENC2, M1, M2, MS = "enc1", "enc2", "m1Hz", "m2Hz", "ms"


def parse(path):
    """返回 (rows, meta); rows = list of (ms, enc1, enc2, m1, m2)"""
    rows, marks = [], 0
    hdr = None
    with open(path, "r", encoding="utf-8-sig", errors="replace") as f:
        for raw in f:
            line = raw.strip()
            if not line:
                continue
            if line.startswith("#"):
                marks += 1
                continue
            if line.startswith("time_s"):
                hdr = [c.strip() for c in line.split(",")]
                continue
            p = line.split(",")
            if hdr is None or len(p) != len(hdr):
                continue
            try:
                ms = int(float(p[hdr.index(MS)]))
                e1 = int(float(p[hdr.index(ENC1)]))
                e2 = int(float(p[hdr.index(ENC2)]))
                m1 = float(p[hdr.index(M1)])
                m2 = float(p[hdr.index(M2)])
            except (ValueError, IndexError):
                continue
            rows.append((ms, e1, e2, m1, m2))
    return rows, {"header": hdr, "marks": marks}


def report(path):
    rows, meta = parse(path)
    hdr = meta["header"]
    if not hdr or ENC1 not in hdr or M1 not in hdr:
        return                      # 老固件没有编码器/命令频率遥测, 跳过
    name = os.path.basename(path)
    if len(rows) < 20:
        print(f"\n== {name}: 数据行不足 ({len(rows)})")
        return
    print(f"\n{'='*78}")
    print(f"== {name}")
    print(f"   列数={len(hdr)}  数据行={len(rows)}  标记={meta['marks']}")

    per_row_1, per_row_2 = [], []
    agg1 = agg2 = aggcmd1 = aggcmd2 = 0.0
    for (ms0, e10, e20, m10, m20), (ms1, e11, e21, m11, m21) in zip(rows, rows[1:]):
        dt = (ms1 - ms0) / 1000.0
        if dt <= 0 or dt > 2.0:          # 跨段/重连
            continue
        d1, d2 = abs(e11 - e10), abs(e21 - e20)
        # 用区间两端 |f| 的均值近似区间平均命令频率
        f1, f2 = (abs(m10) + abs(m11)) / 2.0, (abs(m20) + abs(m21)) / 2.0
        agg1 += d1; aggcmd1 += f1 * dt
        agg2 += d2; aggcmd2 += f2 * dt
        if f1 >= 2000:
            per_row_1.append(d1 / (f1 * dt))
        if f2 >= 2000:
            per_row_2.append(d2 / (f2 * dt))

    def stat(v, label):
        if not v:
            print(f"   {label}: 无样本")
            return None
        v = sorted(v)
        n = len(v)
        q = lambda p: v[min(n - 1, int(p * n))]
        print(f"   {label}: n={n}  p25={q(.25):.4f}  p50={q(.50):.4f}  "
              f"p75={q(.75):.4f}  p90={q(.90):.4f}  max={v[-1]:.4f}")
        return q(.90)

    print("   -- 单行 每微步计数 (= d_enc / (|f|*dt), 仅 |f|>=2000 的行) --")
    p90_1 = stat(per_row_1, "enc1")
    p90_2 = stat(per_row_2, "enc2")

    print("   -- 总量法 --")
    if aggcmd1 > 0:
        r = agg1 / aggcmd1
        print(f"   enc1: 总行程/m1总微步 = {agg1:.0f}/{aggcmd1:.0f} = {r:.4f}")
    else:
        r = None
    if aggcmd2 > 0:
        r2 = agg2 / aggcmd2
        print(f"   enc2: 总行程/m2总微步 = {agg2:.0f}/{aggcmd2:.0f} = {r2:.4f}")

    print("   -- 反推 CPR (= 比值 * MPR) --")
    for ratio, tag in ((p90_1, "enc1 p90"), (r, "enc1 总量")):
        if ratio is None:
            continue
        print(f"   [{tag} = {ratio:.4f}]  1/32(6400)->CPR={ratio*6400:8.0f}   "
              f"1/16(3200)->CPR={ratio*3200:8.0f}   1/8(1600)->CPR={ratio*1600:8.0f}")

    print("   -- 反向: 若 CPR=2048(1024线x2倍频), 则 MPR = --")
    for ratio, tag in ((p90_1, "enc1 p90"), (r, "enc1 总量")):
        if ratio and ratio > 0:
            print(f"   [{tag}]  MPR = {2048/ratio:9.0f}      (1/32=6400, 1/16=3200)")


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        args = glob.glob(str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log_*.csv"))
    for p in args:
        for f in sorted(glob.glob(p)) if any(c in p for c in "*?") else [p]:
            report(f)
