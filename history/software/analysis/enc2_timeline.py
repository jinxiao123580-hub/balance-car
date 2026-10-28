# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""
enc2 健康度时间线。

判据: 平衡车大体走直线, 两轮转过的总行程应该近似相等。
      健康度 = Sigma|d_enc2| / Sigma|d_enc1|
      (总量法, 不受单点噪声影响; 转弯会让该值偏离, 但持续 <0.9 即真故障)
同时给出 每微步计数 比值, 用于反查细分是否变过。
"""
import glob, os, sys

MS, E1, E2, M1, M2, PITCH = "ms", "enc1", "enc2", "m1Hz", "m2Hz", "pitch100"


def load(path):
    hdr = None
    out = []
    for line in open(path, encoding="utf-8-sig", errors="replace"):
        s = line.rstrip("\n").rstrip("\r")
        if not s.strip() or s.startswith("#"):
            continue
        if s.startswith("time_s"):
            hdr = [c.strip() for c in s.split(",")]
            continue
        if hdr is None:
            continue
        p = s.split(",")
        if len(p) != len(hdr):
            continue
        try:
            out.append((int(float(p[hdr.index(MS)])), int(float(p[hdr.index(E1)])),
                        int(float(p[hdr.index(E2)])), float(p[hdr.index(M1)]),
                        float(p[hdr.index(M2)]),
                        float(p[hdr.index(PITCH)]) if PITCH in hdr else 0.0))
        except (ValueError, IndexError):
            continue
    return hdr, out


def q(v, p):
    v = sorted(v)
    return v[min(len(v) - 1, int(p * len(v)))] if v else 0.0


print("%-44s %5s %6s %8s %8s %7s %7s %7s %6s %s" % (
    "file", "rows", "dur_s", "trav_e1", "trav_e2", "e2/e1", "ratio1", "ratio2", "|m1|", "verdict"))
print("-" * 132)
for f in sorted(glob.glob(str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log_*.csv"))):
    hdr, rows = load(f)
    if not hdr or E1 not in hdr or len(rows) < 20:
        continue
    t1 = t2 = c1 = c2 = 0.0
    for (m0, a0, b0, f0, g0, _), (m1, a1, b1, f1, g1, _) in zip(rows, rows[1:]):
        dt = (m1 - m0) / 1000.0
        if dt <= 0 or dt > 2.0:
            continue
        t1 += abs(a1 - a0); t2 += abs(b1 - b0)
        c1 += (abs(f0) + abs(f1)) / 2 * dt
        c2 += (abs(g0) + abs(g1)) / 2 * dt
    dur = (rows[-1][0] - rows[0][0]) / 1000.0
    r1 = t1 / c1 if c1 else 0
    r2 = t2 / c2 if c2 else 0
    h = t2 / t1 if t1 else 0
    mm = sum(abs(r[3]) for r in rows) / len(rows)
    v = "OK" if h > 0.93 else ("DEGRADED" if h > 0.6 else "DEAD")
    print("%-44s %5d %6.1f %8.0f %8.0f %7.3f %7.4f %7.4f %6.0f %s" % (
        os.path.basename(f)[13:], len(rows), dur, t1, t2, h, r1, r2, mm, v))
