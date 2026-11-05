#!/usr/bin/env python3
"""List forward/backward command segments in a BLE telemetry CSV and their balance impact.

Usage: python software/analysis/analyze_fwdback_motion.py <csv> [--window t0 t1]
Segments are split by manualReq1000 (MOVE frame) and jogActive (position jog).
Read-only; no firmware or panel changes.
"""
import argparse
import math
import statistics as st


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


def g(row, idx, name, default=0.0):
    i = idx.get(name)
    if i is None or i >= len(row):
        return default
    try:
        v = float(row[i])
    except ValueError:
        return default
    return v if math.isfinite(v) else default


def summarize(seg, idx):
    pitch = [g(r, idx, "pitch100") / 100 for r in seg]
    off = st.median([g(r, idx, "O_balanceAngleOffset") for r in seg])
    fin = [g(r, idx, "finalHz") for r in seg]
    spd = [g(r, idx, "speed1000") / 1000 for r in seg]
    pos = [g(r, idx, "pos100") / 100 for r in seg]
    cmd = [g(r, idx, "manualReq1000") / 2000 for r in seg]
    lean = [g(r, idx, "manualApplied1000") / 1000 for r in seg]
    return {
        "n": len(seg),
        "cmd_range": (round(min(cmd), 2), round(max(cmd), 2)),
        "applied_peak_deg": round(max((abs(v) for v in lean), default=0), 3),
        "pitch_mean": round(st.fmean(pitch), 2),
        "pitch_dev": (round(min(pitch) - off, 2), round(max(pitch) - off, 2)),
        "speed_peak": round(max((abs(v) for v in spd), default=0), 2),
        "out_peak_hz": round(max((abs(v) for v in fin), default=0), 0),
        "out_saturated": sum(1 for v in fin if abs(v) >= 1749.9),
        "pos_net": round(pos[-1] - pos[0], 1),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--window", nargs=2, type=float, metavar=("T0", "T1"))
    args = ap.parse_args()
    idx, rows = load(args.csv)
    t = [g(r, idx, "ms") / 1000.0 for r in rows]
    print(f"rows={len(rows)} columns={len(idx)} span={t[0]:.1f}..{t[-1]:.1f}s")

    if args.window:
        lo, hi = args.window
        sel = [r for r, tt in zip(rows, t) if lo <= tt <= hi]
        print(f"window {lo}..{hi}s:", summarize(sel, idx))
        return

    groups, cur, kind, gid = [], [], None, None
    for r, tt in zip(rows, t):
        active = g(r, idx, "jogActive") > 0.5
        manual = abs(g(r, idx, "manualReq1000")) > 0.5
        k = "jog" if active else ("manual" if manual else None)
        if k is None:
            if cur:
                groups.append((kind, gid, cur, t0, t1))
            cur, kind, gid = [], None, None
            continue
        if k != kind:
            if cur:
                groups.append((kind, gid, cur, t0, t1))
            cur, kind = [], k
        t0 = t0 if cur else tt
        t1 = tt
        cur.append(r)
        gid = g(r, idx, "jogSequence")
    if cur:
        groups.append((kind, gid, cur, t0, t1))

    print(f"{'kind':7} {'t0':>8} {'t1':>8} {'n':>3} {'cmd':>12} {'lean°':>6} "
          f"{'pitch_dev':>14} {'spd':>5} {'outHz':>6} {'sat':>4} {'posΔ':>7}")
    for kind, gid, seg, a, b in groups:
        s = summarize(seg, idx)
        print(f"{kind:7} {a:8.1f} {b:8.1f} {s['n']:3d} {str(s['cmd_range']):>12} "
              f"{s['applied_peak_deg']:6.3f} {str(s['pitch_dev']):>14} {s['speed_peak']:5.1f} "
              f"{s['out_peak_hz']:6.0f} {s['out_saturated']:4d} {s['pos_net']:7.1f}")


if __name__ == "__main__":
    main()
