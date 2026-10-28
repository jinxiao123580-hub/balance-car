#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分档行程比 + enc2 冻结事件检测。用法: python binned_ratio.py <csv>"""
import sys
from collections import Counter


def load(path):
    header, rows = None, []
    with open(path, encoding='utf-8', errors='replace') as f:
        for raw in f:
            line = raw.rstrip('\n').rstrip('\r')
            if not line:
                continue
            if line.startswith('time_s'):
                if header is None:
                    header = line.split(',')
                continue
            if line.startswith('#'):
                continue
            p = line.split(',')
            try:
                float(p[0])
            except ValueError:
                continue
            rows.append(p)
    return header, rows


def main():
    path = sys.argv[1]
    hdr, rows = load(path)
    ix = {n: i for i, n in enumerate(hdr)}
    g = lambda r, n: (float(r[ix[n]]) if n in ix and r[ix[n]] not in ('', None) else 0.0)

    ms = [g(r, 'ms') for r in rows]
    e1 = [g(r, 'enc1') for r in rows]
    e2 = [g(r, 'enc2') for r in rows]
    md = [int(g(r, 'rawSpeedEncoderMode')) for r in rows]
    m1 = [g(r, 'm1Hz') for r in rows]
    t0, t1 = ms[0], ms[-1]
    print("=" * 88)
    print(f"文件={path.replace(chr(92),'/').split('/')[-1]}  行={len(rows)}  ms {t0:.0f}->{t1:.0f} ({(t1-t0)/1000:.1f}s)")
    print("=" * 88)

    # --- 分档(每4秒)行程 ---
    print("\n【1】每 4 秒分档：行程比 = 总行程enc1 / 总行程enc2（理想=1.00）")
    print(f"  {'起始ms':>8} {'行':>4} {'行程enc1':>9} {'行程enc2':>9} {'比值':>7}  {'mode分布':<16} 判读")
    BIN = 4000.0
    b0 = ms[0]
    i = 0
    while i < len(rows):
        j = i
        while j + 1 < len(rows) and ms[j + 1] - b0 < BIN:
            j += 1
        s1 = s2 = 0.0
        for k in range(i + 1, j + 1):
            s1 += abs(e1[k] - e1[k - 1])
            s2 += abs(e2[k] - e2[k - 1])
        mc = Counter(md[i:j + 1])
        ratio = (s1 / s2) if s2 else float('nan')
        note = ""
        if s2 == 0:
            note = "enc2 完全无计数"
        elif ratio > 1.5:
            note = "enc2 少算很多"
        elif ratio > 1.15:
            note = "enc2 偏少"
        elif 0.85 <= ratio <= 1.15:
            note = "一致"
        else:
            note = "enc1 偏少"
        mds = " ".join(f"{k}:{v}" for k, v in sorted(mc.items()))
        print(f"  {ms[i]:8.0f} {j-i+1:4d} {s1:9.0f} {s2:9.0f} {ratio:7.2f}  {mds:<16} {note}")
        i = j + 1
        if i < len(rows):
            b0 = ms[i]

    # --- enc2 冻结事件 ---
    print("\n【2】enc2 冻结事件（连续 >=2 行 enc2 计数完全不变，而 enc1 在动）")
    print("    这是『丢脉冲』的直接指纹：另一路在计数，这一路一动不动")
    runs = []
    i = 1
    while i < len(rows):
        if e2[i] == e2[i - 1]:
            j = i
            while j + 1 < len(rows) and e2[j + 1] == e2[j]:
                j += 1
            n = j - i + 2
            if n >= 2:
                move1 = sum(abs(e1[k] - e1[k - 1]) for k in range(i, j + 1))
                runs.append((ms[i - 1], ms[j], n, e2[i], move1, md[i]))
            i = j + 1
        else:
            i += 1
    if not runs:
        print("  无")
    else:
        print(f"  {'起始ms':>8} {'结束ms':>8} {'行数':>4} {'enc2值':>8} {'同期enc1行程':>12} {'mode':>4}")
        for a, b, n, v, mv, m in runs:
            flag = "  <<< enc1在动!" if mv > 20 else ""
            print(f"  {a:8.0f} {b:8.0f} {n:4d} {v:8.0f} {mv:12.0f} {m:4d}{flag}")
        tot = sum(r[2] for r in runs)
        print(f"  合计 {len(runs)} 段 / {tot} 行被冻结 ({100.0*tot/len(rows):.1f}%)")

    # --- enc1 冻结事件 ---
    runs1 = []
    i = 1
    while i < len(rows):
        if e1[i] == e1[i - 1]:
            j = i
            while j + 1 < len(rows) and e1[j + 1] == e1[j]:
                j += 1
            n = j - i + 2
            if n >= 2:
                mv = sum(abs(e2[k] - e2[k - 1]) for k in range(i, j + 1))
                runs1.append((ms[i - 1], ms[j], n, mv))
            i = j + 1
        else:
            i += 1
    print(f"\n【3】enc1 冻结事件（对照）: {len(runs1)} 段, "
          f"合计 {sum(r[2] for r in runs1)} 行 ({100.0*sum(r[2] for r in runs1)/len(rows):.1f}%)")

    # --- 谁多谁少 ---
    print("\n【4】逐行方向性统计（两路都动的行）")
    big1 = big2 = eq = 0
    for k in range(1, len(rows)):
        d1 = abs(e1[k] - e1[k - 1])
        d2 = abs(e2[k] - e2[k - 1])
        if d1 and d2:
            if d1 > d2 * 1.15:
                big1 += 1
            elif d2 > d1 * 1.15:
                big2 += 1
            else:
                eq += 1
    print(f"  |Δenc1| 明显更大: {big1} 行")
    print(f"  |Δenc2| 明显更大: {big2} 行")
    print(f"  两者相当(±15%内): {eq} 行")
    if big1 > big2 * 2:
        print("  >>> 系统性偏向 enc1 多计数 => enc2 掉脉冲（而非车在转向）")
    elif abs(big1 - big2) <= max(2, 0.25 * (big1 + big2)):
        print("  >>> 双向对称 => 更像车在被推着转向，而不是单路故障")
    return 0


if __name__ == '__main__':
    sys.exit(main())
