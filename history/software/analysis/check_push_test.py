#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""推车实验: 手动推车时两路编码器是否一致。
用法: python check_push_test.py <csv> [--tail N]"""
import sys


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
    tail = 60
    if '--tail' in sys.argv:
        tail = int(sys.argv[sys.argv.index('--tail') + 1])
    hdr, rows = load(path)
    ix = {n: i for i, n in enumerate(hdr)}
    g = lambda r, n: float(r[ix[n]]) if n in ix and r[ix[n]] not in ('', None) else 0.0

    print("=" * 96)
    print(f"推车实验  文件={path.split(chr(92))[-1].split('/')[-1]}")
    print(f"数据行={len(rows)}  列数={len(hdr)}")
    print("=" * 96)

    # 逐行差分
    pe1 = pe2 = None
    recs = []
    for r in rows:
        e1, e2 = g(r, 'enc1'), g(r, 'enc2')
        d1 = 0.0 if pe1 is None else e1 - pe1
        d2 = 0.0 if pe2 is None else e2 - pe2
        pe1, pe2 = e1, e2
        recs.append(dict(ms=g(r, 'ms'), e1=e1, e2=e2, d1=d1, d2=d2,
                         m1=g(r, 'm1Hz'), m2=g(r, 'm2Hz'),
                         mode=g(r, 'rawSpeedEncoderMode'),
                         p=g(r, 'pitch100') / 100.0,
                         veh=g(r, 'raw1000') / 1000.0,
                         spd=g(r, 'speed1000') / 1000.0))

    net1, net2 = recs[-1]['e1'] - recs[0]['e1'], recs[-1]['e2'] - recs[0]['e2']
    trav1 = sum(abs(x['d1']) for x in recs)
    trav2 = sum(abs(x['d2']) for x in recs)

    print("\n【1】总量")
    print(f"  净位移   enc1={net1:+.0f}  enc2={net2:+.0f}   比值={net1/net2 if net2 else float('nan'):.3f}")
    print(f"  总行程   enc1={trav1:.0f}   enc2={trav2:.0f}   比值={trav1/trav2 if trav2 else float('nan'):.3f}")
    both = sum(1 for x in recs if x['d1'] and x['d2'])
    only1 = sum(1 for x in recs if x['d1'] and not x['d2'])
    only2 = sum(1 for x in recs if x['d2'] and not x['d1'])
    none = sum(1 for x in recs if not x['d1'] and not x['d2'])
    print(f"  采样分类 双动={both}  仅enc1动={only1}  仅enc2动={only2}  都不动={none}")

    print("\n【2】rawSpeedEncoderMode 分布（新遥测列）")
    from collections import Counter
    c = Counter(int(x['mode']) for x in recs)
    nm = {0: '双路平均', 1: '仅enc1', 2: '仅enc2'}
    for k in sorted(c):
        print(f"  mode {k} {nm.get(k,'?'):<8} {c[k]:>4} 行  {100.0*c[k]/len(recs):5.1f}%")

    print("\n【3】逐行明细（最近 %d 行）" % min(tail, len(recs)))
    print(f"  {'ms':>7} {'enc1':>8} {'enc2':>8} {'Δenc1':>7} {'Δenc2':>7} "
          f"{'Δ1/Δ2':>7} {'m1Hz':>7} {'m2Hz':>7} {'pitch':>6} {'mode':>4}")
    for x in recs[-tail:]:
        r = f"{x['d1']/x['d2']:7.2f}" if x['d2'] else "      -"
        print(f"  {x['ms']:7.0f} {x['e1']:8.0f} {x['e2']:8.0f} {x['d1']:7.0f} {x['d2']:7.0f} "
              f"{r} {x['m1']:7.0f} {x['m2']:7.0f} {x['p']:6.2f} {int(x['mode']):4d}")

    print("\n【4】只在两路都动时比较比值（排除单路跳动）")
    rr = [x['d1'] / x['d2'] for x in recs if x['d1'] and x['d2']]
    if rr:
        rr_s = sorted(rr)
        print(f"  样本={len(rr)}  最小={rr_s[0]:.3f}  中位={rr_s[len(rr_s)//2]:.3f}  最大={rr_s[-1]:.3f}")
        bad = [x for x in recs if x['d1'] and x['d2'] and not (0.8 < x['d1']/x['d2'] < 1.25)]
        print(f"  比值不在 [0.8,1.25] 的行数 = {len(bad)} / {len(rr)}")
        for x in bad[:15]:
            print(f"    ms={x['ms']:.0f} Δenc1={x['d1']:+.0f} Δenc2={x['d2']:+.0f} "
                  f"比值={x['d1']/x['d2']:+.2f} m1={x['m1']:.0f}")
    else:
        print("  没有『两路同时动』的采样 —— 两路从未同时报数")

    print("\n【5】符号一致性（推车方向: 前进应两路同号）")
    same = sum(1 for x in recs if x['d1'] and x['d2'] and (x['d1'] > 0) == (x['d2'] > 0))
    opp = sum(1 for x in recs if x['d1'] and x['d2'] and (x['d1'] > 0) != (x['d2'] > 0))
    print(f"  同号={same}   反号={opp}")
    if opp and not same:
        print("  >>> 两路符号系统相反：encoder2Direction 可能设错")
    elif same and not opp:
        print("  >>> 符号一致，方向定义正确")
    return 0


if __name__ == '__main__':
    sys.exit(main())
