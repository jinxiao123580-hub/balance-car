#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""分析18列遥测CSV：站立段/倾角分布/输出饱和/外环权限/失步迹象"""
import sys, csv, math
from collections import defaultdict

def load(path):
    rows = []
    with open(path, newline='', encoding='utf-8-sig') as f:
        r = csv.DictReader(f)
        for row in r:
            if row.get('type') != 'D':
                continue
            try:
                rows.append({
                    't': float(row['time_s']),
                    'pitch': float(row['pitch100']) / 100.0,
                    'gyro': float(row['gyro100']) / 100.0,
                    'target': float(row['target100']) / 100.0,
                    'bal': float(row['balHz']),
                    'spdAng': float(row['spdAng100']) / 100.0,
                    'auth': float(row['auth1000']) / 1000.0,
                    'spdDir': float(row['spdDirHz']),
                    'posDir': float(row['posDirHz']),
                    'final': float(row['finalHz']),
                    'speed': float(row['speed1000']) / 1000.0,
                    'raw': float(row['raw1000']) / 1000.0,
                    'pos': float(row['pos100']) / 100.0,
                    'enc1': int(float(row['enc1'])),
                    'enc2': int(float(row['enc2'])),
                    'm1': float(row['m1Hz']),
                    'm2': float(row['m2Hz']),
                })
            except (KeyError, ValueError) as e:
                pass
    return rows

def main(path):
    rows = load(path)
    if not rows:
        print('NO ROWS')
        return
    n = len(rows)
    t0, t1 = rows[0]['t'], rows[-1]['t']
    print(f'=== {path} ===')
    print(f'行数 {n} | 时长 {t1-t0:.1f}s | 起始t={t0:.1f}s')

    pitches = [r['pitch'] for r in rows]
    finals = [r['final'] for r in rows]
    auths = [r['auth'] for r in rows]

    def pct(lst, fn):
        return 100.0 * sum(1 for v in lst if fn(v)) / len(lst)

    print(f'\n[姿态] pitch min={min(pitches):.1f} max={max(pitches):.1f} '
          f'| >10deg {pct(pitches, lambda v: abs(v)>10):.1f}% | >5deg {pct(pitches, lambda v: abs(v)>5):.1f}% '
          f'| <=2deg {pct(pitches, lambda v: abs(v)<=2):.1f}%')
    print(f'[输出] final min={min(finals):.0f} max={max(finals):.0f} '
          f'| |final|>=12000(饱和) {pct(finals, lambda v: abs(v)>=12000):.1f}% '
          f'| |final|>=6000 {pct(finals, lambda v: abs(v)>=6000):.1f}%')
    print(f'[外环] auth=1.0 {pct(auths, lambda v: v>=0.999):.1f}% | auth<0.2 {pct(auths, lambda v: v<0.2):.1f}%')
    print(f'[死区] |bal|==0 且 |pitch-target|<1.5 {pct(rows, lambda r: abs(r["bal"])<1 and abs(r["pitch"]-r["target"])<1.5):.1f}%')

    # 编码器
    e1 = rows[-1]['enc1'] - rows[0]['enc1']
    e2 = rows[-1]['enc2'] - rows[0]['enc2']
    print(f'[编码器] enc1 Δ{e1:+d} | enc2 Δ{e2:+d} (同号=方向一致)')

    # 站立段：|pitch|<=8 连续>=3行
    segs = []
    cur = []
    for r in rows:
        if abs(r['pitch']) <= 8:
            cur.append(r)
        else:
            if len(cur) >= 3:
                segs.append(cur)
            cur = []
    if len(cur) >= 3:
        segs.append(cur)
    segs.sort(key=len, reverse=True)
    print(f'\n[站立段 |pitch|<=8deg, >=3行] 共{len(segs)}段')
    for i, s in enumerate(segs[:5]):
        dur = s[-1]['t'] - s[0]['t']
        pmax = max(abs(r['pitch']) for r in s)
        pmin = min(abs(r['pitch']) for r in s)
        fmax = max(abs(r['final']) for r in s)
        auth_ok = 100.0 * sum(1 for r in s if r['auth'] >= 0.999) / len(s)
        spd_sat = 100.0 * sum(1 for r in s if abs(r['spdDir']) >= 900) / len(s)
        print(f'  段{i+1}: {s[0]["t"]:.1f}~{s[-1]["t"]:.1f}s dur={dur:.1f}s rows={len(s)} '
              f'|pitch|max={pmax:.1f} auth100={auth_ok:.0f}% spdDir饱和={spd_sat:.0f}% |final|max={fmax:.0f}')

    # 速度测量一致性
    diffs = [abs(r['speed'] - r['raw']) for r in rows]
    big = 100.0 * sum(1 for d in diffs if d > 1.0) / len(diffs)
    print(f'\n[速度] speed-raw 差>1.0 占 {big:.1f}%')

    # 失步迹象：pitch大且|gyro|大但输出饱和
    stall_like = [r for r in rows if abs(r['pitch']) > 15 and abs(r['final']) >= 12000]
    print(f'[失步候选] |pitch|>15 且 |final|>=12000: {len(stall_like)}行 '
          f'({100.0*len(stall_like)/n:.1f}%)')

if __name__ == '__main__':
    p = sys.argv[1] if len(sys.argv) > 1 else 'ble_diag_log.csv'
    main(p)
