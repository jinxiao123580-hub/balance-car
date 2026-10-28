#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""打印指定时间段内的每行细节"""
import sys, csv

def load(path):
    rows = []
    with open(path, newline='', encoding='utf-8-sig') as f:
        for row in csv.DictReader(f):
            if row.get('type') != 'D':
                continue
            try:
                rows.append(dict(
                    t=float(row['time_s']),
                    pitch=float(row['pitch100']) / 100,
                    gyro=float(row['gyro100']) / 100,
                    target=float(row['target100']) / 100,
                    bal=float(row['balHz']),
                    spdAng=float(row['spdAng100']) / 100,
                    auth=float(row['auth1000']) / 1000,
                    spdDir=float(row['spdDirHz']),
                    posDir=float(row['posDirHz']),
                    final=float(row['finalHz']),
                    speed=float(row['speed1000']) / 1000,
                    raw=float(row['raw1000']) / 1000,
                    pos=float(row['pos100']) / 100,
                    enc1=int(float(row['enc1'])),
                    enc2=int(float(row['enc2'])),
                    m1=float(row['m1Hz']),
                    m2=float(row['m2Hz']),
                ))
            except Exception:
                pass
    return rows

if __name__ == '__main__':
    path = sys.argv[1]
    t0 = float(sys.argv[2])
    t1 = float(sys.argv[3]) if len(sys.argv) > 3 else t0 + 10
    rows = load(path)
    print('t    pitch gyro  target bal   auth  spdAng spdDir posDir final  speed  raw    pos')
    for r in rows:
        if t0 <= r['t'] <= t1:
            print(f"{r['t']:.1f} {r['pitch']:+6.1f} {r['gyro']:+6.0f} {r['target']:+6.1f} "
                  f"{r['bal']:+6.0f} {r['auth']:.2f} {r['spdAng']:+6.2f} {r['spdDir']:+7.0f} "
                  f"{r['posDir']:+7.0f} {r['final']:+7.0f} {r['speed']:+7.0f} {r['raw']:+7.0f} "
                  f"{r['pos']:+7.0f}")
