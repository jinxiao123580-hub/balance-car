# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""分析 ble_diag_log.csv：统计 + 关键诊断摘要。"""
import csv
import sys
from collections import deque

F = str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_diag_log.csv")

# 缩放规则
def scale(name):
    if name.endswith("1000"): return 1000
    if name.endswith("100"): return 100
    return 1

rows = []
events = []
header = None
with open(F, newline="", encoding="utf-8") as fh:
    r = csv.reader(fh)
    for line in r:
        if not line: continue
        if header is None:
            header = line
            continue
        if line[0] == "E":
            events.append(line)
            continue
        if line[0] != "D":
            continue
        rows.append(line)

idx = {n: i for i, n in enumerate(header)}
def col(name, row, s=None):
    i = idx.get(name)
    if i is None: return None
    try:
        v = float(row[i])
    except Exception:
        return None
    if s is None: s = scale(name)
    return v / s

n = len(rows)
if n == 0:
    print("no D rows")
    sys.exit()

# 时间
t0, t1 = float(rows[0][1]), float(rows[-1][1])
dur = t1 - t0

pitch = [col("pitch100", r) for r in rows]
gyro  = [col("gyro100", r) for r in rows]
acc   = [col("acc100", r) for r in rows]
trim  = [col("trim100", r) for r in rows]
targ  = [col("target100", r) for r in rows]
spdAng= [col("spdAng100", r) for r in rows]
finalHz=[col("finalHz", r) for r in rows]
balHz = [col("balHz", r) for r in rows]
speed = [col("speed1000", r) for r in rows]
pos   = [col("pos100", r) for r in rows]
auth  = [col("auth100", r) for r in rows]
jmp   = [col("m1JumpMax", r) for r in rows]
stall1= [col("stall1", r) for r in rows]
hold0 = [col("m1Hold0", r) for r in rows]
dirchg= [col("m1DirChg", r) for r in rows]

def stat(x, name):
    if not x: print(f"{name}: empty"); return
    xs = [v for v in x if v is not None]
    if not xs: print(f"{name}: all None"); return
    mn, mx = min(xs), max(xs)
    mean = sum(xs)/len(xs)
    # 标准差
    sd = (sum((v-mean)**2 for v in xs)/len(xs))**0.5
    print(f"  {name:8s} n={len(xs):5d}  min={mn:9.3f}  max={mx:9.3f}  mean={mean:9.3f}  sd={sd:9.3f}")

print(f"=== 总览 ===")
print(f"数据行: {n}  |  时长: {dur:.1f}s  |  采样率约: {n/dur:.2f} 行/s")
print(f"E事件(停机): {len(events)}")

print("\n=== 姿态 ===")
stat(pitch, "pitch°")
stat(gyro, "gyro°/s")
stat(acc, "acc°")

print("\n=== 控制目标 ===")
stat(trim, "trim°")
stat(targ, "target°")
stat(spdAng, "spdAng°")

print("\n=== 输出 ===")
stat(balHz, "balHz")
stat(finalHz, "finalHz")
stat(auth, "auth%")
stat(jmp, "m1JumpMax")
stat(stall1, "stall1")

print("\n=== 运动 ===")
stat(speed, "speed")
stat(pos, "pos counts")

print("\n=== 状态/计数(取最后一行) ===")
last = rows[-1]
for nm in ["imuFailTotal","imuFailNow","imuFailMax","fallCount","lastStop","stopCount",
           "overrunCount","maxLagUs","maxExecUs","dropped","txUsed","m1NaN","m2NaN",
           "m1PwmUpd","m1Hold0","m1DirChg","stall1","stall2"]:
    print(f"  {nm:12s} = {last[idx.get(nm, 0)] if idx.get(nm) is not None else '?'}")

print("\n=== 关键指标 ===")
# 摆幅: pitch 绝对值超过 1°/2° 的占比
abp = [abs(v) for v in pitch if v is not None]
big1 = sum(1 for v in abp if v > 1.0)/len(abp)*100
big2 = sum(1 for v in abp if v > 2.0)/len(abp)*100
big5 = sum(1 for v in abp if v > 5.0)/len(abp)*100
print(f"  |pitch|>1° : {big1:.1f}%   >2°: {big2:.1f}%   >5°: {big5:.1f}%")
# 漂移: pos 起点->终点
p0, p1 = pos[0], pos[-1]
print(f"  位置漂移: {p0:.1f} -> {p1:.1f} counts (Δ={p1-p0:+.1f})")
# 高输出占比
bigHz = sum(1 for v in finalHz if v is not None and abs(v) > 1000)/n*100
print(f"  |finalHz|>1000 : {bigHz:.1f}%")
# 换向速率
print(f"  换向计数变化: 0 -> {dirchg[-1]:.0f} ({(dirchg[-1])/dur:.2f} 次/s)")

print("\n=== E 事件详情 ===")
for e in events[:10]:
    print(f"  t={e[1]} stopReason={e[idx.get('lastStop',0)] if idx.get('lastStop') else '?'}")
if len(events) > 10:
    print(f"  ... 共 {len(events)} 条")
