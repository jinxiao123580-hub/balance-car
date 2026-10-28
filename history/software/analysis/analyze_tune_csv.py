# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""分析 ble_tune_log.csv 的运动趋势. 用法: python analyze_tune_csv.py [csv路径]"""
import sys, csv

path = sys.argv[1] if len(sys.argv) > 1 else str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log.csv")

def conv(k):
    try: return int(float(k))
    except: return 0

rows = []
with open(path, encoding="utf-8") as f:
    for r in csv.DictReader(f):
        try:
            rows.append(dict(
                t=float(r["time_s"]),
                pitch=conv(r["pitch100"]) / 100,
                gyro=conv(r["gyro100"]) / 100,
                target=conv(r["target100"]) / 100,
                bal=conv(r["balHz"]),
                spdAng=conv(r["spdAng100"]) / 100,
                auth=conv(r["auth1000"]) / 1000,
                spdDir=conv(r["spdDirHz"]),
                posDir=conv(r["posDirHz"]),
                final=conv(r["finalHz"]),
                speed=conv(r["speed1000"]) / 1000,
                raw=conv(r["raw1000"]) / 1000,
                pos=conv(r["pos100"]) / 100,
                enc1=conv(r["enc1"]),
                m1=conv(r["m1Hz"]),
            ))
        except Exception:
            pass

n = len(rows)
if n == 0:
    print("无有效数据行"); sys.exit()
dur = rows[-1]["t"] - rows[0]["t"]

pitches = [r["pitch"] for r in rows]
finals = [r["final"] for r in rows]
spds = [r["speed"] for r in rows]

print(f"有效行数 {n}  记录时长约 {dur:.1f}s")
print("-" * 60)
print(f"pitch 角度   min={min(pitches):.2f}° max={max(pitches):.2f}° 摆幅={max(pitches)-min(pitches):.2f}°")
print(f"gyro(ABS)   max={max(abs(r['gyro']) for r in rows):.1f}°/s")
print(f"target      恒定 {rows[0]['target']:.1f}° (平衡目标角, 即 O 命令值)")
print(f"speed 轮速   min={min(spds):.3f} max={max(spds):.3f} (相对速度)")
print(f"pos 位置     min={min(r['pos'] for r in rows):.1f} max={max(r['pos'] for r in rows):.1f}")
print(f"enc1 计数    {min(r['enc1'] for r in rows)} ~ {max(r['enc1'] for r in rows)}  净变化 {rows[-1]['enc1']-rows[0]['enc1']}")
print(f"final 输出Hz min={min(finals)} max={max(finals)}")
print(f"m1Hz(ABS)   max={max(abs(r['m1']) for r in rows)}  (电机实际驱动频率)")
print(f"authority   {min(r['auth'] for r in rows):.2f}~{max(r['auth'] for r in rows):.2f}  (1.0=外环全权限/吃满)")
print(f"spdDirHz    {min(r['spdDir'] for r in rows)}~{max(r['spdDir'] for r in rows)}  (速度直达分量)")
print(f"posDirHz    {min(r['posDir'] for r in rows)}~{max(r['posDir'] for r in rows)}  (位置直达=关,应恒0)")
# 末段稳定度
tail = rows[-min(50, n):]
tp = [r["pitch"] for r in tail]
ip = [abs(r["final"]) for r in tail]
print("-" * 60)
print(f"末50行 pitch 摆幅 {max(tp)-min(tp):.2f}°  |final| 均值 {sum(ip)/len(ip):.0f}Hz")
# 判断: 输出是否频繁翻转/饱和
flips = sum(1 for i in range(1, n) if (finals[i] >= 0) != (finals[i-1] >= 0))
print(f"final 符号翻转次数 {flips} / {n} 行  ({flips/max(n,1)*100:.0f}%)")
sat = sum(1 for f in finals if abs(f) >= 10000) / n * 100
print(f"final 接近限幅(|f|>=10000Hz)占比 {sat:.1f}%")
