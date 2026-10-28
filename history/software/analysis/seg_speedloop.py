# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""修正版: 按 K_speedKp 分段, 正确缩放, 检查速度环两通道(spdAng slow-PI目标角 vs spdDirHz fast直达)"""
import csv, sys, io
import statistics as st
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

p = str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log.csv")
rows = list(csv.DictReader(open(p, encoding='utf-8')))

def f(x):
    try: return float(x)
    except: return None

# 列语义(来自 sendTelemetry):
# pitch100=pitch*100 ; gyro100=gyroRate*100 ; target100=targetBalanceAngle*100
# balHz=balanceOutput(直立PD, Hz)
# spdAng100=speedAngleCorrection*100 (速度PI目标角修正, 度)
# auth1000=outerLoopAuthority*1000 (外环权限)
# spdDirHz=speedDirectOutput*speedDirectAuthority (速度直达STEP, Hz) <- 不需要缩放
# posDirHz=positionDirectOutput (Hz)
# finalHz=finalBalanceOutput (Hz)  <- 不需要缩放
# speed1000=vehicleSpeed*1000 (融合速度)
# raw1000=rawVehicleSpeed*1000
# pos100=vehiclePosition*100
# m1Hz/m2Hz=实际电机命令

seg = {}
order = []
for r in rows:
    k = r['K_speedKp']; v = r['I_speedKi']
    key = f'K={k}/I={v}'
    if key not in seg:
        seg[key] = []; order.append(key)
    seg[key].append(r)

def colmean(seg, name, scale=1.0, absc=False):
    vals = [f(r[name]) for r in seg if f(r[name]) is not None]
    return (st.mean([abs(x) for x in vals])*scale if absc else st.mean(vals)*scale) if vals else float('nan')

print('分段  K/I   行数  pitch摆幅  |gyro|峰 |balHz|  |final|  |spdAng|{\u00b0} |spdDirHz| |speed|  authority')
print('-'*100)
for key in order:
    rr = seg[key]
    pitch = [f(x['pitch100'])/100 for x in rr if f(x['pitch100']) is not None]
    gyro = [f(x['gyro100']) for x in rr if f(x['gyro100']) is not None]
    span = (max(pitch)-min(pitch)) if pitch else float('nan')
    gyro_peak = max(abs(x) for x in gyro) if gyro else float('nan')
    bal = colmean(rr,'balHz',absc=True)
    fin = colmean(rr,'finalHz',absc=True)
    sa  = colmean(rr,'spdAng100',0.01,True)   # /100 -> 度
    sd  = colmean(rr,'spdDirHz',absc=True)
    sp  = colmean(rr,'speed1000',0.001,True)  # /1000 -> 融合速度
    au  = colmean(rr,'auth1000',0.001)
    print(f' {key:<14s} {len(rr):>4}  {span:5.1f}°  {gyro_peak:6.0f}°/s {bal:6.0f} {fin:6.0f}   {sa:6.3f}    {sd:6.0f}  {sp:6.3f}  {au:.2f}')

print()
print('解读:')
print('- balHz(直立PD输出)和 finalHz(最终STEP)是同一个数量级且很大 -> 直立环在高速出力।')
print('- spdAng 速度PI目标角修正: 几乎恒0-0.005° -> 速度PI几乎没往目标角上叠加任何东西।')
print('- spdDirHz 速度直达STEP: 数百Hz, 与final同量级 -> 速度直达阻尼是主要快速分量।')
print('- speed 融合速度: 看车是否真的在动。')
