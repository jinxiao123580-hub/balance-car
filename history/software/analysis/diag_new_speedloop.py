# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""最新CSV专项诊断: 新固件(直立环0.7+速度环直投)后速度环到底出没出力
核心: spdDirHz(直投)占比、spdAng(目标角路,应≈0)、R参数、编码器健康"""
import csv, sys, io
import statistics as st
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

p = str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log.csv")
rows = list(csv.DictReader(open(p, encoding='utf-8')))
def f(x):
    try: return float(x)
    except: return None
def clean(col): return [v for v in col if isinstance(v,(int,float))]

N = len(rows)
# 是否存在新列 R_verticalLoopGain
cols = list(rows[0].keys())
has_R = any('R_' in c for c in cols) or 'R_verticalLoopGain' in cols
print('列数:', len(cols), ' 是否含 R_verticalLoopGain:', 'R_verticalLoopGain' in cols)

# R 参数列取值(若含)
rcol = [c for c in cols if c.startswith('R_')]
if rcol:
    rv = clean([f(r[rcol[0]]) for r in rows])
    print('R(直立环增益) 取值分布:', sorted(set(rv)))

# 各关键列
pit = [f(r['pitch100'])/100 if f(r['pitch100']) is not None else None for r in rows]
gyro= [f(r['gyro100'])/100 if f(r['gyro100']) is not None else None for r in rows]
bal = [f(r['balHz']) for r in rows]
fin = [f(r['finalHz']) for r in rows]
spdDir=[f(r['spdDirHz']) for r in rows]
spdAng=[f(r['spdAng100'])/100 if f(r['spdAng100']) is not None else None for r in rows]
auth=[f(r['auth1000'])/1000 if f(r['auth1000']) is not None else None for r in rows]
spd =clean([f(r['speed1000'])/1000 if f(r['speed1000']) is not None else None for r in rows])
e1=[f(r['enc1']) for r in rows]; e2=[f(r['enc2']) for r in rows]
Pit=[v for v in pit if v is not None]; Auth=[v for v in auth if v is not None]
Bal=clean(bal); Fin=clean(fin); SpdDir=clean(spdDir)
SpdAng=[v for v in spdAng if v is not None]
E1=clean(e1); E2=clean(e2)

print('\n=== 整车状态 ===')
print(f'pitch 摆幅: {min(Pit):.1f}° ~ {max(Pit):.1f}° (span {max(Pit)-min(Pit):.1f}°)')
print(f'gyro 峰值: {max(abs(v) for v in filter(None,gyro)):.1f} °/s')
print(f'finalHz 最大| |: {max(abs(v) for v in Fin):.0f}  均值| |: {sum(abs(v) for v in Fin)/len(Fin):.0f}')
print(f'authority 均值: {sum(Auth)/len(Auth):.2f}')

print('\n=== 速度环到底出没出力 ===')
sd_abs = sum(abs(v) for v in SpdDir)/len(SpdDir)
fin_abs = sum(abs(v) for v in Fin)/len(Fin)
print(f'spdDirHz 直投: 均值| |= {sd_abs:.0f} Hz  最大| |= {max(abs(v) for v in SpdDir):.0f}')
print(f'spdAng 目标角修正: 均值| |= {sum(abs(v) for v in SpdAng)/len(SpdAng):.3f}°  (新固件应≈0,因为去掉了这条路)')
print(f'spdDir 占 final 比例: {sd_abs/max(fin_abs,1e-6):.1%}')
print(f'车速 vehicleSpeed 均值| |= {sum(abs(v) for v in spd)/len(spd):.2f}  (应趋0)')
print(f'R(直立环增益)越线: 若=0.7已被削弱, balanceKp列应还在900')

print('\n=== 编码器健康(抽查) ===')
print(f'enc1: min={min(E1)} max={max(E1)}   enc2: min={min(E2)} max={max(E2)}')
print(f'enc1 恒等首值占比 {sum(1 for v in E1 if v==E1[0])/max(len(E1),1)*100:.1f}%')
print(f'左右完全同步占比:', end=' ')
# 段增量一致性
def cumdiff(e,step=40):
    prev=None;out=[]
    for i in range(0,len(e),step):
        if isinstance(e[i],(int,float)):
            if prev is not None: out.append(e[i]-prev)
            prev=e[i]
    return out
d1,d2=cumdiff(e1),cumdiff(e2); n=min(len(d1),len(d2))
agree=sum(1 for i in range(n) if abs(d1[i]-d2[i])<max(20,(abs(d1[i])+abs(d2[i]))*0.3))
print(f'{agree/max(n,1):.1%}')

print('\n=== 按 R 分段 (若有 R 变化则看差异) ===')
if rcol:
    seg={}
    for r in rows:
        v=f(r[rcol[0]])
        pitch=f(r['pitch100'])/100 if f(r['pitch100']) is not None else None
        sd=f(r['spdDirHz']); fi=f(r['finalHz'])
        key=str(v)
        seg.setdefault(key,[]).append((pitch,sd,fi))
    for k,v in seg.items():
        p=[x[0] for x in v if x[0] is not None]
        sd=[x[1] for x in v if isinstance(x[1],(int,float))]
        fi=[x[2] for x in v if isinstance(x[2],(int,float))]
        if p:
            print(f'  R={k}: 行数{len(v)} pitch摆幅={max(p)-min(p):.1f}° spdDir均值| |={sum(abs(x) for x in sd)/max(len(sd),1):.0f} final均值| |={sum(abs(x) for x in fi)/max(len(fi),1):.0f}')
