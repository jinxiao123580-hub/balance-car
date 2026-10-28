# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""分析最新CSV(17:18,新固件×4标定后). 读最后修改的ble_tune_log.csv."""
import csv, sys, io, statistics as st, glob, os
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
p = str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log.csv")
rows = list(csv.DictReader(open(p, encoding='utf-8')))
def f(x):
    try: return float(x)
    except: return None
def gx(name, div=1.0):
    return [v/div for r in rows if (v:=f(r[name])) is not None]

N=len(rows)
ts=[float(r['time_s']) for r in rows]
cols=list(rows[0].keys())
print(f'行数 {N}  时长 {(max(ts)-min(ts))/60:.1f}min  列 {len(cols)}')
print('含R_verticalLoopGain:', 'R_verticalLoopGain' in cols)

def dist(a,label):
    a=sorted(abs(x) for x in a); n=len(a)
    if not a: print(f'  {label}: 空'); return
    print(f'  {label}: p5={a[int(n*.05)]:.2f} p50={a[int(n*.5)]:.2f} p95={a[int(n*.95)]:.2f} max={a[-1]:.2f} 均值abs={sum(a)/n:.2f}')

pit=gx('pitch100',100); gy=gx('gyro100',100)
fin=gx('finalHz'); sd=gx('spdDirHz'); sa=gx('spdAng100',100)
auth=gx('auth1000',1000); spd=gx('speed1000',1000)
print('\n=== 整车 ===')
print(f'pitch 摆幅: {min(pit):.1f}°~{max(pit):.1f}° span {max(pit)-min(pit):.1f}°')
print(f'gyro 峰值 {max(abs(x) for x in gy):.0f} °/s')
dist(fin,'finalHz(直立环)')
dist(sd,'spdDirHz(直投)')
dist(spd,'speed融合值')
fin_abs=sum(abs(x) for x in fin)/len(fin); sd_abs=sum(abs(x) for x in sd)/len(sd)
print(f'\n>>> 直投占比: {sd_abs/max(fin_abs,1e-9):.1%}  (上次旧固件=3.6%, 目标~15-25%)')
print(f'直投spdDir均值| |= {sd_abs:.0f}Hz  (旧固件同段75Hz)')
print(f'spdAng目标角路均值| |= {sum(abs(x) for x in sa)/len(sa):.3f}°  (==0说明速度PI目标角路仍被移除)')
print(f'authority均值 {sum(auth)/len(auth):.2f}  (1.0=速度环全程可介入, 低=常被大角度关断)')
# Q 列实际发送值
q=[f(r['Q_speedDirectKp']) for r in rows if f(r['Q_speedDirectKp']) is not None]
if q: print(f'Q_speedDirectKp 面板发送值集合: {sorted(set(q))}')
rcol=[f(r['R_verticalLoopGain']) for r in rows if f(r['R_verticalLoopGain']) is not None]
if rcol: print(f'R_verticalLoopGain: {sorted(set(rcol))}')
# 振荡检测: final符号翻转
sg=[1 if x>0 else (-1 if x<0 else 0) for x in fin]
flip=sum(1 for i in range(1,len(sg)) if sg[i]!=0 and sg[i]!=sg[i-1])
print(f'\nfinal符号翻转次数 {flip} ({flip/max(len(fin),1)*100:.0f}% 全帧)  (高=来回震荡)')
# 编码器
e1=gx('enc1'); e2=gx('enc2')
print(f'enc1 {min(e1):.0f}~{max(e1):.0f}  enc2 {min(e2):.0f}~{max(e2):.0f}')
# 找直投占比高的时序段看是否稳定
print('\n=== 直投占比随时间(每10%窗口) ===')
step=max(1,N//10)
for i in range(0,N,step):
    seg2=rows[i:i+step]
    f2=[v for v in (f(r['finalHz']) for r in seg2) if v is not None]
    s2=[v for v in (f(r['spdDirHz']) for r in seg2) if v is not None]
    if f2 and s2:
        fa2=sum(abs(x) for x in f2)/len(f2); sa2=sum(abs(x) for x in s2)/len(s2)
        print(f'  {(i)/(N)*100:3.0f}%: 直投占{sa2/max(fa2,1e-9):5.1%}  spdDir={sa2:.0f} forward={fa2:.0f}Hz')
