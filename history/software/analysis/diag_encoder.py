# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""编码器数据健康诊断: 检查 enc1/enc2 是否正常
覆盖: 范围溢出/跳变/左右一致/方向/与电机命令联动/漂移"""
import csv, sys, io
import statistics as st
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

p = str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log.csv")
rows = list(csv.DictReader(open(p, encoding='utf-8')))
def f(x):
    try: return float(x)
    except: return None
N = len(rows)

def clean(col):
    return [v for v in col if isinstance(v,(int,float))]
def cval(r,k):
    return f(r[k])

e1  = [cval(r,'enc1')    for r in rows]
e2  = [cval(r,'enc2')    for r in rows]
m1  = [cval(r,'m1Hz')    for r in rows]
m2  = [cval(r,'m2Hz')    for r in rows]
pit = [cval(r,'pitch100')/100 if cval(r,'pitch100') is not None else None for r in rows]

E1 = clean(e1); E2 = clean(e2); M1 = clean(m1); M2 = clean(m2)
Pit = [v for v in pit if v is not None]
print(f'总行 {N}')

print('\n[1] 数值可解析性:')
print(f'  enc1 有效值 {len(E1)}/{N}  enc2 有效值 {len(E2)}/{N}')

print('\n[2] 范围检查(计数应平滑有界):')
print(f'  enc1: min={min(E1)} max={max(E1)} span={max(E1)-min(E1)}')
print(f'  enc2: min={min(E2)} max={max(E2)} span={max(E2)-min(E2)}')
print('  |enc|>100000 样本数:', sum(1 for a,b in zip(e1,e2) if isinstance(a,(int,float)) and isinstance(b,(int,float)) and (abs(a)>100000 or abs(b)>100000)))

print('\n[3] 相邻跳变(每行10Hz,正常>50000=跳变/错采):')
def jumps(e):
    j=0; worst=0
    for i in range(1,len(e)):
        if isinstance(e[i],(int,float)) and isinstance(e[i-1],(int,float)):
            d=abs(e[i]-e[i-1])
            if d>50000: j+=1
            worst=max(worst,d)
    return j,worst
j1,w1=jumps(e1); j2,w2=jumps(e2)
print(f'  enc1 跳变={j1} 最大单步差={w1}')
print(f'  enc2 跳变={j2} 最大单步差={w2}')

print('\n[4] 左右一致性 & 方向:')
def cumdiff(e, step=30):
    prev=None; out=[]
    for i in range(0,len(e),step):
        if isinstance(e[i],(int,float)):
            if prev is not None: out.append(e[i]-prev)
            prev=e[i]
    return out
d1=cumdiff(e1); d2=cumdiff(e2)
n=min(len(d1),len(d2))
if n>0:
    agree=sum(1 for i in range(n) if abs(d1[i]-d2[i])<max(20,(abs(d1[i])+abs(d2[i]))*0.3))
    print(f'  左右轮段增量接近占比: {agree/n:.1%} (越高越说明两轮同步,正常平衡车应高)')
    sp1=sum(1 for d in d1[:n] if d>0); sn1=sum(1 for d in d1[:n] if d<0)
    sp2=sum(1 for d in d2[:n] if d>0); sn2=sum(1 for d in d2[:n] if d<0)
    print(f'  enc1 方向: +{sp1} -{sn1}   enc2: +{sp2} -{sn2}')

print('\n[5] "死"编码器检查(全程恒等=没转):')
print(f'  enc1 恒等首值占比 {sum(1 for v in E1 if v==E1[0])/len(E1)*100:.1f}%')
print(f'  enc2 恒等首值占比 {sum(1 for v in E2 if v==E2[0])/len(E2)*100:.1f}%')
print('  两列完全相同:', all(a==b for a,b in zip(E1,E2)))

print('\n[6] 编码器变化与电机命令联动(速度环是否看得到轮转):')
big=[i for i in range(N) if isinstance(m1[i],(int,float)) and abs(m1[i])>5000]
if big:
    i0=big[0]
    a=e1[max(0,i0-10)]; b=e1[min(N-1,i0+10)]
    if isinstance(a,(int,float)) and isinstance(b,(int,float)):
        print(f'  首个|m1Hz|>5000 第{i0}行, 前后20行 enc1 变化={b-a}')
    else:
        print('  (该行附近编码器无效值)')
else:
    print('  无 |m1Hz|>5000 段')

print('\n[7] pitch 最大摆动时编码器是否同步大幅转动:')
if Pit:
    hi=sorted(range(N), key=lambda i: abs(pit[i]) if isinstance(pit[i],(int,float)) else 0)[-50:]
    hiE=[e1[i] for i in hi if isinstance(e1[i],(int,float))]
    print(f'  pitch摆幅最大50行 enc1 有效:{len(hiE)} 范围:{min(hiE)}~{max(hiE)}')

print('\n[8] 编码器量程是否接近int溢出(STM32 int32样式→正常不超过约20亿):')
print('  |enc1|max=%s |enc2|max=%s  (若达2^31/……量级需怀疑溢出)' % (max(abs(x) for x in E1), max(abs(x) for x in E2)))
