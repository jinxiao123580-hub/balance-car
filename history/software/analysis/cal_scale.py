# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""定标前测量: 选 vehicleSpeed 分布决定 WHEEL_TO_HZ_SCALE. 只看新固件段(16:xx 行3592后)."""
import csv, sys, io, statistics as st
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
p = str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log.csv")
rows = list(csv.DictReader(open(p, encoding='utf-8')))
def f(x):
    try: return float(x)
    except: return None
seg = rows[3591:]
def gx(name, div=1.0):
    out=[]
    for r in seg:
        v=f(r[name])
        if v is not None: out.append(v/div)
    return out
spd=gx('speed1000',1000); raw=gx('raw1000',1000)
fin=gx('finalHz'); sd=gx('spdDirHz')
# abs
sa=[abs(x) for x in spd]; ra=[abs(x) for x in raw]; fa=[abs(x) for x in fin]; da=[abs(x) for x in sd]
def dist(a,label):
    a=sorted(a); n=len(a)
    p5=a[int(n*0.05)]; p50=a[int(n*0.5)]; p95=a[int(n*0.95)]
    print(f'  {label}: p5={p5:.2f}  p50={p50:.2f}  p95={p95:.2f}  max={a[-1]:.2f}  均值={sum(a)/n:.2f}')
print('融合速度 vehicleSpeed(abs):'); dist(sa,'speed1000')
print('编码器原始 rawVehicleSpeed(abs):'); dist(ra,'raw1000')
print('直立环 finalHz(abs):'); dist(fa,'final')
print('直投 spdDirHz(abs) 当前Q=5:'); dist(da,'spdDir')
print('\n直投spdDir(current Q=5) / 轮速 speed=Q*5 之比(≈当前无标定系数):', round(sum(da)/max(sum(sa),1e-9),3))
# 需要的放大: 想使 直投 达到 finalHz 的15%时, scale = 0.15*final/ (5*speed)
need = 0.15*sum(fa)/len(fa) / (5* (sum(sa)/len(sa)) )
print(f'\n若想让直投约=finalHz 的15%, 需要 scale≈{need:.1f}')
# clamp 检查: 放大后直投 max = 5*max(speed)*scale
for sc in (10,20,30):
    mx=5*max(sa)*sc
    print(f'scale={sc}: 放大后直投max≈{mx:.0f}Hz (vs MAX4000, final均值{sum(fa)/len(fa):.0f})')
print('\nraw(编码器原始)与speed(融合)差异: 看Kalman是否过度平滑')
diff=[abs(r-f) for (r,f) in zip(raw,spd)]
print(f'  均值|raw-speed|={sum(diff)/len(diff):.2f}')
