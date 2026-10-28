# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""诊断"松手往一边倒": 确认是方向性倾倒还是震荡; 速度环是否在收速度/给修正"""
import csv, sys, io
import statistics as st
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

p = str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log.csv")
rows = list(csv.DictReader(open(p, encoding='utf-8')))
def f(x):
    try: return float(x)
    except: return None

N = len(rows)
pitch = [f(r['pitch100'])/100.0 for r in rows if f(r['pitch100']) is not None]
tgt   = [f(r['target100'])/100.0 for r in rows if f(r['target100']) is not None]
gyro  = [f(r['gyro100'])/100.0 for r in rows if f(r['gyro100']) is not None]
spd   = [f(r['speed1000'])/1000.0 for r in rows if f(r['speed1000']) is not None]
spdAng= [f(r['spdAng100'])/100.0 for r in rows if f(r['spdAng100']) is not None]
spdDir= [f(r['spdDirHz']) for r in rows if f(r['spdDirHz']) is not None]
final = [f(r['finalHz']) for r in rows if f(r['finalHz']) is not None]
bal   = [f(r['balHz']) for r in rows if f(r['balHz']) is not None]

print(f'总行 {N}')
print('\n[1] target(平衡目标角)全程取值:')
print('    分布:', sorted({round(t,1) for t in set(tgt)}))
print('    O参数列最早/最晚值:',
      rows[0].get('O_balanceAngleOffset'), '->', rows[-1].get('O_balanceAngleOffset'))

print('\n[2] pitch 单调倾倒趋势 (车是否一直朝一边偏):')
# 每30行取均值, 看期望方向是否恒定
blk = 60
dirs = []
for i in range(0, N-blk, blk):
    seg = pitch[i:i+blk]
    dirs.append((st.mean(seg[:15]), st.mean(seg[-15:])))  # 段首15 vs 段尾15
mono = sum(1 for a,b in dirs if b - a > 1.0)      # 段内偏正(前倾)增加
mono_neg = sum(1 for a,b in dirs if b - a < -1.0) # 段内偏负(后倾)增加
print(f'    段数={len(dirs)}  段内明显更前倾={mono}  明显更后倾={mono_neg}')
print(f'    pitch 最常停留的范围? 直方: ', end='')
print(f'    min={min(pitch):.1f} max={max(pitch):.1f} mean={st.mean(pitch):.1f}')

print('\n[3] 速度环到底起没起作用:')
print('    车速 vehicleSpeed 均值(应趋0才对)=', round(st.mean([abs(x) for x in spd]),2))
print('    车速 max 绝对值=', round(max(abs(x) for x in spd),2), '  (车在持续移动)')
print('    速度PI目标角修正 spdAng 均值(|.|)=', round(st.mean([abs(x) for x in spdAng]),3), '°  (几乎0)')
print('    速度直达阻尼 spdDirHz 均值(|.|)=', round(st.mean([abs(x) for x in spdDir]),0), 'Hz')
print('    直立环 final 均值(|.|)=', round(st.mean([abs(x) for x in final]),0), 'Hz')
ratio = st.mean([abs(x) for x in spdDir])/max(st.mean([abs(x) for x in final]),1e-6)
print(f'    spdDir 占 final 比例 ≈ {ratio:.1%}   (速度环贡献很小)')
# 速度误差=0-车速; speedAng应由PID输出, 但被N=0.5限幅
print('    N_MAX_SPEED_ANGLE=', rows[0].get('N_MAX_SPEED_ANGLE'), ' (速度PI目标角修正被限在这个±值)')

print('\n[4] 松手瞬间(前~2000行)车倾倒是单调朝一边还是来回:')
pitchearly = pitch[:1500]
signs = [1 if gyro[i]>3 else (-1 if gyro[i]<-3 else 0) for i in range(min(1500,len(gyro)))]
pos_g = sum(1 for s in signs if s>0); neg_g = sum(1 for s in signs if s<0)
print(f'    前1500行: 前倾gyro样本={neg_g}  后倾gyro样本={pos_g}  相对balanced')
