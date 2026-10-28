# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""分析CSV后半段(新固件 16:xx 录制, 行3592~末)。判速度环为何还感受不到。"""
import csv, sys, io, statistics as st
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')
p = str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log.csv")
rows = list(csv.DictReader(open(p, encoding='utf-8')))
def f(x):
    try: return float(x)
    except: return None
seg = rows[3591:]  # 0-based: 行3592起
print('新段行数:', len(seg))
cols = list(seg[0].keys())
print('含R_verticalLoopGain:', 'R_verticalLoopGain' in cols)
# 参数列
P = [(k,v) for (k,v) in seg[0].items() if k.startswith(('P_','D_','O_','Z_','L_','S_','K_','I_','Q_','A_','N_','W_','F_','G_','T_','U_','R_'))]
print('该段时间内发送的参数快照(首行):')
for k,v in P: print(f'  {k}={v}')

def col(name, div=1.0, only_numeric=None):
    out=[]
    for r in seg:
        v=f(r[name]) if name in r else None
        out.append(v/div if v is not None else None)
    return [x for x in out if x is not None]

pit=col('pitch100',100); gy=col('gyro100',100)
bal=col('balHz'); fin=col('finalHz'); sd=col('spdDirHz'); sa=col('spdAng100',100)
auth=col('auth1000',1000); spd=col('speed1000',1000)
e1=col('enc1'); e2=col('enc2')

print('\n=== 新段(16:xx 新固件)整车 ===')
ts=[float(r["time_s"]) for r in seg]
print(f'时间跨度: {min(ts):.0f}~{max(ts):.0f} = {(max(ts)-min(ts))/60:.1f}min')
print(f'pitch摆幅 {min(pit):.1f}°~{max(pit):.1f}° span {max(pit)-min(pit):.1f}°')
print(f'gyro峰值 {max(abs(x) for x in gy):.0f} °/s')
fin_abs=sum(abs(x) for x in fin)/len(fin); sd_abs=sum(abs(x) for x in sd)/len(sd)
print(f'finalHz均值| |= {fin_abs:.0f}   max| |= {max(abs(x) for x in fin):.0f}')
print(f'spdDirHz直投均值| |= {sd_abs:.0f}   (远端:enc1/enc2末值 {e1[-1]}/{e2[-1]})')
print(f'spdDir占final比例: {sd_abs/max(fin_abs,1e-6):.1%}')
print(f'spdAng目标角路均值| |= {sum(abs(x) for x in sa)/len(sa):.3f}°  (新固件移除此路,应≈0)')
print(f'vehicleSpeed均值| |= {sum(abs(x) for x in spd)/len(spd):.1f}')
print(f'authority均值 {sum(auth)/len(auth):.2f}')

print('\n=== 编码器健康(新段) ===')
print(f'enc1 min/max {min(e1)}/{max(e1)}, enc2 min/max {min(e2)}/{max(e2)}')
def cd(e,step=40):
    pv=None;o=[]
    for i in range(0,len(e),step):
        if pv is not None:o.append(e[i]-pv)
        pv=e[i]
    return o
d1,d2=cd(e1),cd(e2);n=min(len(d1),len(d2))
ag=sum(1 for i in range(n) if abs(d1[i]-d2[i])<max(20,(abs(d1[i])+abs(d2[i]))*0.3))
print(f'左右同步 {ag/max(n,1):.0%}')

# pitch倾翻方向:负=前倾还是? 只看平衡性
print('\n=== pitch 分布 ===')
neg=sum(1 for x in pit if x<0)/len(pit)
print(f'负(一侧)占比 {neg:.0%}, 正占比 {1-neg:.0%}')
