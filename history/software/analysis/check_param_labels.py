# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
"""检查 ble_tune_log.csv 里参数列的标签与录制过程中的取值变化"""
import csv, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

p = str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_tune_log.csv")
rows = list(csv.DictReader(open(p, encoding='utf-8')))
if not rows:
    print('CSV 空')
    sys.exit(0)
cols = list(rows[0].keys())

# 命令键 -> 参数名 (对齐固件 handleBtCommand)
KEYS = {'P':'balanceKp','D':'balanceKd','O':'balanceAngleOffset','Z':'deadband',
        'L':'limit','S':'slew','K':'speedKp','I':'speedKi','Q':'speedDirectKp',
        'A':'positionDirectKp','N':'maxSpeedAngle','W':'speedIntegralLimit',
        'F':'outerFull','G':'outerDisable','T':'turnKp','U':'turnKd'}
# 找形如 "<键>_<名>" 的参数列
par_cols = [c for c in cols if c[0:1].upper() in KEYS and '_' in c]
print('表头列数:', len(cols))
print('全部列:', cols)
print()
print('疑似参数列:', par_cols)

def num(x):
    try: return float(x)
    except: return None

print('\n--- 各疑似参数列的取值集合(去重): 是否在录制中变化 ---')
changed_warning = False
for c in par_cols:
    uniq = {}
    for r in rows:
        v = r.get(c,'')
        uniq[v] = uniq.get(v,0)+1
    vals = sorted(uniq.keys(), key=lambda x:(num(x) if num(x) is not None else -1e9))
    distinct = len(uniq)
    if distinct > 1:
        changed_warning = True
    # 只打印前几行与出现次数
    disp = ', '.join(f'{v}x{uniq[v]}' for v in vals)
    if len(disp) > 400: disp = disp[:400]+'...'
    print(f'[{c}] distinct={distinct} -> {disp}')

print()
if changed_warning:
    print('>>> 注意: 有参数列取值在录制过程中发生了变化(可能录制中从面板改过)。')
else:
    print('>>> 所有参数列取值全程恒定,即录制期间未再改参。')

# 对比第一行 vs 最后一行的参数值,并对比固件里 O 是否与 target 一致
print('\n--- 首行 vs 末行 (时间, 参数) ---')
first, last = rows[0], rows[-1]
for c in par_cols:
    a, b = first.get(c), last.get(c)
    mark = '  <== 变了' if a != b else ''
    print(f'{c}: {a} -> {b}{mark}')

# target 与 O 平衡点偏差
if 'target' in cols:
    import collections
    tg = collections.Counter(r['target'] for r in rows)
    print('\ntarget(平衡角X100)全程取值分布:', dict(tg))
