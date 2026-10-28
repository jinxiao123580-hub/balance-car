# -*- coding: utf-8 -*-
# Repository-relative paths; local BLE addresses are supplied via environment.
from pathlib import Path as _ProjectPath
import os as _project_os
_PROJECT_ROOT = _ProjectPath(__file__).resolve().parents[3]

# -*- coding: utf-8 -*-
import csv
F = str(_PROJECT_ROOT / "history" / "data" / "raw" / "ble_diag_log.csv")
rows = list(csv.reader(open(F, newline="", encoding="utf-8")))
h = rows[0]
rows = rows[1:]
idx = {n: i for i, n in enumerate(h)}


def c(name, r):
    i = idx.get(name)
    if i is None:
        return None
    try:
        return float(r[i])
    except Exception:
        return None


print(f"{'t':>6} {'pitch':>7} {'gyro':>7} {'trim':>6} {'tgt':>6} {'spdAng':>6} {'auth':>5} {'finalHz':>7} {'m1Req':>7} {'m1Jmp':>6} {'spd':>7} {'dir':>5}")
for r in rows:
    p = c("pitch100", r)
    print(
        f"{float(r[1]):6.1f} "
        f"{p/100 if p is not None else 0:7.2f} "
        f"{(c('gyro100',r) or 0)/100:7.1f} "
        f"{(c('trim100',r) or 0)/100:6.2f} "
        f"{(c('target100',r) or 0)/100:6.2f} "
        f"{(c('spdAng100',r) or 0)/100:6.2f} "
        f"{(c('auth100',r) or 0):5.0f} "
        f"{(c('finalHz',r) or 0):7.0f} "
        f"{(c('m1Req',r) or 0):7.0f} "
        f"{(c('m1JumpMax',r) or 0):6.0f} "
        f"{(c('speed1000',r) or 0)/1000:7.1f} "
        f"{(c('m1DirChg',r) or 0):5.0f}"
    )
