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


# 找"手扶停稳"段：|gyro| < 40°/s（人基本捏住不动）
print(f"{'t':>7} {'pitch°':>8} {'gyro°/s':>8} {'finalHz':>8}")
seg = []
for r in rows:
    if r[0] != "D":
        continue
    t = float(r[1])
    p = (c("pitch100", r) or 0) / 100
    g = (c("gyro100", r) or 0) / 100
    f = c("finalHz", r) or 0
    mark = "  <== 停稳" if abs(g) < 40 else ""
    print(f"{t:7.1f} {p:8.2f} {g:8.1f} {f:8.0f}{mark}")

# 统计停稳段的 pitch 分布
steady = [
    (c("pitch100", r) or 0) / 100
    for r in rows
    if r[0] == "D" and abs((c("gyro100", r) or 0)) < 40
]
if steady:
    print(f"\n停稳段 pitch: n={len(steady)} min={min(steady):.2f} max={max(steady):.2f} "
          f"mean={sum(steady)/len(steady):.2f}")
