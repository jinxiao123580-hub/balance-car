#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
BLE 调参日志分析器（pinghengche）

用法:
    python analyze_ble_log.py <csv> [--timeline-sec 1] [--frozen-counts 1] [--min-hz 500]

遵守交接文档 §2.5 的分析规矩:
  1. 找 ms 重置点
  2. 找所有参数变化节点
  3. 按节点分段
  4. 区分人工推车/手扶 与 自主运行

日志格式 (由 ble_tune_panel.py 落盘):
  - 表头行: 以 time_s 开头
  - 数据行: 以 unix 时间戳开头, 例如 1789635010.995,...
  - 标记行: # --- reconnect new session @ HH:MM:SS ---
"""

import sys
import argparse
from statistics import median

TELEM = [
    'time_s', 'ms', 'pitch100', 'gyro100', 'target100', 'balPHz', 'balDHz', 'balHz',
    'spdAng100', 'auth1000', 'spdDirRawHz', 'spdDirHz', 'posSpdTarget1000', 'mixHz',
    'limitedHz', 'finalHz', 'turnHz', 'speed1000', 'raw1000', 'speedAccel1000',
    'predSpeed1000', 'predDelta1000', 'spdDirUnclampedHz', 'pos100', 'posError100',
    'enc1', 'enc2', 'm1Hz', 'm2Hz', 'accPitch100', 'kalmanBias1000', 'gyroIntegAngle100',
    'balAccelRawHzStep', 'balAccelLimitedHzStep', 'integratedSpeedHz', 'speedError1000',
    'speedP100', 'speedI100', 'speedAngleUnclamped100', 'speedAngleLimited100',
    'balAccelEffectiveLimitHzStep',
]
PARAM_SHORT = {
    'P_balanceKp': 'P', 'D_balanceKd': 'D', 'O_balanceAngleOffset': 'O',
    'Z_deadband': 'Z', 'L_limitHz': 'L', 'S_maxAccelIncrement': 'S',
    'B_highSpeedAccelStartHz': 'B', 'H_highSpeedAccelLimitHz': 'H',
    'R_balanceAccelScale': 'R', 'K_speedKp': 'K', 'I_speedKi': 'I',
    'N_MAX_SPEED_ANGLE': 'N', 'W_SPEED_INTEGRAL_LIMIT': 'W',
    'J_speedErrorDeadband': 'J', 'E_speedKd': 'E', 'X_positionKp': 'X',
    'Y_maxPositionReturnSpeed': 'Y', 'M_positionDeadband': 'M',
    'F_OUTER_FULL_ANGLE': 'F', 'G_OUTER_DISABLE_ANGLE': 'G',
    'T_turnKp': 'T', 'U_turnKd': 'U',
}


def load(path):
    with open(path, 'r', encoding='utf-8', errors='replace') as fh:
        raw = [ln.rstrip('\r\n') for ln in fh]

    header, rows, markers = None, [], []
    for i, ln in enumerate(raw):
        s = ln.strip()
        if not s:
            continue
        if s.startswith('#'):
            markers.append((i, s))
            continue
        if s.startswith('time_s'):
            if header is None:
                header = [c.strip() for c in s.split(',')]
            continue
        if s[0].isdigit() or s[0] in '+-':
            parts = s.split(',')
            d = {'_line': i}
            for j, name in enumerate(header):
                d[name] = parts[j].strip() if j < len(parts) else ''
            rows.append(d)
    return header, rows, markers


def num(row, key):
    v = row.get(key, '')
    if v == '':
        return None
    try:
        return float(v)
    except ValueError:
        return None


def pct(v, q):
    if not v:
        return 0.0
    s = sorted(v)
    return s[min(len(s) - 1, int(q * len(s)))]


def f0(v):
    return 'None' if v is None else f'{v:.0f}'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv')
    ap.add_argument('--timeline-sec', type=float, default=1.0)
    ap.add_argument('--frozen-counts', type=float, default=1.0,
                    help='判定编码器“冻住”的每采样周期计数阈值')
    ap.add_argument('--min-hz', type=float, default=500.0,
                    help='判定“电机被命令转动”的最小 |m1Hz|')
    ap.add_argument('--window', type=str, default='',
                    help='全分辨率打印该 ms 区间, 例 --window 16000,19000')
    a = ap.parse_args()

    header, rows, markers = load(a.csv)
    if not header or not rows:
        print('!! 没有解析到表头或数据行')
        return 1

    ncol_t = len(TELEM)
    param_cols = [c for c in header if c in PARAM_SHORT]

    print('=' * 78)
    print(f'文件: {a.csv}')
    print(f'表头 {len(header)} 列 (遥测 {ncol_t} + 参数 {len(param_cols)})   '
          f'数据 {len(rows)} 行   标记 {len(markers)} 行')
    missing = [c for c in TELEM if c not in header]
    if missing:
        print(f'!! 表头缺少遥测列: {missing}')
    has_new = 'balAccelEffectiveLimitHzStep' in header
    print(f'新遥测列 balAccelEffectiveLimitHzStep 在位: {has_new}')

    # ---------- 1. 分段: 标记行 + ms 重置 ----------
    print('\n' + '=' * 78)
    print('【1】会话分段 (标记行 / ms 重置)')
    print('=' * 78)
    for ln, txt in markers:
        print(f'  标记 @ 文件行 {ln}: {txt}')

    segs, cur, prev_ms = [], [], None
    marker_lines = sorted(ln for ln, _ in markers)
    for r in rows:
        m = num(r, 'ms')
        new_seg = False
        if prev_ms is not None and m is not None and m < prev_ms:
            new_seg = True
        if cur and any(cur[-1]['_line'] < ln < r['_line'] for ln in marker_lines):
            new_seg = True
        if new_seg and cur:
            segs.append(cur)
            cur = []
        cur.append(r)
        prev_ms = m
    if cur:
        segs.append(cur)

    print(f'\n  共 {len(segs)} 段:')
    print(f'  {"段":>3} {"行数":>5} {"ms起":>8} {"ms止":>8} {"时长s":>7} {"ms步长":>7}')
    for i, s in enumerate(segs):
        m0, m1 = num(s[0], 'ms'), num(s[-1], 'ms')
        steps = [num(s[k], 'ms') - num(s[k - 1], 'ms') for k in range(1, len(s))]
        st = median(steps) if steps else 0
        print(f'  {i:>3} {len(s):>5} {m0:>8.0f} {m1:>8.0f} {(m1 - m0) / 1000:>7.1f} {st:>7.0f}')

    # ---------- 2. 参数变化节点 ----------
    print('\n' + '=' * 78)
    print('【2】参数变化节点')
    print('=' * 78)
    print('  起始值: ' + ' '.join(
        f'{PARAM_SHORT.get(c, c)}={rows[0].get(c, "?")}' for c in param_cols))
    changes = []
    for c in param_cols:
        last = rows[0].get(c)
        for r in rows[1:]:
            v = r.get(c)
            if v != last:
                changes.append((num(r, 'ms'), r['_line'], PARAM_SHORT.get(c, c), last, v))
                last = v
    if not changes:
        print('  全程无参数变化 —— 整份日志是同一组参数下的单一工况。')
    else:
        print(f'  共 {len(changes)} 次变化:')
        for ms, ln, name, o, n in changes:
            print(f'    ms={ms:>7.0f} (文件行 {ln:>4})  {name}: {o} -> {n}')

    # ---------- 3. 编码器运动尺度 ----------
    print('\n' + '=' * 78)
    print('【3】编码器运动尺度 / 指令-运动比')
    print('=' * 78)
    denc1, denc2, ratio = [], [], []
    for k in range(1, len(rows)):
        p, r = rows[k - 1], rows[k]
        dt = (num(r, 'ms') - num(p, 'ms')) / 1000.0
        if dt <= 0:
            continue
        e1a, e1b = num(p, 'enc1'), num(r, 'enc1')
        e2a, e2b = num(p, 'enc2'), num(r, 'enc2')
        if None in (e1a, e1b, e2a, e2b):
            continue
        denc1.append(abs(e1b - e1a))
        denc2.append(abs(e2b - e2a))
        m = num(r, 'm1Hz')
        if m is not None and abs(m) >= a.min_hz:
            ratio.append((abs(e1b - e1a) / dt) / abs(m))

    print(f'  |Δenc1|/采样 分位: p10={pct(denc1,.1):.0f} p50={pct(denc1,.5):.0f} '
          f'p90={pct(denc1,.9):.0f} max={max(denc1) if denc1 else 0:.0f}')
    print(f'  |Δenc2|/采样 分位: p10={pct(denc2,.1):.0f} p50={pct(denc2,.5):.0f} '
          f'p90={pct(denc2,.9):.0f} max={max(denc2) if denc2 else 0:.0f}')
    if ratio:
        print(f'  |enc1速率| / |m1Hz|  (仅 |m1Hz|>={a.min_hz:.0f}, n={len(ratio)}):')
        print(f'    p10={pct(ratio,.1):.4f} p50={pct(ratio,.5):.4f} p90={pct(ratio,.9):.4f}')
        print('    ^ 每个 STEP 脉冲对应的编码器计数 = 细分级数 × 编码器分辨率 的乘积。')
        print('      偏低说明电机在空转/失步, 偏高说明编码器动而指令小。')

    # ---------- 4. 锁定区间 ----------
    print('\n' + '=' * 78)
    print(f'【4】锁定区间 (|Δenc| <= {a.frozen_counts:.0f} 计数 且 |m1Hz| >= {a.min_hz:.0f})')
    print('=' * 78)
    eps, cur_ep = [], None
    for k in range(1, len(rows)):
        p, r = rows[k - 1], rows[k]
        e1a, e1b = num(p, 'enc1'), num(r, 'enc1')
        e2a, e2b = num(p, 'enc2'), num(r, 'enc2')
        m = num(r, 'm1Hz')
        if None in (e1a, e1b, e2a, e2b) or m is None:
            continue
        frozen = (abs(e1b - e1a) <= a.frozen_counts and abs(e2b - e2a) <= a.frozen_counts)
        if frozen and abs(m) >= a.min_hz:
            cur_ep = [k, k] if cur_ep is None else [cur_ep[0], k]
        elif cur_ep is not None:
            eps.append(cur_ep)
            cur_ep = None
    if cur_ep is not None:
        eps.append(cur_ep)

    if not eps:
        print('  未检出“命令在转但编码器不动”的区间。')
    else:
        print(f'  共 {len(eps)} 个区间:')
        for i, (k0, k1) in enumerate(eps):
            w = rows[k0 - 1:k1 + 1]

            def rng(key):
                vs = [num(x, key) for x in w]
                vs = [v for v in vs if v is not None]
                return (min(vs), max(vs)) if vs else (0.0, 0.0)

            lo, hi = rng('m1Hz')
            ilo, ihi = rng('integratedSpeedHz')
            rlo, rhi = rng('raw1000')
            plo, phi = rng('pitch100')
            prot = [v for v in (num(x, 'balAccelEffectiveLimitHzStep') for x in w) if v is not None]
            print(f'  [{i}] ms {num(w[0],"ms"):.0f} -> {num(w[-1],"ms"):.0f}'
                  f'  ({len(w)} 采样, {(num(w[-1],"ms")-num(w[0],"ms"))/1000:.1f} s)')
            print(f'        m1Hz {lo:.0f}..{hi:.0f}    integratedSpeedHz {ilo:.0f}..{ihi:.0f}')
            print(f'        raw1000 {rlo:.0f}..{rhi:.0f}    pitch100 {plo:.0f}..{phi:.0f}')
            print(f'        保护限幅取值集合 {sorted(set(prot))}')

    # ---------- 5. 高速保护命中统计 ----------
    print('\n' + '=' * 78)
    print('【5】高速同向加速保护 (balAccelEffectiveLimitHzStep) 命中统计')
    print('=' * 78)
    S = float(rows[0].get('S_maxAccelIncrement', 0) or 0)
    H = float(rows[0].get('H_highSpeedAccelLimitHz', 0) or 0)
    has_prot = 'balAccelEffectiveLimitHzStep' in header
    if not has_prot:
        print('  !! 该文件是旧格式, 没有 balAccelEffectiveLimitHzStep 列, 本节跳过')
    else:
        print(f'  固件参数: S={S:.0f}  H={H:.0f}  '
              f'B={float(rows[0].get("B_highSpeedAccelStartHz", 0) or 0):.0f}  '
              f'L={float(rows[0].get("L_limitHz", 0) or 0):.0f}  '
              f'R={float(rows[0].get("R_balanceAccelScale", 0) or 0):.4f}')
        limit_vals = {}
        for r in rows:
            v = num(r, 'balAccelEffectiveLimitHzStep')
            limit_vals[v] = limit_vals.get(v, 0) + 1
        for v in sorted(limit_vals, key=lambda x: (x is None, x)):
            tag = ''
            if v == S:
                tag = '  <- = S (常规)'
            elif v == H:
                tag = '  <- = H (保护命中)'
            print(f'    limit={f0(v):>5}: {limit_vals[v]:>4} 次{tag}')
        prot_trans, prev = [], None
        for r in rows:
            v = num(r, 'balAccelEffectiveLimitHzStep')
            if prev is not None and v != prev:
                prot_trans.append((num(r, 'ms'), prev, v))
            prev = v
        print(f'  切换事件 {len(prot_trans)} 次:')
        for ms, o, n in prot_trans:
            print(f'    ms={ms:>7.0f}  {f0(o)} -> {f0(n)}')

    # ---------- 6. 时间线 ----------
    print('\n' + '=' * 78)
    print(f'【6】时间线 (每 {a.timeline_sec:g} s 取样)')
    print('=' * 78)
    hdr = (f'{"ms":>7} {"m1Hz":>7} {"intg":>7} {"raw1000":>8} {"enc1":>8} '
           f'{"enc2":>8} {"d_enc1":>7} {"p100":>6} {"g100":>6} {"prot":>5}')
    print(hdr)
    print('-' * len(hdr))
    steps = [num(rows[k], 'ms') - num(rows[k - 1], 'ms') for k in range(1, len(rows))]
    mstep = median(steps) if steps else 250
    step = max(1, int(round(a.timeline_sec * 1000 / (mstep or 250))))
    for k in range(0, len(rows), step):
        r = rows[k]
        if k == 0:
            d1 = None
        else:
            d1 = (num(r, 'enc1') or 0) - (num(rows[k - 1], 'enc1') or 0)
        flag = ''
        if d1 is not None and abs(d1) <= a.frozen_counts and abs(num(r, 'm1Hz') or 0) >= a.min_hz:
            flag = ' <<< 冻住'
        d1s = f'{d1:>7.0f}' if d1 is not None else '      -'
        print(f'{num(r,"ms"):>7.0f} {num(r,"m1Hz"):>7.0f} '
              f'{num(r,"integratedSpeedHz"):>7.0f} {num(r,"raw1000"):>8.0f} '
              f'{num(r,"enc1"):>8.0f} {num(r,"enc2"):>8.0f} {d1s} '
              f'{num(r,"pitch100"):>6.0f} {num(r,"gyro100"):>6.0f} '
              f'{f0(num(r,"balAccelEffectiveLimitHzStep")):>5}{flag}')

    # ---------- 7. 双编码器一致性 ----------
    print('\n' + '=' * 78)
    print('【7】双编码器一致性 (enc1 = PA0/PA1, enc2 = PB6/PB7, encoder2Direction=-1)')
    print('=' * 78)
    e1s = [num(r, 'enc1') or 0 for r in rows]
    e2s = [num(r, 'enc2') or 0 for r in rows]
    net1, net2 = e1s[-1] - e1s[0], e2s[-1] - e2s[0]
    sum1 = sum(abs(e1s[k] - e1s[k - 1]) for k in range(1, len(rows)))
    sum2 = sum(abs(e2s[k] - e2s[k - 1]) for k in range(1, len(rows)))
    r_net = abs(net1) / abs(net2) if net2 else float('nan')
    r_sum = sum1 / sum2 if sum2 else float('nan')
    print(f'  净位移 : enc1 {net1:+.0f}   enc2 {net2:+.0f}   |enc1|/|enc2| = {r_net:.2f}')
    print(f'  总行程 : enc1 {sum1:.0f}   enc2 {sum2:.0f}   |enc1|/|enc2| = {r_sum:.2f}')
    print('  ^ 直行时两轮行程应接近 1.00。明显偏离 = 打滑/失步/编码器丢脉冲,')
    print('    并且 rawVehicleSpeed = -0.5*(encSpeed1+encSpeed2) 会被污染。')
    cnt = {'both': 0, 'only1': 0, 'only2': 0, 'neither': 0}
    for k in range(1, len(rows)):
        m1 = abs(e1s[k] - e1s[k - 1]) > a.frozen_counts
        m2 = abs(e2s[k] - e2s[k - 1]) > a.frozen_counts
        key = 'both' if (m1 and m2) else 'only1' if m1 else 'only2' if m2 else 'neither'
        cnt[key] += 1
    print(f'  每采样谁在动: 双动 {cnt["both"]}   仅enc1动 {cnt["only1"]}   '
          f'仅enc2动 {cnt["only2"]}   都不动 {cnt["neither"]}   (共 {len(rows) - 1})')
    print('  分时段行程 (每 5 s):')
    print(f'  {"ms起":>7} {"ms止":>7} {"enc1行程":>9} {"enc2行程":>9} {"enc1/enc2":>9}')
    b0 = int(num(rows[0], 'ms') or 0)
    b_end = int(num(rows[-1], 'ms') or 0)
    while b0 <= b_end:
        idx = [k for k in range(1, len(rows))
               if b0 <= (num(rows[k], 'ms') or 0) < b0 + 5000]
        if idx:
            t1 = sum(abs(e1s[k] - e1s[k - 1]) for k in idx)
            t2 = sum(abs(e2s[k] - e2s[k - 1]) for k in idx)
            rr = t1 / t2 if t2 else float('nan')
            print(f'  {b0:>7} {b0 + 5000:>7} {t1:>9.0f} {t2:>9.0f} {rr:>9.2f}')
        b0 += 5000

    # ---------- 8. 瞬时零速 / 失步判据 ----------
    print('\n' + '=' * 78)
    print('【8】瞬时零速判据 (raw1000 == 0 且 |m1Hz| 大)')
    print('=' * 78)
    for thr in (500, 2000, 4000):
        tot = sum(1 for r in rows if abs(num(r, 'm1Hz') or 0) >= thr)
        z = sum(1 for r in rows
                if num(r, 'raw1000') == 0 and abs(num(r, 'm1Hz') or 0) >= thr)
        print(f'  |m1Hz|>={thr:>4} 的采样 {tot:>3} 个, 其中 raw1000==0 的有 {z:>3} 个')
    print('  明细 (|m1Hz|>=2000 且 raw1000==0):')
    for k in range(1, len(rows)):
        r = rows[k]
        if num(r, 'raw1000') == 0 and abs(num(r, 'm1Hz') or 0) >= 2000:
            d1 = e1s[k] - e1s[k - 1]
            d2 = e2s[k] - e2s[k - 1]
            print(f'    ms={num(r,"ms"):>7.0f}  m1Hz={num(r,"m1Hz"):>7.0f}  '
                  f'intg={num(r,"integratedSpeedHz"):>7.0f}  d_enc1={d1:>6.0f}  '
                  f'd_enc2={d2:>6.0f}  p100={num(r,"pitch100"):>6.0f}  '
                  f'g100={num(r,"gyro100"):>6.0f}  prot={f0(num(r,"balAccelEffectiveLimitHzStep")):>4}')

    # ---------- 9. 事件表 ----------
    print('\n' + '=' * 78)
    print('【9】事件表 (大角度 / 频率触顶 / 保护切换 / 瞬时零速)')
    print('=' * 78)
    ev = []
    for k in range(1, len(rows)):
        r, p = rows[k], rows[k - 1]
        ms = num(r, 'ms')
        pit = num(r, 'pitch100') or 0
        m = num(r, 'm1Hz') or 0
        raw = num(r, 'raw1000')
        pr, pp = num(r, 'balAccelEffectiveLimitHzStep'), num(p, 'balAccelEffectiveLimitHzStep')
        if abs(pit) >= 2500:
            ev.append((ms, f'大角度 pitch={pit/100:+.2f}°  m1Hz={m:+.0f}  raw1000={raw:+.0f}'))
        if abs(m) >= 6900:
            ev.append((ms, f'频率触顶 m1Hz={m:+.0f} ≈ L  raw1000={raw:+.0f}  pitch={pit/100:+.2f}°'))
        if pr != pp:
            ev.append((ms, f'保护切换 limit {pp:.0f} -> {pr:.0f}  m1Hz={m:+.0f}  intg={num(r,"integratedSpeedHz"):+.0f}'))
        if raw == 0 and abs(m) >= 2000:
            ev.append((ms, f'瞬时零速 m1Hz={m:+.0f}  raw1000=0  pitch={pit/100:+.2f}°'))
    ev.sort(key=lambda x: x[0])
    prev_ms = None
    for ms, txt in ev:
        mark = '  ' if prev_ms is not None and 0 <= ms - prev_ms <= 300 else '\n'
        print(f'{mark}  ms={ms:>7.0f}  {txt}')
        prev_ms = ms

    # ---------- 10. 指定窗口全分辨率 ----------
    if a.window:
        lo, _, hi = a.window.partition(',')
        lo, hi = float(lo), float(hi or lo)
        print('\n' + '=' * 78)
        print(f'【10】窗口全分辨率 ms {lo:.0f} .. {hi:.0f} (250 ms/行)')
        print('=' * 78)
        h2 = (f'{"ms":>7} {"m1Hz":>7} {"intg":>7} {"raw1000":>8} {"spd1000":>8} '
              f'{"enc1":>8} {"enc2":>8} {"d1":>6} {"d2":>6} {"p100":>6} {"g100":>6} '
              f'{"tgt100":>6} {"turn":>5} {"prot":>4}')
        print(h2)
        print('-' * len(h2))
        for k in range(len(rows)):
            r = rows[k]
            ms = num(r, 'ms')
            if ms < lo or ms > hi:
                continue
            d1 = e1s[k] - e1s[k - 1] if k > 0 else 0
            d2 = e2s[k] - e2s[k - 1] if k > 0 else 0
            print(f'{ms:>7.0f} {num(r,"m1Hz"):>7.0f} {num(r,"integratedSpeedHz"):>7.0f} '
                  f'{num(r,"raw1000"):>8.0f} {num(r,"speed1000"):>8.0f} '
                  f'{e1s[k]:>8.0f} {e2s[k]:>8.0f} {d1:>6.0f} {d2:>6.0f} '
                  f'{num(r,"pitch100"):>6.0f} {num(r,"gyro100"):>6.0f} '
                  f'{num(r,"target100"):>6.0f} {num(r,"turnHz"):>5.0f} '
                  f'{f0(num(r,"balAccelEffectiveLimitHzStep")):>4}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
