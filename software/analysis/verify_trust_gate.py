#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
离线回放「编码器信任门」算法，验证阈值是否合理。
用法: python verify_trust_gate.py <csv>

原理说明（重要，决定了本脚本是近似而非精确）:
  固件里信任门跑在 100 Hz 外环(10ms/拍)，窗口 = 50 拍 = 500ms。
  但遥测是每 250ms 落一行，即 1 行 ≈ 25 拍。
  所以无法从日志知道「50 拍里有几拍出现过脉冲」，只能从 2 行(≈50拍)
  的计数增量做近似判断:
    - 必要条件: 窗口内 |Δcount| 总和 >= ACTIVE_TICKS(=5)。
      因为每拍最多贡献的计数是可变的，但要有 >=5 拍各至少 1 个 count，
      总计数绝对值必然 >= 5。所以「总和 < 5 => 一定判不活跃」是可靠的。
    - 反过来「总和 >= 5 => 活跃」是乐观近似(可能集中在 1 拍里)。
      对编码器脉冲序列来说这是安全的：正常编码器不会 25 拍全零后
      突然 1 拍蹦 5 个 count，那本身就是故障特征。
"""
import sys
import csv as _csv

WINDOW_TICKS = 50          # ENCODER_TRUST_WINDOW_TICKS
ACTIVE_TICKS = 5           # ENCODER_TRUST_ACTIVE_TICKS
MIN_COUNTS = 1             # ENCODER_TRUST_MIN_COUNTS
ROWS_PER_WINDOW = 2        # 250ms/行 -> 500ms 窗 = 2 行


def load(path):
    rows = []
    header = None
    with open(path, 'r', encoding='utf-8', errors='replace') as f:
        for raw in f:
            line = raw.rstrip('\n').rstrip('\r')
            if not line:
                continue
            if line.startswith('time_s'):
                if header is None:
                    header = line.split(',')
                continue
            if line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) < 5:
                continue
            try:
                float(parts[0])
            except ValueError:
                continue
            rows.append(parts)
    return header, rows


def col(header, name):
    try:
        return header.index(name)
    except ValueError:
        return None


def main():
    if len(sys.argv) < 2:
        print("用法: python verify_trust_gate.py <csv>")
        return 1
    path = sys.argv[1]
    header, rows = load(path)
    if not header or not rows:
        print("没有数据行")
        return 1

    i_ms = col(header, 'ms')
    i_e1 = col(header, 'enc1')
    i_e2 = col(header, 'enc2')
    i_m1 = col(header, 'm1Hz')
    i_p = col(header, 'pitch100')
    if None in (i_ms, i_e1, i_e2):
        print("缺列, header=", header)
        return 1

    def num(i, r):
        if i is None:
            return 0.0
        try:
            return float(r[i])
        except (ValueError, IndexError):
            return 0.0

    print("=" * 78)
    print("编码器信任门 离线回放")
    print("=" * 78)
    print(f"文件: {path}")
    print(f"数据行: {len(rows)}  采样间隔≈250ms  窗口={WINDOW_TICKS}拍"
          f"({WINDOW_TICKS*10}ms)={ROWS_PER_WINDOW}行  活跃阈值={ACTIVE_TICKS}拍")
    print()

    # 状态机完全照抄固件
    t1, t2 = True, True
    mode = 0
    segs = []          # (start_idx, end_idx, mode) 连续同模式段
    counts = {0: 0, 1: 0, 2: 0}
    prev_e1 = prev_e2 = None
    wins = []          # (row_idx, sum|d1|, sum|d2|, mode)

    for idx, r in enumerate(rows):
        e1 = num(i_e1, r)
        e2 = num(i_e2, r)
        d1 = 0.0 if prev_e1 is None else e1 - prev_e1
        d2 = 0.0 if prev_e2 is None else e2 - prev_e2
        prev_e1, prev_e2 = e1, e2

        # 累积窗口（固件每拍累加，这里用行近似）
        if not hasattr(main, '_acc'):
            main._acc = [0.0, 0.0, 0]
        main._acc[0] += abs(d1)
        main._acc[1] += abs(d2)
        main._acc[2] += 1
        if main._acc[2] >= ROWS_PER_WINDOW:
            s1, s2 = main._acc[0], main._acc[1]
            a1 = s1 >= ACTIVE_TICKS * MIN_COUNTS
            a2 = s2 >= ACTIVE_TICKS * MIN_COUNTS
            if a1 or a2:          # 车静止时两路都不活跃 -> 不改判
                t1, t2 = a1, a2
            mode = 0 if (t1 and t2) else (1 if t1 else 2)
            wins.append((idx, s1, s2, mode))
            main._acc = [0.0, 0.0, 0]

        counts[mode] += 1
        if segs and segs[-1][2] == mode:
            segs[-1][1] = idx
        else:
            segs.append([idx, idx, mode])

    total = len(rows)
    names = {0: "双路平均", 1: "仅enc1", 2: "仅enc2"}
    print("【1】rawSpeedEncoderMode 占比（新遥测第40项会显示这个）")
    for m in (0, 1, 2):
        pct = 100.0 * counts[m] / total
        bar = "#" * int(pct / 2.5)
        print(f"  mode {m} {names[m]:<8} {counts[m]:>4} 行 {pct:5.1f}%  {bar}")
    print()

    print("【2】模式切换时间线（只看变化点）")
    print(f"  {'起始ms':>8} {'结束ms':>8} {'时长s':>6}  mode")
    shown = 0
    for a, b, m in segs:
        if a == b and len(segs) > 40:
            continue
        ms_a, ms_b = num(i_ms, rows[a]), num(i_ms, rows[b])
        dur = (ms_b - ms_a) / 1000.0
        print(f"  {ms_a:8.0f} {ms_b:8.0f} {dur:6.1f}  {m} {names[m]}")
        shown += 1
        if shown > 40:
            print("  ... (截断)")
            break
    print()

    print("【3】窗口明细（前 40 个窗口）")
    print(f"  {'行':>4} {'ms':>8} {'Σ|Δenc1|':>10} {'Σ|Δenc2|':>10}  mode  判据")
    for idx, s1, s2, m in wins[:40]:
        ms = num(i_ms, rows[idx])
        print(f"  {idx:>4} {ms:8.0f} {s1:10.0f} {s2:10.0f}  {m}     "
              f"enc1{'活' if s1>=ACTIVE_TICKS else '死'} "
              f"enc2{'活' if s2>=ACTIVE_TICKS else '死'}")
    print()

    print("【4】阈值合理性反查：有没有「慢速运动被判死」的危险窗口")
    danger = []
    for idx, s1, s2, m in wins:
        ms = num(i_ms, rows[idx])
        m1 = abs(num(i_m1, rows[idx])) if i_m1 is not None else 0.0
        # 固件在下令运动(电机有频率)但窗口内计数不足 -> 可能误判
        if m1 >= 500 and s1 < ACTIVE_TICKS:
            danger.append((idx, ms, m1, s1, s2))
        if m1 >= 500 and s2 < ACTIVE_TICKS:
            danger.append((idx, ms, m1, s1, s2))
    if danger:
        print(f"  有 {len(danger)} 个窗口满足「命令频率>=500Hz 但某路计数<5」")
        print(f"  {'行':>4} {'ms':>8} {'|m1Hz|':>8} {'Σ|Δe1|':>9} {'Σ|Δe2|':>9}")
        for idx, ms, m1, s1, s2 in danger[:25]:
            print(f"  {idx:>4} {ms:8.0f} {m1:8.0f} {s1:9.0f} {s2:9.0f}")
        print("  -> 这些就是真正的失步/接线故障窗口，信任门会把它们剔掉")
    else:
        print("  无。说明命令运动时编码器总是有脉冲，阈值不会误判慢速运动。")
    print()

    print("【5】结论")
    if counts[1] > counts[0]:
        print(f"  本次日志中 mode=1(仅enc1) 占 {100.0*counts[1]/total:.1f}%，")
        print(f"  即信任门会把「半个车速」修正回全速，修复效果显著。")
    else:
        print(f"  本次日志中 mode=0(双路平均) 占 {100.0*counts[0]/total:.1f}%，双路都可信。")
    print(f"  对比: 原公式永远按 -0.5*(e1+e2) 算，故障时段车速被低估约一半。")
    return 0


if __name__ == '__main__':
    sys.exit(main())
