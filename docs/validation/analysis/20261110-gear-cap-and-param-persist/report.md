# 满档 8→5 与参数蓝牙持久化（2026-11-10）

## 1. 改了什么

| 改动 | 文件 | 说明 |
|---|---|---|
| `MotionSpeedProfile::maxSpeed` 8.0 → **5.0** | `software/firmware/src/motion_drive.h` | vref = 档位/2 × 满档；2.0 档目标由 8 降到 5 counts/50ms |
| 22 项参数存内部 flash | `software/firmware/src/param_store.h`、`param_flash_f4.h`、`main.cpp` | sector 10/11 双备份，追加写 104 B/条，CRC32 + seq 取优 |
| 蓝牙命令 `SAVE` / `LOAD` / `DEF` | `main.cpp` | `?` 回显末尾带 `SAVED=1/0` |
| 面板「保存到车端 / 从车端读回 / 恢复出厂值」 | `software/ble-panel/ble_tune_panel.py` | 未保存橙色提示；旧固件（无 `SAVED=`）自动禁用按钮 |
| 分析脚本默认满档改 5 | `software/analysis/analyze_motion_test.py` | 否则新录制稳态误差整体偏 37% |

为什么要追加写而不是每次擦扇区：F407 的 128 KB 扇区擦除典型 1.1 s，远超 400 Hz 控制环容忍度；
一条记录只编程 26 个字（约 0.5 ms），一个扇区放 1200+ 条，写满才在**开机时**预擦另一块。

## 2. 实测：gear 2.0（满档 5，vref 5）

录制：`docs/validation/recordings/auto_gear20_max5.{csv,fast.csv}`，参数 X=0.02 Y=0.3 M=12。

| 方向 | 段长 | 位移 | 速率 | 目标 | 误差 | lean 均值 | lean 范围 | 贴边 |
|---|---|---|---|---|---|---|---|---|
| 前进 | 2.5 s | +337 counts | 134.8 counts/s | 100 | **+35%** | +0.26° | −0.41…+0.57 | 0% |
| 后退 | 2.8 s | −20.5 counts | −7.5 counts/s | −100 | **−93%** | −0.45° | −0.90…+0.06 | 18% |

**全程 |pitch−O| 最大 1.11°**（旧固件同档位 12.65° 触发中止）。安全目标达成：2.0 档不再冲出。

## 3. 对照：旧记录 auto_run2（gear 1.0，满档 8，vref 4）

| 方向 | 段长 | 位移 | 速率 | 目标 | 误差 |
|---|---|---|---|---|---|
| 前进 | 2.5 s | +139 counts | 55.6 counts/s | 80 | −30% |
| 后退 | 2.8 s | −12 counts | −4.4 counts/s | −80 | −95% |

补录 gear 1.6（满档 5 → vref 4，与旧记录同一工作点）：`auto_gear16_max5`，前进稳定段 +82~86 counts/s（目标 80，**+3~7%**），|pitch−O| 最大 0.93°。

## 4. 两个新结论

1. **后退与满档无关**。旧记录（vref 4）后退就只有 −4.4 counts/s，新记录（vref 5）−7.5 counts/s；
   同时 lean 已到 −0.45~−0.90°，贴 0.9° 上限 18% —— 是"原地摇"不是"滑"：
   瞬时 `speed1000` 均值 −1.68、sd 4.11，而编码器净位移 2.8 s 只有 −12 counts，两者差 8 倍。
2. **旧交接里"后退 3.15 counts/50ms、误差 21%"是瞬时速度均值，不是实际位移**。
   该段实际位移 −12 counts / 2.8 s（`auto_run2.fast.csv` 后退段）。这条结论需要更正。

另：档位重发的实际节拍只有约 4 次/s（2.5 s 内 rxBytes +80 B ≈ 10 条命令），
紧贴固件 350 ms 看门狗，偶尔请求会掉一拍、lean 塌回 0，会让测量偏保守。

## 5. 参数持久化端到端记录

```
?                                    -> FW=20261110 ... X=0.0200 Y=0.30 M=12.00 SAVED=0
SAVE                                 -> SAVED OK seq=1
?                                    -> ... SAVED=1
LOAD                                 -> LOAD OK seq=1      SAVED=1
DEF                                  -> DEF OK ram-only    SAVED=0   X=5 Y=1 M=0（编译默认）
LOAD                                 -> LOAD OK seq=1      SAVED=1   X=0.02 Y=0.3 M=12
【断电重启】
?                                    -> SAVED=1  X=0.0200 Y=0.30 M=12.00
```

断电重启后 `SAVED=1` 且 `X=0.02`（编译默认是 5）—— 证明开机确实从 flash 读回，不再依赖 ST-Link。
快照：`before_save.json` / `after_save.json` / `after_boot.json`（本目录）。
另有旁证：gear 1.6 那次测试故意不传 `--set`，PAR 仍显示 X=0.02/Y=0.3/M=12。

## 6. 未解决

- 后退方向要么加方向配平（`backwardTrim` 目前只能在代码里改，没做成可下发参数），
  要么放宽 `MotionLearnControl::learnMinSpeed=0.8`（现在"不动就学不到配平"是死循环）。
- 前进在 vref 5 超 35%、vref 4 只 +3~7%：满档是否再降到 4.5，需要用同一档位复测两轮。
