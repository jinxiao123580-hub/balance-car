# 新会话启动 prompt（复制下面代码块全部内容）

```text
任务：把平衡车固件代码**一次性、完整地逐行讲解**，产出一份文档。不要分多次问我，不要只讲重点，不要输出到聊天里当长篇回复——直接写文件，写完汇报路径与覆盖度。

## 工程位置（绝对路径，别猜）
项目根：D:\AI项目总库\02_机器人与机械工程\PlatformIO项目\pinghengche\balance-car-showcase
固件源码：<项目根>\software\firmware\src\
  main.cpp            5838 物理行（非空 3283；代码≈2524；注释 759）← 主目标
  motion_drive.h      142 行：MotionSpeedProfile / NetSpeedEstimator / MotionPositionPlan / MotionLeanControl
  position_jog.h      126 行：位置点动状态机
  param_store.h       184 行：22 项参数记录格式 + 双扇区扫描
  manual_motion.h      82 行：`^` 手动俯仰通路（smoothstep 规划、刹车段）
  param_flash_f4.h     66 行：STM32F4 sector 10/11 读写端口
  manual_turn.h        44 行：转向手动通路 + clamp()
构建配置：<项目根>\software\firmware\platformio.ini（board=black_f407zg，upload_protocol=stlink，build_dir=%USERPROFILE%\.platformio\build\pinghengche）

## 必须先读的已有文档（我上一轮已核实，别重复造轮子）
1) <项目根>\docs\code-walkthrough\00-STRUCTURE.md   ← 分层、调度时序、60 个函数索引+行号、全局量四分区、历史坑
2) <项目根>\docs\code-walkthrough\P1-setup-loop.md  ← P1 样板的写法与深度，**严格照这个深度和风格来**
3) <项目根>\docs\validation\analysis\20261119-MOTION-CONTROL.md
4) <项目根>\docs\validation\analysis\20261119-FINDINGS.json

## 读代码的关键前提（不读这条会讲错）
main.cpp 有 **387 处连续空行（最长 7 行）**，语句被拆成 `if (` / 空行 / 条件 / 空行 / `)`。
所以：**按逻辑语句块理解，行号只当定位锚点**；引用代码时可以把空行合并后再展示，但行号要标原始物理行。
读文件请用 offset/limit 分段读（每段 300–500 行），不要试图一次读完。

## 交付物（只有这一个文件）
<项目根>\docs\code-walkthrough\FULL-WALKTHROUGH.md

要求：
- **一份文档讲完全部**（main.cpp 全 5838 行 + 6 个头文件），不要拆成 P2/P3/... 多个文件。
- 按**物理行号顺序**推进，但分成"区段"（每段 100–600 行，用 `## 区段 N：L起-L止 · 主题`）。
- 每个区段统一格式：
  1. **这段干什么**（1–3 句，先说意图再说实现）
  2. **逐块讲解**：贴关键代码（合并空行后），逐条解释"它为什么这么写 / 不这么写会怎样"
  3. **本段涉及的量**（全局变量、常量、单位）
  4. **坑与异味**（代码本身的问题要直说，不要美化）
- 文档开头给：总览（复用 00-STRUCTURE.md 的图）+ 阅读路线 + 术语表（counts/50ms、counts/mm、Hz、O/N/K/I、SP/MS/EG/SV、D 帧/G 帧、PAR/JOGCFG）
- 文档结尾给：① 22 项参数总表（键/变量/默认/单位/作用/实测边界）② 命令与帧契约总表 ③ 三条闭环的完整信号链 ④ "函数清单 × 是否已讲"覆盖度自检表 ⑤ 遗留问题与可疑代码清单

## 已核实的事实（直接引用，不要重新推导）
- 调度：`CONTROL_PERIOD_US=2500`、`CONTROL_DT=0.0025`；400Hz 主拍；`outerLoopDivider>=4`→100Hz 外环；`FAST_TELEMETRY_DIVIDER=100`→4Hz `G` 帧；`TELEMETRY_DIVIDER=400`→1Hz `D` 帧（78 字段）。
- 保护：`IMU_FAIL_CONFIRM_COUNT=8`（≈20ms）、`FALL_CONFIRM_COUNT=8`、`FALL_ANGLE_DEG=75`、`EMERGENCY_FALL_ANGLE_DEG=75`（两者相等 → 紧急通道把 8 帧去抖短路，这是坑，要指出）。
- 串口：`BT_BAUD=9600`（≈960B/s 总预算，遥测与命令回显共用）。
- 结构：单一串级 `SP 速度指令 → 速度 PI(K=-0.10) → 目标角修正 → 直立 PD(P=1000,D=100)`；修正被 `N=MAX_SPEED_ANGLE=0.8°` 硬限幅；`I=speedKi=0`；`X=positionKp=0`（位置环默认关）。
- 参数三层：源码默认值 → 闪存 `SAVED` 记录（开机覆盖默认）→ 正在跑的固件；判"当前值"只信 `?` 回显。
- `SV` 与 `K` 是同一个变量 `speedKp`；`MAX_TARGET_FORWARD_SPEED` 已于 2026-11-20 从 `const 8` 改成可写变量 30，并新增 `MS <v>` 命令（RAM-only，不动 22 项表、`FORMAT_VERSION` 仍为 3）。
- 实测边界（要写进参数表）：`N=2.0` 反向猛拽 2° 翻车；`N=0.8` 是稳定边界；`K≥0.20` 猛冲撞翻；`I≠0` 变极限环且稳态速度反向。当前实车 `O=4.35`（当天从 3.365 漂到 4.35，说明 O 随摆放面变化）。

## 已知的代码异味（必须点名，各写一小段）
1) `loop()` 开头 `uint32_t nowUs = micros();`（L5167）随后被 L5177 覆盖 → 死代码。
2) `EMERGENCY_FALL_ANGLE_DEG == FALL_ANGLE_DEG == 75` → 去抖被短路。
3) `setup()` 里 TMC/IMU 初始化失败用 `while` 死循环重试（无超时、无降级）。
4) 文件被格式化工具打散（每行之间插空行、括号拆行）→ 可读性差、行号不可信。
5) `handleTunableCommand` 里 `EG/SP/SV/MS` 走两字母通路，靠"先于单字母解析被拦截"保证不冲突 —— 讲清这个先后顺序为什么重要。

## 硬约束
- **只读代码**：不要修改任何文件（除了写 FULL-WALKTHROUGH.md），不要编译、不要烧录、不要连 BLE、不要碰硬件。
- 不确定的地方标 `[推断]`，读不到的不要编。
- 每条解释要能落到具体行号/函数名；引用代码片段必须与源码一致（可合并空行，不可改语义）。
- 单条消息写不完就**继续 append 同一个文件**，不要拆文件、不要中途问我。全部写完后，回复我四件事：文件绝对路径、总行数、覆盖了哪些区段（L 范围列表）、覆盖度自检表里"未覆盖项"及原因。

开始之前先回一句"已读 00-STRUCTURE.md 与 P1 样板，开始写"，然后就动手。
```
