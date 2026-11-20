#include "panel/cmd.h"

// ============================================================
// panel/cmd.cpp —— 命令接收、解析、执行与回显（纯上位机交互，不参与控制运算）
//
// 本文件改动的都是控制模块的全局参数（定义在 control/、remote/，声明在 core/state.h），
// 改完是否落 flash 由 params.cpp 的 SAVE 决定；本文件自己不做任何持久化。
// 单位约定见 cmd.h 文件头（10.8 count = 1mm；1 count/50ms = 1.852 mm/s）。
// 回显一律整行入队（QueuedBtReply 析构时 txPushLine），不阻塞 400Hz 控制环。
// ============================================================
namespace panel {

// ---- 命令接收行缓冲（RAM-only，上电清零）----
// btCmdBuf[48]：单行最大 47 个可见字符 + 结尾 '\0'；
// btCmdLen：当前长度，B；btCmdOverflow：本行超长标志（true 时整行丢弃）。
// 这些是 static，生命周期=整个固件运行期，不参与 SAVE。
static char btCmdBuf[48];
static uint8_t btCmdLen = 0;
static bool btCmdOverflow = false;

// ============================================================
// handleTunableCommand() —— 两字母运行时通路（必须在单字母解析之前拦）
//
// 支持的 4 条（大小写不敏感，两字母后必须是空格/TAB 或行尾）：
//   SP <v>  目标速度 speedCmd，单位 count/50ms，钳 ±MAX_TARGET_FORWARD_SPEED(MS)。
//           SP>0 = 后退，SP<0 = 前进（2026-11-19 用 12/12 腿验证过方向）。
//           只改 RAM；^ 手动通路活动时 main 会把 speedCmd 清零（两通路互斥）。
//   MS <v>  MAX_TARGET_FORWARD_SPEED，速度指令总上限，单位 count/50ms，
//           默认 30，钳 [0.5, 60]；改小后立即把已有 speedCmd 拉回范围内。RAM-only。
//   SV <v>  speedKp（= 22 项里的 K），单位 deg per (count/50ms)，
//           默认 -0.10，钳 ±0.5；改的是参数本体，SAVE 时会被一起存。
//   EG <v>  motionLean.errorGain（^ 手动通路的速度误差→倾角增益），
//           单位 deg per (count/50ms)，默认 0.05，钳 [0, 0.15]。RAM-only。
//
// 边界行为（这几条都容易被忽略）：
//   * 值里的空格/TAB 会被整体剔除 —— 发 "SP 1 2" 等于 "SP 12"；
//   * 值缓冲 16 B，超出直接截断（不会越界）；
//   * 用区间判断挡 NaN/Inf（不用 isfinite，主机回归没有该声明）；
//   * 返回 true = 已处理（无论成功还是报了 ERR）；false = 不是这 4 条，交给下层。
// ============================================================
// ------------------------------------------------------------
// 运行时可调的参数（2026-11-19 简化后只剩两个）：
//   EG <deg>   MotionLeanControl::errorGain   速度误差→倾角增益（^ 手动通路用）
//   SP <v>     目标速度（count/50ms，钳 ±MAX_TARGET_FORWARD_SPEED）
//              SP 是唯一的速度设定入口，进入经典速度 PI；与 ^ 手动通路互斥。
// 为什么用两字母：A~Z 已被面板参数占满，再开单字母会和 PAR 回显/面板解析打架；
// 两字母整词既不误命中，也不动 D 帧/CSV 契约。只改 RAM、不写 flash。
// ------------------------------------------------------------
static bool handleTunableCommand(const char * cmd, uint8_t len, QueuedBtReply & reply) {
  uint8_t lead = 0;
  while (lead < len && cmd[lead] == ' ')++lead;  // 前导空格
  if (len < lead + 2) return false;
  if (lead + 2 < len && cmd[lead + 2] != ' ' && cmd[lead + 2] != '\t') return false;
  const char a = (char) toupper((unsigned char) cmd[lead]);
  const char b = (char) toupper((unsigned char) cmd[lead + 1]);
  const bool isErrorGain = (a == 'E' && b == 'G');
  const bool isSpeedCmd = (a == 'S' && b == 'P');
  const bool isSpeedKp = (a == 'S' && b == 'V');  // 2026-11-19：速度环 P 增益在线可调
  const bool isMaxSpeed = (a == 'M' && b == 'S');  // 2026-11-20: speed range cap (RAM-only)
  if (!isErrorGain && !isSpeedCmd && !isSpeedKp && !isMaxSpeed) return false;
  uint8_t vstart = lead + 2;
  while (vstart < len && (cmd[vstart] == ' ' || cmd[vstart] == '\t'))++vstart;
  if (vstart >= len) {
    reply.println("ERR:SP/SV/EG/MS need a number");
    return true;
  }
  char vbuf[16];
  uint8_t vlen = 0;
  for (uint8_t i = vstart; i < len && vlen + 1u < sizeof (vbuf);++i) {
    const char c = cmd[i];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
    vbuf[vlen++] = c;
  }
  vbuf[vlen] = '\0';
  char * end = nullptr;
  const float value = strtof(vbuf, & end);
  // 用区间判断挡 NaN/Inf，而不是 isfinite()：后者在主机回归里没有非限定声明
  if (end == vbuf || * end != '\0' || !(value > - 1.0e6f && value < 1.0e6f)) {
    reply.println("ERR:SP/SV/EG/MS need a number");
    return true;
  }
  // ---- SP：目标速度（count/50ms）----
  // 与 ^ 手动通路互斥（main 侧在 ^ 动作时清 speedCmd）；钳位见 MS。
  // 注意饱和：速度环角修正 = K×speedError 再被 N=0.8° 限幅，
  // |speedError| > N/|K| = 0.8/0.10 = 8 count/50ms 时立刻打满 0.8°，
  // 所以 SP≥12 从静止起步就已经是饱和输出（SP=8 恰好是临界点）。
  if (isSpeedCmd) {
    speedCmd = clampf(value, - MAX_TARGET_FORWARD_SPEED, MAX_TARGET_FORWARD_SPEED);
    reply.print("SP ok ");
    reply.println(speedCmd, 3);
    return true;
  }
  // ---- MS：速度指令总上限 MAX_TARGET_FORWARD_SPEED（count/50ms，RAM-only）----
  // 默认 30（= 55.6 mm/s）。历史上编译期是 8，车被锁在静摩擦主导区间；
  // 2026-11-20 改成运行时变量，可用 MS 在线扫行程范围而不用重烧。
  // 下限 0.5 = 实质"停车档"，上限 60 已远超步进电机可用区间。
  if (isMaxSpeed) {
    // range cap: below 0.5 means standstill, above 60 is far beyond the steppers
    MAX_TARGET_FORWARD_SPEED = clampf(value, 0.5f, 60.0f);
    // shrinking the range may leave speedCmd out of bounds: pull it back at once
    speedCmd = clampf(speedCmd, - MAX_TARGET_FORWARD_SPEED, MAX_TARGET_FORWARD_SPEED);
    reply.print("MS ok ");
    reply.print(MAX_TARGET_FORWARD_SPEED, 3);
    reply.print(" SP=");
    reply.println(speedCmd, 3);
    return true;
  }
  // ---- SV：速度环 P 增益（就是 22 项里的 K，值相同、命令不同）----
  // 单位 deg per (count/50ms)；±0.5 的钳位只防止手滑输错数量级。
  // 实测边界：-0.02 太小（SP=8 只给 0.16°，低于破粘门槛 0.5~0.7°）；
  //           -0.10 是当前值（SP±8 给 +43.8/-43.0mm，方向对称）；
  //           ≥0.20 会猛冲撞翻（SV 0.20/0.30 实测 -47.9/-11.9 mm/s）。
  // 注：本命令不经过 markDirtyIfParam()，改 SV 后 PAR 里的 SAVED 仍是旧值。
  if (isSpeedKp) {
    // 极性与大小都要试：允许 ±，限 ±0.5（SP=8 时 0.12 约合 1.0° 倾角）
    speedKp = clampf(value, - 0.5f, 0.5f);
    reply.print("SV ok ");
    reply.println(speedKp, 4);
    return true;
  }
  // ---- EG：^ 手动通路的 MotionLeanControl 误差增益（RAM-only）----
  // 单位 deg per (count/50ms)，默认 speedErrorGain=0.05，钳 [0, 0.15]。
  // 注意它不是速度环的 K：^ 通路自带前馈 0.06 + 误差项 errorGain，
  // 倾角另受 MotionLeanControl::limit=0.90° 限幅。改大等于把 0.9° 上限一步打满。
  {
    // 误差增益开太大等于把 0.9° 上限一步打满，只允许 0~0.15
    motionLean.errorGain = clampf(value, 0.0f, 0.15f);
  }
  reply.print(" EG ");
  reply.print(motionLean.errorGain, 3);
  reply.println();
  return true;
}

// poll() —— 字节流拼行。
// 参数: 无。返回: void。调用者: main.cpp loop() 每圈（≥400Hz）。
// 边界: 每次最多消费 32 B（readBudget）；'\n' 或 '\r' 都算行结束；
//       超长行（≥47 字符）置 btCmdOverflow 并整行丢弃，换行后复位；
//       空行（连续换行）会走 handleLine(cmd,0)，被 len==0 分支安全忽略。
void poll() {
  uint8_t readBudget = 32;
  while (readBudget--&& BTSerial.available() > 0) {
    char c = (char) BTSerial.read();
    btRxBytes++;
    if (c == '\n' || c == '\r') {
      if (!btCmdOverflow) handleLine(btCmdBuf, btCmdLen);
      btCmdOverflow = false;
      btCmdLen = 0;
    } else if (btCmdLen < sizeof (btCmdBuf) - 1) {
      btCmdBuf[btCmdLen++] = c;
    } else {
      btCmdOverflow = true;
    }
  }
}

// handleLine() —— 一条命令的完整分发链。
// 参数: cmd/len = 行内容（不含换行，允许前后空格）。返回: void。
// 调用者: poll()。频率: 面板手点级别（每条命令一次）。
// 边界: 所有回显经局部 QueuedBtReply，函数返回时整行入队；
//       数值非法回 ERR，未知键回 ERR:unknown cmd，非数字开头的残余单字母命令静默忽略。
//
// 分发顺序（不可调换，否则命令会被拆坏）：
//   两字母 SP/SV/EG/MS → 整词 SAVE/LOAD/DEF → '?' → '!' → '~'/'&' → '^'/'@'
//   → 数字守卫 → 单字母 22 项 + V/C → markDirtyIfParam + 回显。
void handleLine(char * cmd, uint8_t len) {
  QueuedBtReply reply;
  // 运行时可调增益先拦（TF/TB/EG），它们在下面被拆成单字母之前必须命中
  // 两字母通路必须先拦：否则 "SP 8" 会被拆成 key='S' + 值 "P 8"（非法→静默丢弃）。
  // 注：旧注释写的是"先拦 TF/TB/EG"，TF/TB 是历史命令，当前实现只拦 EG/SP/SV/MS。
  if (handleTunableCommand(cmd, len, reply)) return;
  // 整词命令先拦截：SAVE/LOAD/DEF 没有数值，若走下面的"key + 值"解析会被
  // 拆成 S+"AVE"、L+"OAD"、D+"EF" 然后被数值守卫静默丢弃。
  if (handleParamWord(cmd, len, reply)) return;  // SAVE / LOAD / DEF

  // 跳过前导空格
  while (len > 0 && (* cmd == ' ' || * cmd == '\t')) {
    cmd++;
    len--;
  }
  if (len == 0) return;
  char key = * cmd;
  cmd++;
  len--;
  while (len > 0 && (* cmd == ' ' || * cmd == '\t')) {
    cmd++;
    len--;
  }
  uint8_t vlen = len;
  while (vlen > 0 && (cmd[vlen - 1] == ' ' || cmd[vlen - 1] == '\r' || cmd[vlen - 1] == '\n')) vlen--;
  // ============================================================
  // '?' —— 参数回显（PAR 一行 + JOGCFG 一行）
  // ============================================================
  // 回显顺序（面板解析按 KEY=VALUE 取，顺序本身只是显示习惯）：
  //   P D O Z L S R F G T U K I N W J E B H X Y M   ← 22 项（顺序与 flash 存储顺序不同！）
  //   EG SAVED SP SV MS                             ← 运行时量
  //   然后 JOGCFG kp limit                          ← positionLean 配置
  // 单位/默认（与 params.cpp 逐槽注释一致）：
  //   P=直立P 1000 / D=直立D 100 / O=平衡点 2.99° / Z=角度死区 0° /
  //   L=输出限幅 1750Hz / S=每10ms增量限幅 100Hz / R=直立PD→增量比例 0.10 /
  //   F=外环全权限角 5° / G=外环退出角 10° / T=转向P 0 / U=转向D 0 /
  //   K=速度环P -0.10 deg per (count/50ms) / I=速度环I 0 / N=速度修正角上限 0.8° /
  //   W=积分限幅 50 / J=误差死区 2.0 count/50ms / E=速度环D 0 /
  //   B=高速加速保护起点 1000Hz / H=高速每10ms增量上限 40Hz /
  //   X=位置P 0.02 (count/50ms)/count / Y=位置目标速度上限 0.30 count/50ms(=0.56mm/s) /
  //   M=位置死区 12 count(=1.11mm)。
  //   EG=0.05（deg per count/50ms）；SAVED=1 与 flash 一致 / 0 有改动未存；
  //   SP=当前速度指令 count/50ms；SV=speedKp（与 K 显示的是同一个变量，属历史重复）；
  //   MS=速度指令上限 count/50ms（默认 30）。
  // 行首固定标志（面板兼容用的硬编码，不是实时状态）：
  //   TURN=2 MOVE=5 SMOOTH=1 MOTION=2 FW=20261112F
  //   TILTMAX=ManualPitchControl::maxAngle=2.00° —— 这是 ^ 手动俯仰的限幅，
  //   不是倒地角（倒地角是 75°）。
  // ============================================================
  if (key == '?') {
    prepareCommandResponse();
    reply.print("PAR: TURN=2 MOVE=5 SMOOTH=1 MOTION=2 FW=20261112F TILTMAX=");
    reply.print(ManualPitchControl::maxAngle, 2);
    reply.print(" ");
    reply.print("P=");
    reply.print(balanceKp, 2);
    reply.print(" D=");
    reply.print(balanceKd, 2);
    reply.print(" O=");
    reply.print(balanceAngleOffset, 2);
    reply.print(" Z=");
    reply.print(BALANCE_ANGLE_DEADBAND_DEG, 2);
    reply.print(" L=");
    reply.print(BALANCE_OUTPUT_LIMIT_HZ, 1);
    reply.print(" S=");
    reply.print(BALANCE_OUTPUT_SLEW_HZ, 1);
    reply.print(" R=");
    reply.print(BALANCE_ACCEL_SCALE, 4);
    reply.print(" F=");
    reply.print(OUTER_FULL_ANGLE_DEG, 2);
    reply.print(" G=");
    reply.print(OUTER_DISABLE_ANGLE_DEG, 2);
    reply.print(" T=");
    reply.print(turnKp, 2);
    reply.print(" U=");
    reply.print(turnKd, 2);
    reply.print(" K=");
    reply.print(speedKp, 2);
    reply.print(" I=");
    reply.print(speedKi, 2);
    reply.print(" N=");
    reply.print(MAX_SPEED_ANGLE, 2);
    reply.print(" W=");
    reply.print(SPEED_INTEGRAL_LIMIT, 1);
    reply.print(" J=");
    reply.print(SPEED_ERROR_DEADBAND, 2);
    reply.print(" E=");
    reply.print(speedKd, 3);
    reply.print(" B=");
    reply.print(HIGH_SPEED_ACCEL_START_HZ, 1);
    reply.print(" H=");
    reply.print(HIGH_SPEED_ACCEL_LIMIT_HZ, 1);
    reply.print(" X=");
    reply.print(positionKp, 4);
    reply.print(" Y=");
    reply.print(MAX_POSITION_RETURN_SPEED, 2);
    reply.print(" M=");
    reply.print(POSITION_DEADBAND_COUNT, 2);
    // 运动环的运行时增益（EG 可以改；SP 是当前速度指令）
    reply.print(" EG=");
    reply.print(motionLean.errorGain, 3);
    // SAVED=1：当前 RAM 参数与 flash 记录一致；0：改过没存（面板显示橙色提示）
    reply.print(" SAVED=");
    reply.print(savedFlag());
    // SP 追加在 PAR 行最末尾：唯一的速度指令（count/50ms），0=停车
    reply.print(" SP=");
    reply.print(speedCmd, 3);
    // SV：速度环 P 增益（在线可调，用于定极性与标定倾角/速度）
    reply.print(" SV=");
    reply.print(speedKp, 4);
    reply.print(" MS=");
    reply.print(MAX_TARGET_FORWARD_SPEED, 3);
    reply.println();
    reply.print("JOGCFG ");
    reply.print(positionLean.kp, 4);
    reply.print(' ');
    reply.println(positionLean.limit, 3);
    return;
  }
  // '!'（不带值）—— 驱动/调度诊断回显，按编译宏选 TMC 工作模式：
  //   TMC_STANDALONE_STEP_DIR: STEP 边沿数、周期、overrun、maxGapUs、maxWorkUs、
  //                            encoderMode(0 双路/1 仅enc1/2 仅enc2)、softTicks(换向软窗口剩余)
  //   TMC_GPIO_WRITE_ONLY:     仅报告已发送字节数（无读回，不代表配置成功）
  //   其它:                    "TMC READBACK MODE"
  // 单位: 周期/耗时为 us；overrun / 边沿数为次数。全部只读，不改状态。
  if (key == '!' && vlen == 0) {
    prepareCommandResponse();
#if TMC_STANDALONE_STEP_DIR
    reply.println("TMC STEP/DIR standalone: hardware-selected mode");
    reply.print("STEP edges=");
    reply.print(control::step1.risingEdges);
    reply.print('/');
    reply.print(control::step2.risingEdges);
    reply.print(" periodUs=");
    reply.print(control::step1.periodTicks);
    reply.print('/');
    reply.print(control::step2.periodTicks);
    reply.print(" overrun=");
    reply.print(controlOverrunCount);
    reply.print(" maxGapUs=");
    reply.print(controlMaxGapUs);
    reply.print(" maxWorkUs=");
    reply.println(controlMaxWorkUs);
    reply.print("encoderMode=");
    reply.print(rawSpeedEncoderMode);
    reply.print(" softTicks=");
    reply.println(reversalSoftTicksRemaining);
#elif TMC_GPIO_WRITE_ONLY
    reply.print("TMC WRITE-ONLY SENT rearBytes=");
    reply.print(control::rearTmcUart.txCount);
    reply.print(" frontBytes=");
    reply.print(control::frontTmcUart.txCount);
    reply.println(" spreadCycle=REQUESTED microsteps=16 UNVERIFIED");
#else
    reply.println("TMC READBACK MODE");
#endif
    return;
  }
  // 无值的单字母命令到此为止（例如只发一个 "C" 不会捕获位置，
  // 必须带一个数字，如 "C 0" —— 见下面的数值守卫与 'C' 分支）。
  if (vlen == 0) return;
  // 单字母通路的值缓冲：最多 31 个字符 + '\0'；超长直接回 ERR:value too long。
  char vbuf[32];
  if (vlen >= sizeof (vbuf)) {
    reply.println("ERR:value too long");
    return;
  }
  for (uint8_t i = 0; i < vlen; i++) vbuf[i] = cmd[i];
  vbuf[vlen] = '\0';
  // '!'（带值）—— JOGCFG 位置点动配置：格式 "! kp,limit"。
  // kp: positionLean.kp，单位 deg per count，合法范围 [0, 0.02]；
  // limit: positionLean.limit，单位 deg，合法范围 [0.05, 0.50]（默认 0.005 / 0.30）。
  // 边界: 解析失败回 ERR:invalid jog config；点动/倾角动作进行中拒绝修改
  //       （ERR:stop jog before config）。只改 RAM，不持久化。
  if (key == '!') {
    float kp = 0, limit = 0;
    if (!parseJogConfig(vbuf, kp, limit)) {
      reply.println("ERR:invalid jog config");
      return;
    }
    if (positionJog.active || positionLean.busy()) {
      reply.println("ERR:stop jog before config");
      return;
    }
    positionLean.kp = kp;
    positionLean.limit = limit;
    reply.print("JOGCFG ");
    reply.print(kp, 4);
    reply.print(' ');
    reply.println(limit, 3);
    return;
  }
  // '~' / '&' —— 位置点动协议（面板/上位机用）
  //   '~ id,delta'：一帧点动。id 为递增序号（uint32，只认同一次/更新的 id），
  //                delta 单位 count，|delta|<=200；当前实现只接受 delta==0
  //                （等于"停车/保持"，非零会回 ERR:use fixed-angle hold mode）。
  //   '& id'      ：心跳，active 且 id==当前序号时刷新 lastHeartbeat。
  // 边界/守卫：
  //   * 非 0 delta 先拒；id 不新则忽略但回 JOG 状态；
  //   * 起步条件：手动通路不活动、IMU 正常、|pitch-O|<=5°、速度环开着(K!=0)、
  //     positionLean.kp>0、至少一路编码器可信 —— 否则 ERR:jog needs upright idle vehicle；
  //   * 心跳超时 500ms → LINK_LOST；点动总时长 >12s → TIMEOUT；编码器来源切换 → SOURCE_CHANGED。
  // 回显: "JOG <sequence> <active> <reason>"，reason 见 PositionJog::Reason。
  if (key == '~' || key == '&') {
    uint32_t id = 0;
    float delta = 0;
    const char * rest = nullptr;
    if (key == '&') {
      if (parseJogId(vbuf, id, rest) && * rest == '\0') positionJog.heartbeat(id, millis());
      return;
    }
    if (!parseJogFrame(vbuf, id, delta)) {
      reply.println("ERR:invalid jog");
      return;
    }
    if (delta != 0) {
      reply.println("ERR:use fixed-angle hold mode");
      return;
    }
    if (positionJog.newer(id)) {
      if (delta != 0 && (remote::isActive() || imuFailCount != 0 || fabsf(pitch - balanceAngleOffset) > 5 || fabsf(speedKp) < .000001f || positionLean.kp <= 0 || !(encoder1Trusted || encoder2Trusted))) {
        reply.println("ERR:jog needs upright idle vehicle");
        return;
      }
      if (delta == 0) {
        positionJog.start(id, 0, vehiclePosition, rawSpeedEncoderMode, millis());
        remote::captureCurrentPositionTarget();
      } else {
        remote::positionHold();
        positionJog.start(id, delta, vehiclePosition, rawSpeedEncoderMode, millis());
        targetVehiclePosition = positionJog.reference;
        positionHoldWasEnabled = positionKp > 0.000001f;
      }
    }
    if (id == positionJog.sequence) positionJog.heartbeat(id, millis());
    reply.print("JOG ");
    reply.print((unsigned long) positionJog.sequence);
    reply.print(' ');
    reply.print((int) positionJog.active);
    reply.print(' ');
    reply.println(positionJog.reason);
    return;
  }
  // '^' / '@' —— 手动运动通路（前馈+误差的手动倾角，与 SP 速度通路互斥）
  //   '^ turnHz,forwardDeg'：turn 为转向 STEP 频率差分，单位 Hz，钳 ±300Hz；
  //                          forward 为相对平衡点的俯仰角，单位度，|forward|<=2.0°
  //                          （ManualPitchControl::maxAngle），超限回 ERR。
  //   '@' 与 '^' 同格式但只允许 0,0（兼容老面板的"停车"）。
  // 边界/守卫：
  //   * forward!=0 时：若 ^ 不在动作、或 IMU 故障、或 |pitch-O|>5°、
  //     或当前目标角偏移非法，则回 ERR:angle hold needs upright ready vehicle；
  //   * 首次进入用当前目标角 seed，避免目标角跳变；
  //   * 转向非 0 时会先停掉位置点动；
  //   * 命令超时 350ms 未刷新即自动回 0（手动通路最长作用 350ms/帧）。
  // 成功回 "MOTION OK"。
  if (key == '^' || key == '@') {
    float turn, forward;
    if (!remote::parseManualMotion(vbuf, turn, forward)) {
      reply.println("ERR:invalid motion");
      return;
    }
    if (key == '@' && (turn != 0 || forward != 0)) {
      reply.println("ERR:use fixed-angle ^ command");
      return;
    }
    // 2026-11-10: forward is a speed gear now, accept 0..maxAngle instead of only +/-2
    if (forward != 0 && fabsf(forward) > ManualPitchControl::maxAngle) {
      reply.println("ERR:forward gear out of range");
      return;
    }
    if (turn != 0 && positionJog.active) remote::stopPositionJog(PositionJog::STOP);
    if (forward != 0 && !manualForward.busy(millis())) {
      float currentOffset = targetBalanceAngle - balanceAngleOffset;
      if (manualForward.needsZero || imuFailCount != 0 || fabsf(pitch - balanceAngleOffset) > 5 || !isfinite(currentOffset) || fabsf(currentOffset) > ManualPitchControl::maxAngle) {
        reply.println("ERR:angle hold needs upright ready vehicle");
        return;
      }
      manualForward.seed(currentOffset);  // no jump from the current normal-loop target
    }
    uint32_t now = millis();
    manualTurn.command(turn, now);
    manualForward.command(forward, now);
    reply.println("MOTION OK");
    return;
  }
  // 数值守卫：单字母命令的值必须以 0-9 / '-' / '.' 开头，否则静默忽略（不回 ERR）。
  // 目的：SAVE/LOAD/DEF 等已经被整词通路拦掉，剩下的非数字串不应被 strtof 当成 0 误设参数。
  // 值必须为数字，否则忽略（防 "SAVE"/"DEF" 等非数字串被误当 key+val=0 误设）
  {
    char c0 = vbuf[0];
    if (!((c0 >= '0' && c0 <= '9') || c0 == '-' || c0 == '.')) return;
  }
  char * valueEnd = nullptr;
  float val = strtof(vbuf, & valueEnd);
  if (valueEnd == vbuf || * valueEnd != '\0' || !isfinite(val)) {
    reply.println("ERR:invalid value");
    return;
  }
  // ============================================================
  // 单字母参数写入（22 项 + V 转向 / C 位置目标捕获）
  // 回显: "<键大写>=<值> OK"，并 markDirtyIfParam() 置"未保存"。
  // 逐项含义见每个赋值语句的行尾注释；单位/实测边界详见 params.cpp 的 22 槽注释。
  // 注意: 这里只改 RAM；要断电保持必须再发 SAVE。
  // ============================================================
  switch (key) {
    case 'P':
    case 'p':
      balanceKp = val;  // P 直立环P，默认1000：1°角度误差→1000Hz原始输出（再乘R得每10ms增量）
      break;
    case 'D':
    case 'd':
      balanceKd = val;  // D 直立环D，默认100：陀螺角速度阻尼，单位 Hz 每 (度/s)
      break;
    case 'O':
    case 'o':
      balanceAngleOffset = val;  // O 机械平衡点，默认2.99°，实测随摆放面漂移 2.99→3.365→4.35
      break;
    case 'Z':
    case 'z':
      BALANCE_ANGLE_DEADBAND_DEG = val;  // Z 直立环角度死区，默认0°（历史 3→1.5→0.5→0）
      break;
    case 'L':
    case 'l':
      BALANCE_OUTPUT_LIMIT_HZ = fabsf(val);  // L 直立输出限幅，默认1750Hz（finalBalanceOutput 钳位，防高频失步）
      break;
    case 'S':
    case 's':
      BALANCE_OUTPUT_SLEW_HZ = fabsf(val);  // S 每10ms最大增量，默认100Hz/update（=10000Hz/s）
      break;
    case 'R':
    case 'r':
      BALANCE_ACCEL_SCALE = fabsf(val);  // R 直立PD→频率增量比例，默认0.10
      break;
    case 'F':
    case 'f':
      OUTER_FULL_ANGLE_DEG = val;  // F 外环全权限角，默认5°：|pitch-O|<=5° 时外环权限=1
      break;
    case 'G':
    case 'g':
      OUTER_DISABLE_ANGLE_DEG = val;  // G 外环退出角，默认10°：>=10° 外环权限=0，只留直立PD
      break;
    case 'V':
    case 'v':
      if (val != 0 && positionJog.active) remote::stopPositionJog(PositionJog::STOP);
      manualTurn.command(val, millis());  // V 手动转向 STEP 频率差分，单位Hz，钳±300Hz，350ms 无刷新自动回0
      val = manualTurn.requested;
      break;
    case 'T':
    case 't':
      turnKp = val;  // T 转向环P，默认0（转向环当前关闭）
      break;
    case 'U':
    case 'u':
      turnKd = val;  // U 转向环D，默认0（转向环当前关闭）
      break;
    case 'K':
    case 'k':
      speedKp = val;  // K 速度环P，默认-0.10 deg per (count/50ms)；|err|>8 即达 N=0.8° 饱和
      break;
    case 'I':
    case 'i':
      speedKi = val;  // I 速度环I，默认0（加积分在静摩擦下变极限环，稳态反向）
      break;
    case 'N':
    case 'n':
      MAX_SPEED_ANGLE = val;  // N 速度修正角上限，默认0.8°（稳定边界；N=2.0 会反向猛拽翻车）
      break;
    case 'W':
    case 'w':
      SPEED_INTEGRAL_LIMIT = val;  // W 积分累积限幅，默认50（误差×秒，非deg）
      break;
    case 'J':
    case 'j':
      SPEED_ERROR_DEADBAND = val;  // J 速度误差死区，默认2.0 count/50ms（只门控积分）
      break;
    case 'E':
    case 'e':
      speedKd = val;  // E 速度误差微分增益，默认0
      break;
    case 'H':
    case 'h':
      HIGH_SPEED_ACCEL_LIMIT_HZ = fabsf(val);  // H 高速继续加速时每10ms增量上限，默认40Hz
      break;
    case 'B':
    case 'b':
      HIGH_SPEED_ACCEL_START_HZ = fabsf(val);  // B 高速加速保护起点，默认1000Hz
      break;
    case 'X':
    case 'x':
      {
        bool wasEnabled = positionKp > 0.000001f;
        positionKp = fabsf(val);  // X 位置环P，默认0.02；置0=关位置环（2026-11-20 实测用 RAM X=0）
        bool nowEnabled = positionKp > 0.000001f;
        if (!wasEnabled && nowEnabled && !positionHoldSuspended) remote::captureCurrentPositionTarget();
        if (!nowEnabled) {
          positionSpeedTarget = 0.0f;
          targetForwardSpeed = 0.0f;
      }
      positionHoldWasEnabled = nowEnabled;
      break;
    }
    case 'Y':
    case 'y':
      MAX_POSITION_RETURN_SPEED = fabsf(val);  // Y 位置环目标速度限幅，默认0.30 count/50ms（=0.56mm/s）
      break;
    case 'M':
    case 'm':
      POSITION_DEADBAND_COUNT = fabsf(val);  // M 位置死区，默认12 count（约1.11mm）
      break;
    case 'C':
    case 'c':
      remote::captureCurrentPositionTarget();  // C 捕获当前位置为新的位置目标（需带一个数字，值本身不用）
      break;
    default:
      prepareCommandResponse();
      reply.println("ERR:unknown cmd");  // 未知键：回显 ERR:unknown cmd
      return;
  }
  // 单字母写参数后：若键在 22 项表里，把 paramDirty 置 1（PAR 的 SAVED 变 0）。
  // 注意: 两字母通路（SP/SV/EG/MS）在函数开头就 return 了，不走这里 ——
  //   其中 SV 改的 speedKp 属于 22 项参数，所以改 SV 后 SAVED 不会变 0（既有行为）。
  // prepareCommandResponse() 当前是空实现（见 telemetry.cpp），回显直接入队。
  markDirtyIfParam(key);  // 改过就要提示"未保存"
  prepareCommandResponse();
  reply.print(key);
  reply.print("=");
  reply.print(val, 3);
  reply.println(" OK");
}

}  // namespace panel
