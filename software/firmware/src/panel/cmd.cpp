#include "panel/cmd.h"

namespace panel {

static char btCmdBuf[48];
static uint8_t btCmdLen = 0;
static bool btCmdOverflow = false;

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
  if (isSpeedCmd) {
    speedCmd = clampf(value, - MAX_TARGET_FORWARD_SPEED, MAX_TARGET_FORWARD_SPEED);
    reply.print("SP ok ");
    reply.println(speedCmd, 3);
    return true;
  }
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
  if (isSpeedKp) {
    // 极性与大小都要试：允许 ±，限 ±0.5（SP=8 时 0.12 约合 1.0° 倾角）
    speedKp = clampf(value, - 0.5f, 0.5f);
    reply.print("SV ok ");
    reply.println(speedKp, 4);
    return true;
  }
  {
    // 误差增益开太大等于把 0.9° 上限一步打满，只允许 0~0.15
    motionLean.errorGain = clampf(value, 0.0f, 0.15f);
  }
  reply.print(" EG ");
  reply.print(motionLean.errorGain, 3);
  reply.println();
  return true;
}

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

void handleLine(char * cmd, uint8_t len) {
  QueuedBtReply reply;
  // 运行时可调增益先拦（TF/TB/EG），它们在下面被拆成单字母之前必须命中
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
  if (vlen == 0) return;
  char vbuf[32];
  if (vlen >= sizeof (vbuf)) {
    reply.println("ERR:value too long");
    return;
  }
  for (uint8_t i = 0; i < vlen; i++) vbuf[i] = cmd[i];
  vbuf[vlen] = '\0';
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
  switch (key) {
    case 'P':
    case 'p':
      balanceKp = val;
      break;
    case 'D':
    case 'd':
      balanceKd = val;
      break;
    case 'O':
    case 'o':
      balanceAngleOffset = val;
      break;
    case 'Z':
    case 'z':
      BALANCE_ANGLE_DEADBAND_DEG = val;
      break;
    case 'L':
    case 'l':
      BALANCE_OUTPUT_LIMIT_HZ = fabsf(val);
      break;
    case 'S':
    case 's':
      BALANCE_OUTPUT_SLEW_HZ = fabsf(val);
      break;
    case 'R':
    case 'r':
      BALANCE_ACCEL_SCALE = fabsf(val);
      break;
    case 'F':
    case 'f':
      OUTER_FULL_ANGLE_DEG = val;
      break;
    case 'G':
    case 'g':
      OUTER_DISABLE_ANGLE_DEG = val;
      break;
    case 'V':
    case 'v':
      if (val != 0 && positionJog.active) remote::stopPositionJog(PositionJog::STOP);
      manualTurn.command(val, millis());
      val = manualTurn.requested;
      break;
    case 'T':
    case 't':
      turnKp = val;
      break;
    case 'U':
    case 'u':
      turnKd = val;
      break;
    case 'K':
    case 'k':
      speedKp = val;
      break;
    case 'I':
    case 'i':
      speedKi = val;
      break;
    case 'N':
    case 'n':
      MAX_SPEED_ANGLE = val;
      break;
    case 'W':
    case 'w':
      SPEED_INTEGRAL_LIMIT = val;
      break;
    case 'J':
    case 'j':
      SPEED_ERROR_DEADBAND = val;
      break;
    case 'E':
    case 'e':
      speedKd = val;
      break;
    case 'H':
    case 'h':
      HIGH_SPEED_ACCEL_LIMIT_HZ = fabsf(val);
      break;
    case 'B':
    case 'b':
      HIGH_SPEED_ACCEL_START_HZ = fabsf(val);
      break;
    case 'X':
    case 'x':
      {
        bool wasEnabled = positionKp > 0.000001f;
        positionKp = fabsf(val);
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
      MAX_POSITION_RETURN_SPEED = fabsf(val);
      break;
    case 'M':
    case 'm':
      POSITION_DEADBAND_COUNT = fabsf(val);
      break;
    case 'C':
    case 'c':
      remote::captureCurrentPositionTarget();
      break;
    default:
      prepareCommandResponse();
      reply.println("ERR:unknown cmd");
      return;
  }
  markDirtyIfParam(key);  // 改过就要提示"未保存"
  prepareCommandResponse();
  reply.print(key);
  reply.print("=");
  reply.print(val, 3);
  reply.println(" OK");
}

}  // namespace panel
