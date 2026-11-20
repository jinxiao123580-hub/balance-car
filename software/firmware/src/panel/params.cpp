#include "panel/params.h"

#include "remote/remote_control.h"

#include "panel/param_store.h"
#ifdef STM32F4xx
#include "panel/param_flash_f4.h"
#endif

namespace panel {

// 顺序必须与 PAR 回显、面板 22 项一致；改顺序或语义要同步升 ParamStore::FORMAT_VERSION
static const struct ParamSlotDef {
  char key;
  float * ptr;
}

paramSlots[ParamStore::PARAM_COUNT] = {
  { 'P', & balanceKp },
  { 'D', & balanceKd },
  { 'O', & balanceAngleOffset },
  { 'Z', & BALANCE_ANGLE_DEADBAND_DEG },
  { 'L', & BALANCE_OUTPUT_LIMIT_HZ },
  { 'S', & BALANCE_OUTPUT_SLEW_HZ },
  { 'R', & BALANCE_ACCEL_SCALE },
  { 'B', & HIGH_SPEED_ACCEL_START_HZ },
  { 'H', & HIGH_SPEED_ACCEL_LIMIT_HZ },
  { 'K', & speedKp },
  { 'I', & speedKi },
  { 'N', & MAX_SPEED_ANGLE },
  { 'W', & SPEED_INTEGRAL_LIMIT },
  { 'J', & SPEED_ERROR_DEADBAND },
  { 'E', & speedKd },
  { 'X', & positionKp },
  { 'Y', & MAX_POSITION_RETURN_SPEED },
  { 'M', & POSITION_DEADBAND_COUNT },
  { 'F', & OUTER_FULL_ANGLE_DEG },
  { 'G', & OUTER_DISABLE_ANGLE_DEG },
  { 'T', & turnKp },
  { 'U', & turnKd },
};

static float paramDefaults[ParamStore::PARAM_COUNT];  // 编译默认值（开机先抄一份）
static bool paramDirty = true;  // 当前 RAM 值 != flash 里已保存的记录
static bool paramSaved = false;  // flash 里有可 LOAD 的有效记录
static ParamStore::Store::Scan paramScan;
#ifdef STM32F4xx
static Stm32F4FlashPort paramFlashPort;
static ParamStore::FlashPort & paramPort = paramFlashPort;
#endif

static int paramSlotIndexOf(char key) {
  char upper = (char) toupper((unsigned char) key);
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) {
    if (paramSlots[i].key == upper) return (int) i;
  }
  return - 1;
}

static void paramCollect(float * values) {
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) values[i] = * paramSlots[i].ptr;
}

static void paramApply(const float * values) {
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) {
    if (isfinite(values[i])) * paramSlots[i].ptr = values[i];
  }
}

// 写 flash 会停住取指 0.5ms（换扇区擦除可到 1s+），必须车静止才允许
bool paramWriteAllowed() {
  return !remote::isActive() && !positionJog.active && imuFailCount == 0;
}

// 整词命令：只有"整条命令恰好是一个词"才算命中，避免和 P/D/O… 单字母命令抢
enum ParamWordCommand {
  PARAM_WORD_NONE = 0, PARAM_WORD_SAVE, PARAM_WORD_LOAD, PARAM_WORD_DEF
};

static ParamWordCommand paramWordCommand(const char * cmd, uint8_t len) {
  char word[8];
  uint8_t n = 0;
  bool truncated = false;
  while (len > 0 && (* cmd == ' ' || * cmd == '\t')) {
    ++cmd;
    --len;
  }
  while (len > 0 && * cmd != ' ' && * cmd != '\t' && * cmd != '\r' && * cmd != '\n') {
    if (n + 1u < sizeof (word)) word[n++] = (char) toupper((unsigned char) * cmd);
    else truncated = true;
    ++cmd;
    --len;
  }
  word[n] = '\0';
  while (len > 0 && (* cmd == ' ' || * cmd == '\t' || * cmd == '\r' || * cmd == '\n')) {
    ++cmd;
    --len;
  }
  if (truncated || n == 0 || len != 0) return PARAM_WORD_NONE;
  if (!strcmp(word, "SAVE")) return PARAM_WORD_SAVE;
  if (!strcmp(word, "LOAD")) return PARAM_WORD_LOAD;
  if (!strcmp(word, "DEF")) return PARAM_WORD_DEF;
  return PARAM_WORD_NONE;
}

// SAVE / LOAD / DEF 整词命令：返回 true = 已处理（原 handleBtCommand 里的整词 switch）
bool handleParamWord(const char * cmd, uint8_t len, QueuedBtReply & reply) {
  switch (paramWordCommand(cmd, len)) {
    case PARAM_WORD_SAVE:
      {
        if (!paramWriteAllowed()) {
          reply.println("ERR:SAVE needs idle vehicle");
          return true;
      }
      float values[ParamStore::PARAM_COUNT];
      paramCollect(values);
      bool erasedSector = false;
      bool ok = ParamStore::Store().save(paramPort, values, paramScan, erasedSector);
      paramSaved = ok;
      paramDirty = !ok;
      if (!ok) {
        reply.println("ERR:save failed");
        return true;
      }
      reply.print("SAVED OK seq=");
      reply.print((unsigned long) paramScan.seq);
      if (erasedSector) reply.print(" sectorErased=1");
      reply.println();
      return true;
    }
    case PARAM_WORD_LOAD:
      {
        if (!paramWriteAllowed()) {
          reply.println("ERR:LOAD needs idle vehicle");
          return true;
      }
      float values[ParamStore::PARAM_COUNT];
      ParamStore::Store::Scan sc = ParamStore::Store().scan(paramPort, values);
      paramScan = sc;
      if (!sc.found) {
        reply.println("ERR:no saved params");
        return true;
      }
      paramApply(values);
      paramSaved = true;
      paramDirty = false;
      reply.print("LOAD OK seq=");
      reply.println((unsigned long) sc.seq);
      return true;
    }
    case PARAM_WORD_DEF:
      {
        // 只覆盖 RAM，不动 flash：DEF 之后断电仍回到已保存的那一组，
        // 相当于"试一下出厂值"。要固化再发一条 SAVE。
        if (!paramWriteAllowed()) {
          reply.println("ERR:DEF needs idle vehicle");
          return true;
      }
      paramApply(paramDefaults);
      paramDirty = true;
      reply.println("DEF OK ram-only (SAVE to persist) SAVED=0");
      return true;
    }
    default:
      break;
  }

  return false;
}

void markDirtyIfParam(char key) {
  if (paramSlotIndexOf(key) >= 0) paramDirty = true;  // 改过就要提示"未保存"
}

int savedFlag() {
  return paramDirty ? 0: (paramSaved ? 1: 0);
}

void paramsBoot() {
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) {
    paramDefaults[i] = * paramSlots[i].ptr;
  }
  float values[ParamStore::PARAM_COUNT];
  paramScan = ParamStore::Store().scan(paramPort, values);
  paramSaved = paramScan.found;
  paramDirty = !paramScan.found;
  if (paramScan.found) {
    paramApply(values);
    BTSerial.print("PARAMS: loaded from flash seq=");
    BTSerial.print((unsigned long) paramScan.seq);
    BTSerial.print(" sector=");
    BTSerial.println((int) paramScan.sector);
    // 当前扇区写满就先擦好另一块：这 1.1s 放在开机，比放在运行中安全得多
    ParamStore::Store().prepare(paramPort, paramScan);
  } else {
    BTSerial.println("PARAMS: compiled defaults");
  }
}

}  // namespace panel
