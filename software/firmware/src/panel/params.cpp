#include "panel/params.h"

#include "remote/remote_control.h"

#include "panel/param_store.h"
#ifdef STM32F4xx
#include "panel/param_flash_f4.h"
#endif

// ============================================================
// panel/params.cpp —— 22 项参数表的落地 + 三层值语义 + 三个整词命令
//
// 本文件要做的事只有四件：
//   1) 把 22 个全局参数登记成一张"键 → 变量指针"的表（paramSlots）；
//   2) 开机把编译默认值抄一份（paramDefaults），并尝试用 flash 记录覆盖 RAM；
//   3) 整词命令 SAVE / LOAD / DEF；
//   4) 维护两个标志：paramDirty（RAM != flash 已保存）与 paramSaved（flash 里有有效记录），
//      它们合成 PAR 回显里的 SAVED。
//
// 单位换算（与 cmd.h 文件头一致）：
//   10.8 编码器 count = 1 mm；1 count/50ms = 20 count/s = 1.852 mm/s。
//   本文件只搬运 float，不解释单位；每项的单位/默认/实测边界见下面每个槽的注释。
//
// 持久化：只有这里会写 flash（经 ParamStore::Store().save()，扇区 10/11 双备份）。
//   RAM-only 的量（SP/MS/EG）不在本表里，SAVE 永远不保存它们。
// ============================================================
namespace panel {

// ============================================================
// 22 项参数槽表
// （注：旧描述"顺序必须与 PAR 回显、面板 22 项一致"与当前实现不符，以代码为准：
//   本数组的顺序 = flash 记录里 values[] 的顺序 = 历史编译期参数顺序：
//     P D O Z L S R B H K I N W J E X Y M F G T U
//   而 PAR 回显是面板显示顺序：
//     P D O Z L S R F G T U K I N W J E B H X Y M
//   两者键集合相同、顺序不同。改本数组顺序会让已有 flash 记录被按新语义解释，
//   所以必须同步升 ParamStore::FORMAT_VERSION。）
//
// 每个槽 = { 键, 变量指针 }；键区分大小写时由 paramSlotIndexOf() 统一转大写。
// 单位/默认值/实测边界逐个写在下面 22 行里。
// ============================================================
// 顺序必须与 PAR 回显、面板 22 项一致；改顺序或语义要同步升 ParamStore::FORMAT_VERSION
// 槽定义：一个键 + 一个指向全局参数的指针（指针指向 core/ 里定义的变量本体）。
static const struct ParamSlotDef {
  char key;
  float * ptr;
}

// 22 槽数组（下标 = flash 记录里 values[] 的下标）。
// 写法提醒：这些指针指向全局变量，命令与 DEF/LOAD 都直接改内存，立即生效。
paramSlots[ParamStore::PARAM_COUNT] = {
// 1  P  直立环 P：1° 角度误差 → 1000 Hz 原始输出（默认 1000；再乘 R=0.10 得每 10ms 增量）
  { 'P', & balanceKp },
// 2  D  直立环 D：陀螺角速度阻尼，单位 Hz per (度/s)（默认 100）
  { 'D', & balanceKd },
// 3  O  机械平衡点（目标角基准），单位度（默认 2.99）。实测随摆放面/装配漂移：2.99→3.365→4.35
  { 'O', & balanceAngleOffset },
// 4  Z  直立环角度死区，单位度（默认 0；历史 3→1.5→0.5→0，越小纠偏越积极）
  { 'Z', & BALANCE_ANGLE_DEADBAND_DEG },
// 5  L  直立输出总限幅，单位 Hz（默认 1750；finalBalanceOutput 钳位，防高频失步）
  { 'L', & BALANCE_OUTPUT_LIMIT_HZ },
// 6  S  每 10ms 端点的最大频率增量，单位 Hz/update（默认 100 = 10000 Hz/s）
  { 'S', & BALANCE_OUTPUT_SLEW_HZ },
// 7  R  直立 PD 输出 → 每 10ms 频率增量的比例，无量纲（默认 0.10）
  { 'R', & BALANCE_ACCEL_SCALE },
// 8  B  高速"继续加速"保护的启动频率，单位 Hz（默认 1000；制动/反向增量不受它限制）
  { 'B', & HIGH_SPEED_ACCEL_START_HZ },
// 9  H  高速继续加速时每 10ms 的增量上限，单位 Hz/update（默认 40）
  { 'H', & HIGH_SPEED_ACCEL_LIMIT_HZ },
// 10 K  速度环 P，单位 deg per (count/50ms)（默认 -0.10；-0.02 太小、≥0.20 猛冲撞翻）
  { 'K', & speedKp },
// 11 I  速度环 I，单位 deg per (count/50ms)·s（默认 0；加了会变极限环且稳态反向）
  { 'I', & speedKi },
// 12 N  速度修正角上限，单位度（默认 0.8 = 稳定边界；N=2.0 会反向猛拽翻车）
  { 'N', & MAX_SPEED_ANGLE },
// 13 W  积分累积限幅，单位 误差×秒（默认 50；纯 P 时无用）
  { 'W', & SPEED_INTEGRAL_LIMIT },
// 14 J  速度误差死区，单位 count/50ms（默认 2.0；只门控积分，不影响 P）
  { 'J', & SPEED_ERROR_DEADBAND },
// 15 E  速度误差微分增益，单位 deg per (count/50ms)/s（默认 0）
  { 'E', & speedKd },
// 16 X  位置环 P，单位 (count/50ms)/count（默认 0.02；置 0 = 关位置环）
  { 'X', & positionKp },
// 17 Y  位置环目标速度限幅，单位 count/50ms（默认 0.30 = 0.56 mm/s）
  { 'Y', & MAX_POSITION_RETURN_SPEED },
// 18 M  位置死区，单位 编码器 count（默认 12 ≈ 1.11 mm；先扣死区再乘 X）
  { 'M', & POSITION_DEADBAND_COUNT },
// 19 F  外环全权限角，单位度（默认 5：|pitch-O|<=5° 时 outerLoopAuthority=1）
  { 'F', & OUTER_FULL_ANGLE_DEG },
// 20 G  外环退出角，单位度（默认 10：>=10° 外环权限=0，只留直立 PD 救车）
  { 'G', & OUTER_DISABLE_ANGLE_DEG },
// 21 T  转向环 P，默认 0（转向环当前关闭，保留结构）
  { 'T', & turnKp },
// 22 U  转向环 D，默认 0（同上）
  { 'U', & turnKd },
};

// 编译默认值快照（开机 paramsBoot() 抄一份）。
// 用途: DEF 命令把 RAM 恢复成这一组；它不是 flash 记录，也不参与 SAVE。
static float paramDefaults[ParamStore::PARAM_COUNT];  // 编译默认值（开机先抄一份）
// paramDirty：当前 RAM 值是否与 flash 已保存记录不一致（初值 true = 开机尚未核对）。
// SAVE 成功置 false；SAVE 失败/LOAD 失败保持；单字母改参数置 true。
static bool paramDirty = true;  // 当前 RAM 值 != flash 里已保存的记录
// paramSaved：flash 里是否存在一条可 LOAD 的有效记录（开机扫描或 SAVE/LOAD 成功时置 true）。
// 注意它不代表"RAM == flash"，那要看 paramDirty；两者合成 SAVED 标志。
static bool paramSaved = false;  // flash 里有可 LOAD 的有效记录
// 最近一次扫描结果（有效记录所在扇区、seq、各扇区下一个空槽）。
// 只在 paramsBoot()/SAVE/LOAD 时更新；不持久化（每次上电靠扫描重建）。
static ParamStore::Store::Scan paramScan;
// 平台适配：F407 实机上参数区映射到 flash 扇区 10/11；
// 主机回归不定义 STM32F4xx，此时 paramPort 不存在，相关函数只在目标板编译。
#ifdef STM32F4xx
static Stm32F4FlashPort paramFlashPort;
static ParamStore::FlashPort & paramPort = paramFlashPort;
#endif

// paramSlotIndexOf() —— 键 → 槽下标。
// 参数: key 任一大小写字符。返回: 0..21，找不到返回 -1。
// 边界: 只做大写归一化后逐槽比较，O(22)，调用频率=命令级。
static int paramSlotIndexOf(char key) {
  char upper = (char) toupper((unsigned char) key);
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) {
    if (paramSlots[i].key == upper) return (int) i;
  }
  return - 1;
}

// paramCollect() —— 把 22 个参数的当前 RAM 值拷进数组（SAVE 用）。
// 参数: values 长度必须 >= PARAM_COUNT。返回: void。
static void paramCollect(float * values) {
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) values[i] = * paramSlots[i].ptr;
}

// paramApply() —— 把数组写回 22 个全局参数（LOAD / DEF 用）。
// 参数: values 长度必须 >= PARAM_COUNT。返回: void。
// 边界: 只写有限值（isfinite），NaN/Inf 那一项跳过保留原值 —— 防止坏记录把车带失控。
static void paramApply(const float * values) {
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) {
    if (isfinite(values[i])) * paramSlots[i].ptr = values[i];
  }
}

// 写 flash 会停住取指 0.5ms（换扇区擦除可到 1s+），必须车静止才允许
bool paramWriteAllowed() {
  return !remote::isActive() && !positionJog.active && imuFailCount == 0;
}

// 整词命令枚举（参数见 handleParamWord）。
// 整词命令：只有"整条命令恰好是一个词"才算命中，避免和 P/D/O… 单字母命令抢
enum ParamWordCommand {
  PARAM_WORD_NONE = 0, PARAM_WORD_SAVE, PARAM_WORD_LOAD, PARAM_WORD_DEF
};

// paramWordCommand() —— 把整行识别成 SAVE / LOAD / DEF（严格整词）。
// 参数: cmd/len = 整行。返回: 枚举值；不是这三个词则 PARAM_WORD_NONE。
// 边界:
//   * 先跳前导空格，再取一个由空格/TAB/CR/LF 界定的词（只取前 7 个字符）；
//   * 词后必须只剩空白（len==0 才算），否则 NONE —— 避免 "SAVE xxx" 被误当 SAVE；
//   * 词超长（truncated）→ NONE；大小写不敏感（统一转大写）。
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

// handleParamWord() —— 三个整词命令的执行体。
// 参数: cmd/len 整行；reply 回显缓冲。返回: true = 命中三条之一（含报错分支）。
// 调用者: handleLine()，在单字母解析之前。
// 边界: 三条都先过 paramWriteAllowed()（车必须静止）；
//       所有回显都在返回前由 QueuedBtReply 入队。
// SAVE / LOAD / DEF 整词命令：返回 true = 已处理（原 handleBtCommand 里的整词 switch）
bool handleParamWord(const char * cmd, uint8_t len, QueuedBtReply & reply) {
  switch (paramWordCommand(cmd, len)) {
      // SAVE：把 22 项当前 RAM 值写成一条 flash 记录。
      // 成功: "SAVED OK seq=N [sectorErased=1]"，paramSaved=true、paramDirty=false。
      // 失败: "ERR:save failed"，paramSaved=false、paramDirty=true（保持未保存提示）。
      // 可能擦扇区（1.1s 级停住取指）—— 所以先要求车静止。
      // 注意: 不保存 SP/MS/EG（它们不是 22 项），也不保存 O 之外的姿态量。
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
      // LOAD：从 flash 读最新一条记录覆盖 RAM。
      // 成功: "LOAD OK seq=N"（paramSaved=true、paramDirty=false）。
      // 没有记录: "ERR:no saved params"（RAM 保持不变）。
      // 边界: 记录不存在/CRC 错/版本不符/含非有限值 → scan 不认为 found。
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
      // DEF：把 RAM 恢复成编译默认值（只改 RAM，不动 flash）。
      // 回显 "DEF OK ram-only (SAVE to persist) SAVED=0"。
      // 用途: 现场"试一下出厂值"，满意再 SAVE；断电仍回到已保存的那一组。
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

// markDirtyIfParam() —— 单字母参数被改后置"未保存"。
// 参数: key = 命令字符（大小写均可）。返回: void。
// 边界: 键不在 22 项表里则无动作；两字母通路（SP/SV/EG/MS）不调用本函数。
void markDirtyIfParam(char key) {
  if (paramSlotIndexOf(key) >= 0) paramDirty = true;  // 改过就要提示"未保存"
}

// savedFlag() —— PAR 回显 SAVED 字段。
// 返回: 0 = 有改动未存（paramDirty）或 flash 从没有效记录；1 = RAM 与 flash 一致。
int savedFlag() {
  return paramDirty ? 0: (paramSaved ? 1: 0);
}

// paramsBoot() —— 开机参数初始化（main.cpp setup() 调一次）。
// 流程: 抄编译默认值 → 扫 flash → 有记录就覆盖 RAM 并打印 → prepare() 预擦备用扇区。
// 参数/返回: 无。边界: 扫描失败/无记录 → 保持编译默认值，paramSaved=false（SAVED=0）。
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
