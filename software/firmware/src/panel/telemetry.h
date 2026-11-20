#pragma once
// ============================================================
// ============================================================
// panel/telemetry.{h,cpp} —— 蓝牙串口实例、非阻塞发送缓冲、G/D 遥测帧
//
// 一、两个帧（格式串定义在 telemetry.cpp，本文件只声明发送函数）：
//   G 帧 = sendFast()，8 个数值字段，当前 4Hz（core/cfg.h: FAST_TELEMETRY_DIVIDER=100）；
//   D 帧 = sendSlow()，74 个数值字段（含标签共 75 个逗号字段），当前 1Hz
//          （core/cfg.h: TELEMETRY_DIVIDER=400）。列号/单位见 telemetry.cpp 里的表。
//   两帧都走同一条 9600 波特 TX（JDY-23），由 loop 末尾按 800 B/s 预算非阻塞 flush。
//
// 二、单位换算（所有遥测字段通用）：
//   10.8 编码器 count = 1 mm；1 count/50ms = 20 count/s = 1.852 mm/s。
//   pos 字段 = vehiclePosition×100，电脑端 /1080 = mm（= count/10.8）。
//
// 三、调用关系：
//   panel::begin()    main.cpp setup() 一次（BTSerial.begin + 100ms 等待）；
//   panel::sendSlow() 每个 D 帧周期一次（1Hz），入队后清窗口极值；
//   panel::sendFast() 每个 G 帧周期一次（4Hz）；
//   panel::flushTelemetryTx() loop() 每个迭代都调（≥400Hz，非阻塞）。
// ============================================================
// panel/telemetry.{h,cpp} — G 帧(4Hz) / D 帧(1Hz，字段序号不变) + 非阻塞 TX
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"
#include "panel/diag.h"

#include "control/balance.h"
#include "control/cascade.h"
#include "control/motor.h"
#include "remote/manual_pitch.h"
#include "remote/manual_turn.h"
#include "remote/position_jog.h"

namespace panel {

// txPushLine() —— 把一整行（不含换行）写入发送环形缓冲，由 flushTelemetryTx() 择机发出。
// 参数: line/len = 数据与字节数（长度含数字字符，不含 '\n'；函数自己补 '\n'）。
// 返回: true = 整行已入队；false = 长度非法或剩余空间不足（整行丢弃，不部分发送）。
// 边界: 缓冲 4096 B；写满判断用"连续可用空间 = 环形剩余 - 1"，因此实际可用 4095 B；
//       len>=4096 直接丢弃。整行原子入队，所以电脑端不会读到半行。
// 调用者: sendSlow/sendFast、QueuedBtReply 析构。
// 非阻塞发送缓冲（整行入队，空间不足整行丢弃）
bool txPushLine(const char * line, uint16_t len);

// QueuedBtReply —— 命令回显缓冲（继承 Arduino Print）。
// 用法: 函数里 reply.print(...) 拼一整行，函数返回时析构自动 txPushLine()。
// 参数: 无构造参数。返回: write() 返回 1 = 收下，0 = 溢出（并置 overflow）。
// 边界: 缓冲 768 B；一旦溢出，本行整体丢弃（不会发半行）。超过 768 B 的回显目前只有
//       理论可能（PAR 行约 300 B，JOGCFG 行约 20 B），实际不会触发。
// 优点: 回显不直接碰 UART，9600 波特下也不会卡住 400Hz 控制环。
// 命令回显缓冲：析构时整行入队（原 QueuedBtReply）
class QueuedBtReply: public Print {
  public:
  using Print::write;
  size_t write(uint8_t c) override {
    if (used >= sizeof (buffer) - 1) {
      overflow = true;
      return 0;
    }
    buffer[used++] = (char) c;
    return 1;
  }
  ~QueuedBtReply() {
    if (!overflow && used) txPushLine(buffer, used);
  }
  private:
  char buffer[768];
  uint16_t used = 0;
  bool overflow = false;
};

// begin() —— 蓝牙串口初始化（由 main.cpp setup() 调用一次）。
// 参数: 无。返回: void。
// 边界: 内部 delay(100) 等 JDY-23 上电稳定；波特率固定 9600（BT_BAUD），
//       与模块出厂设置一致，改了要同步改模块。
// 蓝牙串口初始化（原 setup 里的 BTSerial.begin + delay(100)）
void begin();

// flushTelemetryTx() —— 按发送预算把环形缓冲里的字推给 UART（非阻塞）。
// 参数: 无。返回: void。调用者: main.cpp loop() 开头与末尾，每个迭代一次（≥400Hz）。
// 边界: 预算按 800 B/s 累加（9600 波特 10bit/字节 ≈ 960 B/s 的保守值），
//       单次最多攒 40 B；BTSerial 写不进去就直接 break，不等待、不丢数据。
// 预算/上限只影响"发多快"，不影响帧内容；积压时表现为遥测延迟变大。
void flushTelemetryTx();
// prepareCommandResponse() —— 历史接口：本意是"回显前丢弃尚未发完的旧遥测、补换行"。
// 参数/返回: 无。
// 注意：当前实现是**空函数**（telemetry.cpp 里函数体为空），即不做任何腾空/补换行，
//       回显行会排在已有遥测之后按预算慢慢发。注释保留原设计意图，以代码为准。
void prepareCommandResponse();

// 遥测帧
// sendSlow() —— 发送一帧 D 遥测（74 个数值字段，当前 1Hz）。
// 参数/返回: 无。调用者: main.cpp loop()，telemetryDivider>=TELEMETRY_DIVIDER 时一次。
// 边界: 帧内容超过 895 字节则整帧丢弃；发完调用 diagResetWindow() 清窗口极值。
void sendSlow();   // 原 sendTelemetry   （D 帧）
// sendFast() —— 发送一帧 G 遥测（8 个数值字段，当前 4Hz）。
// 参数/返回: 无。调用者: main.cpp loop()，fastTelemetryDivider>=FAST_TELEMETRY_DIVIDER 时一次。
// 边界: 行缓冲 96 B；当前字段最长约 70 字符，安全。
void sendFast();   // 原 sendFastTelemetry（G 帧）

}  // namespace panel
