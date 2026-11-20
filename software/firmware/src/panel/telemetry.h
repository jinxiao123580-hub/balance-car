#pragma once
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

// 非阻塞发送缓冲（整行入队，空间不足整行丢弃）
bool txPushLine(const char * line, uint16_t len);

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

// 蓝牙串口初始化（原 setup 里的 BTSerial.begin + delay(100)）
void begin();

void flushTelemetryTx();
void prepareCommandResponse();

// 遥测帧
void sendSlow();   // 原 sendTelemetry   （D 帧）
void sendFast();   // 原 sendFastTelemetry（G 帧）

}  // namespace panel
