#pragma once
// ============================================================
// panel/params.{h,cpp} — 22 项参数表 + SAVE/LOAD/DEF + 三层值语义
// FORMAT_VERSION 保持 3；参数表顺序与 PAR 回显、面板 22 项一致（一个字未动）。
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "panel/telemetry.h"

namespace panel {

// 开机：先抄编译默认值（DEF 用），再尝试用 flash 里的记录覆盖（原 paramStorageBoot）
void paramsBoot();

// SAVE / LOAD / DEF 整词命令（返回 true = 已处理）
bool handleParamWord(const char * cmd, uint8_t len, QueuedBtReply & reply);

// 写 flash 会停住取指：必须车静止（原 paramWriteAllowed）
bool paramWriteAllowed();

// 单字母参数被改动 → 标记"RAM 值 != flash 记录"（原 paramSlotIndexOf(key)>=0 → paramDirty=true）
void markDirtyIfParam(char key);

// PAR 回显里的 SAVED 标志：paramDirty ? 0 : (paramSaved ? 1 : 0)
int savedFlag();

}  // namespace panel
