#include "remote/manual_pitch.h"

namespace remote {

// Parse the entire frame before changing either axis: ^ turnHz,pitchDegrees.
// parseManualMotion — 作用：把 "turnHz,pitchDeg" 两个字段整帧解析成功后才写回输出，
// 避免"改了转向、却发现前后字段非法"的半更新状态。
// 参数：value=命令载荷（不带 ^ 前缀，以 '\0' 结尾的 C 串，长度由 cmd 层限制 ≤31）；
//   turn/forward=输出引用：turn 单位 STEP Hz（差模），forward 单位 deg（档位，±2°）。
// 返回：bool——true 表示两个字段都合法且已写回；false 表示一个字段都没写。
// 调用：panel/cmd.cpp 的 ^ / @ 分支（人工命令级，非周期）。
// 边界：空/非数字/多个逗号/尾随垃圾/NaN/Inf 全部返回 false；strtof 溢出得到 HUGE_VALF 也被非有限检查拦下。
bool parseManualMotion(const char * value, float & turn, float & forward) {
  char * end = nullptr;
  // 先解析第一个字段（turn）：strtof 从 value 起读，end==value 说明没有数字。
  float t = std::strtof(value, & end);
  // 字段 1 必须紧跟逗号（*end==','），并且数值有限；否则整帧拒绝。
  if (end == value || * end != ',' || !std::isfinite(t)) return false;
  // 第二个字段从逗号后一位开始解析（second），并要求 strand 到字符串结尾（*end=='\0'）：
  // 多一个逗号或多一个字符都会拒绝，避免 "1,2,3" 这类误输入被部分接受。
  const char * second = end + 1;
  float f = std::strtof(second, & end);
  if (end == second || * end != '\0' || !std::isfinite(f)) return false;
  // 两个字段都合法后才写回输出引用（原子生效），调用方随后统一 command() 两个轴。
  turn = t;
  forward = f;
  return true;
}

}  // namespace remote
