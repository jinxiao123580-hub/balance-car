#pragma once
// ============================================================
// ============================================================
// panel/diag.h —— 调度/故障计数器与"发送周期窗口"极值的读接口
//
// 本文件没有任何状态，只有两个函数声明；状态定义在 diag.cpp，
// extern 声明集中在 core/state.h（跨模块唯一状态通道）。
//
// 涉及的全局量（全部 RAM-only，上电清零，不属于 22 项参数、不能 SAVE）：
//   controlOverrunCount   控制周期超时（掉拍）累计次数，次
//   controlMaxGapUs       相邻控制拍最大间隔，us（400Hz 拍应约 2500us）
//   controlMaxWorkUs      单拍控制计算最大耗时，us
//   fallStopCount         倒地保护累计触发次数，次（只增不减）
//   btRxBytes             BLE 累计收到字节数，B
//   telemGyroAbsMax       窗口内 max|gyroPitchRate|，度/s
//   telemAccPitchDevMax   窗口内 max|accPitch - gyroIntegAngle|，度
//   telemBalHzAbsMax      窗口内 max|balanceOutput|，Hz
//   telemKalmanBiasMin/Max 窗口内 kalmanBias 最小/最大值，度/s
//
// 窗口时长 = 两次 D 帧发送之间的时间。当前 D 帧 1Hz
// （core/cfg.h: TELEMETRY_DIVIDER=400），所以窗口实际为 1000ms。
// 注：旧描述写"250ms 窗口"，那是 D 帧还为 4Hz 时的历史值，以当前代码为准。
// ============================================================
// panel/diag.{h,cpp} — 计数器与 250ms 窗口极值
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"

namespace panel {

// diagAccumulate() —— 累积当前窗口的极值。
// 参数: 无（直接读 core/state.h 的全局量：gyroPitchRate / accPitch /
//       gyroIntegAngle / balanceOutput / kalmanBias）。
// 返回: void。调用者: main.cpp loop() 的 400Hz 控制段末尾，每拍一次（400Hz）。
// 边界: 只做 > / < 比较后覆盖，不做 NaN 防护；上游 Kalman 保证有限值才成立。
// loop 末尾每 400Hz 控制拍调用：累积 250ms 遥测窗口极值
void diagAccumulate();

// diagResetWindow() —— D 帧取完值后清零窗口，开始下一窗口。
// 参数: 无。返回: void。调用者: panel::sendSlow()，即每帧 D（当前 1Hz）。
// 边界: kalmanBias 的 min/max 以"当拍值"重新起算（不是 ±1e9），
//       因此极值语义是"本窗口相对当拍值的最小/最大偏移"，不是绝对值上下界。
// sendSlow() 取完值后调用：窗口清零、bias 以当拍值重开
void diagResetWindow();

}  // namespace panel
