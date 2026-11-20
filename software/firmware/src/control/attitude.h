#pragma once
// ============================================================
// control/attitude.{h,cpp} — 姿态融合：Kalman → pitch / pitchRate
// （原互补滤波函数为死代码，按 SPEC §5 删除）
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"

namespace control {

// 加速度计倾角（度）
// calculateAccelPitch：由加速度计 X/Z 分量解算重力方向倾角 ｜ 无参 ｜ 返回 单位 deg（±180，atan2 象限）。
//   调用：updateAttitude（400Hz）与 calibrateIMU（开机采样）｜ 边界：atan2(0,0)=0，不会出 NaN。
float calculateAccelPitch();

// Kalman 融合一阶（返回 kalmanAngle）
// updateKalman：二阶 Kalman（角度+陀螺零偏）融合一拍 ｜ 参数 measuredAngle=加速度角 deg、
//   measuredRate=陀螺角速度 deg/s、dt=周期 s（400Hz 时 0.0025）｜ 返回 融合角 kalmanAngle deg。
//   调用：updateAttitude 每 400Hz 拍 ｜ 边界：写入 P00..P11/kalmanBias；dynamicR 由调用方先算好，
//   本函数不做 NaN 守卫（输入来自有限值链路）。
float updateKalman(float measuredAngle, float measuredRate, float dt);

// 400Hz 姿态更新：accPitch / gyroPitchRate / dynamicR / pitch / gyroIntegAngle
// updateAttitude：400Hz 姿态更新总入口（无参/无返回）｜ 顺序：加速度角→俯仰角速度→模长/动态R→
//   Kalman 融合→毛刺抑制→陀螺积分角→前向加速度/可信度→转向角速度。
//   调用：control::tick400() 每拍一次（400Hz）｜ 输出：accPitch/pitch/gyroPitchRate/gyroTurnRate/
//   gyroIntegAngle/imuForwardAccelG/imuSpeedPredictionTrust 等全局量，供平衡环与速度预测读取。
void updateAttitude();

}  // namespace control
