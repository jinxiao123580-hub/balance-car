#pragma once
// ============================================================
// control/speed_est.{h,cpp} — 编码器累计 + 速度 Kalman 预测(400Hz)/修正(100Hz)
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"

namespace control {

// 400Hz：IMU 前向线加速度预测 vehicleSpeed（过程噪声随电机命令增长）
// predictVehicleSpeedKalman：速度 Kalman 的 400Hz 预测步（IMU 前向加速度积分进 vehicleSpeed）。
//   无参/无返回 ｜ 调用：tick400() 每拍 ｜ 边界：加速度死区 0.03g、单拍增量限幅 0.02 count/50ms。
void predictVehicleSpeedKalman();

// 100Hz：编码器速度作为绝对测量修正 vehicleSpeed
// correctVehicleSpeedKalman：速度 Kalman 的 100Hz 测量步（编码器速度作绝对测量）。
//   参数 encoderMeasurement 单位 count/50ms，向前为正 ｜ 无返回 ｜ 调用：tick100()。
//   边界：|meas|<2 → R=0.20（容忍 0/1 count 量化），否则 R=0.08；P 有 0.02 下限防迟钝。
void correctVehicleSpeedKalman(float encoderMeasurement);

// 100Hz 段入口（原 updateOuterLoop 前 4 步）：
// 编码器增量 → 滤波 → 信任门 → 原始速度 → Kalman 修正。
// 必须在 remote::tick100() 之前调用（remote 位置/点动用 vehicleSpeed）。
// tick100：100Hz 段入口（无参/无返回）｜ 编码器增量→count/50ms→低通→信任门→rawVehicleSpeed→Kalman 修正。
//   调用：main.cpp loop 每 4 个 400Hz 拍一次（100Hz/10ms），且必须在 remote::tick100() 之前（远控要用 vehicleSpeed）。
void tick100();

// 400Hz 段入口：读 IMU → 姿态 → 速度预测。成功返回 true。
// tick400：400Hz 段入口（无参）｜ 读 IMU→姿态更新→速度预测。
//   返回 true=本拍 IMU 读取成功；false=失败（main 计 imuFailCount，连续 8 次停机）｜ 调用：main loop 每拍。
bool tick400();

}  // namespace control
