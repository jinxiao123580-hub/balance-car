#pragma once
// ============================================================
// control/motor.{h,cpp} — STEP 频率输出 / 零保持 / 换向软斜坡 / 编码器 ISR
// 以及 TMC2209 双板 UART 初始化（只在非独立 STEP/DIR 模式下编译）。
// ============================================================
#include <Arduino.h>
#include <HardwareTimer.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"

namespace control {

// 1MHz 32-bit timers schedule GPIO STEP edges; no PWM preload latency.
struct StepOutput {
  HardwareTimer * timer;
  uint32_t mask;
  volatile uint32_t periodTicks = 1000000;
  volatile uint32_t risingEdges = 0;
  volatile bool running = false;
  volatile bool high = false;
  bool dir = false;
  StepOutput(HardwareTimer * t, uint32_t m): timer(t), mask(m) {
  }
};

extern StepOutput step1;
extern StepOutput step2;

// ---- 初始化 ---------------------------------------------------
void initMotors();
void initEncoders();

// ---- TMC2209 UART 配置（TMC_STANDALONE_STEP_DIR=1 时整段不编译）----
#if !TMC_STANDALONE_STEP_DIR
bool initTmcDrivers();
bool initTmcDriversWriteOnly(bool & rearLineHigh, bool & frontLineHigh);
#endif

// ---- 电机命令 -------------------------------------------------
void stopMotor1();
void stopMotor2();
void stopBothMotors();
void setMotor1(float commandHz);
void setMotor2(float commandHz);

// 直立输出 ± 转向输出 → 左右电机命令（原 loop 中的电机合成，逻辑逐字不变）
void mixMotors(float turnOffsetHz);

}  // namespace control
