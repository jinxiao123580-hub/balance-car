#pragma once
// ============================================================
// control/imu.{h,cpp} — BMI270 读写 / 400Hz 配置 / 开机校准
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"

namespace control {

// BMI270 实例（定义在 control/imu.cpp）
extern BMI270 imu;

bool bmiWriteRegister(uint8_t reg, uint8_t value);
bool bmiReadRegister(uint8_t reg, uint8_t & value);
bool configureBMI270_400Hz();
bool readBMI270();
bool initBMI270();
bool calibrateIMU();

}  // namespace control
