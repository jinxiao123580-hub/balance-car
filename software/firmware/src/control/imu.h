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
// imu：SparkFun BMI270 驱动实例（定义在 imu.cpp）｜ 单位/范围：无（对象）｜
//   写：beginI2C 配置一次 ｜ 读：getSensorData() 每 400Hz 拍一次（readBMI270）。
extern BMI270 imu;

// bmiWriteRegister：直接写 BMI270 一个寄存器 ｜ 参数 reg=寄存器地址、value=1 字节值 ｜ 返回 true=ACK。
//   调用：configureBMI270_400Hz/initBMI270（开机配置与软复位）｜ 边界：I2C 无应答返回 false，写后固定等 500µs。
bool bmiWriteRegister(uint8_t reg, uint8_t value);
// bmiReadRegister：直接读 BMI270 一个寄存器 ｜ 参数 reg=寄存器地址、value=输出值（仅成功时写）｜
//   返回 true=成功 ｜ 调用：configureBMI270_400Hz 回读校验 ｜ 边界：NACK/无数据返回 false。
bool bmiReadRegister(uint8_t reg, uint8_t & value);
// configureBMI270_400Hz：写 ACC/GYR 配置为 0xAA（400Hz 性能滤波，历史实车值）并读回比对。
//   无参 ｜ 返回 true=两次写+回读全一致 ｜ 调用：main setup() 一次；失败则 setup 重试/报错。
bool configureBMI270_400Hz();
// readBMI270：读一帧六轴数据并做安装方向符号翻转（X 保持、Y/Z 取反），写全局 ax..gz。
//   无参 ｜ 返回 false=I2C/传感器读失败（main 据此累计 imuFailCount）｜ 调用：tick400()，400Hz 每拍一次。
bool readBMI270();
// initBMI270：BMI270 上电初始化（等电源稳定 → 探测地址 → 软复位 → beginI2C → 试读）。
//   无参 ｜ 返回 true=芯片就绪 ｜ 调用：main setup() 一次，失败由 setup 的 while 循环重试。
bool initBMI270();
// calibrateIMU：开机静止校准（估陀螺零偏 + 初始倾角，写 zero/初始姿态/Kalman 初值）。
//   无参 ｜ 返回 false=有效样本不足（<200 或超时 20s），main setup() 会要求摆正重试 ｜ 调用：setup 一次。
bool calibrateIMU();

}  // namespace control
