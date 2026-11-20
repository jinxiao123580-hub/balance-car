#include "control/imu.h"
#include "control/attitude.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// ============================================================
// BMI270 I2C 总线
TwoWire BMIWire(BMI_SDA_PIN, BMI_SCL_PIN);

// 三轴原始读数（g / deg/s，readBMI270() 写入）
float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;
float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

// IMU 零位
float pitchZero = PITCH_MOUNT_OFFSET_DEG;
float gyroYZero = 0.0f;
float gyroXZero = 0.0f;

// IMU 读取连续失败计数
uint8_t imuFailCount = 0;

namespace control {

// BMI270 实例
BMI270 imu;

// ============================================================
// BMI270底层寄存器写（仅用于400Hz配置回读）
// ============================================================
bool bmiWriteRegister(uint8_t reg, uint8_t value) {
  BMIWire.beginTransmission(BMI270_ADDRESS);
  BMIWire.write(reg);
  BMIWire.write(value);
  uint8_t error = BMIWire.endTransmission();
  delayMicroseconds(500);
  return error == 0;

}
// ============================================================
// BMI270底层寄存器读
// ============================================================
bool bmiReadRegister(uint8_t reg, uint8_t & value) {
  BMIWire.beginTransmission(BMI270_ADDRESS);
  BMIWire.write(reg);
  if (BMIWire.endTransmission(false) != 0) {
    return false;
  }
  uint8_t count = BMIWire.requestFrom(BMI270_ADDRESS, (uint8_t) 1);
  if (count != 1 || !BMIWire.available()) {
    return false;
  }
  value = BMIWire.read();
  return true;

}
// ============================================================
// 配置BMI270为400Hz
// ============================================================
bool configureBMI270_400Hz() {
  // 历史实车配置：ODR=400Hz，性能滤波模式。
  if (!bmiWriteRegister(BMI270_REG_ACC_CONF, BMI270_ACC_CONF_400HZ)) {
    return false;
  }
  delay(5);
  if (!bmiWriteRegister(BMI270_REG_GYR_CONF, BMI270_GYR_CONF_400HZ)) {
    return false;
  }
  delay(10);
  uint8_t accConf = 0;
  uint8_t gyrConf = 0;
  return bmiReadRegister(BMI270_REG_ACC_CONF, accConf) && bmiReadRegister(BMI270_REG_GYR_CONF, gyrConf) && accConf == BMI270_ACC_CONF_400HZ && gyrConf == BMI270_GYR_CONF_400HZ;
}
// ============================================================
// BMI270读取；SparkFun库返回g和deg/s，后续融合量纲无需修改
// ============================================================
bool readBMI270() {
  if (imu.getSensorData() != BMI2_OK) return false;
  // 当前实装相对历史竖装方向前后反向：绕X(上下)轴旋转180°。
  // 因此X保持，Y/Z同时翻转；这保证加速度角与俯仰角速度符号一致。
  ax = imu.data.accelX;
  ay = - imu.data.accelY;
  az = - imu.data.accelZ;
  gx = imu.data.gyroX;
  gy = - imu.data.gyroY;
  gz = - imu.data.gyroZ;
  return true;
}
// ============================================================
// BMI270初始化；beginI2C内部完成芯片识别、配置文件装载和传感器使能
// ============================================================
bool initBMI270() {
  // 冷上电时传感器电源可能晚于MCU稳定。初始化阶段先使用100kHz，
  // 完成配置文件装载后，setup()再切换到400kHz。
  delay(1500);
  BMIWire.begin();
  BMIWire.setClock(100000);
  delay(300);
  for (int i = 0; i < 12; i++) {
    BMIWire.beginTransmission(BMI270_ADDRESS);
    uint8_t probeError = BMIWire.endTransmission();
    if (probeError != 0) {
      delay(500);
      continue;
    }
    // 清除上一次未完成的配置装载状态，使每次重试从确定状态开始。
    bmiWriteRegister(BMI270_REG_CMD, BMI270_SOFT_RESET_CMD);
    delay(50);
    int8_t result = imu.beginI2C(BMI270_ADDRESS, BMIWire);
    if (result == BMI2_OK) {
      delay(200);
      if (imu.getSensorData() == BMI2_OK) return true;
    }
    BTSerial.print("BMI270 init retry ");
    BTSerial.print(i + 1);
    BTSerial.print(" err=");
    BTSerial.println((int) result);
    delay(500);
  }
  return false;
}
// ============================================================
// IMU校准
// 开机保持车体静止
// ============================================================
bool calibrateIMU() {
  // 开机静止采样只用于估计陀螺仪零偏和初始绝对倾角。
  // calculateAccelPitch() 直接表示重力相对 IMU 轴的倾角；
  // 只减去固定安装偏置，不减去开机时车体姿态。
  const float CAL_STATIC_GYRO_DPS = 4.0f;  // 静止判据：角速度小于此值(°/s)
  const float CAL_GRAV_MAG_TOL = 0.25f;  // 静止判据：|模长-1g| 容差(g)
  const int SAMPLE_TARGET = 400;  // 需累计的有效样本数
  const uint32_t CAL_TIMEOUT_MS = 20000;  // 总等待上限(毫秒) 保护
  delay(300);  // 上电稳定
  float pitchSum = 0.0f, gyroYSum = 0.0f, gyroXSum = 0.0f;
  int valid = 0;
  uint32_t tStart = millis();
  while (valid < SAMPLE_TARGET && (millis() - tStart) < CAL_TIMEOUT_MS) {
    if (readBMI270()) {
      float mag = sqrtf(ax * ax + ay * ay + az * az);
      bool still = (fabsf(gy) < CAL_STATIC_GYRO_DPS) && (fabsf(mag - 1.0f) < CAL_GRAV_MAG_TOL);
      if (still) {
        pitchSum += calculateAccelPitch();
        gyroYSum += gy;
        gyroXSum += gx;
        valid++;
      }
    }
    delay(3);
  }
  if (valid < SAMPLE_TARGET / 2) {
    return false;
  }
  pitchZero = PITCH_MOUNT_OFFSET_DEG;
  gyroYZero = gyroYSum / (float) valid;  // 陀螺零偏
  gyroXZero = gyroXSum / (float) valid;
  const float initialPitch = pitchSum / (float) valid - pitchZero;
  accPitch = initialPitch;
  pitch = initialPitch;
  kalmanAngle = initialPitch;
  kalmanBias = 0.0f;
  compAngle = initialPitch;
  pitchPrev = initialPitch;
  gyroIntegAngle = initialPitch;
  P00 = 1.0f;
  P01 = 0.0f;
  P10 = 0.0f;
  P11 = 1.0f;
  return true;
}

}  // namespace control
