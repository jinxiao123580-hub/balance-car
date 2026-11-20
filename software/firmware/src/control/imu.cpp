#include "control/imu.h"
#include "control/attitude.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// ============================================================
// BMI270 I2C 总线
// BMIWire：BMI270 的 I2C 总线（PB9=SDA/PB8=SCL）｜ 初始化 100kHz，setup 配置完成后切 400kHz。
// 边界：全局对象，构造即绑定引脚；initBMI270 里才 begin()。
TwoWire BMIWire(BMI_SDA_PIN, BMI_SCL_PIN);

// 三轴原始读数（g / deg/s，readBMI270() 写入）
// ax/ay/az：加速度计三轴 ｜ 单位 g（±2g 量程）｜ 写：readBMI270 400Hz（ay/az 取反）｜
//   读：updateAttitude/calibrateIMU ｜ 初始 0（首帧前无效）。
float ax = 0.0f;
// ay：加速度计 Y ｜ 单位 g ｜ readBMI270 里 = -data.accelY（安装绕 X 转 180°）｜ 400Hz。
float ay = 0.0f;
// az：加速度计 Z ｜ 单位 g ｜ readBMI270 里 = -data.accelZ ｜ 400Hz。
float az = 0.0f;
// gx：陀螺 X（转向轴）｜ 单位 deg/s ｜ 写：readBMI270（不取反）｜ 读：updateAttitude/calibrateIMU ｜ 400Hz。
float gx = 0.0f;
// gy：陀螺 Y（俯仰轴）｜ 单位 deg/s ｜ readBMI270 里 = -data.gyroY ｜ 400Hz。
float gy = 0.0f;
// gz：陀螺 Z（偏航轴）｜ 单位 deg/s ｜ readBMI270 里 = -data.gyroZ ｜ 读数保留，当前控制不用。
float gz = 0.0f;

// IMU 零位
// pitchZero：俯仰安装零点偏置 ｜ 单位 deg ｜ 初值 PITCH_MOUNT_OFFSET_DEG ｜ 写：calibrateIMU 重置 ｜
//   读：updateAttitude 算 accPitch（400Hz）｜ 设计上只补安装误差，不吸收开机姿态。
float pitchZero = PITCH_MOUNT_OFFSET_DEG;
// gyroYZero：俯仰轴陀螺零偏 ｜ 单位 deg/s ｜ 写：calibrateIMU（静止样本均值）｜ 读：updateAttitude 400Hz。
float gyroYZero = 0.0f;
// gyroXZero：转向轴陀螺零偏 ｜ 单位 deg/s ｜ 写：calibrateIMU ｜ 读：updateAttitude 400Hz。
float gyroXZero = 0.0f;

// IMU 读取连续失败计数
// imuFailCount：IMU 连续读取失败计数 ｜ 单位 次（0~255）｜ 写：main loop（失败++、成功清零）｜
//   读：main 停机判据、panel 参数写/点动/命令守卫 ｜ 400Hz。
uint8_t imuFailCount = 0;

namespace control {

// BMI270 实例
// imu：BMI270 驱动实例 ｜ 写：beginI2C 配置一次 ｜ 读：getSensorData()/getSensorData 每个 400Hz 拍。
BMI270 imu;

// ============================================================
// BMI270底层寄存器写（仅用于400Hz配置回读）
// ============================================================
// bmiWriteRegister：I2C 写一个寄存器。参数 reg=地址、value=1 字节；返回 true=ACK。
// 调用：configureBMI270_400Hz（写 ACC_CONF/GYR_CONF）、initBMI270（软复位）。
// 边界：endTransmission()!=0 返回 false；写完固定 delayMicroseconds(500) 等传感器内部就绪。
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
// bmiReadRegister：I2C 读一个寄存器。参数 reg=地址、value=输出（仅成功时写）；返回 true=成功。
// 流程：写地址→重复启动→requestFrom(1)→读。边界：count!=1 或无数据返回 false，value 保持不变。
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
// configureBMI270_400Hz：把 BMI270 配到 400Hz 性能滤波模式（原实车配置）。
// 无参；返回 true=ACC_CONF/GYR_CONF 写入后回读全等于 0xAA。调用：main setup() 一次。
// 边界：任一步 I2C 失败/回读不符 → false（setup 报错）；写后分别 delay 5ms/10ms 等 ODR 生效。
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
// readBMI270：读一帧 BMI270 六轴数据，写 ax..gz（单位 g / deg/s）。
// 无参；返回 false=getSensorData() 失败（I2C/传感器异常）；调用：control::tick400()，400Hz 每拍。
// 安装方向：实装相对历史竖装方向绕 X（上下）轴转 180°，故 X 保持、Y/Z 同时取反，
//   使加速度角与俯仰角速度符号一致（否则直立 PD 的 P/D 会打架）。
bool readBMI270() {
  if (imu.getSensorData() != BMI2_OK) return false;
  // 当前实装相对历史竖装方向前后反向：绕X(上下)轴旋转180°。
  // 因此X保持，Y/Z同时翻转；这保证加速度角与俯仰角速度符号一致。
  // 单位：加速度 g、角速度 deg/s（SparkFun 库已换算，后续融合不再换单位）。
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
// initBMI270：BMI270 上电初始化。无参；返回 true=芯片可用。调用：main setup() 一次。
// 流程：delay(1500) 等传感器电源稳定 → I2C 100kHz → 最多 12 次：探测地址→软复位(0x7E=0xB6,delay50)
//   →beginI2C→delay200→试读 getSensorData。
// 边界：任一次失败打印 retry 并 delay(500)；12 次都失败返回 false，由 setup 的 while 重试。
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
// calibrateIMU：开机静止校准。无参；返回 false=有效样本 <200（车在动/超时 20s），调用方重试。
// 判据（每个样本）：|gy|<4deg/s 且 | |a|-1g |<0.25g，累计 400 个有效样本，最长等 20s。
// 输出：gyroYZero/gyroXZero=样本均值（陀螺零偏）；pitchZero=固定安装偏置（不把开机姿态当零点）；
//   accPitch/pitch/kalmanAngle/compAngle/pitchPrev/gyroIntegAngle=初始倾角；kalmanBias=0；P=[1,0;0,1]。
// 边界：只做一次性初始化，运行中不再重标定；样本不足直接 false，不写坏状态。
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
