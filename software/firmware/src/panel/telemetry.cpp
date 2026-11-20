#include "panel/telemetry.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地）
// ============================================================
// 蓝牙串口实例
// MKS TMC2209 V2.0 以独立 STEP/DIR 模式运行；不依赖 PDN_UART 启动。
// ============================================================
// STM32F407ZGT6 + BMI270 + DRV8825
// 正式控制版
//
// BMI270：ACC 400Hz + GYRO 400Hz
// 控制循环：固定400Hz
// Kalman：Angle + Gyro Bias
// 新增：基于加速度模长的动态R，降低运动/振动时accPitch权重
// 当前为增强版：小角度速度/位置拉回 + 3.5~6°平滑退外环 + >6°直立PD优先 + 短时0输出保持
// 平衡点：1.0°（balanceAngleOffset，用户实测调1更好）
// 当前：直立PD + 串级速度PI外环 + 位置拉回（转向环关闭）
//
// 已开启串级控制：融合速度作为速度PI反馈，速度PI输出目标倾角，直立PD输出STEP频率。
// 转向PD保留结构，当前参数为0。
// OLED完全不参与运行。
// ============================================================
HardwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

// 遥测分频
uint32_t telemetryDivider = 0;
uint32_t fastTelemetryDivider = 0;
// 收到命令后暂停遥测 400ms，把发送缓冲让给 PAR/参数回显（9600 波特下两者共用同一条 TX）

// 非阻塞遥测 txBuffer（9600 波特很慢，直接 print 会阻塞 400Hz 控制循环）
static const uint16_t TX_BUF_SIZE = 4096;
static char txBuffer[TX_BUF_SIZE];
static volatile uint16_t txHead = 0;
static volatile uint16_t txTail = 0;

// 每控制周期(2.5ms)累加的发送预算（字节）
// 800B/s * 0.0025s = 2B/周期
static const float TX_BUDGET_PER_CYCLE = 2.0f;
static float txBudgetBytes = 0.0f;
static const float TX_BUDGET_MAX = 40.0f;  // 预算上限，避免积压暴写

namespace panel {

void begin() {
  BTSerial.begin(BT_BAUD);
  delay(100);
}

// 写入环形缓冲（整行入队；空间不足则丢弃该行）
bool txPushLine(const char * line, uint16_t len) {
  if (len == 0 || len >= TX_BUF_SIZE) return false;
  // 检查是否有连续空间放 [len 字节 + 换行]
  uint16_t space;
  if (txHead >= txTail) space = (TX_BUF_SIZE - 1) - (txHead - txTail);
  else space = (txTail - txHead) - 1;
  if (space < len + 1) return false;
  for (uint16_t i = 0; i < len; i++) {
    txBuffer[txHead] = line[i];
    txHead = (txHead + 1) % TX_BUF_SIZE;
  }
  txBuffer[txHead] = '\n';
  txHead = (txHead + 1) % TX_BUF_SIZE;
  return true;
}

// loop 末尾按预算 flush（非阻塞：只写 UART 当前可容纳的字节）
void flushTelemetryTx() {
  static uint32_t lastBudgetUs = micros();
  uint32_t budgetNowUs = micros();
  txBudgetBytes += (uint32_t)(budgetNowUs - lastBudgetUs) * 0.0008f;
  lastBudgetUs = budgetNowUs;
  if (txBudgetBytes > TX_BUDGET_MAX) txBudgetBytes = TX_BUDGET_MAX;
  while (txHead != txTail && txBudgetBytes >= 1.0f) {
    int avail = BTSerial.availableForWrite();
    if (avail <= 0) break;
    int toSend = (int) txBudgetBytes;
    if (toSend > avail) toSend = avail;
    if (toSend <= 0) break;
    uint16_t n = 0;
    while (n < toSend && txHead != txTail) {
      BTSerial.write((uint8_t) txBuffer[txTail]);
      txTail = (txTail + 1) % TX_BUF_SIZE;
      n++;
    }
    txBudgetBytes -= (float) n;
  }
}

// 参数回显前丢弃尚未发送完的旧遥测，并先补换行，防止确认文本插入D行中间。
void prepareCommandResponse() {
}

// ============================================================
// 蓝牙发送
// 网页原来的解析格式无需修改：
// time_ms,accPitch,kalmanPitch
// ============================================================
void sendSlow() {
  // 完整控制链：直立P/D、目标角通道、直接制动原始/权限后、
  // 混合、总限幅、slew最终值、转向及左右电机命令。
  char line[896];  // 74 numeric fields; fixed-angle rate, trajectory phase and normal-loop blend appended.
  int n = snprintf(line, sizeof (line), "D,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%d,%d,%d,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", (unsigned long) millis(), (int)(pitch * 100.0f), // 1  Kalman倾角 *100
  (int)(gyroPitchRate * 100.0f), // 2  角速度 *100
  (int)(targetBalanceAngle * 100.0f), // 3  目标角 *100
  (int) balancePOutput, // 4  直立P项 Hz
  (int) balanceDOutput, // 5  直立D项 Hz
  (int) balanceOutput, // 6  直立PD合计 Hz
  (int)(speedAngleCorrection * 100.0f), // 7  速度目标角修正 *100
  (int)(outerLoopAuthority * 1000.0f), // 8  外环权限 *1000
  (int) 0, // 9  直接制动通道已删除（SPEC §5），列位保留填 0
  (int) 0, // 10 直接制动通道已删除（SPEC §5），列位保留填 0
  (int)(positionSpeedTarget * 1000.0f), // 11 位置环生成的目标速度 *1000
  (int) mixedBalanceOutput, // 12 直立+直接制动，限幅前 Hz
  (int) limitedBalanceOutput, // 13 总输出限幅后、slew前 Hz
  (int) finalBalanceOutput, // 14 slew后最终前后输出 Hz
  (int) turnOutput, // 15 转向环输出 Hz
  (int)(vehicleSpeed * 1000.0f), // 16 融合速度 *1000
  (int)(rawVehicleSpeed * 1000.0f), // 17 编码器原始速度 *1000
  (int) 0, // 18 旧预测通道已删除（SPEC §5），列位保留填 0
  (int)(telemetrySpeedSnapshot100Hz * 1000.0f), // 19 外环时刻速度快照 *1000（列位/语义不变）
  (int) 0, // 20 旧预测通道已删除（SPEC §5），列位保留填 0
  (int) 0, // 21 直接制动通道已删除（SPEC §5），列位保留填 0
  (int)(vehiclePosition * 100.0f), // 22 位置 *100
  (int)(positionError * 100.0f), // 23 位置误差 *100
  (long) encoder1Count, // 24 编码器1计数
  (long) encoder2Count, // 25 编码器2计数
  (int) motor1LastCommandHz, // 26 电机1实际命令 Hz
  (int) motor2LastCommandHz, // 27 电机2实际命令 Hz
  (int)(accPitch * 100.0f), // 28 加速度计角度 *100
  (int)(kalmanBias * 1000.0f), // 29 Kalman偏置 *1000
  (int)(gyroIntegAngle * 100.0f), // 30 陀螺积分角 *100
  (int) balanceAccelRawOutput, // 31 直立PD换算的原始频率增量 Hz/update
  (int) balanceAccelLimitedOutput, // 32 S限幅后的频率增量 Hz/update
  (int) integratedMotorSpeedCommand, // 33 增量积分后的STEP频率 Hz
  (int)(speedError * 1000.0f), // 34 速度误差 *1000
  (int)(speedPContribution * 100.0f), // 35 速度P项角度贡献 *100
  (int)(speedIContribution * 100.0f), // 36 速度I项角度贡献 *100
  (int)(speedAngleUnclamped * 100.0f), // 37 N限幅前速度目标角 *100
  (int)(speedAngleLimited * 100.0f), // 38 N限幅后、slew前目标角 *100
  (int) balanceAccelEffectiveLimit, // 39 本拍实际STEP频率增量限幅 Hz/update
  (int) rawSpeedEncoderMode, // 40 编码器来源 0=双路平均 1=仅enc1 2=仅enc2
  (int)(dynamicR * 1000.0f), // 41 dynamicR*1000（0.02~1.20，越大越不信accPitch）
  (int)(accelMagnitudeError * 1000.0f), // 42 |加速度模长-1g|*1000（动态R的驱动量，判据是否选错看这里）
  (unsigned long) fallStopCount, // 43 倒地保护累计触发次数（抓250ms采样漏掉的短促误触发）
  (int)(telemGyroAbsMax * 100.0f), // 44 窗口内max|gyro| *100  ← 判别陀螺是否有尖峰
  (int)(telemAccPitchDevMax * 100.0f), // 45 窗口内max|accPitch-gyroIntegAngle| *100
  (int) telemBalHzAbsMax, // 46 窗口内max|balanceOutput| Hz（饱和深度）
  (int)(telemKalmanBiasMin * 1000.0f), // 47 窗口内 kalmanBias 最小值 *1000
  (int)(telemKalmanBiasMax * 1000.0f), // 48 窗口内 kalmanBias 最大值 *1000
  (int)(manualForward.requested * 1000.0f), (int)(manualForward.applied * 1000.0f), (int)(0.0f), // manualI1000：自学习已移除，固定 0（保持 D 帧列数不变）
  (int) manualTurn.requested, (int) positionHoldSuspended, (manualForward.needsZero ? 1: 0) | (manualTurn.needsZero ? 2: 0), (unsigned long)(millis() - manualForward.receivedMs), 5, // manualControlMode: 5=fixed +/-2deg smooth hold
  (int)(targetVehiclePosition * 100), (int) positionJog.active, (unsigned long) positionJog.sequence, positionJog.reason, (encoder1Trusted && encoder2Trusted) ? 0: (encoder1Trusted ? 1: (encoder2Trusted ? 2: 3)), (int)(positionJog.speed * 1000), (int)(positionJog.goal * 100), (int)(positionJog.referenceSpeed * 1000), (int)(positionLean.requested * 1000), (int)(positionLean.applied * 1000), (int)(positionJog.arrival.error * 100), positionJog.arrival.goodWindows, (int)(positionLean.kp * 1000000), (int)(positionLean.limit * 1000), (int)(manualForward.rate * 1000), manualForward.phase, (int)(manualForward.normalAuthority() * 1000));
  if (n > 0 && (size_t) n < sizeof (line)) txPushLine(line, (uint16_t) n);
  // 窗口极值清零，开始累积下一个 250ms 窗口（必须在上面取值之后）
  diagResetWindow();
}

void sendFast() {
  char line[96];
  int n = snprintf(line, sizeof (line), "G,%lu,%d,%d,%d,%d,%d,%d,%d", (unsigned long) millis(), (int)(vehiclePosition * 100.0f), (int)(vehicleSpeed * 1000.0f), (int)(pitch * 100.0f), (int)(targetBalanceAngle * 100.0f), (int) finalBalanceOutput, (int)(manualForward.requested * 1000.0f), (int) btRxBytes);
  if (n > 0) txPushLine(line, (uint16_t) n);
}

}  // namespace panel
