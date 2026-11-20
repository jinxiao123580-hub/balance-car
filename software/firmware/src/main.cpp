#include <Arduino.h>
#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"
#include "control/motor.h"
#include "control/imu.h"
#include "control/attitude.h"
#include "control/speed_est.h"
#include "control/balance.h"
#include "control/cascade.h"
#include "remote/remote_control.h"
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <HardwareTimer.h>
#include <math.h>
#include "manual_motion.h"
#include "position_jog.h"
#include "motion_drive.h"
ManualPitchControl manualForward;

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
// ------------------------------------------------------------
// 转向环 PD
// 当前关闭
// ------------------------------------------------------------
float turnKp = 0.0f;
float turnKd = 0.0f;

uint8_t motor1ZeroHoldCount = 0;
uint8_t motor2ZeroHoldCount = 0;

uint8_t fallConfirmCount = 0;

uint32_t fallStopCount = 0;  // 倒地保护累计触发次数（遥测第43项，只增不减）
// ---- 250ms 遥测窗口极值保持（穿透 250ms 采样盲区，抓短促尖峰）----
// 用途：判定陀螺/加速度计是真的有尖峰（IMU 故障），还是车体真在剧烈振荡（控制增益问题）。
// 每控制拍（400Hz）在 loop 末尾累积，sendTelemetry() 读取后清零。
float telemGyroAbsMax = 0.0f;  // 窗口内 max|gyroPitchRate| 度/s
float telemAccPitchDevMax = 0.0f;  // 窗口内 max|accPitch - gyroIntegAngle| 度
float telemBalHzAbsMax = 0.0f;  // 窗口内 max|balanceOutput| Hz（量化饱和深度）
float telemKalmanBiasMin = 1e9f;  // 窗口内 kalmanBias 最小/最大（首次发送后由 sendTelemetry 重置为当拍值）
float telemKalmanBiasMax = - 1e9f;

// 车辆相对开机/重新站立位置的累计位移，单位为平均编码器count
float vehiclePosition = 0.0f;

// 位置零点对应的两个编码器累计计数
int32_t positionZeroCount1 = 0;
int32_t positionZeroCount2 = 0;

// 目标位置固定为0，即尽量回到启动位置
float targetVehiclePosition = 0.0f;

void paramStorageBoot();  // 开机从 flash 读回 22 项参数（定义在蓝牙命令区）

void clearControlOutputDiagnostics() {
  manualTurn.clear();
  manualForward.clear();
  positionJog.cancel(PositionJog::FAULT);
  positionLean.reset();
  positionSpeedReference.reset();  // Actual motor-stop fault only; odometry remains continuous.
  positionHoldSuspended = false;
  positionHoldWasEnabled = false;
  balancePOutput = 0.0f;
  balanceDOutput = 0.0f;
  balanceOutput = 0.0f;
  turnOutput = 0.0f;
  telemetrySpeedSnapshot100Hz = 0.0f;  // 遥测第19列快照（原 speedPredicted 列位）
  positionSpeedTarget = 0.0f;
  positionError = 0.0f;
  mixedBalanceOutput = 0.0f;
  limitedBalanceOutput = 0.0f;
  finalBalanceOutput = 0.0f;
  balanceAccelRawOutput = 0.0f;
  balanceAccelLimitedOutput = 0.0f;
  balanceAccelEffectiveLimit = 0.0f;
  highSpeedAccelProtectionActive = false;
  balanceAccelSubstepOutput = 0.0f;
  integratedMotorSpeedCommand = 0.0f;
  prevIntegratedSpeedCmdSign = 0;
  reversalSoftTicksRemaining = 0;
  balanceAccelDivider = 0;
  lastFinalBalanceOutput = 0.0f;
}

// ============================================================
// 调度
// ============================================================
uint32_t lastControlUs = 0;
uint32_t controlOverrunCount = 0;
uint32_t controlMaxGapUs = 0;
uint32_t controlMaxWorkUs = 0;
uint32_t outerLoopDivider = 0;

uint32_t telemetryDivider = 0;

uint32_t fastTelemetryDivider = 0;

// 收到命令后暂停遥测 400ms，把发送缓冲让给 PAR/参数回显（9600 波特下两者共用同一条 TX）
uint32_t telemetryMuteUntilMs = 0;

// BLE 接收诊断：面板是否真的把命令送到了车端（放在 4Hz 调试帧最后一列）
uint32_t btRxBytes = 0;
uint32_t btCmdCount = 0;

// ============================================================
// 非阻塞遥测 txBuffer（9600 波特很慢，直接 print 会阻塞 400Hz 控制循环）
// 方案：sendTelemetry 只把整行写入环形缓冲；loop 末尾按 800B/s 预算 flush。
// 预算不足时丢整行（不部分发送）。
// ============================================================
static const uint16_t TX_BUF_SIZE = 4096;
static char txBuffer[TX_BUF_SIZE];
static volatile uint16_t txHead = 0;
static volatile uint16_t txTail = 0;

// 每控制周期(2.5ms)累加的发送预算（字节）
// 800B/s * 0.0025s = 2B/周期
static const float TX_BUDGET_PER_CYCLE = 2.0f;
static float txBudgetBytes = 0.0f;
static const float TX_BUDGET_MAX = 40.0f;  // 预算上限，避免积压暴写

// 写入环形缓冲（整行入队；空间不足则丢弃该行）
static bool txPushLine(const char * line, uint16_t len) {
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

class QueuedBtReply: public Print {
  public:
  using Print::write;
  size_t write(uint8_t c) override {
    if (used >= sizeof (buffer) - 1) {
      overflow = true;
      return 0;
    }
    buffer[used++] = (char) c;
    return 1;
  }
  ~QueuedBtReply() {
    if (!overflow && used) txPushLine(buffer, used);
  }
  private:
  char buffer[768];
  uint16_t used = 0;
  bool overflow = false;
};

// ============================================================
// 蓝牙发送
// 网页原来的解析格式无需修改：
// time_ms,accPitch,kalmanPitch
// ============================================================
void sendTelemetry() {
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
  telemGyroAbsMax = 0.0f;
  telemAccPitchDevMax = 0.0f;
  telemBalHzAbsMax = 0.0f;
  telemKalmanBiasMin = kalmanBias;
  telemKalmanBiasMax = kalmanBias;
}

// ============================================================
// setup
// ============================================================
void sendFastTelemetry() {
  char line[96];
  int n = snprintf(line, sizeof (line), "G,%lu,%d,%d,%d,%d,%d,%d,%d", (unsigned long) millis(), (int)(vehiclePosition * 100.0f), (int)(vehicleSpeed * 1000.0f), (int)(pitch * 100.0f), (int)(targetBalanceAngle * 100.0f), (int) finalBalanceOutput, (int)(manualForward.requested * 1000.0f), (int) btRxBytes);
  if (n > 0) txPushLine(line, (uint16_t) n);
}

void setup() {
  control::initMotors();
  control::initEncoders();
  // --------------------------------------------------------
  // YahBoom_BL UART5
  // --------------------------------------------------------
  BTSerial.begin(BT_BAUD);
  delay(100);

#if TMC_STANDALONE_STEP_DIR
  // STEP/DIR 由原有 TIM1/TIM3 和 DIR 逻辑驱动，不访问 PA9/PA2。
  // MKS 的 EN(低有效)、VREF 电流、MS1/MS2 细分和 SPREAD 模式由硬件决定。
  BTSerial.println("TMC STEP/DIR standalone: UART disabled; mode/current/microsteps hardware-selected");
#elif TMC_GPIO_WRITE_ONLY
  control::stopBothMotors();
  bool rearLineHigh = false, frontLineHigh = false;
  while (!control::initTmcDriversWriteOnly(rearLineHigh, frontLineHigh)) {
    control::stopBothMotors();
    BTSerial.print("TMC WRITE-ONLY WAIT rearLineHigh=");
    BTSerial.print(rearLineHigh);
    BTSerial.print(" frontLineHigh=");
    BTSerial.println(frontLineHigh);
    delay(1000);
  }
  BTSerial.print("TMC WRITE-ONLY SENT rearBytes=");
  BTSerial.print(rearTmcUart.txCount);
  BTSerial.print(" frontBytes=");
  BTSerial.print(frontTmcUart.txCount);
  BTSerial.println(" spreadCycle=REQUESTED current=1200mA microsteps=16 UNVERIFIED");
#else
  // TMC2209 spreadCycle 配置(后轮)+PA9, 前轮+PA2；每次上电重写
  while (!control::initTmcDrivers()) {
    control::stopBothMotors();
    delay(1000);
  }
  BTSerial.println("TMC UART OK: both spreadCycle, 1/16 step");
#endif
  // --------------------------------------------------------
  // BMI270初始化
  // --------------------------------------------------------
  // 22 项参数：先抄编译默认值，能读到 flash 记录就覆盖（2026-11-10）
  paramStorageBoot();
  while (!control::initBMI270()) {
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    BTSerial.println("BOOT RETRY: BMI270 init failed");
    delay(1000);
  }
  BMIWire.setClock(400000);
  delay(20);
  // --------------------------------------------------------
  // 配置400Hz
  // --------------------------------------------------------
  while (!control::configureBMI270_400Hz()) {
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    BTSerial.println("BOOT RETRY: BMI270 400Hz config failed");
    delay(500);
  }
  // --------------------------------------------------------
  // 校准
  // --------------------------------------------------------
  while (!control::calibrateIMU()) {
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    BTSerial.println("BOOT RETRY: IMU calibration failed; keep car still");
    delay(1000);
  }
  BTSerial.println("BOOT OK: BMI270 ready");
  // --------------------------------------------------------
  // 编码器初值
  // --------------------------------------------------------
  noInterrupts();
  lastEncoder1Count = encoder1Count;
  lastEncoder2Count = encoder2Count;
  positionZeroCount1 = encoder1Count;
  positionZeroCount2 = encoder2Count;
  interrupts();
  vehiclePosition = 0.0f;
  targetVehiclePosition = 0.0f;
  positionError = 0.0f;
  positionSpeedTarget = 0.0f;
  positionHoldWasEnabled = false;
  // --------------------------------------------------------
  // 速度Kalman + 串级速度PI初始化
  // --------------------------------------------------------
  vehicleSpeed = 0.0f;
  rawVehicleSpeed = 0.0f;
  imuForwardAccelG = 0.0f;
  imuForwardAccelFilteredG = 0.0f;
  imuSpeedPredictionTrust = 1.0f;
  speedKalmanP = 1.0f;
  speedKalmanQ = SPEED_KALMAN_Q_BASE;
  speedKalmanR = SPEED_KALMAN_R_LOW_SPEED;
  speedError = 0.0f;
  finalBalanceOutput = 0.0f;
  targetForwardSpeed = 0.0f;
  targetBalanceAngle = balanceAngleOffset;
  outerLoopDivider = 0;
  telemetryDivider = 0;
  imuFailCount = 0;
  fallConfirmCount = 0;
  motor1ZeroHoldCount = 0;
  motor2ZeroHoldCount = 0;
  outerLoopAuthority = 1.0f;
  // 网页会忽略这一行表头
  BTSerial.println("time_ms,fused_speed,speed_direct_hz");
  lastControlUs = micros();

}

// ============================================================
// loop
// 固定400Hz
// ============================================================
// ============================================================
// 蓝牙在线调参（在 loop 每圈开头调用，不影响400Hz控制时序）
// 命令格式：一行一个，回车发送。 字母 + 空格 + 数值
//   P <kp>    改 balanceKp
//   D <kd>    改 balanceKd
//   O <offset> 改 balanceAngleOffset（平衡点）
//   Z <deg>   改 BALANCE_ANGLE_DEADBAND_DEG（死区）
//   L <hz>    改 BALANCE_OUTPUT_LIMIT_HZ（输出限幅）
//   S <hz>    改 BALANCE_OUTPUT_SLEW_HZ（每周期最大变化）
//   ?         回显当前全部参数
//   !         驱动诊断；! kp,limit 配置位置倾角点动
// ============================================================
// ============================================================
// 参数持久化（2026-11-10）：蓝牙 SAVE / LOAD / DEF
// ------------------------------------------------------------
// 记录格式与槽位扫描见 param_store.h，扇区读写见 param_flash_f4.h（sector 10/11）。
// 这里只做三件事：把 22 个全局参数登记成表、开机读回、处理三个整词命令。
// 为什么不再依赖 ST-Link：整定一次要重烧一次，现场根本跟不上调参节奏。
// ============================================================
#include "param_store.h"
#ifdef STM32F4xx
#include "param_flash_f4.h"
#endif

// 顺序必须与 PAR 回显、面板 22 项一致；改顺序或语义要同步升 ParamStore::FORMAT_VERSION
static const struct ParamSlotDef {
  char key;
  float * ptr;
}

paramSlots[ParamStore::PARAM_COUNT] = {
  { 'P', & balanceKp },
  { 'D', & balanceKd },
  { 'O', & balanceAngleOffset },
  { 'Z', & BALANCE_ANGLE_DEADBAND_DEG },
  { 'L', & BALANCE_OUTPUT_LIMIT_HZ },
  { 'S', & BALANCE_OUTPUT_SLEW_HZ },
  { 'R', & BALANCE_ACCEL_SCALE },
  { 'B', & HIGH_SPEED_ACCEL_START_HZ },
  { 'H', & HIGH_SPEED_ACCEL_LIMIT_HZ },
  { 'K', & speedKp },
  { 'I', & speedKi },
  { 'N', & MAX_SPEED_ANGLE },
  { 'W', & SPEED_INTEGRAL_LIMIT },
  { 'J', & SPEED_ERROR_DEADBAND },
  { 'E', & speedKd },
  { 'X', & positionKp },
  { 'Y', & MAX_POSITION_RETURN_SPEED },
  { 'M', & POSITION_DEADBAND_COUNT },
  { 'F', & OUTER_FULL_ANGLE_DEG },
  { 'G', & OUTER_DISABLE_ANGLE_DEG },
  { 'T', & turnKp },
  { 'U', & turnKd },
};

static float paramDefaults[ParamStore::PARAM_COUNT];  // 编译默认值（开机先抄一份）
static bool paramDirty = true;  // 当前 RAM 值 != flash 里已保存的记录
static bool paramSaved = false;  // flash 里有可 LOAD 的有效记录
static ParamStore::Store::Scan paramScan;
#ifdef STM32F4xx
static Stm32F4FlashPort paramFlashPort;
static ParamStore::FlashPort & paramPort = paramFlashPort;
#endif
static int paramSlotIndexOf(char key) {
  char upper = (char) toupper((unsigned char) key);
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) {
    if (paramSlots[i].key == upper) return (int) i;
  }
  return - 1;
}

static void paramCollect(float * values) {
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) values[i] = * paramSlots[i].ptr;
}

static void paramApply(const float * values) {
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) {
    if (isfinite(values[i])) * paramSlots[i].ptr = values[i];
  }
}

// 写 flash 会停住取指 0.5ms（换扇区擦除可到 1s+），必须车静止才允许
static bool paramWriteAllowed() {
  return !remote::isActive() && !positionJog.active && imuFailCount == 0;
}

// 开机：先抄编译默认值（DEF 用），再尝试用 flash 里的记录覆盖。
void paramStorageBoot() {
  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT;++i) {
    paramDefaults[i] = * paramSlots[i].ptr;
  }
  float values[ParamStore::PARAM_COUNT];
  paramScan = ParamStore::Store().scan(paramPort, values);
  paramSaved = paramScan.found;
  paramDirty = !paramScan.found;
  if (paramScan.found) {
    paramApply(values);
    BTSerial.print("PARAMS: loaded from flash seq=");
    BTSerial.print((unsigned long) paramScan.seq);
    BTSerial.print(" sector=");
    BTSerial.println((int) paramScan.sector);
    // 当前扇区写满就先擦好另一块：这 1.1s 放在开机，比放在运行中安全得多
    ParamStore::Store().prepare(paramPort, paramScan);
  } else {
    BTSerial.println("PARAMS: compiled defaults");
  }
}

// 整词命令：只有"整条命令恰好是一个词"才算命中，避免和 P/D/O… 单字母命令抢
enum ParamWordCommand {
  PARAM_WORD_NONE = 0, PARAM_WORD_SAVE, PARAM_WORD_LOAD, PARAM_WORD_DEF
};

static ParamWordCommand paramWordCommand(const char * cmd, uint8_t len) {
  char word[8];
  uint8_t n = 0;
  bool truncated = false;
  while (len > 0 && (* cmd == ' ' || * cmd == '\t')) {
    ++cmd;
    --len;
  }
  while (len > 0 && * cmd != ' ' && * cmd != '\t' && * cmd != '\r' && * cmd != '\n') {
    if (n + 1u < sizeof (word)) word[n++] = (char) toupper((unsigned char) * cmd);
    else truncated = true;
    ++cmd;
    --len;
  }
  word[n] = '\0';
  while (len > 0 && (* cmd == ' ' || * cmd == '\t' || * cmd == '\r' || * cmd == '\n')) {
    ++cmd;
    --len;
  }
  if (truncated || n == 0 || len != 0) return PARAM_WORD_NONE;
  if (!strcmp(word, "SAVE")) return PARAM_WORD_SAVE;
  if (!strcmp(word, "LOAD")) return PARAM_WORD_LOAD;
  if (!strcmp(word, "DEF")) return PARAM_WORD_DEF;
  return PARAM_WORD_NONE;
}

static char btCmdBuf[48];
static uint8_t btCmdLen = 0;
static bool btCmdOverflow = false;
void handleBtCommand(char * cmd, uint8_t len);  // 前置声明

// ------------------------------------------------------------
// 运行时可调的参数（2026-11-19 简化后只剩两个）：
//   EG <deg>   MotionLeanControl::errorGain   速度误差→倾角增益（^ 手动通路用）
//   SP <v>     目标速度（count/50ms，钳 ±MAX_TARGET_FORWARD_SPEED）
//              SP 是唯一的速度设定入口，进入经典速度 PI；与 ^ 手动通路互斥。
// 为什么用两字母：A~Z 已被面板参数占满，再开单字母会和 PAR 回显/面板解析打架；
// 两字母整词既不误命中，也不动 D 帧/CSV 契约。只改 RAM、不写 flash。
// ------------------------------------------------------------
static bool handleTunableCommand(const char * cmd, uint8_t len, QueuedBtReply & reply) {
  uint8_t lead = 0;
  while (lead < len && cmd[lead] == ' ')++lead;  // 前导空格
  if (len < lead + 2) return false;
  if (lead + 2 < len && cmd[lead + 2] != ' ' && cmd[lead + 2] != '\t') return false;
  const char a = (char) toupper((unsigned char) cmd[lead]);
  const char b = (char) toupper((unsigned char) cmd[lead + 1]);
  const bool isErrorGain = (a == 'E' && b == 'G');
  const bool isSpeedCmd = (a == 'S' && b == 'P');
  const bool isSpeedKp = (a == 'S' && b == 'V');  // 2026-11-19：速度环 P 增益在线可调
  const bool isMaxSpeed = (a == 'M' && b == 'S');  // 2026-11-20: speed range cap (RAM-only)
  if (!isErrorGain && !isSpeedCmd && !isSpeedKp && !isMaxSpeed) return false;
  uint8_t vstart = lead + 2;
  while (vstart < len && (cmd[vstart] == ' ' || cmd[vstart] == '\t'))++vstart;
  if (vstart >= len) {
    reply.println("ERR:SP/SV/EG/MS need a number");
    return true;
  }
  char vbuf[16];
  uint8_t vlen = 0;
  for (uint8_t i = vstart; i < len && vlen + 1u < sizeof (vbuf);++i) {
    const char c = cmd[i];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;
    vbuf[vlen++] = c;
  }
  vbuf[vlen] = '\0';
  char * end = nullptr;
  const float value = strtof(vbuf, & end);
  // 用区间判断挡 NaN/Inf，而不是 isfinite()：后者在主机回归里没有非限定声明
  if (end == vbuf || * end != '\0' || !(value > - 1.0e6f && value < 1.0e6f)) {
    reply.println("ERR:SP/SV/EG/MS need a number");
    return true;
  }
  if (isSpeedCmd) {
    speedCmd = ManualTurnControl::clamp(value, - MAX_TARGET_FORWARD_SPEED, MAX_TARGET_FORWARD_SPEED);
    reply.print("SP ok ");
    reply.println(speedCmd, 3);
    return true;
  }
  if (isMaxSpeed) {
    // range cap: below 0.5 means standstill, above 60 is far beyond the steppers
    MAX_TARGET_FORWARD_SPEED = ManualTurnControl::clamp(value, 0.5f, 60.0f);
    // shrinking the range may leave speedCmd out of bounds: pull it back at once
    speedCmd = ManualTurnControl::clamp(speedCmd, - MAX_TARGET_FORWARD_SPEED, MAX_TARGET_FORWARD_SPEED);
    reply.print("MS ok ");
    reply.print(MAX_TARGET_FORWARD_SPEED, 3);
    reply.print(" SP=");
    reply.println(speedCmd, 3);
    return true;
  }
  if (isSpeedKp) {
    // 极性与大小都要试：允许 ±，限 ±0.5（SP=8 时 0.12 约合 1.0° 倾角）
    speedKp = ManualTurnControl::clamp(value, - 0.5f, 0.5f);
    reply.print("SV ok ");
    reply.println(speedKp, 4);
    return true;
  }
  {
    // 误差增益开太大等于把 0.9° 上限一步打满，只允许 0~0.15
    motionLean.errorGain = ManualTurnControl::clamp(value, 0.0f, 0.15f);
  }
  reply.print(" EG ");
  reply.print(motionLean.errorGain, 3);
  reply.println();
  return true;
}

void pollBtCommand() {
  uint8_t readBudget = 32;
  while (readBudget--&& BTSerial.available() > 0) {
    char c = (char) BTSerial.read();
    btRxBytes++;
    if (c == '\n' || c == '\r') {
      if (!btCmdOverflow) handleBtCommand(btCmdBuf, btCmdLen);
      btCmdOverflow = false;
      btCmdLen = 0;
    } else if (btCmdLen < sizeof (btCmdBuf) - 1) {
      btCmdBuf[btCmdLen++] = c;
    } else {
      btCmdOverflow = true;
    }
  }
}

void handleBtCommand(char * cmd, uint8_t len) {
  QueuedBtReply reply;
  // 运行时可调增益先拦（TF/TB/EG），它们在下面被拆成单字母之前必须命中
  if (handleTunableCommand(cmd, len, reply)) return;
  // 整词命令先拦截：SAVE/LOAD/DEF 没有数值，若走下面的"key + 值"解析会被
  // 拆成 S+"AVE"、L+"OAD"、D+"EF" 然后被数值守卫静默丢弃。
  switch (paramWordCommand(cmd, len)) {
    case PARAM_WORD_SAVE:
      {
        if (!paramWriteAllowed()) {
          reply.println("ERR:SAVE needs idle vehicle");
          return;
      }
      float values[ParamStore::PARAM_COUNT];
      paramCollect(values);
      bool erasedSector = false;
      bool ok = ParamStore::Store().save(paramPort, values, paramScan, erasedSector);
      paramSaved = ok;
      paramDirty = !ok;
      if (!ok) {
        reply.println("ERR:save failed");
        return;
      }
      reply.print("SAVED OK seq=");
      reply.print((unsigned long) paramScan.seq);
      if (erasedSector) reply.print(" sectorErased=1");
      reply.println();
      return;
    }
    case PARAM_WORD_LOAD:
      {
        if (!paramWriteAllowed()) {
          reply.println("ERR:LOAD needs idle vehicle");
          return;
      }
      float values[ParamStore::PARAM_COUNT];
      ParamStore::Store::Scan sc = ParamStore::Store().scan(paramPort, values);
      paramScan = sc;
      if (!sc.found) {
        reply.println("ERR:no saved params");
        return;
      }
      paramApply(values);
      paramSaved = true;
      paramDirty = false;
      reply.print("LOAD OK seq=");
      reply.println((unsigned long) sc.seq);
      return;
    }
    case PARAM_WORD_DEF:
      {
        // 只覆盖 RAM，不动 flash：DEF 之后断电仍回到已保存的那一组，
        // 相当于"试一下出厂值"。要固化再发一条 SAVE。
        if (!paramWriteAllowed()) {
          reply.println("ERR:DEF needs idle vehicle");
          return;
      }
      paramApply(paramDefaults);
      paramDirty = true;
      reply.println("DEF OK ram-only (SAVE to persist) SAVED=0");
      return;
    }
    default:
      break;
  }
  // 跳过前导空格
  while (len > 0 && (* cmd == ' ' || * cmd == '\t')) {
    cmd++;
    len--;
  }
  if (len == 0) return;
  char key = * cmd;
  cmd++;
  len--;
  while (len > 0 && (* cmd == ' ' || * cmd == '\t')) {
    cmd++;
    len--;
  }
  uint8_t vlen = len;
  while (vlen > 0 && (cmd[vlen - 1] == ' ' || cmd[vlen - 1] == '\r' || cmd[vlen - 1] == '\n')) vlen--;
  if (key == '?') {
    prepareCommandResponse();
    reply.print("PAR: TURN=2 MOVE=5 SMOOTH=1 MOTION=2 FW=20261112F TILTMAX=");
    reply.print(ManualPitchControl::maxAngle, 2);
    reply.print(" ");
    reply.print("P=");
    reply.print(balanceKp, 2);
    reply.print(" D=");
    reply.print(balanceKd, 2);
    reply.print(" O=");
    reply.print(balanceAngleOffset, 2);
    reply.print(" Z=");
    reply.print(BALANCE_ANGLE_DEADBAND_DEG, 2);
    reply.print(" L=");
    reply.print(BALANCE_OUTPUT_LIMIT_HZ, 1);
    reply.print(" S=");
    reply.print(BALANCE_OUTPUT_SLEW_HZ, 1);
    reply.print(" R=");
    reply.print(BALANCE_ACCEL_SCALE, 4);
    reply.print(" F=");
    reply.print(OUTER_FULL_ANGLE_DEG, 2);
    reply.print(" G=");
    reply.print(OUTER_DISABLE_ANGLE_DEG, 2);
    reply.print(" T=");
    reply.print(turnKp, 2);
    reply.print(" U=");
    reply.print(turnKd, 2);
    reply.print(" K=");
    reply.print(speedKp, 2);
    reply.print(" I=");
    reply.print(speedKi, 2);
    reply.print(" N=");
    reply.print(MAX_SPEED_ANGLE, 2);
    reply.print(" W=");
    reply.print(SPEED_INTEGRAL_LIMIT, 1);
    reply.print(" J=");
    reply.print(SPEED_ERROR_DEADBAND, 2);
    reply.print(" E=");
    reply.print(speedKd, 3);
    reply.print(" B=");
    reply.print(HIGH_SPEED_ACCEL_START_HZ, 1);
    reply.print(" H=");
    reply.print(HIGH_SPEED_ACCEL_LIMIT_HZ, 1);
    reply.print(" X=");
    reply.print(positionKp, 4);
    reply.print(" Y=");
    reply.print(MAX_POSITION_RETURN_SPEED, 2);
    reply.print(" M=");
    reply.print(POSITION_DEADBAND_COUNT, 2);
    // 运动环的运行时增益（EG 可以改；SP 是当前速度指令）
    reply.print(" EG=");
    reply.print(motionLean.errorGain, 3);
    // SAVED=1：当前 RAM 参数与 flash 记录一致；0：改过没存（面板显示橙色提示）
    reply.print(" SAVED=");
    reply.print(paramDirty ? 0: (paramSaved ? 1: 0));
    // SP 追加在 PAR 行最末尾：唯一的速度指令（count/50ms），0=停车
    reply.print(" SP=");
    reply.print(speedCmd, 3);
    // SV：速度环 P 增益（在线可调，用于定极性与标定倾角/速度）
    reply.print(" SV=");
    reply.print(speedKp, 4);
    reply.print(" MS=");
    reply.print(MAX_TARGET_FORWARD_SPEED, 3);
    reply.println();
    reply.print("JOGCFG ");
    reply.print(positionLean.kp, 4);
    reply.print(' ');
    reply.println(positionLean.limit, 3);
    return;
  }
  if (key == '!' && vlen == 0) {
    prepareCommandResponse();
#if TMC_STANDALONE_STEP_DIR
    reply.println("TMC STEP/DIR standalone: hardware-selected mode");
    reply.print("STEP edges=");
    reply.print(control::step1.risingEdges);
    reply.print('/');
    reply.print(control::step2.risingEdges);
    reply.print(" periodUs=");
    reply.print(control::step1.periodTicks);
    reply.print('/');
    reply.print(control::step2.periodTicks);
    reply.print(" overrun=");
    reply.print(controlOverrunCount);
    reply.print(" maxGapUs=");
    reply.print(controlMaxGapUs);
    reply.print(" maxWorkUs=");
    reply.println(controlMaxWorkUs);
    reply.print("encoderMode=");
    reply.print(rawSpeedEncoderMode);
    reply.print(" softTicks=");
    reply.println(reversalSoftTicksRemaining);
#elif TMC_GPIO_WRITE_ONLY
    reply.print("TMC WRITE-ONLY SENT rearBytes=");
    reply.print(rearTmcUart.txCount);
    reply.print(" frontBytes=");
    reply.print(frontTmcUart.txCount);
    reply.println(" spreadCycle=REQUESTED microsteps=16 UNVERIFIED");
#else
    reply.println("TMC READBACK MODE");
#endif
    return;
  }
  if (vlen == 0) return;
  char vbuf[32];
  if (vlen >= sizeof (vbuf)) {
    reply.println("ERR:value too long");
    return;
  }
  for (uint8_t i = 0; i < vlen; i++) vbuf[i] = cmd[i];
  vbuf[vlen] = '\0';
  if (key == '!') {
    float kp = 0, limit = 0;
    if (!parseJogConfig(vbuf, kp, limit)) {
      reply.println("ERR:invalid jog config");
      return;
    }
    if (positionJog.active || positionLean.busy()) {
      reply.println("ERR:stop jog before config");
      return;
    }
    positionLean.kp = kp;
    positionLean.limit = limit;
    reply.print("JOGCFG ");
    reply.print(kp, 4);
    reply.print(' ');
    reply.println(limit, 3);
    return;
  }
  if (key == '~' || key == '&') {
    uint32_t id = 0;
    float delta = 0;
    const char * rest = nullptr;
    if (key == '&') {
      if (parseJogId(vbuf, id, rest) && * rest == '\0') positionJog.heartbeat(id, millis());
      return;
    }
    if (!parseJogFrame(vbuf, id, delta)) {
      reply.println("ERR:invalid jog");
      return;
    }
    if (delta != 0) {
      reply.println("ERR:use fixed-angle hold mode");
      return;
    }
    if (positionJog.newer(id)) {
      if (delta != 0 && (remote::isActive() || imuFailCount != 0 || fabsf(pitch - balanceAngleOffset) > 5 || fabsf(speedKp) < .000001f || positionLean.kp <= 0 || !(encoder1Trusted || encoder2Trusted))) {
        reply.println("ERR:jog needs upright idle vehicle");
        return;
      }
      if (delta == 0) {
        positionJog.start(id, 0, vehiclePosition, rawSpeedEncoderMode, millis());
        remote::captureCurrentPositionTarget();
      } else {
        remote::positionHold();
        positionJog.start(id, delta, vehiclePosition, rawSpeedEncoderMode, millis());
        targetVehiclePosition = positionJog.reference;
        positionHoldWasEnabled = positionKp > 0.000001f;
      }
    }
    if (id == positionJog.sequence) positionJog.heartbeat(id, millis());
    reply.print("JOG ");
    reply.print((unsigned long) positionJog.sequence);
    reply.print(' ');
    reply.print((int) positionJog.active);
    reply.print(' ');
    reply.println(positionJog.reason);
    return;
  }
  if (key == '^' || key == '@') {
    float turn, forward;
    if (!parseManualMotion(vbuf, turn, forward)) {
      reply.println("ERR:invalid motion");
      return;
    }
    if (key == '@' && (turn != 0 || forward != 0)) {
      reply.println("ERR:use fixed-angle ^ command");
      return;
    }
    // 2026-11-10: forward is a speed gear now, accept 0..maxAngle instead of only +/-2
    if (forward != 0 && fabsf(forward) > ManualPitchControl::maxAngle) {
      reply.println("ERR:forward gear out of range");
      return;
    }
    if (turn != 0 && positionJog.active) remote::stopPositionJog(PositionJog::STOP);
    if (forward != 0 && !manualForward.busy(millis())) {
      float currentOffset = targetBalanceAngle - balanceAngleOffset;
      if (manualForward.needsZero || imuFailCount != 0 || fabsf(pitch - balanceAngleOffset) > 5 || !isfinite(currentOffset) || fabsf(currentOffset) > ManualPitchControl::maxAngle) {
        reply.println("ERR:angle hold needs upright ready vehicle");
        return;
      }
      manualForward.seed(currentOffset);  // no jump from the current normal-loop target
    }
    uint32_t now = millis();
    manualTurn.command(turn, now);
    manualForward.command(forward, now);
    reply.println("MOTION OK");
    return;
  }
  // 值必须为数字，否则忽略（防 "SAVE"/"DEF" 等非数字串被误当 key+val=0 误设）
  {
    char c0 = vbuf[0];
    if (!((c0 >= '0' && c0 <= '9') || c0 == '-' || c0 == '.')) return;
  }
  char * valueEnd = nullptr;
  float val = strtof(vbuf, & valueEnd);
  if (valueEnd == vbuf || * valueEnd != '\0' || !isfinite(val)) {
    reply.println("ERR:invalid value");
    return;
  }
  switch (key) {
    case 'P':
    case 'p':
      balanceKp = val;
      break;
    case 'D':
    case 'd':
      balanceKd = val;
      break;
    case 'O':
    case 'o':
      balanceAngleOffset = val;
      break;
    case 'Z':
    case 'z':
      BALANCE_ANGLE_DEADBAND_DEG = val;
      break;
    case 'L':
    case 'l':
      BALANCE_OUTPUT_LIMIT_HZ = fabsf(val);
      break;
    case 'S':
    case 's':
      BALANCE_OUTPUT_SLEW_HZ = fabsf(val);
      break;
    case 'R':
    case 'r':
      BALANCE_ACCEL_SCALE = fabsf(val);
      break;
    case 'F':
    case 'f':
      OUTER_FULL_ANGLE_DEG = val;
      break;
    case 'G':
    case 'g':
      OUTER_DISABLE_ANGLE_DEG = val;
      break;
    case 'V':
    case 'v':
      if (val != 0 && positionJog.active) remote::stopPositionJog(PositionJog::STOP);
      manualTurn.command(val, millis());
      val = manualTurn.requested;
      break;
    case 'T':
    case 't':
      turnKp = val;
      break;
    case 'U':
    case 'u':
      turnKd = val;
      break;
    case 'K':
    case 'k':
      speedKp = val;
      break;
    case 'I':
    case 'i':
      speedKi = val;
      break;
    case 'N':
    case 'n':
      MAX_SPEED_ANGLE = val;
      break;
    case 'W':
    case 'w':
      SPEED_INTEGRAL_LIMIT = val;
      break;
    case 'J':
    case 'j':
      SPEED_ERROR_DEADBAND = val;
      break;
    case 'E':
    case 'e':
      speedKd = val;
      break;
    case 'H':
    case 'h':
      HIGH_SPEED_ACCEL_LIMIT_HZ = fabsf(val);
      break;
    case 'B':
    case 'b':
      HIGH_SPEED_ACCEL_START_HZ = fabsf(val);
      break;
    case 'X':
    case 'x':
      {
        bool wasEnabled = positionKp > 0.000001f;
        positionKp = fabsf(val);
        bool nowEnabled = positionKp > 0.000001f;
        if (!wasEnabled && nowEnabled && !positionHoldSuspended) remote::captureCurrentPositionTarget();
        if (!nowEnabled) {
          positionSpeedTarget = 0.0f;
          targetForwardSpeed = 0.0f;
      }
      positionHoldWasEnabled = nowEnabled;
      break;
    }
    case 'Y':
    case 'y':
      MAX_POSITION_RETURN_SPEED = fabsf(val);
      break;
    case 'M':
    case 'm':
      POSITION_DEADBAND_COUNT = fabsf(val);
      break;
    case 'C':
    case 'c':
      remote::captureCurrentPositionTarget();
      break;
    default:
      prepareCommandResponse();
      reply.println("ERR:unknown cmd");
      return;
  }
  if (paramSlotIndexOf(key) >= 0) paramDirty = true;  // 改过就要提示"未保存"
  prepareCommandResponse();
  reply.print(key);
  reply.print("=");
  reply.print(val, 3);
  reply.println(" OK");
}

void loop() {
  uint32_t nowUs = micros();
  // 蓝牙在线调参：每圈都轮询，独立于400Hz时序
  pollBtCommand();
  flushTelemetryTx();
  nowUs = micros();
  // --------------------------------------------------------
  // 没到2.5ms就直接返回
  // --------------------------------------------------------
  if ((uint32_t)(nowUs - lastControlUs) < CONTROL_PERIOD_US) {
    return;
  }
  // --------------------------------------------------------
  // 保持固定时间基准
  // --------------------------------------------------------
  uint32_t controlGapUs = nowUs - lastControlUs;
  if (controlGapUs > controlMaxGapUs) controlMaxGapUs = controlGapUs;
  if (controlGapUs >= CONTROL_PERIOD_US * 2UL) {
    controlOverrunCount++;
    lastControlUs = nowUs;
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    outerLoopDivider = 0;
  } else {
    lastControlUs += CONTROL_PERIOD_US;
  }
  uint32_t controlWorkStartUs = micros();
  // --------------------------------------------------------
  // 读取BMI270
  // --------------------------------------------------------
  if (!control::readBMI270()) {
    imuFailCount++;
    if (imuFailCount < IMU_FAIL_CONFIRM_COUNT) {
      // 保持上一有效STEP输出
      return;
    }
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    balanceOutput = 0.0f;
    finalBalanceOutput = 0.0f;
    speedIntegral = 0.0f;
    speedAngleCorrection = 0.0f;
    speedErrorPrev = 0.0f;
    speedErrorDotFilt = 0.0f;
    positionSpeedTarget = 0.0f;
    targetForwardSpeed = 0.0f;
    return;
  }
  imuFailCount = 0;
  // --------------------------------------------------------
  // 控制段首：Kalman姿态 + 多源速度Kalman预测 400Hz
  // （IMU负责快速感知“开始加速”的趋势；编码器会在外环20Hz时做绝对速度修正）
  // --------------------------------------------------------
  control::tick400();
  // --------------------------------------------------------
  // 蓝牙遥测20Hz
  // 放在倒地保护之前，这样即使触发倒地保护，
  // 手机上仍能继续看到IMU/Kalman数据。
  // --------------------------------------------------------
  telemetryDivider++;
  if (telemetryDivider >= TELEMETRY_DIVIDER) {
    telemetryDivider = 0;
    sendTelemetry();
  }
  fastTelemetryDivider++;
  if (fastTelemetryDivider >= FAST_TELEMETRY_DIVIDER) {
    fastTelemetryDivider = 0;
    sendFastTelemetry();
  }
  // --------------------------------------------------------
  // 倒地保护（连续确认，避免单帧尖峰突然停机）
  // --------------------------------------------------------
  float fallAngleError = fabsf(pitch - balanceAngleOffset);
  if (fallAngleError >= FALL_ANGLE_DEG) {
    fallConfirmCount++;
  } else {
    fallConfirmCount = 0;
  }
  bool confirmedFall = fallConfirmCount >= FALL_CONFIRM_COUNT;
  bool emergencyFall = fallAngleError >= EMERGENCY_FALL_ANGLE_DEG;
  if (confirmedFall || emergencyFall) {
    control::stopBothMotors();
    clearControlOutputDiagnostics();
    fallStopCount++;  // 倒地保护触发计数：250ms遥测窗无法分辨20ms确认窗内的短促误触发
    balanceOutput = 0.0f;
    turnOutput = 0.0f;
    speedError = 0.0f;
    speedIntegral = 0.0f;
    speedAngleCorrection = 0.0f;
    speedErrorPrev = 0.0f;
    speedErrorDotFilt = 0.0f;
    finalBalanceOutput = 0.0f;
    outerLoopAuthority = 0.0f;
    motor1ZeroHoldCount = 0;
    motor2ZeroHoldCount = 0;
    targetForwardSpeed = 0.0f;
    noInterrupts();
    int32_t resetCount1 = encoder1Count;
    int32_t resetCount2 = encoder2Count;
    interrupts();
    lastEncoder1Count = resetCount1;
    lastEncoder2Count = resetCount2;
    positionZeroCount1 = resetCount1;
    positionZeroCount2 = resetCount2;
    vehiclePosition = 0.0f;
    positionError = 0.0f;
    positionSpeedTarget = 0.0f;
    positionHoldWasEnabled = false;
    vehicleSpeed = 0.0f;
    rawVehicleSpeed = 0.0f;
    imuForwardAccelG = 0.0f;
    imuForwardAccelFilteredG = 0.0f;
    speedKalmanP = 1.0f;
    targetBalanceAngle = balanceAngleOffset;
    return;
  }
  // --------------------------------------------------------
  // 先在400Hz读取累计位置，20Hz位置外环使用最新位置。
  remote::tick(CONTROL_DT);
  remote::positionHold();
  // 速度PI外环100Hz（成熟步进平衡车串级结构）
  //
  // 400Hz控制循环，每4次执行一次 = 10ms。
  // --------------------------------------------------------
  outerLoopDivider++;
  if (outerLoopDivider >= 4) {
    outerLoopDivider = 0;
    control::tick100();      // 编码器速度估计 + Kalman 修正
    bool jogPos = false;
    bool jogAngle = false;
    if (!remote::tick100(jogPos, jogAngle)) {      // 远控运动驱动优先时接管本拍
      control::outerLoop(jogPos, jogAngle, remote::motionWasBusy(),
                          remote::manualPitchBusy(), remote::jogLeanOffset());
    }
    if (leanResetRequested) {                      // 速度环整体关闭 → 立刻清 jog 倾角
      remote::resetLean();
      leanResetRequested = false;
    }
  }
  // --------------------------------------------------------
  // 外环权限管理400Hz
  // 大角度时速度/位置环平滑退出，直立PD优先
  // --------------------------------------------------------
  control::updateAuthority(remote::normalAuthority(), remote::leanOffset());
  // --------------------------------------------------------
  // 位置已在外环计算前更新
  // --------------------------------------------------------
  // remote::positionHold() 已在本周期前面执行
  // --------------------------------------------------------
  // 直立环400Hz
  // --------------------------------------------------------
  control::updateBalance();
  // --------------------------------------------------------
  // 转向环400Hz
  // --------------------------------------------------------
  remote::updateTurn();
  // --------------------------------------------------------
  // 电机合成
  // --------------------------------------------------------
  control::mixMotors(remote::turnOffset());
  // ---- 250ms 遥测窗口极值累积（必须在所有控制量算完后）----
  {
    float aGyro = fabsf(gyroPitchRate);
    if (aGyro > telemGyroAbsMax) telemGyroAbsMax = aGyro;
    float aDev = fabsf(accPitch - gyroIntegAngle);
    if (aDev > telemAccPitchDevMax) telemAccPitchDevMax = aDev;
    float aBal = fabsf(balanceOutput);
    if (aBal > telemBalHzAbsMax) telemBalHzAbsMax = aBal;
    if (kalmanBias < telemKalmanBiasMin) telemKalmanBiasMin = kalmanBias;
    if (kalmanBias > telemKalmanBiasMax) telemKalmanBiasMax = kalmanBias;
  }
  // 非阻塞遥测 flush（每控制周期按预算发送缓冲内容）
  flushTelemetryTx();
  uint32_t workUs = micros() - controlWorkStartUs;
  if (workUs > controlMaxWorkUs) controlMaxWorkUs = workUs;
}
