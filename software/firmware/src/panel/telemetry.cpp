#include "panel/telemetry.h"

// ============================================================
// panel/telemetry.cpp —— 蓝牙实例、发送环形缓冲、D/G 帧
//
// 本文件落地的东西：
//   BTSerial            HardwareSerial 实例（USART3: TX=PB10, RX=PB11, 9600 波特）
//   telemetryDivider    D 帧分频计数（400Hz/400 = 1Hz）
//   fastTelemetryDivider G 帧分频计数（400Hz/100 = 4Hz）
//   txBuffer/txHead/txTail/txBudgetBytes  非阻塞发送环形缓冲（4096 B，纯 RAM）
//   sendSlow()          D 帧：74 个数值字段（含标签 75 个逗号字段）
//   sendFast()          G 帧：8 个数值字段
//
// 单位换算（所有字段通用）：
//   10.8 编码器 count = 1 mm；1 count/50ms = 20 count/s = 1.852 mm/s；
//   pos 字段 = vehiclePosition×100，电脑端 /1080 = mm。
// 持久化：本文件所有变量都是 RAM-only，上电清零；不进 22 项参数、SAVE 不涉及。
// ============================================================
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
// （注：旧描述与当前实现不符，以代码为准：上面 line 3~23 是历史版本说明 ——
//   ① "平衡点 1.0°（用户实测调 1 更好）"是早期结论，当前 O 默认 2.99 且随摆放面漂移
//      （实测 2.99 → 3.365 → 4.35）；
//   ② "STM32F407ZGT6 + BMI270 + DRV8825"里，后轮当前为 MKS TMC2209（STEP/DIR 独立模式），
//      前轮仍是 DRV8825；
//   ③ "转向PD保留结构"仍是事实，但 T/U 当前均为 0（转向环关闭）。）
// BTSerial —— 蓝牙串口实例（JDY-23，USART3）。
// 引脚: BT_RX_PIN=PB11 / BT_TX_PIN=PB10（core/cfg.h）；波特率 BT_BAUD=9600。
// 定义在这里 = core/state.h 里 extern HardwareSerial BTSerial 的落地。
// 全项目共用这一条 TX：遥测帧与命令回显都排队走它（见 txPushLine）。
HardwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);

// 遥测分频
// D 帧分频计数：每 400Hz 控制拍 +1，到 TELEMETRY_DIVIDER(400) 归零并发一帧 → 1Hz。
// RAM-only；上电 0（main.cpp setup 里也会重置）。
uint32_t telemetryDivider = 0;
// G 帧分频计数：每 400Hz 控制拍 +1，到 FAST_TELEMETRY_DIVIDER(100) 归零并发一帧 → 4Hz。
// 2026-11-12 起从 10Hz 降回 4Hz：10Hz 会挤占命令通道（实测 O 命令延迟 0.7~0.9s）。
uint32_t fastTelemetryDivider = 0;
// 注：下面这段"收到命令后暂停遥测 400ms…"是历史设计说明，当前实现没有该暂停逻辑
// （prepareCommandResponse() 是空函数）。
// 收到命令后暂停遥测 400ms，把发送缓冲让给 PAR/参数回显（9600 波特下两者共用同一条 TX）

// 非阻塞遥测 txBuffer（9600 波特很慢，直接 print 会阻塞 400Hz 控制循环）
// 发送环形缓冲大小：4096 B（RAM 常驻）。
// 9600 波特 ≈ 960 B/s，一帧 D 最长约 900 B，4KB 足够吸收"发一帧 D + 一条回显"的突发。
static const uint16_t TX_BUF_SIZE = 4096;
// 环形缓冲本体（纯 RAM，上电清零）。整行入队，flush 时按预算出队。
static char txBuffer[TX_BUF_SIZE];
// 写指针（生产者：sendSlow/sendFast/QueuedBtReply）。volatile：与 flush 同处主循环，
// 保留 volatile 是因为历史上曾在中断上下文写过，现仅主循环访问。
static volatile uint16_t txHead = 0;
// 读指针（消费者：flushTelemetryTx）。
static volatile uint16_t txTail = 0;

// 每控制周期(2.5ms)累加的发送预算（字节）
// 800B/s * 0.0025s = 2B/周期
// 每控制周期的发送预算：800 B/s × 2.5ms = 2 B/周期。
// 注：该常量目前**未被使用** —— flushTelemetryTx() 里直接写的是 0.0008f 字面量
//     （每 us 0.0008 B），两者数值一致；保留不删只为兼容/可读性。
static const float TX_BUDGET_PER_CYCLE = 2.0f;
// 累计可发送字节预算（float，B）：按时间累加、每发一个字节减 1。
// 只在 flushTelemetryTx() 里读写；上电 0。
static float txBudgetBytes = 0.0f;
// 预算上限 40 B：积压再久一次也只补发 40 B，避免长时间中断后突然猛写。
static const float TX_BUDGET_MAX = 40.0f;  // 预算上限，避免积压暴写

namespace panel {

// begin() —— 蓝牙串口初始化（main.cpp setup() 调一次）。
// 参数/返回: 无。边界: 9600 波特 + delay(100) 等模块稳定；调用后即可收发。
void begin() {
  BTSerial.begin(BT_BAUD);
  delay(100);
}

// 写入环形缓冲（整行入队；空间不足则丢弃该行）
// txPushLine() —— 整行写入环形缓冲（长度不含 '\n'，由本函数补）。
// 参数: line/len；返回: true=已入队，false=丢弃（len==0、len>=4096、剩余空间不足 len+1）。
// 边界/实现细节：
//   * 剩余空间按"环形留一格"计算：head>=tail 时 = 4095-(head-tail)，否则 tail-head-1；
//   * 整行原子性：先确认空间够，才逐字节拷入，所以不会出现半行；
//   * 空间不足时丢弃的是**整行**（调用方拿不到中间态），旧数据保留。
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
// flushTelemetryTx() —— 把环形缓冲按预算推给 UART（非阻塞，不等待发送完成）。
// 参数: 无。返回: void。调用者: main.cpp loop() 开头与末尾（每迭代一次，≥400Hz）。
// 实现要点：
//   * 用 micros() 差值累加预算：bytes += dt_us × 0.0008（= 800 B/s），上限 40 B；
//   * 每轮先看 BTSerial.availableForWrite()，<=0 直接 break（UART 满就下个周期再发）；
//   * 单次最多发 min(预算字节数, UART 可写字节数)，每发 1 字节预算 -1；
//   * 预算/上限只控制"发多快"，不改变帧内容；积压时表现为遥测延迟变大。
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
// prepareCommandResponse() —— 历史接口（设计意图见函数上方旧注释）。
// 参数/返回: 无。
// 注意：当前函数体为空 —— 不做"丢弃旧遥测/补换行"，命令回显与遥测帧共用同一条
// TX 队列、按入队顺序与 800B/s 预算发送。旧注释描述的是设计意图，以代码为准。
void prepareCommandResponse() {
}

// ============================================================
// 蓝牙发送
// 网页原来的解析格式无需修改：
// time_ms,accPitch,kalmanPitch
// ============================================================
// ============================================================
// sendSlow() —— D 帧（当前 1Hz，74 个数值字段）
//
// 帧结构: "D,<列1>,<列2>,...,<列74>" —— 按逗号切分共 75 个字段（第 0 个是标签 "D"）。
// 列号约定: 下表"列N"= 第 N 个数值字段（1 起）；电脑端去掉前导 "D," 再 split 得到的
//           数组下标 k 对应"列 k+1"。注意旧行内注释的编号 1..48 对应的是"列 2..49"
//           （它把 millis 当作了 API 维度上的第 0 项），两套编号不要混用。
// 单位/换算: 10.8 count = 1mm；1 count/50ms = 1.852 mm/s；
//           ×100 字段单位 = 0.01，×1000 字段单位 = 0.001。
// 边界: char line[896]；snprintf 返回 n>=896 时整行丢弃（不发送）。
//       格式串一个字符都不许动：电脑端脚本、CSV 契约、分析文档都按列号解析。
//
// 逐列清单：
//   列 1  millis                   上电毫秒计数，ms，%lu
//   列 2  pitch                    Kalman 倾角 ×100 → 0.01°
//   列 3  gyroPitchRate            俯仰角速度 ×100 → 0.01°/s
//   列 4  targetBalanceAngle       目标平衡角 ×100 → 0.01°
//   列 5  balancePOutput           直立 P 项，Hz
//   列 6  balanceDOutput           直立 D 项，Hz
//   列 7  balanceOutput            直立 PD 合计，Hz
//   列 8  speedAngleCorrection     速度环目标角修正 ×100 → 0.01°
//   列 9  outerLoopAuthority       外环权限 ×1000 → 0.001（0~1）
//   列 10 固定 0                    原速度直投通道（SPEC §5 删除，列位保留）
//   列 11 固定 0                    原速度直投限幅后（同上）
//   列 12 positionSpeedTarget      位置环目标速度 ×1000 → 0.001 count/50ms
//   列 13 mixedBalanceOutput       直立+直投混合、限幅前，Hz
//   列 14 limitedBalanceOutput     总限幅后、slew 前，Hz
//   列 15 finalBalanceOutput       最终 STEP 频率，Hz
//   列 16 turnOutput               转向环输出，Hz
//   列 17 vehicleSpeed             Kalman 融合速度 ×1000 → 0.001 count/50ms
//   列 18 rawVehicleSpeed          编码器原始速度 ×1000 → 0.001 count/50ms
//   列 19 固定 0                    原预测增量通道（SPEC §5 删除）
//   列 20 telemetrySpeedSnapshot100Hz 外环时刻速度快照 ×1000 → 0.001 count/50ms
//   列 21 固定 0                    原预测速度通道（SPEC §5 删除）
//   列 22 固定 0                    原直接制动通道（SPEC §5 删除）
//   列 23 vehiclePosition          位置 ×100 → 0.01 count（/1080 = mm）
//   列 24 positionError            位置误差 ×100 → 0.01 count
//   列 25 encoder1Count            编码器1 累计计数，count（%ld）
//   列 26 encoder2Count            编码器2 累计计数，count（%ld）
//   列 27 motor1LastCommandHz      电机1 实际命令，Hz
//   列 28 motor2LastCommandHz      电机2 实际命令，Hz
//   列 29 accPitch                 加速度计角度 ×100 → 0.01°
//   列 30 kalmanBias               Kalman 陀螺零偏 ×1000 → 0.001°/s
//   列 31 gyroIntegAngle           陀螺积分角 ×100 → 0.01°
//   列 32 balanceAccelRawOutput    未限幅的频率增量，Hz/10ms
//   列 33 balanceAccelLimitedOutput 限幅后的频率增量，Hz/10ms
//   列 34 integratedMotorSpeedCommand 增量积分后的 STEP 频率指令，Hz
//   列 35 speedError               速度误差 ×1000 → 0.001 count/50ms
//   列 36 speedPContribution       速度 P 项角度贡献 ×100 → 0.01°
//   列 37 speedIContribution       速度 I 项角度贡献 ×100 → 0.01°
//   列 38 speedAngleUnclamped      N 限幅前目标角 ×100 → 0.01°
//   列 39 speedAngleLimited        N 限幅后、slew 前目标角 ×100 → 0.01°
//   列 40 balanceAccelEffectiveLimit 本拍实际增量限幅，Hz/10ms
//   列 41 rawSpeedEncoderMode      编码器来源 0=双路平均 1=仅enc1 2=仅enc2
//   列 42 dynamicR                 动态 R ×1000（0.02~1.20，越大越不信 accPitch）
//   列 43 accelMagnitudeError      |加速度模长-1g| ×1000 → 0.001g
//   列 44 fallStopCount            倒地保护累计触发次数（%lu）
//   列 45 telemGyroAbsMax          窗口内 max|gyroPitchRate| ×100 → 0.01°/s
//   列 46 telemAccPitchDevMax      窗口内 max|accPitch-gyroIntegAngle| ×100 → 0.01°
//   列 47 telemBalHzAbsMax         窗口内 max|balanceOutput|，Hz
//   列 48 telemKalmanBiasMin       窗口内 kalmanBias 最小 ×1000 → 0.001°/s
//   列 49 telemKalmanBiasMax       窗口内 kalmanBias 最大 ×1000 → 0.001°/s
//   列 50 manualForward.requested  ^ 手动俯仰目标 ×1000 → 0.001°
//   列 51 manualForward.applied    ^ 手动俯仰当前值 ×1000 → 0.001°
//   列 52 固定 0                   manualI1000（自学习已移除，保持列数不变）
//   列 53 manualTurn.requested     手动转向命令，Hz（STEP 频率差分，非标定偏航率）
//   列 54 positionHoldSuspended    位置保持挂起，0/1
//   列 55 needsZero 位图           bit0=manualForward.needsZero，bit1=manualTurn.needsZero
//   列 56 ms 距上次 ^ 前向命令     (millis()-manualForward.receivedMs)，ms（%lu）
//   列 57 固定 5                   manualControlMode（5=fixed ±2° smooth hold）
//   列 58 targetVehiclePosition    位置目标 ×100 → 0.01 count
//   列 59 positionJog.active       位置点动进行中，0/1
//   列 60 positionJog.sequence     点动帧序号（%lu）
//   列 61 positionJog.reason       原因码 0 NONE / 1 RUNNING / 2 ARRIVED / 3 STOP /
//                                 4 LINK_LOST / 5 SOURCE_CHANGED / 6 FAULT / 7 TIMEOUT
//   列 62 编码器来源（信任门后）   0=双路 1=仅enc1 2=仅enc2 3=都不可信
//   列 63 positionJog.speed        ×1000 → 0.001 count/50ms（当前实现恒 0）
//   列 64 positionJog.goal         ×100 → 0.01 count
//   列 65 positionJog.referenceSpeed ×1000 → 0.001 count/50ms
//   列 66 positionLean.requested   ×1000 → 0.001°
//   列 67 positionLean.applied     ×1000 → 0.001°
//   列 68 positionJog.arrival.error ×100 → 0.01 count
//   列 69 positionJog.arrival.goodWindows 连续达标窗口数，个
//   列 70 positionLean.kp          ×1000000 → 1e-6 deg/count
//   列 71 positionLean.limit       ×1000 → 0.001°
//   列 72 manualForward.rate       ×1000 → 0.001°/s
//   列 73 manualForward.phase      0 IDLE / 1 MOVE / 2 BRAKE
//   列 74 manualForward.normalAuthority ×1000 → 0.001（0~1，^ 通路回归权重）
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

// ============================================================
// sendFast() —— G 帧（当前 4Hz，8 个数值字段）
// 帧结构: "G,<列1>,...,<列8>"，按逗号切分共 9 个字段（第 0 个是标签 "G"）。
// 电脑端脚本（software/analysis/ble_*.py）里先去掉 "G," 再 split：
//   f[0]=millis f[1]=pos f[2]=speed f[3]=pitch f[4]=target f[5]=output f[6]=manualReq f[7]=btRx
//   pos 换算 mm = f[1]/1080；speed 换算 mm/s = f[2]/1000 × 1.852。
// 逐列清单：
//   列 1 millis                    上电毫秒计数，ms，%lu
//   列 2 vehiclePosition           位置 ×100 → 0.01 count（/1080 = mm）
//   列 3 vehicleSpeed              Kalman 融合速度 ×1000 → 0.001 count/50ms
//   列 4 pitch                     Kalman 倾角 ×100 → 0.01°
//   列 5 targetBalanceAngle        目标平衡角 ×100 → 0.01°
//   列 6 finalBalanceOutput        最终 STEP 频率，Hz
//   列 7 manualForward.requested   ^ 手动俯仰目标 ×1000 → 0.001°
//   列 8 btRxBytes                 BLE 累计接收字节数，B
// 边界: char line[96]；当前字段最长约 70 字符，安全。
//   注意：这里只判 n>0、没有判 n<sizeof(line)（sendSlow 有）；
//   若以后往 G 帧加列导致超过 96 B，txPushLine 会按 snprintf 的"应写入长度"越界读，
//   加列时必须同时把这段判断补齐。
// 格式串一个字符都不许动（电脑端脚本按列号解析）。
// ============================================================
void sendFast() {
  char line[96];
  int n = snprintf(line, sizeof (line), "G,%lu,%d,%d,%d,%d,%d,%d,%d", (unsigned long) millis(), (int)(vehiclePosition * 100.0f), (int)(vehicleSpeed * 1000.0f), (int)(pitch * 100.0f), (int)(targetBalanceAngle * 100.0f), (int) finalBalanceOutput, (int)(manualForward.requested * 1000.0f), (int) btRxBytes);
  if (n > 0) txPushLine(line, (uint16_t) n);
}

}  // namespace panel
