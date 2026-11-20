#include "panel/diag.h"

// ============================================================
// panel/diag.cpp —— 计数器与窗口极值的落地定义
//
// 本文件里所有全局量都是 RAM-only：上电由 .bss/.data 初始化，复位即丢，
// 不进 param_store 的 22 项参数表，面板 SAVE 不会保存它们。
// 单位见 diag.h 的清单；D 帧第 43~49 列（CSV 列号，见 telemetry.cpp）是这些量的遥测。
// ============================================================
// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地）
// ============================================================
// ---- 调度/掉拍/耗时计数（D 帧 43 列之外由 `!` 诊断命令回读）----
// 单位: controlOverrunCount=次；controlMaxGapUs/controlMaxWorkUs=us，只增不减。
// 20000us 的间隔即判一次掉拍；! 命令打印 overrun / maxGapUs / maxWorkUs。
// 调度/掉拍/工作耗时计数器
uint32_t controlOverrunCount = 0;
uint32_t controlMaxGapUs = 0;
uint32_t controlMaxWorkUs = 0;

// 倒地保护累计触发次数（遥测第43项/CSV 列 44，只增不减）。
// 用途：250ms→现在的 1s 采样仍可能漏掉 20ms 确认窗内的短促误触发，
//       这个只增计数是"是否真的触发过"的唯一可靠证据。
// 倒地保护累计触发次数（遥测第43项，只增不减）
uint32_t fallStopCount = 0;  // 倒地保护累计触发次数（遥测第43项，只增不减）

// BLE 接收累计字节数（G 帧最后一列 / D 帧无）。
// 用途: 判断面板命令到底有没有到车端（=0 说明 UART/接线/波特率问题）。
// BLE 接收诊断（面板是否真的把命令送到了车端）
// BLE 接收诊断：面板是否真的把命令送到了车端（放在 4Hz 调试帧最后一列）
uint32_t btRxBytes = 0;

// ---- 窗口极值保持（穿透 D 帧采样盲区，抓短促尖峰）----
// 用途：判定陀螺/加速度计是真的有尖峰（IMU 故障），还是车体真在剧烈振荡（增益问题）。
// 每控制拍（400Hz）在 loop 末尾累积，sendSlow() 读取后由 diagResetWindow() 清零。
// 注：下面 18~23 行有内容相同的两段旧注释（重构时重复保留），不删，以代码为准。
// ---- 250ms 遥测窗口极值保持（穿透 250ms 采样盲区，抓短促尖峰）----
// 用途：判定陀螺/加速度计是真的有尖峰（IMU 故障），还是车体真在剧烈振荡（控制增益问题）。
// 每控制拍（400Hz）在 loop 末尾累积，sendSlow() 读取后清零。
// ---- 250ms 遥测窗口极值保持（穿透 250ms 采样盲区，抓短促尖峰）----
// 用途：判定陀螺/加速度计是真的有尖峰（IMU 故障），还是车体真在剧烈振荡（控制增益问题）。
// 每控制拍（400Hz）在 loop 末尾累积，sendTelemetry() 读取后清零。
// ---- 窗口极值量（下面 5 行；行尾旧注释已在，这里补默认值与极值语义）----
// 默认/初值：telemGyroAbsMax=0；telemAccPitchDevMax=0；telemBalHzAbsMax=0；
//             telemKalmanBiasMin=+1e9、telemKalmanBiasMax=-1e9（表示窗口尚未累积，
//             第一次 diagResetWindow() 后会被当拍 kalmanBias 覆盖）。
// 单位：度/s、度、Hz、度/s、度/s。范围：物理上受 IMU 量程与限幅约束；这里的 max 只增到窗口结束。
// 持久化：RAM-only，上电清零。
float telemGyroAbsMax = 0.0f;  // 窗口内 max|gyroPitchRate| 度/s
float telemAccPitchDevMax = 0.0f;  // 窗口内 max|accPitch - gyroIntegAngle| 度
float telemBalHzAbsMax = 0.0f;  // 窗口内 max|balanceOutput| Hz（量化饱和深度）
float telemKalmanBiasMin = 1e9f;  // 窗口内 kalmanBias 最小/最大（首次发送后由 sendTelemetry 重置为当拍值）
float telemKalmanBiasMax = - 1e9f;

namespace panel {

// diagAccumulate() —— 400Hz 每拍累积窗口极值（实现见 diag.h 的接口说明）。
// 参数: 无。返回: void。调用者: main.cpp loop() 400Hz 控制段末尾。
// 边界: 逐项比较赋值；不判 NaN。
void diagAccumulate() {
  float aGyro = fabsf(gyroPitchRate);
  if (aGyro > telemGyroAbsMax) telemGyroAbsMax = aGyro;
  float aDev = fabsf(accPitch - gyroIntegAngle);
  if (aDev > telemAccPitchDevMax) telemAccPitchDevMax = aDev;
  float aBal = fabsf(balanceOutput);
  if (aBal > telemBalHzAbsMax) telemBalHzAbsMax = aBal;
  if (kalmanBias < telemKalmanBiasMin) telemKalmanBiasMin = kalmanBias;
  if (kalmanBias > telemKalmanBiasMax) telemKalmanBiasMax = kalmanBias;
}

// diagResetWindow() —— D 帧入队后重置窗口。
// 参数: 无。返回: void。调用者: panel::sendSlow()（当前 1Hz）。
// 边界: kalmanBias 以当拍值重开 min/max，避免下一窗口两端从极端值起步。
void diagResetWindow() {
  telemGyroAbsMax = 0.0f;
  telemAccPitchDevMax = 0.0f;
  telemBalHzAbsMax = 0.0f;
  telemKalmanBiasMin = kalmanBias;
  telemKalmanBiasMax = kalmanBias;
}

}  // namespace panel
