#include "panel/diag.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地）
// ============================================================
// 调度/掉拍/工作耗时计数器
uint32_t controlOverrunCount = 0;
uint32_t controlMaxGapUs = 0;
uint32_t controlMaxWorkUs = 0;

// 倒地保护累计触发次数（遥测第43项，只增不减）
uint32_t fallStopCount = 0;  // 倒地保护累计触发次数（遥测第43项，只增不减）

// BLE 接收诊断（面板是否真的把命令送到了车端）
// BLE 接收诊断：面板是否真的把命令送到了车端（放在 4Hz 调试帧最后一列）
uint32_t btRxBytes = 0;

// ---- 250ms 遥测窗口极值保持（穿透 250ms 采样盲区，抓短促尖峰）----
// 用途：判定陀螺/加速度计是真的有尖峰（IMU 故障），还是车体真在剧烈振荡（控制增益问题）。
// 每控制拍（400Hz）在 loop 末尾累积，sendSlow() 读取后清零。
// ---- 250ms 遥测窗口极值保持（穿透 250ms 采样盲区，抓短促尖峰）----
// 用途：判定陀螺/加速度计是真的有尖峰（IMU 故障），还是车体真在剧烈振荡（控制增益问题）。
// 每控制拍（400Hz）在 loop 末尾累积，sendTelemetry() 读取后清零。
float telemGyroAbsMax = 0.0f;  // 窗口内 max|gyroPitchRate| 度/s
float telemAccPitchDevMax = 0.0f;  // 窗口内 max|accPitch - gyroIntegAngle| 度
float telemBalHzAbsMax = 0.0f;  // 窗口内 max|balanceOutput| Hz（量化饱和深度）
float telemKalmanBiasMin = 1e9f;  // 窗口内 kalmanBias 最小/最大（首次发送后由 sendTelemetry 重置为当拍值）
float telemKalmanBiasMax = - 1e9f;

namespace panel {

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

void diagResetWindow() {
  telemGyroAbsMax = 0.0f;
  telemAccPitchDevMax = 0.0f;
  telemBalHzAbsMax = 0.0f;
  telemKalmanBiasMin = kalmanBias;
  telemKalmanBiasMax = kalmanBias;
}

}  // namespace panel
