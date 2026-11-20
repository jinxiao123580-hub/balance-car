#include "control/balance.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// 以下声明全部从 main.cpp 原样搬出，初始值未改。
// ============================================================
// 整体平衡方向
int balanceDirection = - 1;
// ============================================================
// 机械平衡角
// ============================================================
float balanceAngleOffset = 2.99f;  // 用户面板参数 O：上电默认平衡目标角。
// 2026-11-19 实测定为 2.99（烧录重启后二分：4.24 会漂 +28.5mm/s，2.99 时 −0.11mm/s）。
// 注意 O 是随装配/上电变化的量：每次上电先 `?` 看 O=，再用 ble_find_o.py 重测；
// 闪存里有 SAVED 记录时，本默认值只在擦除后首次启动才生效。
// 历史：2026-11-12 曾因"O=4 持续单侧倾倒"降到 2.0，11-19 找到真因是 N=2.0（见 MAX_SPEED_ANGLE）。
// ============================================================
// PID参数
// ============================================================
// ------------------------------------------------------------
// 直立环 PD
// ------------------------------------------------------------
float balanceKp = 1000.0f;  // 直立环P：角度误差1°→STEP加速度原始输出
float balanceKd = 100.0f;  // 直立环D：角速度阻尼
// 直立环最终输出限幅 Hz（finalBalanceOutput 钳位）。防大角度时输出冲到±25kHz超高频失步抱死。
// 注意：限幅不能太小（试过±2000/±4000站不住=恢复力被砍），12000 是折中值。
float BALANCE_OUTPUT_LIMIT_HZ = 1750.0f;
// 8/21 输出斜坡限速：每控制周期(2.5ms) finalBalanceOutput 最多变化这么多Hz。
// 修复"运行中突然卡死"(失步)：2000/4000限幅站不住(砍恢复力)，30000等效无限幅能站但
// 大角度瞬间0→25kHz失步卡死。斜坡+12000限幅：保留恢复力且电机加速跟得上。
// 成熟步进平衡结构：直立PD不再直接指定STEP速度，而是每10ms给速度增加一个增量。
// S为100Hz每次允许的最大频率增量；40Hz/update = 4000Hz/s。
float BALANCE_OUTPUT_SLEW_HZ = 100.0f;
float BALANCE_ACCEL_SCALE = 0.10f;  // 蓝牙R：直立PD输出到每次STEP频率增量的比例
// 角度死区：|pitch - targetBalanceAngle| < 该值时，直立环输出0（电机不顶）
// 死区越大越不易震荡，但偏了拉不回来；越小纠偏越积极。当前0.5°：8/21从3°→1.5°→0.5°逐步调小
float BALANCE_ANGLE_DEADBAND_DEG = 0.0f;
// 高速驱动保护：只限制"继续加速"，不限制制动/反向所需的增量。
// B=保护启动频率，H=高速继续加速的每10ms最大频率增量。
float HIGH_SPEED_ACCEL_START_HZ = 1000.0f;
float HIGH_SPEED_ACCEL_LIMIT_HZ = 40.0f;
// ============================================================
// 控制变量
// ============================================================
float targetBalanceAngle = 4.0f;  // 初始值：与 balanceAngleOffset 一致
float balanceOutput = 0.0f;
float balancePOutput = 0.0f;
float balanceDOutput = 0.0f;
// 直立PD + 速度直接STEP阻尼后的最终前后控制量
float finalBalanceOutput = 0.0f;
float mixedBalanceOutput = 0.0f;
float limitedBalanceOutput = 0.0f;
float balanceAccelRawOutput = 0.0f;  // 100Hz本次未限幅STEP频率增量 Hz/update
float balanceAccelLimitedOutput = 0.0f;  // 100Hz加速度限幅后的增量 Hz/update
float balanceAccelEffectiveLimit = 0.0f;  // 本拍实际使用的增量限幅 Hz/update
bool highSpeedAccelProtectionActive = false;
float balanceAccelSubstepOutput = 0.0f;  // 把一个100Hz端点增量均分到4个400Hz小步
float integratedMotorSpeedCommand = 0.0f;  // 对增量积分得到的STEP目标频率 Hz
uint8_t balanceAccelDivider = 0;
float lastFinalBalanceOutput = 0.0f;
// 换向软过渡状态（见 updateBalanceLoop）
int32_t prevIntegratedSpeedCmdSign = 0;  // 上一端点周期 integratedMotorSpeedCommand 符号
uint16_t reversalSoftTicksRemaining = 0;  // 换向后剩余软过渡端点周期数

// 倒地保护连续确认计数
uint8_t fallConfirmCount = 0;

namespace control {

// ============================================================
// 直立环 PD
// ============================================================
void updateBalance() {
  float angleError = targetBalanceAngle - pitch;
  // --------------------------------------------------------
  // 1. 直立PD
  // --------------------------------------------------------
  // 连续角度死区：死区外扣除死区宽度，避免 P 项在边界从 0 突跳。
  // 死区内只将 P 项误差置零，保留陀螺仪 D 阻尼来抑制车体晃动。
  float effectiveAngleError = 0.0f;
  if (angleError > BALANCE_ANGLE_DEADBAND_DEG) {
    effectiveAngleError = angleError - BALANCE_ANGLE_DEADBAND_DEG;
  } else if (angleError < - BALANCE_ANGLE_DEADBAND_DEG) {
    effectiveAngleError = angleError + BALANCE_ANGLE_DEADBAND_DEG;
  }
  balancePOutput = balanceDirection * balanceKp * effectiveAngleError;
  balanceDOutput = balanceDirection * (- balanceKd * gyroPitchRate);
  balanceOutput = balancePOutput + balanceDOutput;
  // --------------------------------------------------------
  // 2. 直接速度/位置修正权限
  //
  // 与目标角使用同一个 outerLoopAuthority。
  // 大角度时速度环和位置环全部退出。
  // --------------------------------------------------------
  // --------------------------------------------------------
  // 3. 步进电机加速度式直立控制
  //
  // B-ROBOT类成熟结构：直立PD给出速度增量，累加后才得到STEP频率。
  // 100Hz计算下一个频率端点；端点增量在随后的4个400Hz周期线性执行。
  // 这样R/S/B/H/L和每10ms累计变化量均保持不变，只消除频率阶跃。
  // --------------------------------------------------------
  balanceAccelDivider++;
  if (balanceAccelDivider >= BALANCE_ACCEL_UPDATE_DIVIDER) {
    balanceAccelDivider = 0;
    balanceAccelRawOutput = BALANCE_ACCEL_SCALE * balanceOutput;
    // 低速时保留 S 的完整纠偏能力；只有已经处于高速、且本拍仍在
    // 朝同一方向继续加速时，才收紧增量，避免STEP频率再陡升而丢步。
    // 反向增量是制动，始终使用 S，不能被这层保护削弱。
    balanceAccelEffectiveLimit = BALANCE_OUTPUT_SLEW_HZ;
    highSpeedAccelProtectionActive = false;
    const bool acceleratingFurther = integratedMotorSpeedCommand * balanceAccelRawOutput > 0.0f;
    if (fabsf(integratedMotorSpeedCommand) >= HIGH_SPEED_ACCEL_START_HZ && acceleratingFurther) {
      balanceAccelEffectiveLimit = HIGH_SPEED_ACCEL_LIMIT_HZ;
      if (balanceAccelEffectiveLimit > BALANCE_OUTPUT_SLEW_HZ) balanceAccelEffectiveLimit = BALANCE_OUTPUT_SLEW_HZ;
      highSpeedAccelProtectionActive = true;
    }
    // 换向软过渡生效：翻转后的窗口内把增量限幅再压小一档
    if (reversalSoftTicksRemaining > 0) {
      reversalSoftTicksRemaining--;
      balanceAccelEffectiveLimit *= REVERSAL_SOFT_LIMIT_SCALE;
    }
    balanceAccelLimitedOutput = balanceAccelRawOutput;
    if (balanceAccelLimitedOutput > balanceAccelEffectiveLimit) balanceAccelLimitedOutput = balanceAccelEffectiveLimit;
    else if (balanceAccelLimitedOutput < - balanceAccelEffectiveLimit) balanceAccelLimitedOutput = - balanceAccelEffectiveLimit;
    balanceAccelSubstepOutput = balanceAccelLimitedOutput / (float) BALANCE_ACCEL_UPDATE_DIVIDER;
  }
  // 最小平滑层：每2.5ms推进原100Hz端点增量的1/4。
  // 不重新计算PD，不改变端点、总斜率或任何现有调参量。
  float stepIncrement = balanceAccelSubstepOutput;
  float candidateSpeed = integratedMotorSpeedCommand + stepIncrement;
  int nextSign = candidateSpeed > 1.0f ? 1: (candidateSpeed < - 1.0f ? - 1: 0);
  if (nextSign != 0 && prevIntegratedSpeedCmdSign != 0 && nextSign != prevIntegratedSpeedCmdSign) {
    if (reversalSoftTicksRemaining == 0) {
      balanceAccelSubstepOutput *= REVERSAL_SOFT_LIMIT_SCALE;
      balanceAccelLimitedOutput *= REVERSAL_SOFT_LIMIT_SCALE;
      balanceAccelEffectiveLimit *= REVERSAL_SOFT_LIMIT_SCALE;
      stepIncrement = balanceAccelSubstepOutput;
    }
    reversalSoftTicksRemaining = REVERSAL_SOFT_TICKS;
  }
  integratedMotorSpeedCommand += stepIncrement;
  int actualSign = integratedMotorSpeedCommand > 1.0f ? 1: (integratedMotorSpeedCommand < - 1.0f ? - 1: 0);
  if (actualSign != 0) prevIntegratedSpeedCmdSign = actualSign;
  if (integratedMotorSpeedCommand > BALANCE_OUTPUT_LIMIT_HZ) integratedMotorSpeedCommand = BALANCE_OUTPUT_LIMIT_HZ;
  else if (integratedMotorSpeedCommand < - BALANCE_OUTPUT_LIMIT_HZ) integratedMotorSpeedCommand = - BALANCE_OUTPUT_LIMIT_HZ;
  mixedBalanceOutput = integratedMotorSpeedCommand;
  if (mixedBalanceOutput > BALANCE_OUTPUT_LIMIT_HZ) limitedBalanceOutput = BALANCE_OUTPUT_LIMIT_HZ;
  else if (mixedBalanceOutput < - BALANCE_OUTPUT_LIMIT_HZ) limitedBalanceOutput = - BALANCE_OUTPUT_LIMIT_HZ;
  else limitedBalanceOutput = mixedBalanceOutput;
  // 频率已经由100Hz加速度积分平滑变化，不再对结果做第二次slew。
  finalBalanceOutput = limitedBalanceOutput;
  lastFinalBalanceOutput = finalBalanceOutput;

}

// ============================================================
// 停机/故障统一清零：控制侧输出与积分（原 clearControlOutputDiagnostics 的控制部分）
// ============================================================
void clearOutputDiagnostics() {
  balancePOutput = 0.0f;
  balanceDOutput = 0.0f;
  balanceOutput = 0.0f;
  turnOutput = 0.0f;
  telemetrySpeedSnapshot100Hz = 0.0f;  // 遥测第19列快照（原预测速度列位）
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

}  // namespace control
