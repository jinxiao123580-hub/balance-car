#include "control/balance.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// 以下声明全部从 main.cpp 原样搬出，初始值未改。
// ============================================================
// 整体平衡方向
// balanceDirection：直立 PD 输出方向系数 ｜ 无量纲 ±1 ｜ 当前 -1（实测定向；翻号=整台车反向推）｜
//   写：无（编译期常量）｜ 读：updateBalance 算 P/D 项 ｜ 400Hz。
int balanceDirection = - 1;
// ============================================================
// 机械平衡角
// ============================================================
// balanceAngleOffset：机械平衡角 O ｜ 单位 deg ｜ 默认 2.99（随装配/上电漂移；实测定 4.24 会漂 +28.5mm/s）｜
//   写：panel O/LOAD/DEF、flash 读回 ｜ 读：updateAuthority 的相对基准、main 倒地判据、remote 位置/点动、遥测 ｜ 400Hz 级。
//   死区判定在 updateBalance 里用 |targetBalanceAngle - pitch|，与本基准无关。
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
// balanceKp：直立环 P 增益 ｜ 单位 Hz/deg（乘 R 后成为 Hz/100Hz 增量）｜ 默认 1000 ｜
//   写：panel P/LOAD/DEF ｜ 读：updateBalance ｜ 400Hz。
float balanceKp = 1000.0f;  // 直立环P：角度误差1°→STEP加速度原始输出
// balanceKd：直立环 D 增益 ｜ 单位 Hz/(deg/s) ｜ 默认 100 ｜ 写：panel D/LOAD/DEF ｜ 读：updateBalance ｜ 400Hz。
//   边界：D 项与 P 项同乘 balanceDirection；角速度含符号，正反馈会自激，参数只能实测定。
float balanceKd = 100.0f;  // 直立环D：角速度阻尼
// 直立环最终输出限幅 Hz（finalBalanceOutput 钳位）。防大角度时输出冲到±25kHz超高频失步抱死。
// 注意：限幅不能太小（试过±2000/±4000站不住=恢复力被砍），12000 是折中值。
// BALANCE_OUTPUT_LIMIT_HZ：直立最终输出绝对限幅 ｜ 单位 Hz ｜ 默认 1750（≈109mm/s；1Hz=0.0625mm/s）｜
//   写：panel L/LOAD/DEF ｜ 读：updateBalance 积分后钳位、remote 转向上限计算 ｜ 400Hz。
//   （注：旧描述里的 12000 是历史值，以当前常量 1750 为准；限幅对象是“积分后的 STEP 频率”，不是每拍增量。）
float BALANCE_OUTPUT_LIMIT_HZ = 1750.0f;
// 8/21 输出斜坡限速：每控制周期(2.5ms) finalBalanceOutput 最多变化这么多Hz。
// 修复"运行中突然卡死"(失步)：2000/4000限幅站不住(砍恢复力)，30000等效无限幅能站但
// 大角度瞬间0→25kHz失步卡死。斜坡+12000限幅：保留恢复力且电机加速跟得上。
// 成熟步进平衡结构：直立PD不再直接指定STEP速度，而是每10ms给速度增加一个增量。
// S为100Hz每次允许的最大频率增量；40Hz/update = 4000Hz/s。
// BALANCE_OUTPUT_SLEW_HZ（S）：每个 100Hz 端点允许的 STEP 频率增量上限 ｜ 单位 Hz/update ｜ 默认 100（=10000Hz/s）｜
//   写：panel S/LOAD/DEF ｜ 读：updateBalance 100Hz 端点（balanceAccelEffectiveLimit 初值）｜ 100Hz。
//   （注：旧描述“每控制周期 2.5ms 最多变化这么多”与当前实现不符，以代码为准：本值限的是 100Hz 端点增量，
//    400Hz 实际每小步只执行该增量的 1/4；旧注释里 12000/30000 也是历史限幅值。）
float BALANCE_OUTPUT_SLEW_HZ = 100.0f;
// BALANCE_ACCEL_SCALE（R）：直立 PD 输出→频率增量的比例 ｜ 无量纲 ｜ 默认 0.10 ｜
//   写：panel R/LOAD/DEF ｜ 读：updateBalance（balanceAccelRawOutput=R*balanceOutput，单位 Hz/100Hz 端点）｜ 100Hz。
float BALANCE_ACCEL_SCALE = 0.10f;  // 蓝牙R：直立PD输出到每次STEP频率增量的比例
// 角度死区：|pitch - targetBalanceAngle| < 该值时，直立环输出0（电机不顶）
// 死区越大越不易震荡，但偏了拉不回来；越小纠偏越积极。当前0.5°：8/21从3°→1.5°→0.5°逐步调小
// BALANCE_ANGLE_DEADBAND_DEG（Z）：直立角度死区 ｜ 单位 deg ｜ 默认 0.0 ｜
//   死区内 P 项误差置零、保留 D 阻尼；死区越大越不晃但偏了拉不回来。
//   写：panel Z/LOAD/DEF ｜ 读：updateBalance ｜ 400Hz。
//   （注：旧注释“当前 0.5°”与当前默认值 0.0 不符，以代码为准（面板可改，8/21 曾从 3°→1.5°→0.5° 调小）。）
float BALANCE_ANGLE_DEADBAND_DEG = 0.0f;
// 高速驱动保护：只限制"继续加速"，不限制制动/反向所需的增量。
// B=保护启动频率，H=高速继续加速的每10ms最大频率增量。
// HIGH_SPEED_ACCEL_START_HZ（B）：高速加速保护启动频率 ｜ 单位 Hz（|integratedMotorSpeedCommand| 门限）｜ 默认 1000 ｜
//   写：panel B/LOAD/DEF ｜ 读：updateBalance 100Hz 端点 ｜ 100Hz。
float HIGH_SPEED_ACCEL_START_HZ = 1000.0f;
// HIGH_SPEED_ACCEL_LIMIT_HZ（H）：高速段“继续同向加速”的增量上限 ｜ 单位 Hz/update ｜ 默认 40（=4000Hz/s）｜
//   写：panel H/LOAD/DEF ｜ 读：updateBalance 100Hz；只收紧同向加速，制动/反向不限制 ｜ 100Hz。
float HIGH_SPEED_ACCEL_LIMIT_HZ = 40.0f;
// ============================================================
// 控制变量
// ============================================================
// targetBalanceAngle：直立环目标倾角 ｜ 单位 deg ｜ 写：updateAuthority 每 400Hz 拍、setup/故障清零 ｜
//   读：updateBalance 算角度误差、手动通路 seed、遥测第3列 ｜ 400Hz。
//   （注：旧注释“初始值 4.0，与 balanceAngleOffset 一致”与当前 O=2.99 不符；且 main setup 会用 O 覆盖，
//    4.0 只在 setup 之前的瞬间有意义，以代码为准。）
float targetBalanceAngle = 4.0f;  // 初始值：与 balanceAngleOffset 一致
// balanceOutput：直立 PD 原始输出 = P+D ｜ 单位 Hz（等效）｜ 写：updateBalance 400Hz ｜
//   读：速度预测 Q 计算、遥测第6列、diag 窗口 ｜ 400Hz。
float balanceOutput = 0.0f;
// balancePOutput：直立 P 项 = dir*Kp*有效角度误差 ｜ 单位 Hz ｜ 写：updateBalance ｜ 读：遥测第4列 ｜ 400Hz。
float balancePOutput = 0.0f;
// balanceDOutput：直立 D 项 = dir*(-Kd*gyroPitchRate) ｜ 单位 Hz ｜ 写：updateBalance ｜ 读：遥测第5列 ｜ 400Hz。
float balanceDOutput = 0.0f;
// 直立PD + 速度直接STEP阻尼后的最终前后控制量
// finalBalanceOutput：直立最终前后输出（积分后、L 限幅后）｜ 单位 Hz ｜ 写：updateBalance 400Hz、故障清零 ｜
//   读：mixMotors（左右轮共模）、转向环限幅计算、遥测第14列 ｜ 400Hz。
float finalBalanceOutput = 0.0f;
// mixedBalanceOutput：限幅前输出（增量积分后的 STEP 目标频率）｜ 单位 Hz ｜ 写：updateBalance ｜ 读：限幅、遥测第12列 ｜ 400Hz。
float mixedBalanceOutput = 0.0f;
// limitedBalanceOutput：总限幅（±L）后输出 ｜ 单位 Hz ｜ 写：updateBalance ｜ 读：finalBalanceOutput 赋值、遥测第13列 ｜ 400Hz。
float limitedBalanceOutput = 0.0f;
// balanceAccelRawOutput：100Hz 端点未限幅频率增量 = R*balanceOutput ｜ 单位 Hz/update ｜ 写：updateBalance ｜
//   读：限幅计算、遥测第31列（量化直立 PD 有多猛）｜ 100Hz。
float balanceAccelRawOutput = 0.0f;  // 100Hz本次未限幅STEP频率增量 Hz/update
// balanceAccelLimitedOutput：经 S/H/换向软窗口限幅后的增量 ｜ 单位 Hz/update ｜ 写：updateBalance ｜
//   读：均分小步、遥测第32列 ｜ 100Hz。
float balanceAccelLimitedOutput = 0.0f;  // 100Hz加速度限幅后的增量 Hz/update
// balanceAccelEffectiveLimit：本拍实际增量限幅（S 或 H，软窗口再乘 0.25）｜ 单位 Hz/update ｜
//   写：updateBalance ｜ 读：限幅、遥测第39列 ｜ 100Hz。
float balanceAccelEffectiveLimit = 0.0f;  // 本拍实际使用的增量限幅 Hz/update
// highSpeedAccelProtectionActive：高速加速保护本次是否生效 ｜ bool ｜ 写：updateBalance ｜
//   读：当前无读者（诊断保留）｜ 100Hz。
bool highSpeedAccelProtectionActive = false;
// balanceAccelSubstepOutput：100Hz 端点增量均分到 4 个 400Hz 小步后的值 ｜ 单位 Hz/2.5ms ｜ 写/读：updateBalance ｜ 400Hz。
float balanceAccelSubstepOutput = 0.0f;  // 把一个100Hz端点增量均分到4个400Hz小步
// integratedMotorSpeedCommand：对增量积分得到的 STEP 目标频率 ｜ 单位 Hz ｜ 范围 ±BALANCE_OUTPUT_LIMIT_HZ ｜
//   写：updateBalance 400Hz ｜ 读：加速保护判据、遥测第33列 ｜ 400Hz。
float integratedMotorSpeedCommand = 0.0f;  // 对增量积分得到的STEP目标频率 Hz
// balanceAccelDivider：400Hz→100Hz 分频计数 ｜ 0~BALANCE_ACCEL_UPDATE_DIVIDER-1 ｜ 写/读：updateBalance ｜ 400Hz。
uint8_t balanceAccelDivider = 0;
// lastFinalBalanceOutput：finalBalanceOutput 的上一拍副本 ｜ 单位 Hz ｜ 写：updateBalance/clearOutputDiagnostics ｜
//   读：当前无读者（历史诊断）｜ 400Hz。
float lastFinalBalanceOutput = 0.0f;
// 换向软过渡状态（见 updateBalanceLoop）
// prevIntegratedSpeedCmdSign：上一拍 integratedMotorSpeedCommand 的符号（-1/0/1，±1Hz 死区）｜ 无量纲 ｜
//   写/读：updateBalance 换向检测 ｜ 400Hz。
int32_t prevIntegratedSpeedCmdSign = 0;  // 上一端点周期 integratedMotorSpeedCommand 符号
// reversalSoftTicksRemaining：换向软窗口剩余端点拍数 ｜ 单位 100Hz 拍（0~25，1 拍=10ms）｜
//   写：updateBalance 检测到换向时置 REVERSAL_SOFT_TICKS(25)=250ms ｜ 读：每端点递减、诊断命令 ! ｜ 100Hz。
uint16_t reversalSoftTicksRemaining = 0;  // 换向后剩余软过渡端点周期数

// 倒地保护连续确认计数
// fallConfirmCount：倒地确认连续帧计数 ｜ 单位 帧（0~255）｜ 写：main.cpp loop ｜
//   读：main.cpp 确认倒地（>=FALL_CONFIRM_COUNT=8）｜ 400Hz。
uint8_t fallConfirmCount = 0;

namespace control {

// ============================================================
// 直立环 PD
// ============================================================
// updateBalance：400Hz 直立环（无参/无返回）。调用：main.cpp loop 每拍。
// 顺序与算式：
// ① angleError=targetBalanceAngle-pitch；连续死区：死区外扣掉 Z 宽度，死区内 P 误差=0（D 保留）；
// ② P=balanceDirection*balanceKp*effErr（单位 Hz）；D=balanceDirection*(-balanceKd*gyroPitchRate)；
//    balanceOutput=P+D；
// ③ 每 4 拍（100Hz）算端点增量：raw=R*balanceOutput；限幅=S，若 |integrated|>=B 且同向加速则改用 H；
//    换向软窗口内限幅再×0.25；limited=clamp(raw,±limit)；substep=limited/4；
// ④ 每 400Hz 拍：candidate=integrated+substep，若符号翻转（±1Hz 判定）且不在软窗口则把 substep 再×0.25
//    并置软窗口 25 拍；integrated+=substep；
// ⑤ 绝对限幅：integrated 夹到 ±BALANCE_OUTPUT_LIMIT_HZ；mixed/limited=该值；final=limited。
// 边界/顺序要点：角度 PD 先按比例转成“增量”，限幅在积分之前（防失步），绝对限幅在积分之后（防超速）；
//   NaN 不在此处理（上游 pitch/gyro 均为有限值，电机侧 commandStepOutput 还会再兜底 NaN）。
//   integrated 是增量积分状态，停机清零后由 PD 重新爬升，不会保留旧速度。
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
    // 100Hz 端点增量 = R[无量纲] × PD[Hz] ⇒ 单位 Hz/update；S/H/软窗口限的正是这个增量。
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
  // 400Hz 小步：每 2.5ms 只推进 100Hz 端点增量的 1/4，使 STEP 频率连续变化而不是阶跃。
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
// clearOutputDiagnostics：停机/故障统一清零控制侧输出与积分（无参/无返回）。
// 清零项：P/D/PD 输出、turnOutput、遥测快照、位置目标/误差、mixed/limited/final、增量及其限幅、
//   integrated、换向状态、分频计数、lastFinalBalanceOutput。
// 调用：main 的倒地/掉拍/IMU 失效/欠压路径（400Hz 事件）｜ 注意 turnOutput 定义在 remote 但历史上一并清零。
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
