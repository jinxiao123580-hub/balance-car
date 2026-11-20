#include "control/cascade.h"

// ============================================================
// 全局状态定义（core/state.h 的 extern 在此落地，必须在全局作用域）
// ============================================================
// ------------------------------------------------------------
// 速度P直接STEP快速阻尼
//
// 用于解决“只改变目标角度，轮速响应滞后明显”的问题。
// 这一路直接叠加到直立环STEP频率输出，但权限受姿态限制。
// ------------------------------------------------------------
// speedLoopDirection：速度误差→角度修正的符号 ｜ 无量纲 ±1 ｜ 当前 -1（2026-09-02 架空实测定反后翻号）｜
//   写：无（编译期常量）｜ 读：outerLoop、remote 点动方向 ｜ 100Hz。
//   （注：上面“速度P直接STEP快速阻尼/叠加到直立环STEP频率输出”是已删除的 speedDirect 通道，
//    现在速度环只改目标倾角，不再直接叠加 STEP 频率。）
int speedLoopDirection = - 1;  // 2026-09-02 1→-1：架空实测定反。原=轮只往单方向转；翻号后=轮来回正反转（方向盘反）
// 经典速度PI外环（改目标角）：speedAngleCorrection 叠加到目标平衡角，让车倾斜以移动。
// speedKp：速度环 P 增益（面板 K / SV 命令在线可调）｜ 单位 deg/(count/50ms) ｜ 默认 -0.10。
// 整定依据：-0.02 太小（SP=8 只给 0.16°，低于静摩擦破粘 0.5~0.7°）；-0.10 在 SP±8 给 ±43mm 对称位移；
//   >=0.20 会猛冲撞翻。写：panel K/SV/LOAD/DEF ｜ 读：outerLoop 100Hz。
float speedKp = - 0.10f;  // 速度PI到目标角的比例增益；实车已确认该号为正确方向。
// 2026-11-19：−0.02 太小（SP=8 只产生 0.16°，低于破粘门槛 0.5~0.7°），
// 实测 −0.10 在 SP±8 下给出 +43.8/−43.0mm 的对称位移（2% 差）。
// 上限：≥0.20 会猛冲撞翻（SV 0.20/0.30 实测 −47.9/−11.9mm/s）。
// speedKi：速度环 I 增益（面板 I）｜ 单位 deg/((count/50ms)·s) ｜ 默认 0（当前用纯 P 验证方向）｜
//   写：panel I/LOAD/DEF ｜ 读：outerLoop（条件积分+抗饱和）｜ 100Hz。
float speedKi = 0.0f;  // 先用纯P验证速度方向；启用后采用条件积分/抗饱和
// speedKd：速度环 D 增益（面板 E）｜ 单位 deg·s/(count/50ms) ｜ 默认 0 ｜ 写：panel E/LOAD/DEF ｜ 读：outerLoop ｜ 100Hz。
//   注意不要与直立环 balanceKd 混淆：本值作用在 speedErrorDotFilt 上，单位差一个时间量纲。
float speedKd = 0.0f;
// MAX_SPEED_ANGLE（N）：速度修正目标角硬限幅 ｜ 单位 deg ｜ 默认 0.8；实测定为稳定边界：
//   N=2.0 时速度误差一大外环就反向拽 2° 翻车；0.8 起步 0.96° 截到 0.8°（仍破粘）、反向最大 0.8° 拽不翻。
//   写：panel N/LOAD/DEF ｜ 读：outerLoop 钳位、面板回显 ｜ 100Hz。
//   饱和点：|speedAngleUnclamped|>=N 即饱和；K=-0.10 时对应 |speedError|>=N/|K|=8 count/50ms（≈14.8mm/s）。
float MAX_SPEED_ANGLE = 0.8f;  // 速度修正角上限（参数 N）。
// SPEED_INTEGRAL_LIMIT（W）：速度积分累积上限 ｜ 单位 (count/50ms)·s（误差×秒）｜ 默认 50 ｜
//   写：panel W/LOAD/DEF ｜ 读：outerLoop 抗饱和候选钳位 ｜ 100Hz。
float SPEED_INTEGRAL_LIMIT = 50.0f;  // 速度PI积分累积上限（误差×秒，非deg）
// ------------------------------------------------------------
// 串级速度PI外环
//
// 控制结构：
//
// targetForwardSpeed
//        ↓
// 速度误差 = target - Kalman融合速度
//        ↓
// 速度直投（P×比例 + 积分Gain）→ 直接叠加STEP Hz（该通道已退出控制，仅历史说明）
//        ↓
// 直立PD（400Hz）
//        ↓
// STEP频率
//
// 速度外环每50ms执行一次 = 20Hz。
// （旧"速度PI→目标倾角→改targetBalanceAngle"路径已移除：见 .bak_deadcode_20260902）
// ------------------------------------------------------------
// （注：上方结构说明里的“速度外环每 50ms 执行一次 = 20Hz”与当前实现不符，以代码为准：
//   main.cpp 每 4 个 400Hz 拍调用一次 outerLoop = 100Hz/10ms；N 限幅仍是 deg，饱和点见 MAX_SPEED_ANGLE 注释。）
// （注：上方“旧速度PI→目标倾角→改 targetBalanceAngle 路径已移除”与当前实现不符：
//   现在 speedAngleCorrection 正是经 updateAuthority() 叠加到 targetBalanceAngle（串级结构已恢复），
//   被删除的是更早的 speedDirect（直接叠 STEP Hz）通道，看代码为准。）
// 很小的速度误差不积分，避免零附近反复积累
// SPEED_ERROR_DEADBAND（J）：速度误差积分门控死区 ｜ 单位 count/50ms ｜ 默认 2（≈3.7mm/s）｜
//   只有 |speedError|>J 且外环权限>=0.2 才继续累积积分 ｜ 写：panel J/LOAD/DEF ｜ 读：outerLoop ｜ 100Hz。
float SPEED_ERROR_DEADBAND = 2.0f;  // 速度误差死区(速度环积分门控) 2026-09-02 可调=J
// 目标速度变化限幅。
// 当前targetForwardSpeed通常为0，这个参数主要为后续遥控前进保留。
// 2026-11-19：8 太小——车被锁在"几 mm/s"的静摩擦主导区间，任何环都稳不住。
// 放到 30，允许进入正常巡航速度（SP 命令仍受此钳制）。
// 2026-11-20: runtime variable + `MS <v>` command (RAM-only), no reflash for range tuning.
// MAX_TARGET_FORWARD_SPEED（MS 命令，RAM-only）：目标速度上限 ｜ 单位 count/50ms ｜ 默认 30（≈55.5mm/s）｜
//   历史：8 太小，车被锁在静摩擦主导的几 mm/s 区间；放到 30 才允许进入正常巡航。
//   写：panel MS 命令（0.5~60 夹紧）、SP 钳位 ｜ 读：outerLoop 安全限幅 ｜ 100Hz。
//   （注：旧注释“当前 targetForwardSpeed 通常为 0，主要为后续遥控前进保留”已过时：SP/MS 已启用。）
float MAX_TARGET_FORWARD_SPEED = 30.0f;
// ============================================================
// 大角度直立优先
//
// 小角度：速度环 + 位置环全部参与
// 中等角度：外环平滑减弱
// 大角度：外环完全退出，只保留直立PD救车
// ============================================================
// OUTER_FULL_ANGLE_DEG（F）：外环全权限角 ｜ 单位 deg（相对机械平衡角 O）｜ 默认 5.0 ｜
//   |pitch-O|<=F 时 outerLoopAuthority=1 ｜ 写：panel F/LOAD/DEF ｜ 读：updateAuthority 400Hz。
float OUTER_FULL_ANGLE_DEG = 5.0f;  // |pitch-平衡点| <= 此角：外环全权限
// OUTER_DISABLE_ANGLE_DEG（G）：外环退出角 ｜ 单位 deg（相对 O）｜ 默认 10.0 ｜
//   |pitch-O|>=G 时 authority=0，只留直立 PD 救车 ｜ 写：panel G/LOAD/DEF ｜ 读：updateAuthority 400Hz。
float OUTER_DISABLE_ANGLE_DEG = 10.0f;  // >=此角：外环退出，只留直立PD
// 直立环衰减的历史参数（已并入外环权限，不再单独存在）。range 0~1。
// 用户方案：削弱直立环(如0.7)给速度环让权重，速度环直接叠加到轮速而非改目标角。
// 外环当前权限：0~1
// outerLoopAuthority：外环权限 0~1 ｜ 无量纲 ｜ 写：updateAuthority（5°~10° 线性插值）400Hz、故障清零 ｜
//   读：outerLoop 积分门控、遥测第8列 ｜ 400Hz 更新 / 100Hz 使用。
float outerLoopAuthority = 1.0f;
// ============================================================
// 目标值
// ============================================================
// targetForwardSpeed：速度环目标速度 ｜ 单位 count/50ms，向前为正（1.85mm/s per count/50ms）｜
//   写：outerLoop（SP/点动）、remote 位置保持/运动计划、故障清零 ｜ 读：outerLoop 算 speedError、遥测 ｜ 100Hz。
float targetForwardSpeed = 0.0f;
// SP 命令设定的目标速度（count/50ms）。与 ^ 手动通路互斥：^ 活动时被清零。
// speedCmd：SP 命令设定的目标速度（唯一速度设定入口）｜ 单位 count/50ms ｜
//   范围 ±MAX_TARGET_FORWARD_SPEED ｜ 写：panel SP 命令、MS 收缩时回钳 ｜ 读：outerLoop（^ 运动时清零）｜ 100Hz。
float speedCmd = 0.0f;
// ============================================================
// 目标角阶跃测试
//
// TEST_TARGET_ANGLE_OFFSET:
// 当前关闭阶跃测试：TEST_TARGET_ANGLE_OFFSET = 0.0f
// 速度PI同样关闭。
// 本版只用于观察：
//   1. Kalman pitch
//   2. gyroPitchRate
//
// 用于判断落地运动时：
// 是真实车体仍在持续旋转，还是Kalman角度被动态加速度拉偏。
// ============================================================
// 目标角阶跃诊断已关闭，正式外环版本不再使用固定阶跃。
// ============================================================
// 速度PI控制变量
// ============================================================
// speedError：速度环误差 = targetForwardSpeed - vehicleSpeed ｜ 单位 count/50ms ｜
//   写：outerLoop、故障清零 ｜ 读：outerLoop 的 P/D 与积分门控、遥测第34列 ｜ 100Hz。
float speedError = 0.0f;
// speedPContribution：速度 P 项 = speedKp*speedError ｜ 单位 deg（已含 K 的量纲）｜ 写：outerLoop ｜
//   读：outerLoop 合成、遥测第35列 ｜ 100Hz。
float speedPContribution = 0.0f;
// speedIContribution：速度 I 项 = speedKi*speedIntegral ｜ 单位 deg ｜ 写：outerLoop ｜
//   读：outerLoop 合成、遥测第36列 ｜ 100Hz。
float speedIContribution = 0.0f;
// speedAngleUnclamped：N 限幅前的目标角修正 = speedLoopDirection*(P+I+D)+jogLeanOffset ｜ 单位 deg。
//   量纲：每一项都是 deg；P=speedKp[deg/(count/50ms)]*err[count/50ms]，D=speedKd[deg·s/(count/50ms)]*dot，
//   jogLeanOffset 是远控点动倾角（deg，不乘方向系数）。写：outerLoop ｜ 读：限幅、遥测第37列 ｜ 100Hz。
float speedAngleUnclamped = 0.0f;
// speedAngleLimited：N 限幅后、slew 前的目标角 ｜ 单位 deg，范围 ±MAX_SPEED_ANGLE(0.8) ｜
//   写：outerLoop ｜ 读：outerLoop slew 计算、遥测第38列 ｜ 100Hz。
float speedAngleLimited = 0.0f;
// 速度直投积分（治"往一边溜"）：对准 speedError 累积、直接叠到 STEP Hz，不进平衡角。
// 只加直投积分不碰速度PI目标角路，前进/转弯(targetForwardSpeed/turnKp)语义保留。
// speedIntegral：速度 I 的积分累积（经典串级，会进目标角）｜ 单位 (count/50ms)·s ｜
//   范围 ±SPEED_INTEGRAL_LIMIT（W=50）｜ 写：outerLoop、故障/运动清零、remote 捕获位置 ｜ 读：outerLoop ｜ 100Hz。
//   （注：上方“速度直投积分…直接叠到 STEP Hz、不进平衡角”是已删除通道的历史说明，以当前代码为准：
//    本积分经 speedIContribution 进入目标角，不再直接叠 STEP。）
float speedIntegral = 0.0f;  // 经典速度PI积分累积（纯串级，会进目标角）
// speedAngleCorrection：速度环输出、实际叠加到目标倾角的修正量（slew 后）｜ 单位 deg ｜
//   写：outerLoop 每 100Hz 朝 speedAngleLimited 最多迈 0.10°；clearSpeedLoopForMotionBypass 清 0 ｜
//   读：updateAuthority 合成 targetBalanceAngle、remote 运动 seed、遥测第7列 ｜ 100Hz。
float speedAngleCorrection = 0.0f;  // 经典速度PI输出的目标倾角修正（叠加到 targetBalanceAngle）
// speedErrorPrev：上一拍速度误差（微分用）｜ 单位 count/50ms ｜ 写：outerLoop、故障清零 ｜ 读：outerLoop ｜ 100Hz。
float speedErrorPrev = 0.0f;  // 速度误差微分阻尼：上一拍误差
// speedErrorDotFilt：低通后的误差变化率 ｜ 单位 (count/50ms)/s ｜ 写：outerLoop（新值*0.3 + 旧值*0.7）｜
//   读：outerLoop 的 D 项 ｜ 100Hz。
float speedErrorDotFilt = 0.0f;  // 速度误差微分阻尼：低通后的误差变化率
// ------------------------------------------------------------
// 位置快速拉回
//
// 作用：车只要离开开机位置一点点，就立即给出反向STEP修正。
// 这一路直接读取编码器累计位置，400Hz更新，不等待50ms速度窗口。
// ------------------------------------------------------------
// 标准串级位置外环：位置误差先生成一个受限目标速度，不直接抢占STEP输出。
// positionKp（X）：位置 P 外环增益 ｜ 单位 (count/50ms)/count ｜ 默认 0.02（实测安全；原 5.0 会把静止漂移
//   放大到 230counts/s）。写：panel X/LOAD/DEF ｜ 读：remote 位置保持/运动计划 ｜ 100Hz。
//   （注：上方“位置快速拉回…400Hz 直接读取编码器…立即给反向 STEP 修正”与当前实现不符：
//    现在位置环在 remote/position_jog.cpp，只生成受限目标速度、不直接抢占 STEP；cascade 只保存参数与诊断量。）
float positionKp = 0.02f;  // 蓝牙X：实测安全值（原 5.0 会把静止漂移放大到 230counts/s，2026-11-12 改）
// MAX_POSITION_RETURN_SPEED（Y）：位置环目标回位速度限幅 ｜ 单位 count/50ms ｜ 默认 0.30（≈0.56mm/s）｜
//   写：panel Y/LOAD/DEF ｜ 读：remote 位置保持/运动计划 ｜ 100Hz。
float MAX_POSITION_RETURN_SPEED = 0.30f;  // 蓝牙Y：目标回位速度限幅，count/50ms（实测安全值）
// POSITION_DEADBAND_COUNT（M）：位置死区 ｜ 单位 平均编码器 count ｜ 默认 12（≈1.11mm）｜
//   写：panel M/LOAD/DEF ｜ 读：remote 位置保持/运动计划 ｜ 100Hz。
float POSITION_DEADBAND_COUNT = 12.0f;  // 蓝牙M：位置死区，平均编码器count（实测安全值）
// 位置串级外环诊断量
// positionError：位置误差 = targetVehiclePosition - vehiclePosition ｜ 单位 count ｜
//   写：remote 点动/位置保持、故障清零 ｜ 读：位置 P 外环、遥测第23列 ｜ 400Hz 更新/100Hz 使用。
float positionError = 0.0f;
// positionSpeedTarget：位置环生成的目标速度 ｜ 单位 count/50ms ｜ 写：remote 位置保持/运动计划、outerLoop(点动)、
//   故障清零 ｜ 读：遥测第11列、remote/main ｜ 100Hz。
float positionSpeedTarget = 0.0f;

// 遥测第 19 列：外环时刻的 vehicleSpeed 快照（原预测速度列位，值不变）
// telemetrySpeedSnapshot100Hz：外环时刻的 vehicleSpeed 快照（遥测第19列，列位/语义固定）｜ 单位 count/50ms ｜
//   写：outerLoop/故障清零 ｜ 读：遥测第19列 ｜ 100Hz。
float telemetrySpeedSnapshot100Hz = 0.0f;

// 速度环整体关闭时，请求远控清零 jog 倾角（由 main 立即转发 remote::resetLean()）
// leanResetRequested：请求清零 jog 倾角的握手标志 ｜ bool ｜ 写：outerLoop（速度环 K/I/D 全 0 时置 true）｜
//   读：main.cpp 100Hz 分支（转发 remote::resetLean() 后清 false）｜ 事件型。
bool leanResetRequested = false;

namespace control {

// ============================================================
// 漂移拉回外环
// 成熟步进平衡车串级外环：100Hz速度PI -> 小目标倾角。
// 编码器增量换算为既有count/50ms量纲，保持参数和遥测单位不变。
// ============================================================
// outerLoop：100Hz 速度串级外环（无返回）。调用：main.cpp loop 每 4 个 400Hz 拍一次（10ms），
//   仅在 remote::tick100() 返回 false（运动驱动未接管）时调用。
// 参数：jogPositionControl=位置点动调度中；jogAngleControl=jog 倾角/位置状态机忙；motionWasBusy=^ 运动接管；
//   manualPitchBusy=手动俯仰忙；jogLeanOffset=点动倾角 deg。
// 步骤：① SP 覆盖 targetForwardSpeed（^ 运动时清 SP）；② 目标速度限幅 ±MAX_TARGET_FORWARD_SPEED；
//   ③ speedError=target-vehicleSpeed（vehicleSpeedFiltered 当前直接透传）；④ 微分：errDot=(err-errPrev)*100，
//     低通 0.7/0.3；⑤ 条件积分：jog/手动忙或 Ki=0 → 清积分；否则 authority>=0.2 且 |err|>J 时
//     candidate=integral+err*0.01，夹 ±W，且仅当“候选角仍<=N 或朝 N 反向”才接受（抗饱和）；
//   ⑥ speedAngleUnclamped=dir*(P+I+D)+jogLeanOffset → 硬限幅 ±N 得 speedAngleLimited；
//   ⑦ slew：speedAngleCorrection 每拍朝 Limited 最多变 0.10°（100Hz → 10°/s）；
//   ⑧ 若 K/I/D 全 0：清全部状态并置 leanResetRequested（速度环整体关闭）。
// 饱和点：|speedAngleUnclamped|>=N=0.8° 即饱和；K=-0.10、dir=-1 时对应 |speedError|>=8 count/50ms（≈14.8mm/s），
//   超过后目标角固定 0.8°，变成 bang-bang 式最大倾角——这就是 SP>=16 抖动、SP=20 跑飞的直接原因之一。
void outerLoop(bool jogPositionControl, bool jogAngleControl, bool motionWasBusy,
               bool manualPitchBusy, float jogLeanOffset) {
  // --------------------------------------------------------
  // 4. 标准串级位置P外环：位置误差 -> 受限目标速度
  // --------------------------------------------------------
  // ---- SP 速度指令：唯一的速度设定入口 ----
  // 与 ^ 手动通路互斥：^ 正在动作时清掉 SP，避免两套通路同时写目标速度。
  if (motionWasBusy) {
    speedCmd = 0.0f;
  } else if (speedCmd != 0.0f) {
    targetForwardSpeed = speedCmd;  // 直接进入下面的经典速度 PI
  }
  if (jogPositionControl) positionSpeedTarget = targetForwardSpeed;
  // --------------------------------------------------------
  // 5. 总目标速度安全限幅
  // --------------------------------------------------------
  if (targetForwardSpeed > MAX_TARGET_FORWARD_SPEED) {
    targetForwardSpeed = MAX_TARGET_FORWARD_SPEED;
  }
  if (targetForwardSpeed < - MAX_TARGET_FORWARD_SPEED) {
    targetForwardSpeed = - MAX_TARGET_FORWARD_SPEED;
  }
  // --------------------------------------------------------
  // 6. 速度误差
  // --------------------------------------------------------
  // 2026-11-12 回退：τ=0.25s 的速度低通把速度外环推过临界（pitch sd 0.17°→3.0°），
  // 回到原始瞬时速度；滤波思路保留在 SPEED_MEAS_TAU_S 注释里，要用只能小步试。
  vehicleSpeedFiltered = vehicleSpeed;
  speedError = targetForwardSpeed - vehicleSpeed;
  // --------------------------------------------------------
  // 7. 速度PI生成目标角。第一版严格采用成熟串级结构，
  //
  // 不使用速度预测、STEP直投或位置回位。
  // --------------------------------------------------------
  // 经典速度PI目标角通道：100Hz更新。
  // 微分：100Hz 下 (err-errPrev)/0.01s 等价 ×100，得到 count/50ms 每秒的变化率；随后 0.7/0.3 一阶低通。
  float speedErrDot = (speedError - speedErrorPrev) * 100.0f;
  speedErrorDotFilt = 0.7f * speedErrorDotFilt + 0.3f * speedErrDot;  // 低通滤噪声
  speedErrorPrev = speedError;
  speedPContribution = speedKp * speedError;
  float speedDContribution = speedKd * speedErrorDotFilt;
  // 条件积分 + 抗饱和：禁止在外环无权限或输出继续撞向N限幅时累积。
  if (jogAngleControl || manualPitchBusy || fabsf(speedKi) < 0.000001f) {
    speedIntegral = 0.0f;
  } else if (outerLoopAuthority >= 0.2f && fabsf(speedError) > SPEED_ERROR_DEADBAND) {
    float candidateIntegral = speedIntegral + speedError * 0.01f;
    if (candidateIntegral > SPEED_INTEGRAL_LIMIT) candidateIntegral = SPEED_INTEGRAL_LIMIT;
    else if (candidateIntegral < - SPEED_INTEGRAL_LIMIT) candidateIntegral = - SPEED_INTEGRAL_LIMIT;
    float currentUnclamped = speedLoopDirection * (speedPContribution + speedKi * speedIntegral + speedDContribution);
    float candidateUnclamped = speedLoopDirection * (speedPContribution + speedKi * candidateIntegral + speedDContribution);
    if (fabsf(candidateUnclamped) <= MAX_SPEED_ANGLE || fabsf(candidateUnclamped) < fabsf(currentUnclamped)) {
      speedIntegral = candidateIntegral;
    }
  }
  speedIContribution = speedKi * speedIntegral;
  // Manual forward/reverse uses the operator target with P/D only; no hidden I trim.
  // 量纲逐项：P=speedKp[deg/(count/50ms)]×err[count/50ms]→deg；I=speedKi[deg/((count/50ms)·s)]×integral→deg；
  //   D=speedKd[deg·s/(count/50ms)]×dot[(count/50ms)/s]→deg；dir[±1] 只翻符号；jogLeanOffset 直接加在限幅前。
  // 饱和：|speedAngleUnclamped|>=N=0.8° 即被下一行硬限幅；K=-0.10、dir=-1 时 |err|>=8 count/50ms（≈14.8mm/s）就饱和。
  speedAngleUnclamped = speedLoopDirection * (speedPContribution + speedIContribution + speedDContribution) + jogLeanOffset;
  float desiredSpeedAngle = speedAngleUnclamped;
  // 遥测第 19 列：保留外环时刻的速度快照（原预测速度列位，数值不变）。
  telemetrySpeedSnapshot100Hz = vehicleSpeed;
  if (desiredSpeedAngle > MAX_SPEED_ANGLE) desiredSpeedAngle = MAX_SPEED_ANGLE;
  if (desiredSpeedAngle < - MAX_SPEED_ANGLE) desiredSpeedAngle = - MAX_SPEED_ANGLE;
  speedAngleLimited = desiredSpeedAngle;
  // 速度环全部关闭时立即清零历史修正，保证纯直立环测试不被残留目标角污染。
  if (fabsf(speedKp) < 0.000001f && fabsf(speedKi) < 0.000001f && fabsf(speedKd) < 0.000001f) {
    leanResetRequested = true;
    speedAngleCorrection = 0.0f;
    speedIntegral = 0.0f;
    speedPContribution = 0.0f;
    speedIContribution = 0.0f;
    speedAngleUnclamped = 0.0f;
    speedAngleLimited = 0.0f;
    speedErrorDotFilt = 0.0f;
    speedErrorPrev = speedError;
    telemetrySpeedSnapshot100Hz = vehicleSpeed;
    positionSpeedTarget = 0.0f;
    targetForwardSpeed = 0.0f;
  } else {
  // slew：每 100Hz 拍朝目标最多挪 0.10° ⇒ 最大 10°/s，防目标角阶跃直接砸进直立环。
    float speedAngleDelta = desiredSpeedAngle - speedAngleCorrection;
    if (speedAngleDelta > SPEED_ANGLE_SLEW_DEG_PER_UPDATE) speedAngleDelta = SPEED_ANGLE_SLEW_DEG_PER_UPDATE;
    else if (speedAngleDelta < - SPEED_ANGLE_SLEW_DEG_PER_UPDATE) speedAngleDelta = - SPEED_ANGLE_SLEW_DEG_PER_UPDATE;
    speedAngleCorrection += speedAngleDelta;
  }
  // --------------------------------------------------------

}
// ============================================================
// 外环权限管理
//
// 使用相对机械平衡角，而不是相对 targetBalanceAngle。
// 这样速度PI改变目标角时，不会影响“大角度判定”。
// ============================================================
// updateAuthority：400Hz 外环权限 + 目标角合成（无返回）。调用：main.cpp loop 每拍。
// 参数：normalAuthority=手动俯仰正常权限 0~1（ManualPitchControl 平滑驻留）；leanOffset=运动倾角 deg。
// ① 相对机械平衡角判定：uprightError=pitch-O（不用 targetBalanceAngle，避免速度环改目标反过来影响权限判定）；
// ② |err|<=F(5°) → authority=1；F~G(5~10°) 线性降到 0；>=G → 0；
// ③ pitchOffset=normalAuthority*speedAngleCorrection+leanOffset；
//    targetBalanceAngle=O+authority*pitchOffset（大角度时 target 回落到 O，直立 PD 全力救车）；
// ④ authority<0.2 时 speedIntegral*=0.95（每 400Hz 拍；1s 后约 1e-9，防恢复后旧积分推车）。
// 边界：normalAuthority/leanOffset 均为有限值（由 remote 状态机保证）；authority 限在 0~1。
void updateAuthority(float normalAuthority, float leanOffset) {
  float uprightError = pitch - balanceAngleOffset;
  float absUprightError = fabsf(uprightError);
  if (absUprightError <= OUTER_FULL_ANGLE_DEG) {
    outerLoopAuthority = 1.0f;
  } else if (absUprightError < OUTER_DISABLE_ANGLE_DEG) {
    outerLoopAuthority = (OUTER_DISABLE_ANGLE_DEG - absUprightError) / (OUTER_DISABLE_ANGLE_DEG - OUTER_FULL_ANGLE_DEG);
  } else {
    outerLoopAuthority = 0.0f;
  }
  // 目标角必须受同一个权限控制。
  // 大角度时直接回到机械平衡角，让直立PD全力救车。
  // Normal operation uses the speed-loop angle; held motion replaces it smoothly.
  // The held angle is relative to O. 2026-11-05：前后运动改为速度闭环，不再写死角度。
  float pitchOffset = normalAuthority * speedAngleCorrection + leanOffset;
  targetBalanceAngle = balanceAngleOffset + outerLoopAuthority * pitchOffset;
  // 大角度时释放历史积分，避免恢复后旧积分继续把车推走
  if (outerLoopAuthority < 0.2f) {
    speedIntegral *= SPEED_INTEGRAL_DECAY_BIG_ANGLE;
  }

}

// ============================================================
// ^ 运动驱动接管时清零速度环历史（原 updateOuterLoop 运动分支里的 3 行清零）
// ============================================================
// clearSpeedLoopForMotionBypass：^ 运动驱动接管时清速度环历史（无参/无返回）。
// 清：speedError/speedIntegral/speedErrorPrev/speedErrorDotFilt/P项/I项/修正角/未限幅量/限幅量。
// 调用：remote::motionDriveTick（100Hz，运动入口与释放段）｜ 防止运动接管瞬间速度环残留把目标角顶跳。
void clearSpeedLoopForMotionBypass() {
  speedError = speedIntegral = speedErrorPrev = speedErrorDotFilt = 0.0f;
  speedPContribution = speedIContribution = 0.0f;
  speedAngleCorrection = speedAngleUnclamped = speedAngleLimited = 0.0f;
}

}  // namespace control
