#include "remote/position_jog.h"
#include "remote/remote_control.h"

// vehiclePosition — 车体累计位移坐标（单位：平均编码器 count，10.8 counts = 1 mm，故 mm=count/10.8）。
// 写：positionHold()/captureCurrentPositionTarget()（400Hz/事件级，经 PositionOdometry）；倒地保护/开机时被清零。
// 读：positionJog、位置 P 外环、motion 位置计划、遥测（D 帧 pos/100）。更新频率：400Hz。
// 坐标零点 = 首次建立里程基准时（开机/跌倒恢复后）的位置，之后只累计相对位移。
// 车辆相对开机/重新站立位置的累计位移，单位为平均编码器count
float vehiclePosition = 0.0f;

// positionZeroCount1/2 — 位置原点对应的两路编码器累计计数（count）。
// 写：main setup 与倒地保护分支快照；读：当前无读者（历史/诊断字段保留，不参与坐标计算）。
// 位置零点对应的两个编码器累计计数
int32_t positionZeroCount1 = 0;
int32_t positionZeroCount2 = 0;

// targetVehiclePosition — 驻车目标位置（单位 count，与 vehiclePosition 同坐标）。
// 写：captureCurrentPositionTarget()/positionHold()/positionJogTick()；读：positionHold100() 算位置误差。
// 更新频率：400Hz（跟随）或事件级（capture）。
// （注：旧注释"目标位置固定为0，即尽量回到启动位置"与当前实现不符，以代码为准：
//   目标会被 capture 成"当前位置"（X 环启用/跌倒恢复/点动收尾），并随点动参考 reference 移动；
//   只有开机初始值才是 0。）
// 目标位置固定为0，即尽量回到启动位置
float targetVehiclePosition = 0.0f;

namespace remote {

// ============================================================
// 100Hz 点动调度（原 updateOuterLoop 开头的 positionJog 分支）
// ============================================================
// positionJogTick — 100Hz 点动调度：推进状态机；点动进行中把 reference 当驻车目标，结束那一拍做收尾。
// 参数 jogPositionControl：输出标志（true=本拍 jog 位置控制接管，tick100 后续会据此清速度目标）。返回：无。
// 调用：remote::tick100() 每 100Hz。边界：点动结束但非 ARRIVED 时也会 capture（强制以当前位置停车）。
void positionJogTick(bool & jogPositionControl) {
  jogPositionControl = false;
  // 只有 active 才推进；mode 由两路编码器信任门决定（0=双路平均、1/2=单路、3=都不信）。
  if (positionJog.active) {
    int mode = (encoder1Trusted && encoder2Trusted) ? 0: (encoder1Trusted ? 1: (encoder2Trusted ? 2: 3));
    // 状态机推进：内部可能因 到位/超时/心跳丢失/通道变化/故障 自行 cancel（active 变 false）。
    positionJog.update(vehiclePosition, vehicleSpeed, mode, millis(), .01f);
    // 本拍刚结束：先按当前位置重新 capture 驻车目标；若是"到位"结束，则用到位窗口平均中心做目标（更准，抵消抖动）。
    if (!positionJog.active) {
      captureCurrentPositionTarget();
      if (positionJog.reason == PositionJog::ARRIVED) {
        targetVehiclePosition = positionJog.arrival.center;
        positionError = targetVehiclePosition - vehiclePosition;
      }
    } else {
      // 点动进行中：目标位置 = 参考轨迹（reference），位置误差交给 jog 倾角环；
      // 同时清掉速度环的两个目标，避免 400Hz 速度 PI 与点动抢目标。
      jogPositionControl = true;
      targetVehiclePosition = positionJog.reference;
      positionError = targetVehiclePosition - vehiclePosition;
      positionSpeedTarget = targetForwardSpeed = 0;
    }
  }
}

// ============================================================
// 100Hz 位置 P 外环（原 updateOuterLoop 的 !jogPositionControl 段）
// ============================================================
// positionHold100 — 100Hz 位置 P 外环：非点动时把位置误差变成目标速度（count/50ms）。
// 启用条件：positionKp（面板 X）>1e-6，编译默认 0.02（实测安全值；旧值 5.0 会把静止漂移放大到 230counts/s）。
// 公式：speed = clamp(positionKp × 去死区误差, ±MAX_POSITION_RETURN_SPEED)；
//   0.02 (count/50ms)/count、Y=0.30 count/50ms ≈0.56mm/s、M=12 count ≈1.1mm。
// 边界：点动接管时整段跳过；positionHoldSuspended 时输出 0；速度输出再经 positionSpeedReference 限斜率。
void positionHold100(bool jogPositionControl) {
  bool positionHoldEnabled = positionKp > 0.000001f;
  // 点动接管时整段不执行（避免两套位置控制同时改 targetForwardSpeed）。
  if (!jogPositionControl) {
    // 上升沿（X 由关到开）：以当前位置为目标（不然会把"开机以来的位移"当成误差去追，猛冲）。
    if (positionHoldEnabled && !positionHoldWasEnabled && !positionHoldSuspended) {
      captureCurrentPositionTarget();
      positionError = 0.0f;
    }
    // 记住本拍启用状态，供下一拍边沿检测；随后按启用/悬挂与否决定输出。
    positionHoldWasEnabled = positionHoldEnabled;
    if (positionHoldEnabled && !positionHoldSuspended) {
      // 死区：|误差|≤POSITION_DEADBAND_COUNT(M=12count≈1.1mm) 直接当 0；超出则先减去死区再算 P（避免死区边缘抖动）。
      float effectivePositionError = positionError;
      if (effectivePositionError > POSITION_DEADBAND_COUNT) effectivePositionError -= POSITION_DEADBAND_COUNT;
      else if (effectivePositionError < - POSITION_DEADBAND_COUNT) effectivePositionError += POSITION_DEADBAND_COUNT;
      else effectivePositionError = 0.0f;
      // P 环：目标速度 = positionKp × 去死区误差（count/50ms）；再钳 ±MAX_POSITION_RETURN_SPEED(Y=0.30)。
      positionSpeedTarget = positionKp * effectivePositionError;
      if (positionSpeedTarget > MAX_POSITION_RETURN_SPEED) positionSpeedTarget = MAX_POSITION_RETURN_SPEED;
      else if (positionSpeedTarget < - MAX_POSITION_RETURN_SPEED) positionSpeedTarget = - MAX_POSITION_RETURN_SPEED;
      targetForwardSpeed = positionSpeedTarget;
    } else {
      // 位置环关闭/悬挂：速度目标清零（不参与速度环）。
      positionSpeedTarget = 0.0f;
      targetForwardSpeed = 0.0f;
    }
  }  // !jogPositionControl
}

// ============================================================
// 位置快速拉回（原 updatePositionHold）
//
// 400Hz直接读取编码器累计位置。
// 不经过50ms速度窗口，所以只要编码器出现少量位移就开始反向拉回。
// ============================================================
// positionHold — 400Hz 位置里程 + 驻车目标维护（原 updatePositionHold）。
// 每拍原子读两路编码器 → 更新 vehiclePosition；再处理点动故障、编码器通道变化、手动通路悬挂、恢复时的目标重建。
// 不经过 50ms 速度窗口：编码器只要有少量位移就更新位置（位置环比速度环更早发现漂移）。
// 调用：main loop 远控 400Hz 段，必须在 100Hz 外环之前（外环要用最新位置）。
void positionHold() {
  int32_t a, b;
  // 临界区快照两路计数（32 位、ISR 写），保证同一时刻采样；随后按信任模式算里程。
  noInterrupts();
  a = encoder1Count;
  b = encoder2Count;
  interrupts();
  int mode = (encoder1Trusted && encoder2Trusted) ? 0: (encoder1Trusted ? 1: (encoder2Trusted ? 2: 3));
  vehiclePosition = positionOdom.update(a, b, mode);
  // 点动安全门：倾角误差 >5° 或速度环关闭（|K|<1e-6）→ 以 FAULT 停止点动（车没立稳就不该继续走位）。
  if (positionJog.active && (fabsf(pitch - balanceAngleOffset) > 5 || fabsf(speedKp) < .000001f)) {
    stopPositionJog(PositionJog::FAULT);
    return;
  }
  // 编码器可信通道在点动中变化：坐标可能跳变，立即以 SOURCE_CHANGED 停止。
  if (positionJog.active && mode != positionJog.source) {
    stopPositionJog(PositionJog::SOURCE_CHANGED);
    return;
  }
  // Follow actual position throughout manual tilt, return-to-O and speed-loop handoff.
  // The parking loop must not accumulate an error against a stale pre-motion target.
  // followPosition：手动 ^ 正在动作（isActive）或释放斜坡未满（normalAuthority<1）时，驻车目标跟随实际位置，
  // 不累计位置误差——否则手动走过的距离会在松手瞬间被驻车环当成"要拉回"而猛拽。
  bool followPosition = isActive() || manualForward.normalAuthority() < 1.0f;
  // resume：悬挂刚结束（本拍不再跟随）→ 需要以当前位置重建目标，避免用悬挂前的旧目标。
  bool resume = positionHoldSuspended && !followPosition;
  positionHoldSuspended = followPosition;
  // 悬挂期间：若有点动先停掉（STOP）；目标=实际、速度参考/误差清零，然后直接返回（本拍不做位置闭环）。
  if (followPosition) {
    if (positionJog.active) stopPositionJog(PositionJog::STOP);
    targetVehiclePosition = vehiclePosition;
    positionSpeedReference.reset();
    positionError = positionSpeedTarget = targetForwardSpeed = 0;
    return;
  }
  // 恢复驻车：若刚结束悬挂且没有点动，则以当前位置为新目标（无扰接管）。
  if (resume && !positionJog.active) targetVehiclePosition = vehiclePosition;
  // 点动中：驻车目标跟随参考轨迹（reference），供 jog 倾角环/位置误差使用。
  if (positionJog.active) targetVehiclePosition = positionJog.reference;
  // 位置误差 = 目标 − 实际（count）；100Hz 位置 P 外环（positionHold100）消费它。
  positionError = targetVehiclePosition - vehiclePosition;
}

// 把当前位置设为新的驻车位置。用于启用位置环、跌倒恢复和面板手动重设。
// captureCurrentPositionTarget — 把当前位置锁定为新的驻车目标（无扰接管）。
// 调用：启用位置环（X 0→非0）、跌倒恢复、点动收尾、面板 C 命令（事件级）。
// 边界：先停点动；只重置目标/误差/速度参考，不改编码器计数与里程坐标（vehiclePosition 连续）。
void captureCurrentPositionTarget() {
  if (positionJog.active) positionJog.cancel(PositionJog::STOP);
  int32_t a, b;
  noInterrupts();
  a = encoder1Count;
  b = encoder2Count;
  interrupts();
  int mode = (encoder1Trusted && encoder2Trusted) ? 0: (encoder1Trusted ? 1: (encoder2Trusted ? 2: 3));
  vehiclePosition = positionOdom.update(a, b, mode);
  // Capturing a parking target never rewrites encoder counters or the odometry coordinate.
  targetVehiclePosition = vehiclePosition;
  positionError = positionSpeedTarget = 0;
  // 无扰接管：把"当前实际下发速度参考"直接当目标，并把速度环的微分历史也对齐，
  // 这样速度 PI 不会因为参考突变而猛冲（speedIntegral 清零、speedErrorPrev 置为当前误差）。
  targetForwardSpeed = positionSpeedReference.applied;
  speedIntegral = 0;
  speedErrorPrev = targetForwardSpeed - vehicleSpeed;
  speedErrorDotFilt = 0;
}

// stopPositionJog — 按原因停止点动并立即以当前位置收尾（= cancel + capture）。
// 参数 reason：PositionJog::Reason（STOP/FAULT/STOP 等）。返回：无。调用：故障/转向/新命令/上层接管（事件级）。
void stopPositionJog(int reason) {
  positionJog.cancel(reason);
  captureCurrentPositionTarget();
}

}  // namespace remote
