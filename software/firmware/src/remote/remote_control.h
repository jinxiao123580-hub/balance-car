#pragma once
// ============================================================
// ============================================================
// remote_control — 远控层唯一对外入口：把 ^ 手动通路/位置点动/运动驱动封装成 control 可消费的少量接口。
// 依赖方向：remote → control/core 允许；remote 不 include panel（panel 反过来 include remote）。
// 调用关系：main loop 在每个 400Hz/100Hz 段按固定顺序调用 tick/positionHold/tick100/updateTurn；
//   control 侧只通过包含的函数看到远控：normalAuthority/leanOffset/turnOffset/motionWasBusy/jogLeanOffset；
//   panel/cmd.cpp 调命令入口：captureCurrentPositionTarget/stopPositionJog。
// ============================================================
// remote/remote_control.{h,cpp} — ^ 手动通路 / 点动调度 / EG 增益
// 以及远控对控制核心的数据接口（leanOffset / turnOffset / isActive）。
//
// 依赖方向：remote → control/core 允许；remote 不 include panel。
// ============================================================
#include <Arduino.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"

#include "control/balance.h"
#include "control/cascade.h"
#include "control/motor.h"

// 远控子模块
#include "remote/manual_pitch.h"
#include "remote/manual_turn.h"
#include "remote/motion_drive.h"
#include "remote/position_jog.h"

namespace remote {

// ---- 400Hz -----------------------------------------------------
// 手动俯仰目标平滑（原 loop 中的 manualForward.update 一行）
// tick — 作用：400Hz 推进手动俯仰物体（ManualPitchControl）的轨迹。
// 参数 dt：控制周期（s，传 CONTROL_DT=0.0025）。返回：无。
// 调用：main loop 远控 400Hz 段，每 400Hz 一次。
// 边界：轨迹内的超时/大角度保护由 ManualPitchControl::update() 负责（≥10° 清状态并闩锁）。
void tick(float dt);
// 位置保持：400Hz 读累计位置，维护停车目标（原 updatePositionHold）
// positionHold — 作用：400Hz 位置里程维护 + 驻车目标维护（详见 position_jog.cpp）。
// 参数：无。返回：无。调用：main loop 远控 400Hz 段，必须在 100Hz 外环之前（外环要最新位置）。
// 边界：内部处理点动故障/编码器通道变化/手动通路悬挂；异常时以 FAULT/SOURCE_CHANGED 停点动。
void positionHold();
// 转向环：legacy PD + 手动转向状态机 → turnOutput（原 updateTurnLoop）
// updateTurn — 作用：400Hz 转向环（legacy PD + 转向状态机 → turnOutput）。
// 参数：无。返回：无。调用：main loop 每 400Hz，必须在直立环之后（要用 finalBalanceOutput 算剩余空间）。
// 边界：限幅/超时/大角度禁止转向全部在 ManualTurnControl::update() 内（见 manual_turn.h）。
void updateTurn();

// ---- 100Hz -----------------------------------------------------
// 运动驱动/点动调度（原 updateOuterLoop 第 4 步的远控部分）。
// 返回 true = 运动驱动接管本拍（原代码里的 "return"，速度 PI 必须跳过）。
// jogPos / jogAngle 为回传的调度标志，交给 control::outerLoop 用于积分清零。
// tick100 — 作用：远控 100Hz 调度器（等价原 updateOuterLoop 里远控那几段），按固定顺序执行：
//   ① 点动调度 positionJogTick → ② 运动驱动 motionDriveTick（接管则本拍直接返回 true）
//   → ③ 位置 P 外环 positionHold100 → ④ jog 倾角环 → ⑤ 速度参考斜率限制。
// 参数 jogPos/jogAngle：回传标志（true=本拍位置由 jog 控制/jog 倾角参与），main 传给 control::outerLoop() 用于积分清零。
// 返回：true=运动驱动接管本拍（main 必须跳过速度 PI，避免两套目标角打架）；false=常规路径。
// 调用：main loop 每 4 拍（100Hz）一次。**必须**在 control::tick100() 之后（位置/速度已更新）。
bool tick100(bool & jogPos, bool & jogAngle);

// ---- 位置点动入口（panel/cmd 调用）------------------------------
// captureCurrentPositionTarget — 作用：把当前位置设为新的驻车目标并停止点动。
// 参数：无。返回：无。调用：panel/cmd.cpp 的 C 命令、启停位置环、点动收尾、跌倒恢复。
void captureCurrentPositionTarget();
// stopPositionJog — 作用：按原因停止点动并收尾（cancel + capture）。
// 参数 reason：PositionJog::Reason（STOP/FAULT 等）。返回：无。调用：panel/cmd.cpp/positionHold() 故障路径。
void stopPositionJog(int reason);

// ---- 数据接口：control 只能通过这些函数"看到"远控，不得直接访问 remote 的对象 ----------
// ---- 数据接口（control 只能通过这些量看到远控，见 SPEC §2）--------
// isActive — 作用：转向或手动俯仰任一仍在占用（等价原 manualMotionIsActive()）。
// 参数：无。返回 bool。调用：params 写盘前置检查、cmd 的 ^/~ 前置检查、高频状态查询。
bool isActive();                        // 等价原 manualMotionIsActive()
// leanOffset — 作用：运动/jog 叠加的目标倾角修正（等价 motionLean.applied）。
// 单位 deg（±0.90）。调用：main 每 400Hz 传给 control::updateAuthority()。
float leanOffset();                     // 等价 motionLean.applied
// turnOffset — 作用：转向差模输出（等价 turnOutput）。单位 STEP Hz。
// 调用：main 每 400Hz 传给 control::mixMotors()；无转向命令时恒 0。
float turnOffset();                     // 等价 turnOutput
// normalAuthority — 作用：手动俯仰释放斜坡（0..1），给外环权限用（等价 manualForward.normalAuthority()）。
// 调用：main 每 400Hz 传给 control::updateAuthority()。
float normalAuthority();                // 等价 manualForward.normalAuthority()
// motionWasBusy — 作用：上一拍运动驱动是否接管（等价 manualMotionWasBusy）。
// 调用：main 每 100Hz 传给 control::outerLoop()（运动旁路时清速度环历史）。
bool motionWasBusy();                   // 等价 manualMotionWasBusy
// resetLean — 作用：清零 jog 倾角（位置点动的 PositionLeanControl）。
// 调用：main 在 leanResetRequested 时（速度环被整体关闭）每 100Hz 检查一次。
void resetLean();                       // 速度环整体关闭时清零 jog 倾角
// resetFault — 作用：停机/故障统一清零远控侧状态机（转向/俯仰/点动/倾角/速度参考）。
// 调用：main 的 clearControlOutputDiagnostics() → setup 重试、掉拍、IMU 故障、倒地（事件级）。
void resetFault();                      // 停机/故障统一清零远控状态机（原 clearControlOutputDiagnostics 远控部分）
// manualPitchBusy — 作用：手动俯仰是否占用（等价 manualForward.busy(millis())）。
// 调用：main 每 100Hz 传给 control::outerLoop()；cmd 的 ^ 前置检查。
bool manualPitchBusy();                 // 等价 manualForward.busy(millis())
// jogLeanOffset — 作用：jog 倾角当前输出（等价 positionLean.applied）。单位 deg（±0.30）。
// 调用：main 每 100Hz 传给 control::outerLoop()。
float jogLeanOffset();                  // 等价 positionLean.applied

}  // namespace remote
