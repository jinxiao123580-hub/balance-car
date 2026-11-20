#pragma once
// ============================================================
// remote/manual_pitch.{h,cpp} — 手动俯仰角目标（^ 通路的前后分量）
// （ManualPitchControl 原样搬自 src/manual_motion.h，parseManualMotion 同）
// ============================================================
#include <Arduino.h>
#include <cstdlib>
#include <cmath>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"
#include "remote/manual_turn.h"


// ============================================================
// ManualPitchControl — ^ 手动通路的前后分量：把"固定相对倾角"做成连续角度/角速率轨迹。
// 作用：把一条 ^ turnHz,pitchDeg 命令（pitchDeg ∈ [-2°, +2°]）平滑成 planned/applied 倾角轨迹，
//   并用 normalAuthority() 给外环一个 0→1 的释放斜坡，避免手动通路与速度环交接时目标角跳变。
// 单位：角度 deg、角速度 deg/s、角加速度 deg/s²、时间 ms/s。
// 调用者/频率：update() 由 remote::tick() 在 loop 400Hz 调用；command() 由 panel/cmd.cpp 收到 ^ 时调用（人工级）；
//   normalAuthority()/busy() 由 remote::normalAuthority()/isActive() 与 main 每次 400Hz 读取。
// 边界：角度硬限 ±2°；命令 350ms 超时自动回 0；|pitch-平衡点| ≥10° 直接 clear() 并闩锁 needsZero；
//   NaN/Inf/非正 dt 一律 clear() 并返回 0（见 update()）。
// 轨迹形状：MOVE 段用三次 smoothstep（起止速率=0），中途改目标先 BRAKE（按 4°/s² 减速到 0）再重新 MOVE，
//   所以任何时刻加速度有界、角度/速率连续；释放（^ 命令消失）时 applied 不会突变。
// ============================================================
// Fixed relative pitch with continuous angle/rate transitions, including release mid-ramp.
struct ManualPitchControl {
    // phase：轨迹状态机。IDLE=静止/已到位；MOVE=沿 smoothstep 从 start 到 planned；
    // BRAKE=改目标或释放时先按 maxAcceleration 把当前 rate 刹到 0（含 elapsed/duration 计时）。
    enum Phase {IDLE=0, MOVE=1, BRAKE=2};
    // maxAngle=2.0：手动倾角硬上限（deg）。^ 的 forward 参数被钳在 ±2°；
    // motion_drive 再把它归一化成速度档位：planned/maxAngle ∈ [-1,+1] × 满档速度 5 count/50ms。
    // 2° 是历史实测值：旧做法直接写 2° 死倾角，现做法它只当"档位刻度"，真正倾角由速度环生成。
    static constexpr float maxAngle=2.0f;
    // maxRate=2.0 deg/s：轨迹最大角速率。smoothstep 的 duration 下界取 1.5|delta|/maxRate，
    // 即 |delta|=2° → 至少 1.5 s、|delta|=4°（-2°→+2°）→ 至少 3 s，保证不出现阶跃。
    static constexpr float maxRate=2.0f;       // deg/s
    // maxAcceleration=4.0 deg/s²：轨迹最大角加速度。smoothstep 的 duration 下界取
    // sqrt(6|delta|/maxAcceleration)（三次曲线两端 |a|=6|delta|/duration²）；BRAKE 段也用它减速。
    static constexpr float maxAcceleration=4.0f; // deg/s^2
    // requested：最新一条 ^ 命令要求的目标倾角（deg，±2°）；command() 写入，update() 消费。
    // applied：当前实际输出倾角（deg，±2°）；每拍 update() 后由轨迹重新赋值，读方（遥测/运动环）只看它。
    // rate：当前角速率（deg/s，±maxRate 量级）；由 MOVE/BRAKE 段解析式给出，用于判断是否需要先刹车。
    // 更新频率：400Hz（applied/rate）；requested 只在收到命令时变。
    float requested=0,applied=0,rate=0;
    // receivedMs：最后一条有效 ^ 命令的 millis() 时间戳（ms）。busy()/update() 用它判 350ms 超时；
    // 超时后 requested=0 → 轨迹平滑回 0（不是硬切）。
    uint32_t receivedMs=0;
    // needsZero：安全闩锁（bool）。clear() 置 true；此状态下 command() 会忽略一切非 0 命令，
    // 直到收到一条 0 命令才解锁。用途：故障/倒地/大角度退出后，必须先明确"回零"才允许再次给倾角。
    bool needsZero=false;
    // phase：见 Phase 枚举（IDLE/MOVE/BRAKE），只在 beginMove()/update() 内迁移。
    int phase=IDLE;
    // planned：当前轨迹目标倾角（deg，±2°）= requested 的钳位副本；elapsed：当前段已走时间（s）；
    // duration：当前段总时长（s，由 beginMove() 按 maxRate/maxAcceleration 解析计算）；
    // start：当前段起始倾角（deg）；delta = planned − start（deg）；brakeRate：进入 BRAKE 时的角速率（deg/s）；
    // resumeTime：释放斜坡计时（0..1 s），normalAuthority() 对它做 smoothstep → 0→1 权限，1 s 内恢复满权。
    float planned=0,elapsed=0,duration=0,start=0,delta=0,brakeRate=0,resumeTime=1;
    // clear — 作用：故障/大角度退出时把轨迹与命令全部清零并闩锁。
    // 参数：无。返回：无。调用：手动通路故障路径、update() 的 |角度误差|≥10° 保护、remote::resetFault()。
    // 边界：置 needsZero=true 并 resumeTime=0（权限从 0 重新爬升）；applied/requested/rate/planned 全为 0。
    void clear(){requested=applied=rate=planned=0;phase=IDLE;elapsed=duration=0;resumeTime=0;needsZero=true;}
    // seed — 作用：进入手动通路时用"当前已输出的外环倾角"做初值，避免入口处目标角跳变。
    // 参数 offset：当前速度环已输出的倾角修正（deg，一般 |offset|≤N≈0.8°）。返回：无。
    // 调用：panel/cmd.cpp 收到 ^ 且 manualForward 空闲时（人工级，低频）。边界：NaN→0；钳到 ±maxAngle。
    void seed(float offset){
        applied=ManualTurnControl::clamp(offset,-maxAngle,maxAngle);rate=0;
        planned=applied;phase=IDLE;elapsed=duration=0;resumeTime=0;
    }
    // command — 作用：登记一条新的 ^ 目标倾角（人工命令入口）。
    // 参数 angle：目标倾角（deg，±2°，超出钳位）；now：millis() 时间戳（ms）。返回：无。
    // 调用：panel/cmd.cpp 解析 ^ 后，两侧轴各自调用（人工级）。
    // 边界：angle==0 先解锁 needsZero 再 return（只解锁不登记，保持 requested=0）；needsZero 时忽略非 0 命令；
    //   NaN/Inf → requested=0。
    void command(float angle,uint32_t now){
        if(angle==0)needsZero=false;
        if(needsZero)return;
        requested=std::isfinite(angle) ? ManualTurnControl::clamp(angle,-maxAngle,maxAngle) : 0;
        receivedMs=now;
    }
    // busy — 作用：判断手动俯仰是否仍在"占用"（有新鲜命令、正在走轨迹、或还有残余倾角）。
    // 参数 now：millis()。返回：bool（true=占用）。调用：remote::isActive()、cmd 的 ^ 前置检查、500ms 级状态查询。
    // 边界：350ms 内且 requested≠0 算占用；phase≠IDLE 算占用；|applied|>1e-6 算占用（释放段仍占）。
    bool busy(uint32_t now)const{
        return (requested!=0 && uint32_t(now-receivedMs)<=350) || phase!=IDLE || std::fabs(applied)>.000001f;
    }
    // normalAuthority — 作用：把"释放斜坡"变成 0..1 的外环权限（三次 smoothstep）。
    // 参数/返回：无参，返回 0..1（1=手动通路完全退出、速度环可满权）。
    // 调用：main 每 400Hz 传给 control::updateAuthority()（remote::normalAuthority() 包装）。
    // 边界：busy 时 resumeTime=0 → 权限 0；释放后 resumeTime 以 1/s 爬升，1 s 到 1。
    float normalAuthority()const{
        float u=ManualTurnControl::clamp(resumeTime,0,1);
        return u*u*(3-2*u);
    }
    // beginMove — 作用：从当前 applied 出发规划一段新的 smoothstep（MOVE）到 planned。
    // 参数/返回：无；只改内部状态。调用：update() 内（改目标且当前无速率时）。
    // 边界：|delta|≈0 直接到位、phase=IDLE；否则 duration = max(1.5|delta|/maxRate, sqrt(6|delta|/maxAcceleration))。
    void beginMove(){
        start=applied;delta=planned-start;elapsed=0;
        if(std::fabs(delta)<.000001f){applied=planned;rate=0;phase=IDLE;return;}
        // Cubic smoothstep starts/ends with zero rate. These bounds hold analytically.
        duration=std::fmax(1.5f*std::fabs(delta)/maxRate,std::sqrt(6*std::fabs(delta)/maxAcceleration));
        phase=MOVE;
    }
    // update — 作用：400Hz 轨迹推进；每拍最多串行处理 3 个状态段（BRAKE→MOVE）保证不超时。
    // 参数：now=millis()；dt=控制周期 s（400Hz 时 0.0025）；angleError=pitch−平衡点（deg，供保护用）。
    // 返回：无（结果写 applied/rate/phase）。调用：remote::tick() → 本函数，loop 每 400Hz 一次。
    // 边界：350ms 无新命令 → requested=0；非有限值或 |angleError|≥10° → clear() 并 return（保护：
    //   车已经明显偏离平衡点时不再叠加倾角命令）；结束位置硬钳 ±maxAngle。
    void update(uint32_t now,float dt,float angleError){
        if(uint32_t(now-receivedMs)>350)requested=0;
        // 保护：角度误差 ≥10°（deg）说明车已经要倒/被拿起来，手动倾角会雪上加霜 → 立即清零并闩锁 needsZero。
        // 10° 是经验门限，远大于正常平衡误差（±1° 量级），不会误伤正常 ^ 操作。
        if(!std::isfinite(angleError)||!std::isfinite(dt)||dt<=0||std::fabs(angleError)>=10){clear();return;}
        // 目标变化时重新规划：若当前还在走（rate≠0），先插一段 BRAKE 把速率刹到 0，再 beginMove()，
        // 保证轨迹速度连续（不会在 MOVE 中途直接瞬移目标）。
        if(requested!=planned){
            planned=requested;
            if(phase!=BRAKE){
                if(std::fabs(rate)>.000001f){
                    start=applied;brakeRate=rate;elapsed=0;duration=std::fabs(rate)/maxAcceleration;phase=BRAKE;
                }else beginMove();
            }
        }
        float left=dt;
        // 每拍最多推进 3 个状态段：BRAKE 结束后可能立刻接 MOVE，剩 dt 要接着算；
        // 限制 3 次是为了防异常参数下死循环（实际最多 BRAKE+MOVE 两段）。
        for(int transition=0;transition<3 && left>0 && phase!=IDLE;transition++){
            float step=std::fmin(left,duration-elapsed);elapsed+=step;left-=step;
            // BRAKE 段：匀减速 applied = start + brakeRate·t − ½·sign·maxAcceleration·t²，
            // 时长 duration=|brakeRate|/maxAcceleration；速率归零后立即 beginMove() 接管。
            if(phase==BRAKE){
                float sign=brakeRate>0 ? 1.0f : -1.0f;
                applied=start+brakeRate*elapsed-.5f*sign*maxAcceleration*elapsed*elapsed;
                rate=brakeRate-sign*maxAcceleration*elapsed;
                if(elapsed>=duration){rate=0;beginMove();}
            }else{
                // MOVE 段：三次 smoothstep applied = start + delta·u²(3−2u)，u=elapsed/duration ∈[0,1]；
                // rate = delta·6u(1−u)/duration。两端 u=0/1 时 rate=0，所以角度与速率都连续。
                float u=ManualTurnControl::clamp(elapsed/duration,0,1);
                applied=start+delta*u*u*(3-2*u);
                rate=delta*6*u*(1-u)/duration;
                if(elapsed>=duration){applied=planned;rate=0;phase=IDLE;}
            }
        }
        // 结束位置硬钳 ±maxAngle（deg）：数值安全网，正常轨迹不会触发。
        applied=ManualTurnControl::clamp(applied,-maxAngle,maxAngle);
        // 释放斜坡：busy 时 resumeTime 归 0；空闲时 1/s 爬升到 1 → normalAuthority() 做 smoothstep。
        resumeTime=busy(now) ? 0 : std::fmin(1.0f,resumeTime+dt);
    }
};

// （注：下面两行 extern 声明内容完全相同，属重构时合并留下的重复声明；C++ 允许重复声明，保留原样。）
// 手动俯仰实例（定义在 remote/remote_control.cpp）
// manualForward：^ 手动俯仰通路唯一实例（定义在 remote/remote_control.cpp）。
// 含义：planned/applied 为倾角（deg，±2°）；写：panel/cmd.cpp 命令 + remote::tick() 每 400Hz 推轨迹；
// 读：motion_drive（planned 当速度档位）、remote::isActive()/normalAuthority()、遥测（1Hz D 帧）。
extern ManualPitchControl manualForward;

// 手动俯仰实例（定义在 remote/remote_control.cpp）
extern ManualPitchControl manualForward;

namespace remote {

// remote::parseManualMotion — 作用：解析一条 ^ 命令的载荷 "turnHz,pitchDeg"（不是倾角本身）。
// 参数 value：逗号分隔字符串；turn/forward：输出引用（单位：STEP Hz 差模 / deg 档位）。
// 返回：bool（true=格式合法且两个数都有限；false=拒绝，调用方不得改动输出）。
// 调用：panel/cmd.cpp 的 ^ 与 @ 分支（人工命令，低频）。边界：见 .cpp 的逐行说明。
// 解析整帧再改两个轴：^ turnHz,pitchDegrees
bool parseManualMotion(const char * value, float & turn, float & forward);

}  // namespace remote
