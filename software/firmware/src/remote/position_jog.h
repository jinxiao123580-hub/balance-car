#pragma once
// ============================================================
// remote/position_jog.{h,cpp} — 位置点动 / 位置保持 / 位置 P 外环
// 结构体原样搬自 src/position_jog.h；调度与保持函数搬自 main.cpp。
// ============================================================
#include <Arduino.h>
#include <cmath>
#include <cstdlib>
#include <cerrno>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"
#include "remote/manual_turn.h"


// ============================================================
// PositionOdometry — 位置里程：把编码器累计计数换算成"车体位移坐标"（平均编码器 count）。
// 单位：position 为编码器 count；外部换算 10.8 counts = 1 mm（相机对表），故 mm = count/10.8。
//   1 count/50ms 速度 = 20 counts/s ≈ 1.85 mm/s。
// 调用：positionHold()/captureCurrentPositionTarget()（400Hz 与事件级）读编码器后调用 update()。
// 为什么不用位置 = count − zero：编码器可能丢脉冲/只信单路，这里按"可信通道"选测速方案并做回绕处理。
// ============================================================
struct PositionOdometry {
    // last1/last2：上一次读到的两路编码器累计计数（count，int32，可回绕）。每次 update() 末更新。
    int32_t last1=0,last2=0;
    // position：累计位移（count，带符号；符号由采样模式与电机方向共同决定，向前为正）。
    // 读者：positionHold/positionJogTick/遥测（D 帧 ×100 上报）。更新：每次 update()。
    float position=0;
    // initialized：是否已建立基准。首次 update() 只 reset 并返回 0，避免把开机前计数差算成位移。
    bool initialized=false;
    // reset — 作用：用给定两路计数重建基准并把 position 归零。
    // 参数 a/b：encoder1/2 当前计数（count）。返回：无。调用：首次 update()、故障重建（事件级）。
    void reset(int32_t a,int32_t b){last1=a;last2=b;position=0;initialized=true;}
    // update — 作用：读两路计数，算增量并累计位置。
    // 参数 a/b：encoder1/2 累计计数（count）；mode：可信通道 0=两路平均、1=仅1路、2=仅2路、3=都不信（保持不动）。
    // 返回：position（count）。调用：positionHold() 每 400Hz、captureCurrentPositionTarget() 事件级。
    // 边界：未初始化首调用只建基准返回 0；计数回绕由无符号差自动处理；mode=3 时位置冻结。
    float update(int32_t a,int32_t b,int mode){
        if(!initialized){reset(a,b);return position;}
        // 无符号回绕写法：编码器计数是 int32 环，直接相减在回绕瞬间会得到巨大跳变；
        // (uint32_t)差再转回 int32 得到"真实增量"（±2^31 内正确）。
        // Unsigned subtraction handles encoder counter rollover.
        float d1=(int32_t)((uint32_t)a-(uint32_t)last1);
        float d2=(int32_t)((uint32_t)b-(uint32_t)last2);
        last1=a;last2=b;
        // 位置累加方向：position -= 增量，即编码器计数增加的方向对应车体后退（由电机/编码器装配方向决定）。
        // mode 0：两路平均（都可信，降噪）；1/2：只用可信那一路（另一路接线故障）；3：都不信→增量为 0（位置冻结）。
        // 注意：单路模式不补偿两轮差速，转弯时会有少量位置偏差——这是可靠优先的取舍。
        position -= mode==0 ? .5f*(d1+d2) : (mode==1 ? d1 : (mode==2 ? d2 : 0));
        return position;
    }
};
// PositionSpeedReference — 速度参考的斜率限制器：jog/驻车/速度环交接时速度请求不能阶跃。
// 单位：target/applied 均为 count/50ms；限斜率 3.0 (count/50ms)/s。
// 调用：remote::tick100() 每 100Hz（位置 P 外环之后）。
// Shared by jog and parking: changes of control mode cannot step the speed request.
struct PositionSpeedReference {
    // applied：当前实际下发的速度参考（count/50ms，±? 由上游位置环限幅决定，斜率另限 3.0/s）。
    // 写：update() 每 100Hz；读：captureCurrentPositionTarget()/tick100()/遥测。
    float applied=0;
    // reset — 作用：清速度参考。调用：停车/进入手动跟随/故障（事件级）。
    void reset(){applied=0;}
    // update — 作用：限斜率逼近 target，返回 applied。
    // 参数 target：目标速度（count/50ms）；dt：周期 s（100Hz→0.01）。返回：applied（count/50ms）。
    // 调用：remote::tick100() 每 100Hz。边界：非有限/dt≤0 → 原值返回；每拍最多变 3.0×dt。
    float update(float target,float dt){
        if(!std::isfinite(target)||!std::isfinite(dt)||dt<=0)return applied;
        applied += ManualTurnControl::clamp(target-applied,-3.0f*dt,3.0f*dt);
        return applied;
    }
};
// PositionLeanControl — 位置点动专用的"位置误差→倾角"P 环（独立于速度环的 jog 倾角）。
// 单位：error 为 count，kp 为 deg per count，输出 applied/limit 为 deg。
// 公式：requested = direction×kp×error（钳 ±limit），再按斜率逼近 applied（建立段 0.10 deg/s，释放段 0.30 deg/s）。
// 调用：remote::tick100() 每 100Hz（jog 期间），结果经 remote::jogLeanOffset() 进 control::outerLoop/权限。
// 默认：kp=0.005（deg/count）、limit=0.30（deg）；面板 X/Y 是位置环参数，jog 的 kp/limit 用 `! kp,limit` 配。
// ============================================================
// Independent position-P angle; speed feedback keeps the existing damping gain.
struct PositionLeanControl {
    // kp：位置误差→倾角增益（deg per count），默认 0.005，合法范围 [0, 0.02]（validConfig）；
    //   例：位置误差 10 count（≈0.93mm）→ 0.05° 倾角。limit：jog 倾角上限（deg），默认 0.30，范围 [0.05, 0.50]。
    // requested：本拍期望倾角（deg，±limit）；applied：实际输出（deg，按斜率逼近）。
    // 更新频率：100Hz；写：本类 update()；读：telemetry D 帧、remote::jogLeanOffset()。
    float kp=.005f,limit=.30f,requested=0,applied=0;
    // validConfig — 作用：校验 `! kp,limit` 命令配置是否在安全范围。
    // 参数 p：kp（deg/count，[0,0.02]）；cap：limit（deg，[0.05,0.50]）。返回：bool。
    // 边界：p=0 合法（等于关闭 jog 倾角，只靠速度环）；cap 下限 0.05 防止小到推不动。
    static bool validConfig(float p,float cap){
        return std::isfinite(p)&&std::isfinite(cap)&&p>=0&&p<=.02f&&cap>=.05f&&cap<=.50f;
    }
    // busy — 作用：是否还有残余 jog 倾角（释放段仍算占用）。返回 bool；调用：tick100() 判定 jogAngleControl。
    bool busy()const{return std::fabs(applied)>.00001f;}
    // reset — 作用：清零 jog 倾角。调用：remote::resetLean()（速度环关闭）、resetFault()。
    void reset(){requested=applied=0;}
    // update — 作用：位置误差→jog 倾角，含符号、限幅与建立/释放斜率。
    // 参数：error 位置误差（count）；moving 是否在点动；dt s（100Hz→0.01）；direction 方向符号（±1，来自 speedLoopDirection×speedKp）。
    // 返回：无（写 applied）。调用：remote::tick100() 每 100Hz。
    // 边界：非有限或 dt≤0 → reset()；不 moving 时 requested=0 并按 0.30deg/s 释放到 0。
    void update(float error,bool moving,float dt,float direction){
        if(!std::isfinite(error)||!std::isfinite(dt)||dt<=0||!std::isfinite(direction)){reset();return;}
        // 方向：direction 把"位置误差方向"映射成"该往哪边倾"，与速度环极性保持一致（否则点动会反向跑）。
        // 误差乘 kp 得倾角（deg），钳到 ±limit（默认 0.30°）。
        requested=moving ? ManualTurnControl::clamp(direction*kp*error,-limit,limit) : 0;
        // 建立段（同向且绝对值在增大）用慢斜率 0.10 deg/s（起步柔和、防过冲）；其余情况用 0.30 deg/s（可快速回中）。
        bool building=moving && requested*applied>=0 && std::fabs(requested)>std::fabs(applied);
        float rate=building ? .10f : .30f;
        // 斜率限制：每拍最多 rate×dt，避免倾角阶跃；不 moving 时最终会平滑到 0。
        applied += ManualTurnControl::clamp(requested-applied,-rate*dt,rate*dt);
        if(!moving && std::fabs(applied)<.00001f)applied=0;
    }
};
// JogArrivalWindow — 点动到位判定窗口（1 s 统计窗，连续 2 个好窗确认）。
// 只做"到位统计"，不用它的滤波值参与控制（位置/速度仍用实时量）。
// 单位：goal/reference/position/first/low/high/center/error 为 count；referenceSpeed 为 count/50ms；speed 为 count/50ms。
// 调用：PositionJog::update() 每 100Hz 喂数据；positionJogTick() 读 center 作为新驻车目标。
// ============================================================
// Arrival statistics do not filter the position/speed signals used for control.
struct JogArrivalWindow {
    // elapsed：当前统计窗已累计时间（s，满 1s 结算）；sumPosition/sumSpeed：窗内积分（用于算均值）；
    // first：窗起点位置（count）；low/high：窗内位置极值（count）；center：窗平均位置（count）；
    // error = goal − center（count）；goodWindows：连续合格窗计数（≥2 判定到位）。
    float elapsed=0,sumPosition=0,sumSpeed=0,first=0,low=0,high=0,center=0,error=0;
    int goodWindows=0;
    // reset — 作用：清窗口统计，用给定位置初始化极值。参数 position：当前位置（count）。
    // 调用：start() 起点、参考还在动时中止窗口、到达后不再统计。
    void reset(float position=0){elapsed=sumPosition=sumSpeed=0;first=low=high=center=position;error=0;goodWindows=0;}
    // update — 作用：累计 1 s 窗口，判定"是否已稳定停在目标附近"。
    // 参数：goal 目标位置（count）；reference 参考位置（count）；referenceSpeed 参考速度（count/50ms）；
    //   position 实测位置（count）；speed 实测速度（count/50ms）；dt s。返回：true=连续 2 个合格窗（到位）。
    // 调用：PositionJog::update() 每 100Hz。边界：参考还在移动（|goal−ref|>2 或 |refSpeed|>0.2）→ 清窗重来。
    bool update(float goal,float reference,float referenceSpeed,float position,float speed,float dt){
        // 参考还没停稳就不计窗：参考本身在动时统计没有意义（判到位的前提是参考已到目标附近）。
        if(std::fabs(goal-reference)>2 || std::fabs(referenceSpeed)>.2f){reset(position);return false;}
        if(elapsed==0){first=low=high=position;}
        low=std::fmin(low,position);high=std::fmax(high,position);
        elapsed+=dt;sumPosition+=position*dt;sumSpeed+=speed*dt;
        if(elapsed<.9999f)return false;
        center=sumPosition/elapsed;error=goal-center;
        // 合格窗判据（单位：count / count/50ms）：
        //   |error|≤5 count（≈0.46mm）、窗均速 |v|≤1 count/50ms、窗内漂移 |pos−first|≤10 count、峰峰差 ≤60 count；
        //   连续 2 个窗合格才返回 true（≈2s 稳定驻留，压掉"路过目标点时恰好平均为零"的假到位）。
        bool good=std::fabs(error)<=5 && std::fabs(sumSpeed/elapsed)<=1 &&
            std::fabs(position-first)<=10 && high-low<=60;
        goodWindows=good ? goodWindows+1 : 0;
        elapsed=sumPosition=sumSpeed=0;
        return goodWindows>=2;
    }
};
// PositionJog — 位置点动状态机（~ / ^ 命令背后的位置闭环）。
// 流程：panel 收到 ~id,delta → start() 生成参考轨迹（goal=当前位置+delta，clamp ±200 count）；
//   100Hz update() 用"参考位置 + 参考速度"做点到点：接近目标自动减速，同时检查心跳/超时/编码器通道；
//   positionJogTick() 把 reference 作为 targetVehiclePosition 交给位置 P 外环，倾角由 jog/速度环执行；
//   到位（JogArrivalWindow 连续 2 窗合格）或异常 → cancel(reason)，由 captureCurrentPositionTarget() 收尾。
// 单位：位置/增量 count（10.8 counts = 1 mm，故 ±200 count ≈ ±18.5 mm）；速度 count/50ms。
// 频率：start/heartbeat 事件级（命令）；update 100Hz；状态被 tick400 位置保持每拍读取。
// 安全：心跳 500ms 丢失→LINK_LOST；总时长 12s→TIMEOUT；倾角>5° 或速度环关闭→FAULT；编码器通道变化→SOURCE_CHANGED。
// ============================================================
struct PositionJog {
    // Reason：结束原因。NONE=未结束；RUNNING=进行中；ARRIVED=到位；STOP=人为停止（新命令/转向/位置环接管）；
    //   LINK_LOST=500ms 无心跳；SOURCE_CHANGED=可信编码器通道变了；FAULT=倾角过大/速度环关闭/非有限值；
    //   TIMEOUT=总时长超 12s。写：start/cancel/update；读：panel `~` 回显、遥测 D 帧。
    enum Reason {NONE=0,RUNNING=1,ARRIVED=2,STOP=3,LINK_LOST=4,SOURCE_CHANGED=5,FAULT=6,TIMEOUT=7};
    // active：是否正在点动（状态机主标志）；seen：是否收到过命令（用 sequence 判新旧）。
    bool active=false,seen=false;
    // sequence：命令序号（uint32，单调递增，回绕安全）；started：本次点动开始时刻（ms）；
    // lastHeartbeat：最后心跳时刻（ms），`&id` 命令刷新。
    uint32_t sequence=0,started=0,lastHeartbeat=0;
    // source：启动时的编码器可信模式（0/1/2/3，见 PositionOdometry::update）；运行中模式变了立刻取消（避免坐标跳变）。
    // reason：见 Reason。
    int source=0,reason=NONE;
    // goal：目标位置（count，绝对坐标）；reference：参考位置（count，轨迹当前位置）；referenceSpeed：参考速度（count/50ms）；
    // speed：本功能恒 0（MOVE 用零速阻尼，不作速度指令）；settled：到位后驻留计时（count/ms 量纲，未使用于判据）。
    float goal=0,reference=0,referenceSpeed=0,speed=0,settled=0;
    // arrival：到位统计窗口（1s 窗×2），到位判定只看它。
    JogArrivalWindow arrival;
    // newer — 作用：判断来命令 id 是否比当前新（防止乱序/重发旧命令）。
    // 参数 id：命令序号。返回 bool（true=新）。边界：用 (int32_t)(id-sequence)>0 兼容回绕；首条命令直接接受。
    bool newer(uint32_t id)const{return !seen || (int32_t)(id-sequence)>0;}
    // cancel — 作用：结束点动并记录原因。参数 why：Reason。返回：无。
    // 边界：只清运动状态（referenceSpeed/speed/settled=0），不动位置坐标/目标——收尾由调用方 capture。
    void cancel(int why){active=false;referenceSpeed=speed=settled=0;reason=why;}
    // start — 作用：接受一条新的点动命令并生成参考轨迹。
    // 参数：id 命令序号；delta 位移增量（count，钳 ±200 ≈±18.5mm；0=停车命令）；position 当前位置（count）；
    //   mode 编码器可信模式；now millis()。返回：bool（true=已接受）。
    // 调用：panel/cmd.cpp 收到 ~id,delta 时（命令级）。边界：旧 id 拒绝；delta=0 立即 cancel(STOP) 并返回 true。
    bool start(uint32_t id,float delta,float position,int mode,uint32_t now){
        if(!newer(id))return false;
        seen=true;sequence=id;
        if(delta==0){cancel(STOP);return true;}
        // 改目标时不重置轨迹：若已在点动中（retarget），沿用当前 reference/referenceSpeed，从当前位置平滑改道，
        // 不把参考位置拽回实测位置（否则会突然减速/反向）。
        // Retarget from the existing trajectory, without resetting its position or velocity.
        if(!active){reference=position;referenceSpeed=0;speed=0;}
        // goal = 当前位置 + 钳位后的增量（count）；随后记录通道/时刻、清到位窗口并置 RUNNING。
        goal=position+ManualTurnControl::clamp(delta,-200,200);
        source=mode;started=lastHeartbeat=now;
        settled=0;arrival.reset(position);active=true;reason=RUNNING;
        return true;
    }
    // heartbeat — 作用：刷新心跳（`&id` 命令，上位机周期性发送以证明在线）。
    // 参数 id/now。边界：只接受与当前 sequence 相同的 id（旧心跳无效）。调用：panel/cmd.cpp，命令级（通常 ~2Hz）。
    void heartbeat(uint32_t id,uint32_t now){if(active && id==sequence)lastHeartbeat=now;}
    // update — 作用：点动 100Hz 主循环：安全门 → 参考轨迹 → 到位判定。
    // 参数：position 实测位置（count）；measuredSpeed 融合速度（count/50ms）；mode 当前编码器可信模式；now millis()；dt s。
    // 返回：无（结果写 reference/referenceSpeed 与 reason；结束时 active=false）。
    // 调用：positionJogTick() 每 100Hz。边界：以下任一条件立即 cancel 并 return（见各行）。
    void update(float position,float measuredSpeed,int mode,uint32_t now,float dt){
        if(!active)return;
        if(!std::isfinite(position)||!std::isfinite(measuredSpeed)||!std::isfinite(dt)||dt<=0){
            cancel(FAULT);return;
        }
        // 编码器可信通道在运行中变化（一路掉线/恢复）→ 位置坐标可能跳变，点动立即取消（SOURCE_CHANGED）。
        if(mode!=source){cancel(SOURCE_CHANGED);return;}
        // 心跳丢失 >500ms → LINK_LOST（上位机死了/蓝牙断了：不能在无人监控下继续走）。
        if(uint32_t(now-lastHeartbeat)>500){cancel(LINK_LOST);return;}
        // 单次点动总时长 >12s → TIMEOUT（防卡死/防误判目标永远到不了）。
        if(uint32_t(now-started)>12000){cancel(TIMEOUT);return;}
        // remaining：剩余距离（count）；direction：该往哪个方向走（±1）。
        float remaining=goal-reference;
        float direction=remaining>=0 ? 1.0f : -1.0f;
        // count/50ms -> count/s uses factor 20. Slow the reference if feedback lags.
        // leadRoom："车还在参考后面多少"（count，钳到 ≥0）——用于接近目标时提前减速（借鉴前视刹车）。
        // 50 是最大前视余量（count ≈4.6mm）：参考领先实测 50 count 以上时不再额外限速。
        float leadRoom=std::fmax(0.0f,50.0f-direction*(reference-position));
        // leadCap：由 leadRoom 生成的速度上限（count/50ms）：min(0.3·room, sqrt(0.3·room))。
        // room 大时线性项主导，room 小时平方根项更保守——两种形状在 room≈0.3 count 处切换（实测整定）。
        float leadCap=std::fmin(.3f*leadRoom,std::sqrt(.3f*leadRoom));
        // wanted：期望参考速度 = direction × min(3.0, 0.08·|remaining|, leadCap)：
        //   0.08 count/50ms per count → 剩余 37.5 count 时到 3.0 上限；离目标越近越慢；leadCap 限制"追得太快"。
        float wanted=direction*std::fmin(3.0f,std::fmin(.08f*std::fabs(remaining),leadCap));
        // 梯形积分：先保存旧参考速度 oldSpeed，再给 referenceSpeed 加斜率限幅后的增量。
        float oldSpeed=referenceSpeed;
        referenceSpeed += ManualTurnControl::clamp(wanted-referenceSpeed,-3.0f*dt,3.0f*dt);
        // reference += ½(v_旧+v_新)×20×dt：速度 count/50ms ×20 = count/s，×dt(s) = count，½(v旧+v新) 为梯形平均。
        reference += .5f*(oldSpeed+referenceSpeed)*20.0f*dt;
        // speed 恒 0：本版点动只做"位置闭环 + 零速阻尼"，不走速度指令通路（MOVE=4/JOGANGLE=1 模式语义）。
        speed=0; // MOVE=4/JOGANGLE=1 uses zero-speed damping, not a velocity command.
        // 到位判定：由 1s 统计窗 ×2 决定；返回 true 时 cancel(ARRIVED)，reason 供上层收尾。
        if(arrival.update(goal,reference,referenceSpeed,position,measuredSpeed,dt))cancel(ARRIVED);
    }
};
// parseJogId — 作用：解析 ~/& 命令开头的无符号十进制 id。
// 参数 text：输入串；id/rest：输出 id 与剩余串指针。返回 bool。
// 边界：非数字开头/errno=ERANGE/超过 UINT32_MAX 都拒绝；id=0 合法（序号从 0 起也可能是首条）。
inline bool parseJogId(const char* text,uint32_t& id,const char*& rest){
    if(*text<'0'||*text>'9')return false;
    errno=0;char* end=nullptr;unsigned long value=std::strtoul(text,&end,10);
    if(end==text || errno==ERANGE || value>UINT32_MAX)return false;
    id=(uint32_t)value;rest=end;return true;
}
// parseJogFrame — 作用：解析 ~id,delta 整帧（id 与位移一次解析）。
// 参数 text：载荷；id/delta：输出（delta 单位 count）。返回 bool。
// 边界：delta 必须 |delta|≤200 且（delta==0 或 |delta|≥5）；"1,2,3"/空/非有限全部拒绝。
inline bool parseJogFrame(const char* text,uint32_t& id,float& delta){
    const char* rest=nullptr;if(!parseJogId(text,id,rest)||*rest!=',')return false;
    const char* value=rest+1;char* end=nullptr;float v=std::strtof(value,&end);
    // 范围规则：0 表示停车；|delta|<5 count（≈0.46mm）视为无意义抖动直接拒绝；上限 ±200 count ≈ ±18.5mm。
    if(end==value||*end!='\0'||!std::isfinite(v)||std::fabs(v)>200||(v!=0 && std::fabs(v)<5))return false;
    delta=v;return true;
}

// parseJogConfig — 作用：解析 `! kp,limit` 点动倾角配置。
// 参数 text：载荷；kp/limit：输出（deg/count、deg）。返回 bool；两个字段都合法才写回。
// 边界：范围由 PositionLeanControl::validConfig 校验（kp 0~0.02、limit 0.05~0.50）。
inline bool parseJogConfig(const char* text,float& kp,float& limit){
    char* end=nullptr;float p=std::strtof(text,&end);
    if(end==text||*end!=',')return false;
    const char* capText=end+1;float cap=std::strtof(capText,&end);
    if(end==capText||*end!='\0'||!PositionLeanControl::validConfig(p,cap))return false;
    kp=p;limit=cap;return true;
}

// 位置/点动实例：定义在 remote/remote_control.cpp，此处 extern 供本模块与 panel/telemetry 读。
// 位置/点动实例（定义在 remote/remote_control.cpp）
// positionOdom：位置里程实例（position 单位 count）。写：positionHold/capture（400Hz/事件级）。
extern PositionOdometry positionOdom;
// positionJog：点动状态机实例。写：panel 命令/100Hz update；读：positionHold、panel 遥测与参数写盘前置检查。
extern PositionJog positionJog;
// positionSpeedReference：速度参考斜率器实例。applied 单位 count/50ms。写/读：tick100() 每 100Hz。
extern PositionSpeedReference positionSpeedReference;
// positionLean：jog 倾角 P 环实例。applied 单位 deg（±0.30）。写：tick100()；读：jogLeanOffset()/遥测。
extern PositionLeanControl positionLean;
// positionHoldWasEnabled：驻车位置环上一拍的启用状态（X>0 视为启用），用于"刚启用时重新 capture 目标"的边沿检测。
// 写：positionHold100()/复位清零处；读：positionHold100()。更新：每 100Hz。
extern bool positionHoldWasEnabled;
// positionHoldSuspended：驻车环是否被悬挂（手动 ^ 动作或权限释放期间不累计位置误差，防回位拉扯）。
// 写：positionHold() 每 400Hz；读：positionHold100()/cmd 的 X 参数/params 写盘检查。
extern bool positionHoldSuspended;

// （注：下面第二组 extern 声明与上面 142–147 完全重复，属重构合并时的重复声明；C++ 允许，保留原样。）
// 位置/点动实例（定义在 remote/remote_control.cpp）
extern PositionOdometry positionOdom;
extern PositionJog positionJog;
extern PositionSpeedReference positionSpeedReference;
extern PositionLeanControl positionLean;
extern bool positionHoldWasEnabled;
extern bool positionHoldSuspended;

namespace remote {

// positionJogTick — 作用：100Hz 点动调度：推进状态机、把参考位置交给位置 P 外环。
// 参数 jogPositionControl：输出标志（true=本拍位置由点动参考驱动）。返回：无。
// 调用：remote::tick100() 每 100Hz，在运动驱动之前。边界：点动结束那一拍负责"收尾对齐目标"，见 .cpp。
// 100Hz 点动调度（原 updateOuterLoop 开头 positionJog 分支）
void positionJogTick(bool & jogPositionControl);

// positionHold100 — 作用：100Hz 位置 P 外环（非点动时）：位置误差 → 目标速度（count/50ms）。
// 参数 jogPositionControl：true 时整段跳过（点动接管）。返回：无。
// 调用：remote::tick100() 每 100Hz。边界：X=positionKp≤1e-6 视为关闭；死区由 M 扣除；输出限幅 Y。
// 100Hz 位置 P 外环（原 updateOuterLoop 的 !jogPositionControl 段）
void positionHold100(bool jogPositionControl);

// positionHold — 作用：400Hz 位置保持/里程维护（原 updatePositionHold）：读编码器 → 更新 vehiclePosition → 维护驻车目标。
// 参数：无。返回：无。调用：main loop 每 400Hz（远控 400Hz 段），必须在外环之前。
// 边界：点动中倾角>5° 或速度环关闭 → stopPositionJog(FAULT)；编码器通道变化 → SOURCE_CHANGED；
//   手动通路活动/权限释放期间 → 目标跟随实际位置（悬挂驻车环，防误差累积）。
// 400Hz 位置保持（原 updatePositionHold）
void positionHold();

// captureCurrentPositionTarget — 作用：把当前位置设为新的驻车目标（同时取消点动）。
// 参数：无。返回：无。调用：启用位置环（X 0→非0）、跌倒恢复、点动收尾、面板 C 命令（事件级）。
// 边界：只写目标/清误差，不改编码器计数与里程坐标（坐标连续）。
void captureCurrentPositionTarget();
// stopPositionJog — 作用：按原因停止点动并收尾（= cancel + capture）。
// 参数 reason：PositionJog::Reason。返回：无。调用：故障/转向/新命令/上层接管（事件级）。
void stopPositionJog(int reason);

}  // namespace remote
