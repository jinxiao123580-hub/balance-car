#pragma once
#include "manual_turn.h"
#include <cstdlib>
#include <cerrno>

struct PositionOdometry {
    int32_t last1=0,last2=0;
    float position=0;
    bool initialized=false;
    void reset(int32_t a,int32_t b){last1=a;last2=b;position=0;initialized=true;}
    float update(int32_t a,int32_t b,int mode){
        if(!initialized){reset(a,b);return position;}
        // Unsigned subtraction handles encoder counter rollover.
        float d1=(int32_t)((uint32_t)a-(uint32_t)last1);
        float d2=(int32_t)((uint32_t)b-(uint32_t)last2);
        last1=a;last2=b;
        position -= mode==0 ? .5f*(d1+d2) : (mode==1 ? d1 : (mode==2 ? d2 : 0));
        return position;
    }
};
// Shared by jog and parking: changes of control mode cannot step the speed request.
struct PositionSpeedReference {
    float applied=0;
    void reset(){applied=0;}
    float update(float target,float dt){
        if(!std::isfinite(target)||!std::isfinite(dt)||dt<=0)return applied;
        applied += ManualTurnControl::clamp(target-applied,-3.0f*dt,3.0f*dt);
        return applied;
    }
};
// Independent position-P angle; speed feedback keeps the existing damping gain.
struct PositionLeanControl {
    float kp=.005f,limit=.30f,requested=0,applied=0;
    static bool validConfig(float p,float cap){
        return std::isfinite(p)&&std::isfinite(cap)&&p>=0&&p<=.02f&&cap>=.05f&&cap<=.50f;
    }
    bool busy()const{return std::fabs(applied)>.00001f;}
    void reset(){requested=applied=0;}
    void update(float error,bool moving,float dt,float direction){
        if(!std::isfinite(error)||!std::isfinite(dt)||dt<=0||!std::isfinite(direction)){reset();return;}
        requested=moving ? ManualTurnControl::clamp(direction*kp*error,-limit,limit) : 0;
        bool building=moving && requested*applied>=0 && std::fabs(requested)>std::fabs(applied);
        float rate=building ? .10f : .30f;
        applied += ManualTurnControl::clamp(requested-applied,-rate*dt,rate*dt);
        if(!moving && std::fabs(applied)<.00001f)applied=0;
    }
};
// Arrival statistics do not filter the position/speed signals used for control.
struct JogArrivalWindow {
    float elapsed=0,sumPosition=0,sumSpeed=0,first=0,low=0,high=0,center=0,error=0;
    int goodWindows=0;
    void reset(float position=0){elapsed=sumPosition=sumSpeed=0;first=low=high=center=position;error=0;goodWindows=0;}
    bool update(float goal,float reference,float referenceSpeed,float position,float speed,float dt){
        if(std::fabs(goal-reference)>2 || std::fabs(referenceSpeed)>.2f){reset(position);return false;}
        if(elapsed==0){first=low=high=position;}
        low=std::fmin(low,position);high=std::fmax(high,position);
        elapsed+=dt;sumPosition+=position*dt;sumSpeed+=speed*dt;
        if(elapsed<.9999f)return false;
        center=sumPosition/elapsed;error=goal-center;
        bool good=std::fabs(error)<=5 && std::fabs(sumSpeed/elapsed)<=1 &&
            std::fabs(position-first)<=10 && high-low<=60;
        goodWindows=good ? goodWindows+1 : 0;
        elapsed=sumPosition=sumSpeed=0;
        return goodWindows>=2;
    }
};
struct PositionJog {
    enum Reason {NONE=0,RUNNING=1,ARRIVED=2,STOP=3,LINK_LOST=4,SOURCE_CHANGED=5,FAULT=6,TIMEOUT=7};
    bool active=false,seen=false;
    uint32_t sequence=0,started=0,lastHeartbeat=0;
    int source=0,reason=NONE;
    float goal=0,reference=0,referenceSpeed=0,speed=0,settled=0;
    JogArrivalWindow arrival;
    bool newer(uint32_t id)const{return !seen || (int32_t)(id-sequence)>0;}
    void cancel(int why){active=false;referenceSpeed=speed=settled=0;reason=why;}
    bool start(uint32_t id,float delta,float position,int mode,uint32_t now){
        if(!newer(id))return false;
        seen=true;sequence=id;
        if(delta==0){cancel(STOP);return true;}
        // Retarget from the existing trajectory, without resetting its position or velocity.
        if(!active){reference=position;referenceSpeed=0;speed=0;}
        goal=position+ManualTurnControl::clamp(delta,-200,200);
        source=mode;started=lastHeartbeat=now;
        settled=0;arrival.reset(position);active=true;reason=RUNNING;
        return true;
    }
    void heartbeat(uint32_t id,uint32_t now){if(active && id==sequence)lastHeartbeat=now;}
    void update(float position,float measuredSpeed,int mode,uint32_t now,float dt){
        if(!active)return;
        if(!std::isfinite(position)||!std::isfinite(measuredSpeed)||!std::isfinite(dt)||dt<=0){
            cancel(FAULT);return;
        }
        if(mode!=source){cancel(SOURCE_CHANGED);return;}
        if(uint32_t(now-lastHeartbeat)>500){cancel(LINK_LOST);return;}
        if(uint32_t(now-started)>12000){cancel(TIMEOUT);return;}
        float remaining=goal-reference;
        float direction=remaining>=0 ? 1.0f : -1.0f;
        // count/50ms -> count/s uses factor 20. Slow the reference if feedback lags.
        float leadRoom=std::fmax(0.0f,50.0f-direction*(reference-position));
        float leadCap=std::fmin(.3f*leadRoom,std::sqrt(.3f*leadRoom));
        float wanted=direction*std::fmin(3.0f,std::fmin(.08f*std::fabs(remaining),leadCap));
        float oldSpeed=referenceSpeed;
        referenceSpeed += ManualTurnControl::clamp(wanted-referenceSpeed,-3.0f*dt,3.0f*dt);
        reference += .5f*(oldSpeed+referenceSpeed)*20.0f*dt;
        speed=0; // MOVE=4/JOGANGLE=1 uses zero-speed damping, not a velocity command.
        if(arrival.update(goal,reference,referenceSpeed,position,measuredSpeed,dt))cancel(ARRIVED);
    }
};
inline bool parseJogId(const char* text,uint32_t& id,const char*& rest){
    if(*text<'0'||*text>'9')return false;
    errno=0;char* end=nullptr;unsigned long value=std::strtoul(text,&end,10);
    if(end==text || errno==ERANGE || value>UINT32_MAX)return false;
    id=(uint32_t)value;rest=end;return true;
}
inline bool parseJogFrame(const char* text,uint32_t& id,float& delta){
    const char* rest=nullptr;if(!parseJogId(text,id,rest)||*rest!=',')return false;
    const char* value=rest+1;char* end=nullptr;float v=std::strtof(value,&end);
    if(end==value||*end!='\0'||!std::isfinite(v)||std::fabs(v)>200||(v!=0 && std::fabs(v)<5))return false;
    delta=v;return true;
}

inline bool parseJogConfig(const char* text,float& kp,float& limit){
    char* end=nullptr;float p=std::strtof(text,&end);
    if(end==text||*end!=',')return false;
    const char* capText=end+1;float cap=std::strtof(capText,&end);
    if(end==capText||*end!='\0'||!PositionLeanControl::validConfig(p,cap))return false;
    kp=p;limit=cap;return true;
}
