#pragma once
#include "manual_turn.h"
#include <cstdlib>

// Fixed relative pitch with continuous angle/rate transitions, including release mid-ramp.
struct ManualPitchControl {
    enum Phase {IDLE=0, MOVE=1, BRAKE=2};
    static constexpr float maxAngle=2.0f;
    static constexpr float maxRate=2.0f;       // deg/s
    static constexpr float maxAcceleration=4.0f; // deg/s^2
    float requested=0,applied=0,rate=0;
    uint32_t receivedMs=0;
    bool needsZero=false;
    int phase=IDLE;
    float planned=0,elapsed=0,duration=0,start=0,delta=0,brakeRate=0,resumeTime=1;
    void clear(){requested=applied=rate=planned=0;phase=IDLE;elapsed=duration=0;resumeTime=0;needsZero=true;}
    void seed(float offset){
        applied=ManualTurnControl::clamp(offset,-maxAngle,maxAngle);rate=0;
        planned=applied;phase=IDLE;elapsed=duration=0;resumeTime=0;
    }
    void command(float angle,uint32_t now){
        if(angle==0)needsZero=false;
        if(needsZero)return;
        requested=std::isfinite(angle) ? ManualTurnControl::clamp(angle,-maxAngle,maxAngle) : 0;
        receivedMs=now;
    }
    bool busy(uint32_t now)const{
        return (requested!=0 && uint32_t(now-receivedMs)<=350) || phase!=IDLE || std::fabs(applied)>.000001f;
    }
    float normalAuthority()const{
        float u=ManualTurnControl::clamp(resumeTime,0,1);
        return u*u*(3-2*u);
    }
    void beginMove(){
        start=applied;delta=planned-start;elapsed=0;
        if(std::fabs(delta)<.000001f){applied=planned;rate=0;phase=IDLE;return;}
        // Cubic smoothstep starts/ends with zero rate. These bounds hold analytically.
        duration=std::fmax(1.5f*std::fabs(delta)/maxRate,std::sqrt(6*std::fabs(delta)/maxAcceleration));
        phase=MOVE;
    }
    void update(uint32_t now,float dt,float angleError){
        if(uint32_t(now-receivedMs)>350)requested=0;
        if(!std::isfinite(angleError)||!std::isfinite(dt)||dt<=0||std::fabs(angleError)>=10){clear();return;}
        if(requested!=planned){
            planned=requested;
            if(phase!=BRAKE){
                if(std::fabs(rate)>.000001f){
                    start=applied;brakeRate=rate;elapsed=0;duration=std::fabs(rate)/maxAcceleration;phase=BRAKE;
                }else beginMove();
            }
        }
        float left=dt;
        for(int transition=0;transition<3 && left>0 && phase!=IDLE;transition++){
            float step=std::fmin(left,duration-elapsed);elapsed+=step;left-=step;
            if(phase==BRAKE){
                float sign=brakeRate>0 ? 1.0f : -1.0f;
                applied=start+brakeRate*elapsed-.5f*sign*maxAcceleration*elapsed*elapsed;
                rate=brakeRate-sign*maxAcceleration*elapsed;
                if(elapsed>=duration){rate=0;beginMove();}
            }else{
                float u=ManualTurnControl::clamp(elapsed/duration,0,1);
                applied=start+delta*u*u*(3-2*u);
                rate=delta*6*u*(1-u)/duration;
                if(elapsed>=duration){applied=planned;rate=0;phase=IDLE;}
            }
        }
        applied=ManualTurnControl::clamp(applied,-maxAngle,maxAngle);
        resumeTime=busy(now) ? 0 : std::fmin(1.0f,resumeTime+dt);
    }
};

// Parse the entire frame before changing either axis: ^ turnHz,pitchDegrees.
inline bool parseManualMotion(const char* value, float& turn, float& forward) {
    char* end=nullptr;
    float t=std::strtof(value,&end);
    if (end==value || *end!=',' || !std::isfinite(t)) return false;
    const char* second=end+1;
    float f=std::strtof(second,&end);
    if (end==second || *end!='\0' || !std::isfinite(f)) return false;
    turn=t; forward=f;
    return true;
}
