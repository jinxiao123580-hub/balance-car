"""Actual fixed-angle trajectory, speed bypass and target composition regression."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1];s=(root/"src/main.cpp").read_text(encoding="utf-8-sig")
a=s.index("    if (manualForward.busy(millis())) {\n        // Direct pitch mode");b=s.index("    bool positionHoldEnabled = positionKp",a);bypass=s[a:b]
a=s.index("    float pitchOffset = manualForward.busy(millis())");b=s.index(";",s.index("    targetBalanceAngle = balanceAngleOffset +",a))+1;compose=s[a:b]
mock=r'''
#include "manual_motion.h"
#include <cassert>
#include <cstdio>
#include <initializer_list>
ManualPitchControl manualForward;
uint32_t now=0;uint32_t millis(){return now;}
float positionSpeedTarget=1,targetForwardSpeed=1,speedError=100,speedIntegral=20;
float speedErrorPrev=1,speedErrorDotFilt=2,speedPContribution=4,speedIContribution=5;
float speedAngleCorrection=.4f,speedAngleUnclamped=1,speedAngleLimited=1;
float balanceAngleOffset=4,outerLoopAuthority=1,targetBalanceAngle=4.4f;
void speedBranch(){BYPASS}
void target(){COMPOSE}
void step(ManualPitchControl& v,uint32_t t){
    float p=v.applied,r=v.rate;v.update(t,.0025f,0);
    assert(std::fabs(v.applied)<=2.00001f && std::fabs(v.rate)<=2.00001f);
    assert(std::fabs(v.applied-p)<=.00501f && std::fabs(v.rate-r)<=.01002f);
}
int main(){
    for(float sign:{1.f,-1.f}){
        ManualPitchControl v;
        for(unsigned n=0;n<1000;n++){if(n%40==0)v.command(2*sign,n*2.5f);step(v,n*2.5f);}
        assert(v.applied==2*sign && v.rate==0 && v.busy(2500));
        v.command(0,2500);
        for(unsigned n=0;n<1600;n++)step(v,2500+n*2.5f);
        assert(v.applied==0 && v.rate==0 && !v.busy(6500) && v.normalAuthority()==1);
    }
    // Releasing while the requested lean is still building preserves angle/rate.
    for(unsigned release:{80u,300u,500u,650u}){
        ManualPitchControl v;
        for(unsigned n=0;n<release;n++){if(n%40==0)v.command(2,n*2.5f);step(v,n*2.5f);}
        float before=v.applied;v.command(0,release*2.5f);assert(v.applied==before);
        for(unsigned n=release;n<release+1600;n++)step(v,n*2.5f);
        assert(v.applied==0 && v.rate==0);
    }
    ManualPitchControl random;uint32_t rng=1;
    for(unsigned n=0;n<12000;n++){
        if(n%47==0){rng=rng*1664525u+1013904223u;random.command((int(rng%3)-1)*2,n*2.5f);}
        if(n%40==0)random.command(random.requested,n*2.5f);
        step(random,n*2.5f);
    }
    ManualPitchControl reverse;
    for(unsigned n=0;n<1000;n++){if(n%40==0)reverse.command(2,n*2.5f);step(reverse,n*2.5f);}
    for(unsigned n=0;n<1400;n++){if(n%40==0)reverse.command(-2,2500+n*2.5f);step(reverse,2500+n*2.5f);}
    assert(reverse.applied==-2 && reverse.rate==0);
    reverse.update(7000,.0025f,0);assert(reverse.requested==0); // link timeout returns smoothly
    reverse.update(7010,.0025f,10);assert(reverse.needsZero && reverse.applied==0);
    reverse.command(2,7020);assert(reverse.requested==0);
    reverse.command(0,7030);reverse.command(2,7040);assert(reverse.requested==2);
    reverse.update(7041,.0025f,NAN);assert(reverse.needsZero && reverse.applied==0);
    ManualPitchControl wrap;wrap.command(2,UINT32_MAX-100);wrap.update(20,.0025f,0);assert(wrap.requested==2);
    wrap.update(300,.0025f,0);assert(wrap.requested==0);
    // Actual firmware selection: enter from the current normal-loop target without a target step.
    manualForward.seed(.4f);manualForward.command(2,0);now=0;target();assert(std::fabs(targetBalanceAngle-4.4f)<.00001f);
    for(unsigned n=0;n<1000;n++){now=n*2.5f;if(n%40==0)manualForward.command(2,now);manualForward.update(now,.0025f,0);}
    speedError=100;speedBranch();assert(speedAngleCorrection==0 && speedIntegral==0 && targetForwardSpeed==0);
    target();assert(targetBalanceAngle==6); // O + 2, not canceled by a speed error
    for(unsigned n=0;n<1400;n++){now=2500+n*2.5f;if(n%40==0)manualForward.command(-2,now);manualForward.update(now,.0025f,0);}
    target();assert(targetBalanceAngle==2); // O - 2
    manualForward.command(0,6000);
    for(unsigned n=0;n<694;n++){now=6000+n*2.5f;manualForward.update(now,.0025f,0);}
    assert(manualForward.applied==0);speedAngleCorrection=.8f;target();assert(std::fabs(targetBalanceAngle-4)<.0001f);
    float oldAuthority=manualForward.normalAuthority();
    for(unsigned n=0;n<400;n++){
        now+=3;manualForward.update(now,.0025f,0);
        float a=manualForward.normalAuthority();assert(a>=oldAuthority && a-oldAuthority<.004f);oldAuthority=a;
    }
    target();assert(std::fabs(targetBalanceAngle-4.8f)<.0001f);
    puts("PASS: fixed +/-2, bounded angle/rate/acceleration, early release/reversal, timeout/fault, bumpless seed, real speed bypass and smooth normal-loop handoff");
}
'''
code=mock.replace("BYPASS",bypass).replace("COMPOSE",compose)
(root/".pio").mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix="fixed-angle-",dir=root/".pio") as tmp:
    cpp=Path(tmp)/"test.cpp";exe=Path(tmp)/"test.exe";cpp.write_text(code,encoding="utf-8")
    subprocess.run(["g++","-std=c++11","-O2","-I",str(root/"src"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
