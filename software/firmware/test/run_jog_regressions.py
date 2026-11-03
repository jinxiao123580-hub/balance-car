"""Host checks of the actual point-move controller and odometry; no physical plant claim."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
code=r'''
#include "position_jog.h"
#include <cassert>
#include <cstdio>
int main(){
    PositionOdometry od;od.reset(0,0);
    assert(od.update(-20,-20,0)==20);
    assert(od.update(-30,-1000,1)==30);
    assert(od.update(-40,-2000,1)==40);
    assert(od.update(-42,-2002,0)==42);
    assert(od.update(-50,-2010,3)==42);
    od.reset(INT32_MAX,0);assert(od.update(INT32_MIN,0,1)==-1);
    uint32_t id=0;float delta=0;
    assert(parseJogFrame("12,50",id,delta) && id==12 && delta==50);
    for(const char* bad:{"-1,50","4294967296,50","1,nan","1,201","1,1","1,5x","1,50,2"})
        assert(!parseJogFrame(bad,id,delta));
    PositionJog j;PositionSpeedReference request;
    assert(j.start(10,50,1000,1,0));
    assert(j.goal==1050 && j.reference==1000 && j.referenceSpeed==0);
    float position=1000;
    for(unsigned n=0;n<100;n++){
        float lastRef=j.reference,lastV=j.referenceSpeed,lastRequest=request.applied;
        j.heartbeat(10,n*10);j.update(position,0,1,n*10,.01f);
        assert(std::fabs(j.reference-lastRef)<=.6001f); // no position setpoint step
        assert(std::fabs(j.referenceSpeed-lastV)<=.03001f);
        assert(std::fabs(request.update(j.speed,.01f)-lastRequest)<=.03001f);
        assert(std::fabs(j.speed)<=4);
        position=j.reference; // ideal tracker only verifies trajectory, not a balancing plant
    }
    float ref=j.reference,vel=j.referenceSpeed;
    assert(!j.start(10,50,position,1,1000) && j.reference==ref);
    assert(j.start(11,-50,position,1,1000));
    assert(j.reference==ref && j.referenceSpeed==vel); // reverse/retarget without state reset
    j.update(position,0,1,1010,.01f);
    assert(std::fabs(j.reference-ref)<=.6001f && std::fabs(j.referenceSpeed-vel)<=.03001f);
    j.start(12,0,position,1,1020);
    assert(!j.active && j.reason==j.STOP && !j.start(11,50,position,1,1030));
    float before=request.applied;
    request.update(0,.01f);assert(std::fabs(request.applied-before)<=.03001f);
    // Stalled feedback must not let the reference run to a far endpoint.
    PositionJog stalled;stalled.start(1,200,0,0,0);
    for(unsigned n=0;n<700;n++){
        stalled.heartbeat(1,n*10);stalled.update(0,0,0,n*10,.01f);
    }
    assert(stalled.reference<51 && stalled.reference>0);
    // Ideal follower in either direction must reach a low-speed finish.
    for(float distance:{50.f,-50.f,200.f,-200.f}){
        PositionJog follow;follow.start(1,distance,0,0,0);float p=0,v=0;
        for(unsigned n=0;n<1200 && follow.active;n++){
            follow.heartbeat(1,n*10);follow.update(p,v,0,n*10,.01f);
            v=follow.referenceSpeed;p=follow.reference;
        }
        assert(!follow.active && follow.reason==follow.ARRIVED);
        assert(std::fabs(p-distance)<=5);
    }
    PositionLeanControl lean;
    for(unsigned n=0;n<300;n++){
        float old=lean.applied;lean.update(50,true,.01f,1);
        assert(std::fabs(lean.applied-old)<=.00101f);
    }
    assert(std::fabs(lean.applied-.25f)<.0001f); // independent of weak velocity K
    lean.update(1000,true,.01f,1);assert(lean.requested==.30f && lean.applied<=.30f);
    float beforeLean=lean.applied;lean.update(-50,true,.01f,1);
    assert(lean.applied>0 && std::fabs(lean.applied-beforeLean)<=.00301f);
    for(unsigned n=0;n<200;n++)lean.update(0,false,.01f,1);
    assert(!lean.busy() && lean.applied==0);
    float pGain=9,cap=9;assert(parseJogConfig("0.005,0.30",pGain,cap));
    assert(std::fabs(pGain-.005f)<.000001f && cap==.30f);
    for(const char* bad:{"-1,0.3","0.021,0.3","0.01,0.51","0.01,0.01","nan,0.3","0.01,inf","1,","0.01,0.3,2"})
        assert(!parseJogConfig(bad,pGain,cap));
    // Position-center arrival tolerates normal balancing sway; no control signal is filtered.
    JogArrivalWindow window;bool arrived=false;
    for(unsigned n=0;n<220 && !arrived;n++){
        float phase=2*3.14159265f*2*n*.01f;
        arrived=window.update(50,50,0,50+20*std::sin(phase),4*3.14159265f*std::cos(phase),.01f);
    }
    assert(arrived && window.goodWindows==2 && std::fabs(window.center-50)<.1f);
    window.reset();
    for(unsigned n=0;n<300;n++)assert(!window.update(50,50,0,65,0,.01f)); // wrong center never arrives
    window.reset();
    for(unsigned n=0;n<300;n++)assert(!window.update(50,50,0,50+40*std::sin(n*.1256637f),0,.01f));
    PositionJog timeout;timeout.start(1,50,0,0,0);timeout.update(0,0,0,501,.01f);
    assert(timeout.reason==timeout.LINK_LOST && !timeout.active);
    timeout.start(2,50,0,0,1000);timeout.heartbeat(2,13010);timeout.update(0,0,0,13010,.01f);
    assert(timeout.reason==timeout.TIMEOUT);
    timeout.start(3,50,0,0,10000);timeout.update(0,0,1,10010,.01f);assert(timeout.reason==timeout.SOURCE_CHANGED);
    timeout.start(4,50,0,0,11000);timeout.update(NAN,0,0,11010,.01f);assert(timeout.reason==timeout.FAULT);
    PositionJog wrap;wrap.start(UINT32_MAX,50,0,0,UINT32_MAX-100);
    wrap.heartbeat(UINT32_MAX,20);wrap.update(0,0,0,20,.01f);assert(wrap.active);
    assert(wrap.start(0,0,0,0,30) && !wrap.active);
    puts("PASS: continuous reference/velocity, reverse/retarget, stalled follower, independent lean/config, windowed sway arrival, stop slew, parser, timeout/source/fault, counter/sequence wrap");
}
'''
code="#include <initializer_list>\n"+code
(root/".pio").mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix="jog-tests-",dir=root/".pio") as tmp:
    cpp=Path(tmp)/"jog.cpp";exe=Path(tmp)/"jog.exe"
    cpp.write_text(code,encoding="utf-8")
    subprocess.run(["g++","-std=c++11","-O2","-Wall","-Wextra","-I",str(root/"src"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
