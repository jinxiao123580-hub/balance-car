"""Run actual main.cpp jog routing, capture and shared speed-P calculations on host."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1]
s=(root/"src/main.cpp").read_text(encoding="utf-8-sig")
a=s.index("    bool jogPositionControl=false;");b=s.index("    if (manualForward.busy(millis()))",a)
jog=s[a:b]
a=s.index("    bool positionHoldEnabled = positionKp");b=s.index("    // 5. 总目标速度安全限幅",a)
parking=s[a:b]
a=s.index("    speedError =\n",s.index("// 6. 速度误差"));b=s.index("    // 条件积分",a)
speed=s[a:b]
a=s.index("    if (jogAngleControl ||");b=s.index("    // Manual forward/reverse",a)
integral=s[a:b]
a=s.index("    speedAngleUnclamped = speedLoopDirection *",b);b=s.index("    // 旧预测",a)
compose=s[a:b]
a=s.index("void captureCurrentPositionTarget()\n{");b=s.index("// ============================================================",s.index("void stopPositionJog(int reason)\n{",a))
capture=s[a:b]
mock=r'''
#include "position_jog.h"
#include <cassert>
#include <cstdio>
uint32_t now=0;uint32_t millis(){return now;}
void noInterrupts(){}void interrupts(){}
int32_t encoder1Count=-100,encoder2Count=0;
bool encoder1Trusted=true,encoder2Trusted=false,positionHoldWasEnabled=true,positionHoldSuspended=false;
float vehiclePosition=100,targetVehiclePosition=100,positionError=0;
float positionSpeedTarget=0,targetForwardSpeed=0,positionKp=5,POSITION_DEADBAND_COUNT=0,MAX_POSITION_RETURN_SPEED=1;
float vehicleSpeed=0,speedIntegral=0,speedErrorPrev=0,speedErrorDotFilt=0,speedError=0;
float speedKp=-.02f,speedKd=0,speedPContribution=0,speedAngleCorrection=.4f;
PositionOdometry positionOdom;PositionJog positionJog;PositionSpeedReference positionSpeedReference;PositionLeanControl positionLean;
float speedKi=.5f,speedIContribution=0,speedLoopDirection=-1,speedAngleUnclamped=0;
float outerLoopAuthority=1,SPEED_ERROR_DEADBAND=2,SPEED_INTEGRAL_LIMIT=50,MAX_SPEED_ANGLE=2;
struct Forward{bool busy(uint32_t){return false;}}manualForward;
'''
cases=r'''
int main(){
    positionOdom.reset(encoder1Count,encoder2Count);positionOdom.position=100;
    positionJog.start(1,50,vehiclePosition,1,0);
    tick();assert(targetVehiclePosition<100.01f && targetVehiclePosition<positionJog.goal);
    assert(targetForwardSpeed==0 && positionLean.applied>0 && positionLean.applied<=.00101f);
    for(unsigned n=1;n<400;n++){
        now=n*10;positionJog.heartbeat(1,now);tick();
    }
    assert(targetForwardSpeed==0 && positionLean.applied>.15f);
    assert(speedIntegral==0 && speedIContribution==0);
    assert(std::fabs(speedAngleUnclamped-positionLean.applied)<.00001f);
    float before=targetForwardSpeed,ref=targetVehiclePosition;
    vehicleSpeed=7;tick();assert(std::fabs(speedPContribution-speedKp*(targetForwardSpeed-7))<.00001f);
    assert(std::fabs(speedAngleUnclamped-(positionLean.applied-.14f))<.00001f);
    ref=targetVehiclePosition;
    positionJog.start(2,-50,vehiclePosition,1,now);
    assert(positionJog.reference==ref);tick();
    assert(std::fabs(targetForwardSpeed-before)<=.061f);
    before=targetForwardSpeed;stopPositionJog(PositionJog::STOP);
    assert(vehiclePosition==100 && targetVehiclePosition==100 && encoder1Count==-100);
    assert(speedAngleCorrection==.4f); // parking capture does not drop the current angle correction
    tick();assert(std::fabs(targetForwardSpeed-before)<=.03001f);
    puts("PASS: real main.cpp smooth target, independent lean + original K damping, no integral, retarget/stop handoff, continuous odometry/correction");
}
'''
(root/".pio").mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix="jog-integration-",dir=root/".pio") as tmp:
    cpp=Path(tmp)/"test.cpp";exe=Path(tmp)/"test.exe"
    cpp.write_text(mock+capture+"\nvoid tick(){\n"+jog+parking+speed+integral+compose+"\n}\n"+cases,encoding="utf-8")
    subprocess.run(["g++","-std=c++11","-O2","-I",str(root/"src"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
