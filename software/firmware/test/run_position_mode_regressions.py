"""Exercise actual firmware position sampling and outer-loop position arbitration."""
from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[1]
s=(root/"src/main.cpp").read_text(encoding="utf-8-sig")
position=s[s.index("void updatePositionHold()\n{"):s.index("// 把当前位置设为新的驻车位置")]
start=s.index("    bool positionHoldEnabled = positionKp")
end=s.index("    // 5. 总目标速度安全限幅",start)
outer=s[start:end]
a=s.index("void captureCurrentPositionTarget()\n{");b=s.index("// ============================================================",s.index("void stopPositionJog(int reason)\n{",a))
capture=s[a:b]
mock=r'''
#include "position_jog.h"
#include <cassert>
#include <cstdint>
#include <cstdio>
int32_t encoder1Count=0,encoder2Count=0,positionZeroCount1=0,positionZeroCount2=0;
float pitch=0,balanceAngleOffset=0;
float vehiclePosition=0,targetVehiclePosition=0,positionError=0;
float positionSpeedTarget=0,targetForwardSpeed=0,positionKp=5;
float POSITION_DEADBAND_COUNT=0,MAX_POSITION_RETURN_SPEED=1;
PositionOdometry positionOdom; PositionJog positionJog; PositionSpeedReference positionSpeedReference; PositionLeanControl positionLean;
float speedLoopDirection=-1,speedKp=-.02f;
bool jogPositionControl=false;
float speedIntegral=0,speedErrorPrev=0,speedErrorDotFilt=0,vehicleSpeed=0;
bool encoder1Trusted=true,encoder2Trusted=true;
bool positionHoldWasEnabled=true,positionHoldSuspended=false,manualActive=false;
struct Forward {float applied=0,authority=1;float normalAuthority()const{return authority;}} manualForward;
bool manualMotionIsActive(){return manualActive;}
void noInterrupts(){} void interrupts(){}
void captureCurrentPositionTarget();
void stopPositionJog(int reason);
'''
cases=r'''
int main(){
    positionOdom.reset(0,0);
    // A distant old parking reference must not create a speed demand while turning.
    targetVehiclePosition=100;encoder1Count=-20;encoder2Count=-20;
    positionSpeedReference.applied=.8f;
    manualActive=true;updatePositionHold();outerPosition();
    assert(positionHoldSuspended && vehiclePosition==20);
    assert(positionSpeedTarget==0 && targetForwardSpeed==0 && positionError==0);
    assert(targetVehiclePosition==vehiclePosition && positionSpeedReference.applied==0); // track, discard old request
    encoder1Count=-50;encoder2Count=-50;
    updatePositionHold();outerPosition();
    assert(targetForwardSpeed==0 && vehiclePosition==50 && targetVehiclePosition==50);
    // Turning command released but still decelerating: same active arbitration input.
    manualActive=true;updatePositionHold();outerPosition();
    assert(positionHoldSuspended && targetForwardSpeed==0);
    // Returning to O is not enough: follow during the entire normal-loop blend too.
    manualActive=false;manualForward.authority=.4f;encoder1Count=-53;encoder2Count=-53;
    updatePositionHold();outerPosition();
    assert(positionHoldSuspended && targetVehiclePosition==53 && positionError==0 && targetForwardSpeed==0);
    manualForward.authority=.999f;encoder1Count=-55;encoder2Count=-55;
    updatePositionHold();outerPosition();
    assert(positionHoldSuspended && targetVehiclePosition==55 && targetForwardSpeed==0);
    // After full handoff, lock exactly once at the latest position, never the old point.
    manualForward.authority=1;encoder1Count=-56;encoder2Count=-56;
    updatePositionHold();outerPosition();
    assert(!positionHoldSuspended && targetVehiclePosition==56 && targetForwardSpeed==0);
    encoder1Count=-58;encoder2Count=-58;
    updatePositionHold();outerPosition();
    assert(targetVehiclePosition==56 && positionError==-2 && targetForwardSpeed<0 && targetForwardSpeed>=-.031f);
    // Direct pitch motion bypasses position P without writing degrees into a speed target.
    manualActive=true;manualForward.applied=.3f;
    updatePositionHold();outerPosition();
    assert(positionHoldSuspended && positionSpeedTarget==0 && targetForwardSpeed==0);
    positionKp=0;outerPosition();assert(targetForwardSpeed==0);
    manualForward.applied=0;manualActive=false;updatePositionHold();outerPosition();
    // Disabling X must stay disabled after manual motion; config is never overwritten.
    positionKp=0;manualActive=true;updatePositionHold();outerPosition();
    manualActive=false;updatePositionHold();outerPosition();
    assert(positionKp==0 && targetForwardSpeed==0);
    // Repeated manual sessions always take a fresh parking reference.
    positionKp=5;manualActive=true;encoder1Count=-80;encoder2Count=-80;
    updatePositionHold();outerPosition();
    manualActive=false;updatePositionHold();outerPosition();
    assert(targetVehiclePosition==80 && targetForwardSpeed==0);
    // Source and excessive tilt changes cancel an in-progress goal and capture a fresh origin.
    positionJog.start(1,50,vehiclePosition,0,0);encoder2Trusted=false;
    updatePositionHold();assert(!positionJog.active && positionJog.reason==PositionJog::SOURCE_CHANGED);
    positionJog.start(2,50,vehiclePosition,1,0);pitch=6;
    updatePositionHold();assert(!positionJog.active && positionJog.reason==PositionJog::FAULT);
    pitch=0;encoder1Count=-90;updatePositionHold();float before=vehiclePosition;
    int32_t rawBefore=encoder1Count;captureCurrentPositionTarget();
    assert(vehiclePosition==before && targetVehiclePosition==before && encoder1Count==rawBefore);
    encoder1Count-=3;updatePositionHold();assert(vehiclePosition==before+3);
    // A new command during a partial handoff keeps tracking, without a one-frame pullback.
    manualForward.authority=.5f;manualActive=true;encoder1Count-=10;
    updatePositionHold();outerPosition();
    assert(positionHoldSuspended && targetVehiclePosition==vehiclePosition && targetForwardSpeed==0);
    puts("PASS: live parking target, stale ramp reset, complete handoff tracking, one-shot final lock, continuous encoders, X disable, repeated motion");
}
'''
(root/".pio").mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix="position-mode-",dir=root/".pio") as tmp:
    cpp=Path(tmp)/"position.cpp";exe=Path(tmp)/"position.exe"
    cpp.write_text(mock+position+capture+"\nvoid outerPosition(){\n"+outer+"\n}\n"+cases,encoding="utf-8")
    subprocess.run(["g++","-I",str(root/"src"),"-std=c++11","-O2","-Wall","-Wextra",str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
