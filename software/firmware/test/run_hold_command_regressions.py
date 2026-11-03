"""Run the actual fixed-angle command branch against malformed and stale-mode frames."""
from pathlib import Path
import subprocess,tempfile
root=Path(__file__).resolve().parents[1];s=(root/"src/main.cpp").read_text(encoding="utf-8-sig")
a=s.index("    if (key == '^' || key == '@') {");b=s.index("    // 值必须为数字",a);branch=s[a:b]
mock=r'''
#include "manual_motion.h"
#include "position_jog.h"
#include <cassert>
#include <cstdio>
#include <cstring>
using std::isfinite;
struct Reply{int errors=0;void println(const char* s){if(std::strncmp(s,"ERR:",4)==0)errors++;}}reply;
ManualPitchControl manualForward;ManualTurnControl manualTurn;PositionJog positionJog;
uint32_t now=0;uint32_t millis(){return now;}
float targetBalanceAngle=4.3f,balanceAngleOffset=4,pitch=4;
int imuFailCount=0;void stopPositionJog(int reason){positionJog.cancel(reason);}
'''
cases=r'''
int main(){
    handle('^',"0,2");assert(manualForward.requested==2 && std::fabs(manualForward.applied-.3f)<.00001f);
    int errors=reply.errors;handle('^',"0,3");assert(reply.errors==errors+1 && manualForward.requested==2);
    handle('^',"0,nan");assert(manualForward.requested==2);
    handle('@',"0,-2");assert(manualForward.requested==2);
    handle('^',"150,2");assert(manualTurn.requested==150);
    handle('^',"0,0");assert(manualForward.requested==0 && manualTurn.requested==0);
    manualForward.clear();pitch=10;handle('^',"0,0");handle('^',"0,-2");assert(manualForward.requested==0);
    pitch=4;targetBalanceAngle=4;handle('^',"0,-2");assert(manualForward.requested==-2);
    manualForward.clear();handle('^',"0,2");assert(manualForward.requested==0 && manualForward.needsZero);
    handle('^',"0,0");handle('^',"0,2");assert(manualForward.requested==2);
    puts("PASS: actual fixed-angle protocol, +/-2 only, atomic rejection, zero stop, entry seed, upright and zero-rearm guards");
}
'''
(root/".pio").mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix="hold-command-",dir=root/".pio") as tmp:
    cpp=Path(tmp)/"test.cpp";exe=Path(tmp)/"test.exe"
    cpp.write_text(mock+"\nvoid handle(char key,const char* vbuf){\n"+branch+"}\n"+cases,encoding="utf-8")
    subprocess.run(["g++","-std=c++11","-O2","-I",str(root/"src"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
