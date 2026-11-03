"""Exercise the real manual_turn.h limits and fault behaviour on the host."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[1]
(root / ".pio").mkdir(exist_ok=True)
code = r'''
#include "manual_turn.h"
#include <cassert>
#include <cstdio>
int main() {
    ManualTurnControl t;
    t.command(1000, 0);
    assert(t.requested == 300);
    assert(t.update(0, .0025f, 0, 1750, 0, 0) == .75f);
    for (unsigned n=1;n<=100;n++) {
        t.command(100,n*10);
        t.update(n*10,.01f,0,1750,0,0);
    }
    assert(t.applied == 100);
    assert(t.update(1001,.01f,0,1750,1745,0) == 5); // balance gets priority
    assert(t.update(1002,.01f,10,1750,0,0) == 0); // tilt cuts turn immediately
    assert(t.requested == 0);
    t.command(100,1999);
    assert(t.requested == 0); // must release after tilt/protection
    t.command(0,1999);
    t.command(100,2000);
    for(unsigned n=0;n<30;n++) t.update(2000+n*10,.01f,0,1750,0,0);
    t.update(2351,.01f,0,1750,0,0);
    assert(t.requested == 0);
    for(unsigned n=0;n<110;n++) t.update(2400+n*10,.01f,0,1750,0,0);
    assert(t.applied == 0 && !t.busy(4000));
    t.command(-100,UINT32_MAX-10);
    t.update(20,.01f,0,1750,0,0); // millis wrap
    assert(t.requested == -100);
    t.update(400,.01f,0,1750,0,0);
    assert(t.requested == 0);
    t.command(100,500);
    t.update(500,.01f,NAN,1750,0,0);
    assert(t.applied == 0 && t.requested == 0);
    for(float balance : {-1750.f,-1740.f,-100.f,0.f,100.f,1740.f,1750.f}) {
        t.command(0,0);
        t.command(100,0);
        for(unsigned n=0;n<30;n++) {
            float d=t.update(n*10,.01f,8,1750,balance,10000);
            assert(std::fabs(balance+d)<=1750 && std::fabs(balance-d)<=1750);
            assert(std::fabs(d)<=120.001f);
        }
        t.clear();
    }
    puts("PASS: limits, balance priority, tilt, timeout, zero, clock wrap, NaN");
}
'''
code = "#include <initializer_list>\n" + code
with tempfile.TemporaryDirectory(prefix="turn-tests-", dir=root/".pio") as temp:
    cpp=Path(temp)/"turn.cpp";exe=Path(temp)/"turn.exe"
    cpp.write_text(code,encoding="utf-8")
    subprocess.run(["g++","-std=c++11","-O2","-Wall","-Wextra","-I",str(root/"src"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
