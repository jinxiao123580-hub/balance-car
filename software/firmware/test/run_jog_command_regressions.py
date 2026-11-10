"""Exercise actual Bluetooth prefix/config routing, including the legacy bare ! command."""
from pathlib import Path
import subprocess,tempfile,re
root=Path(__file__).resolve().parents[1];s=(root/"src/main.cpp").read_text(encoding="utf-8-sig")
a=s.index("void handleBtCommand(char* cmd, uint8_t len)\n{");reply=s.index("    QueuedBtReply reply;",a);b=s.index("    if (key == '?')",reply)
# 2026-11-10：`?` 之前新增了 SAVE/LOAD/DEF 整词分支（依赖 flash 驱动，主机上没有）。
# 从"跳过前导空格"接回来，被测的 jog/诊断路由与原来一致；整词命令本身由
# test/run_param_store_regressions.py 覆盖。
skip=s.index("    // 跳过前导空格",reply)
prefix=s[reply:reply+len("    QueuedBtReply reply;")]+"\n"+s[skip:b]
diag=re.search(r"    if \(key == '!'[^\n]+",s).group(0)
a=s.index("    if (vlen == 0) return;",b);b=s.index("    if(key=='~' || key=='&')",a);config=s[a:b]
mock=r'''
#include "position_jog.h"
#include <cassert>
#include <cstring>
#include <string>
#include <cstdio>
struct QueuedBtReply{template<class...T>void print(T...args){}template<class...T>void println(T...args){}};
PositionJog positionJog;PositionLeanControl positionLean;bool diagnostic=false;
'''
cases=r'''
void send(std::string text){diagnostic=false;handle(&text[0],text.size());}
int main(){
    send("!");assert(diagnostic);
    send(" ! 0.0080,0.250 ");assert(!diagnostic && std::fabs(positionLean.kp-.008f)<.000001f && positionLean.limit==.25f);
    send("! nan,0.3");assert(positionLean.limit==.25f);
    send("! 0.01,0.8");assert(positionLean.limit==.25f);
    positionJog.start(1,50,0,0,0);send("! 0.01,0.30");assert(positionLean.limit==.25f);
    positionJog.cancel(PositionJog::STOP);positionLean.applied=.1f;
    send("! 0.01,0.30");assert(positionLean.limit==.25f);
    positionLean.reset();send("! 0.01,0.30");assert(positionLean.limit==.30f && positionLean.kp==.01f);
    puts("PASS: real Bluetooth config routing, legacy diagnostic, bounds and active/release interlock");
}
'''
(root/".pio").mkdir(exist_ok=True)
with tempfile.TemporaryDirectory(prefix="jog-command-",dir=root/".pio") as tmp:
    cpp=Path(tmp)/"test.cpp";exe=Path(tmp)/"test.exe"
    cpp.write_text(mock+"\nvoid handle(char* cmd,uint8_t len){\n"+prefix+diag+"diagnostic=true;return;}\n"+config+"}\n"+cases,encoding="utf-8")
    subprocess.run(["g++","-std=c++11","-O2","-I",str(root/"src"),str(cpp),"-o",str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
