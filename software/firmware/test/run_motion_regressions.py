"""Host regression checks execute the STEP and reversal code extracted from main.cpp.

Run with Python and g++ available. MCU interrupt latency still needs bench measurement.
"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / "src/main.cpp").read_text(encoding="utf-8-sig")
driver = source[source.index("struct StepOutput {"):source.index("void stopMotor1(")]
# The closing comments between driver and stopMotor1 are harmless C++ comments.
crossing = source[source.index("    float stepIncrement = balanceAccelSubstepOutput;"):
                  source.index("    if (integratedMotorSpeedCommand > BALANCE_OUTPUT_LIMIT_HZ)")]
mock = r'''
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
using std::isfinite;
struct TIM_TypeDef { uint32_t CR1=0, SR=0, ARR=0, CNT=0; };
struct GPIO_TypeDef { uint32_t BSRR=0; } gpio;
GPIO_TypeDef* GPIOA=&gpio;
constexpr uint32_t TIM_CR1_CEN=1;
struct Handle { TIM_TypeDef* Instance; };
struct HardwareTimer {
    TIM_TypeDef regs; Handle handle{&regs};
    Handle* getHandle(){return &handle;}
    uint32_t getTimerClkFreq(){return 84000000;}
    void setPrescaleFactor(uint32_t){}
    void setOverflow(uint32_t n,int){regs.ARR=n-1;}
    void setPreloadEnable(bool){}
    void attachInterrupt(void(*)()){}
    void resume(){regs.CR1|=TIM_CR1_CEN;}
} motor1Timer,motor2Timer;
uint32_t primask=0, waited=0;
uint32_t __get_PRIMASK(){return primask;}
void __disable_irq(){primask=1;}
void __set_PRIMASK(uint32_t v){primask=v;}
void delayMicroseconds(uint32_t n){waited+=n;}
void digitalWrite(uint32_t,int){}
void pinMode(uint32_t,int){}
constexpr int OUTPUT=1,LOW=0,HIGH=1,TICK_FORMAT=0;
constexpr int MOTOR1_DIR_PIN=7,MOTOR2_DIR_PIN=16,MOTOR1_STEP_PIN=6,MOTOR2_STEP_PIN=8;
constexpr float MOTOR_STOP_THRESHOLD_HZ=1;
'''
tests = r'''
float balanceAccelSubstepOutput, integratedMotorSpeedCommand;
float balanceAccelLimitedOutput, balanceAccelEffectiveLimit;
constexpr float REVERSAL_SOFT_LIMIT_SCALE=.25f;
constexpr uint16_t REVERSAL_SOFT_TICKS=25;
uint16_t reversalSoftTicksRemaining=0;
int32_t prevIntegratedSpeedCmdSign=0;
void crossStep() {
REPLACE_CROSSING
}
int main(){
    initMotors();
    assert(!step1.running && !step2.running);
    commandStepOutput(step1,7,false,1.0f);
    assert(step1.running && step1.periodTicks==1000000);
    stepEdge(step1); // rising edge
    assert(step1.high && motor1Timer.regs.ARR==3 && step1.risingEdges==1);
    commandStepOutput(step1,7,false,3500.0f); // don't shorten HIGH
    assert(step1.high && motor1Timer.regs.ARR==3);
    stepEdge(step1); // falling edge uses newest period
    assert(!step1.high && motor1Timer.regs.ARR==281);
    commandStepOutput(step1,7,false,1.0f);
    motor1Timer.regs.CNT=500000;
    commandStepOutput(step1,7,false,3500.0f);
    assert(motor1Timer.regs.ARR==281 && motor1Timer.regs.CNT<282);
    assert(motor1Timer.regs.CNT>=140); // progress preserved; no long old-period wait
    stepEdge(step1);
    uint32_t before=waited;
    commandStepOutput(step1,7,false,0);
    assert(!step1.running && !step1.high && waited-before>=4);
    assert(GPIOA->BSRR==(step1.mask<<16));
    commandStepOutput(step1,7,false,-500);
    assert(step1.running && !step1.dir);
    commandStepOutput(step1,7,false,NAN);
    assert(!step1.running);
    prevIntegratedSpeedCmdSign=1;
    integratedMotorSpeedCommand=0;
    balanceAccelSubstepOutput=-40;
    balanceAccelLimitedOutput=-160;
    balanceAccelEffectiveLimit=400;
    crossStep(); // positive -> zero -> negative must activate immediately
    assert(reversalSoftTicksRemaining==25);
    assert(integratedMotorSpeedCommand==-10 && prevIntegratedSpeedCmdSign==-1);
    reversalSoftTicksRemaining=0;
    prevIntegratedSpeedCmdSign=1;
    integratedMotorSpeedCommand=20;
    balanceAccelSubstepOutput=-100;
    crossStep(); // direct crossing must soften this very substep
    assert(integratedMotorSpeedCommand==-5 && reversalSoftTicksRemaining==25);
    puts("PASS: STEP retiming, pulse width, stop/restart, invalid command, zero-cross protection");
}
'''.replace("REPLACE_CROSSING", crossing)
with tempfile.TemporaryDirectory(prefix="balance-motion-tests-", dir=root / ".pio") as temp:
    cpp = Path(temp) / "motion.cpp"
    exe = Path(temp) / "motion.exe"
    cpp.write_text(mock + driver + tests, encoding="utf-8")
    subprocess.run(["g++", "-std=c++11", "-O2", "-Wall", "-Wextra", str(cpp), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
