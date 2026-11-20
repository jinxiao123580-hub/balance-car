#include "control/motor.h"

namespace control {

// ============================================================
// TMC2209 双板 UART 配置（独立 STEP/DIR 模式下不编译）
// 后轮TMC (Motor2: PA8 STEP / PB0 DIR) PDN_UART = PA9 (USART1)
// 前轮TMC (Motor1: PA6 STEP / PA7 DIR) PDN_UART = PA2 (USART2)
// 仅配置寄存器(每次上电重写)；STEP/DIR 仍由原逻辑驱动
// ============================================================
#if !TMC_STANDALONE_STEP_DIR
// 轻量半双工 Stream：USART 半双工单线 (HDSEL=1)，TMC2209 PDN_UART 用


// 后轮/前轮 TMC UART 实例：USART1+PA9 / USART2+PA2（TMC2209 PDN_UART 单线半双工）。
// 当前 TMC_STANDALONE_STEP_DIR=1，本段不编译。
HalfDuplexSerial rearTmcUart(USART1, GPIOA, 9);  // 后轮 PA9/USART1
HalfDuplexSerial frontTmcUart(USART2, GPIOA, 2);  // 前轮 PA2/USART2
// 候选驱动对象：TMC2209 地址由 MS1/MS2 决定（0~3），上电时逐个探测 version==0x21 找到实际地址。
static TMC2209Stepper rearTmc[4] = {
  { & rearTmcUart, R_SENSE, 0 },
  { & rearTmcUart, R_SENSE, 1 },
  { & rearTmcUart, R_SENSE, 2 },
  { & rearTmcUart, R_SENSE, 3 }
};

// 前轮候选对象数组：TMC_FRONT_ACTIVE=false（前轮 DRV8825）时不会用到，但对象保留以支持 UART 模式编译。
static TMC2209Stepper frontTmc[4] = {
  { & frontTmcUart, R_SENSE, 0 },
  { & frontTmcUart, R_SENSE, 1 },
  { & frontTmcUart, R_SENSE, 2 },
  { & frontTmcUart, R_SENSE, 3 }
};

// configureTmcDriver：配置一片 TMC2209（spreadCycle、1200mA、1/16、toff=3）并逐项回读校验。
// 参数 tmc=候选对象（地址已定）｜ 返回 true=全部回读通过；false=version 不对或回读不符。
// 边界：回读前先用 IFCNT() 判断写入计数器至少前进 5 次，防止把“没写进去”误判为成功。
bool configureTmcDriver(TMC2209Stepper & tmc) {
  if (tmc.version() != 0x21) return false;
  const uint8_t ifcntBefore = tmc.IFCNT();
  tmc.begin();  // pdn_disable=1，mstep_reg_select=1
  tmc.toff(3);  // 使能斩波器
  tmc.en_spreadCycle(true);  // 显式关闭 stealthChop，强制 spreadCycle
  tmc.rms_current(1200);  // 以载板 Rsense=0.11 Ω 为前提
  tmc.microsteps(16);  // UART 寄存器优先于 MS1/MS2 微步设置
  const uint8_t ifcntAfter = tmc.IFCNT();
  return (uint8_t)(ifcntAfter - ifcntBefore) >= 5 && tmc.version() == 0x21 && tmc.en_spreadCycle() && tmc.mstep_reg_select() && tmc.toff() == 3 && tmc.microsteps() == 16;
}

// findAndConfigureTmc：在地址 0~3 中找第一片 version==0x21 且配置成功的 TMC2209。
// 参数 candidates[4]=候选对象；versions[4] 回传每地址读到的 version（诊断）；address 回传命中地址。
// 返回 true=找到并配置成功；false=4 个地址都没有。调用：initTmcDrivers()（当前模式不编译）。
bool findAndConfigureTmc(TMC2209Stepper(& candidates)[4], uint8_t(& versions)[4], uint8_t & address) {
  address = 0xFF;
  for (uint8_t i = 0; i < 4;++i) {
    versions[i] = candidates[i].version();
    if (versions[i] == 0x21 && configureTmcDriver(candidates[i])) {
      address = i;
      return true;
    }
  }
  return false;
}

// initTmcDrivers：TMC UART 初始化总入口：开串口→延时→前后轮探测配置→失败打印诊断。
// 返回 true=前后轮都成功；false=有失败（main setup() 会 while 重试并停电机）。当前模式不编译。
// 边界：TMC_FRONT_ACTIVE=false 时前轮跳过（DRV8825 无 UART）。
bool initTmcDrivers() {
  rearTmcUart.begin(115200);
  if (TMC_FRONT_ACTIVE) frontTmcUart.begin(115200);
  delay(50);
  uint8_t rearVersions[4], frontVersions[4];
  uint8_t rearAddress, frontAddress;
  const bool rearOk = findAndConfigureTmc(rearTmc, rearVersions, rearAddress);
  const bool frontOk = TMC_FRONT_ACTIVE ? findAndConfigureTmc(frontTmc, frontVersions, frontAddress): true;  // 前轮 DRV8825 或前轮不参与 UART 时跳过
  if (!rearOk || !frontOk) {
    BTSerial.print("TMC UART FAIL rear=");
    BTSerial.print(rearOk);
    BTSerial.print(" front=");
    BTSerial.println(frontOk);
    BTSerial.print("TMC DIAG R[0..3]=");
    for (uint8_t i = 0; i < 4;++i) {
      BTSerial.print(rearVersions[i], HEX);
      BTSerial.print(' ');
    }
    BTSerial.print("F[0..3]=");
    for (uint8_t i = 0; i < 4;++i) {
      BTSerial.print(frontVersions[i], HEX);
      BTSerial.print(' ');
    }
    BTSerial.println();
    BTSerial.print("TMC RX rear=");
    BTSerial.print(rearTmcUart.rxCount);
    BTSerial.print(" front=");
    BTSerial.print(frontTmcUart.rxCount);
    BTSerial.print(" err=");
    BTSerial.print(rearTmcUart.rxErrors, HEX);
    BTSerial.print('/');
    BTSerial.print(frontTmcUart.rxErrors, HEX);
    BTSerial.print(" last=");
    BTSerial.print(rearTmcUart.lastRx, HEX);
    BTSerial.print('/');
    BTSerial.print(frontTmcUart.lastRx, HEX);
    BTSerial.print(" PA9/PA2=");
    BTSerial.print((GPIOA->IDR >> 9) & 1);
    BTSerial.print('/');
    BTSerial.println((GPIOA->IDR >> 2) & 1);
  } else {
    BTSerial.print("TMC UART address rear=");
    BTSerial.print(rearAddress);
    BTSerial.print(" front=");
    BTSerial.println(frontAddress);
  }
  return rearOk && frontOk;
}

#if TMC_GPIO_WRITE_ONLY
// configureTmcWriteOnly：只写不读回的配置序列（toff=0 先关斩波→begin→电流/细分→最后 toff=3 使能）。
// 参数 candidates[4]=地址 0~3 的候选对象，全部盲写（只有地址匹配的驱动会接收）。当前模式不编译。
void configureTmcWriteOnly(TMC2209Stepper(& candidates)[4]) {
  // 四个可能地址逐个写入，实际只有地址匹配的驱动会接收。
  for (auto & tmc: candidates) {
    tmc.toff(0);  // 配置阶段不使能斩波器
    tmc.begin();  // pdn_disable=1, mstep_reg_select=1
    tmc.I_scale_analog(false);  // 运行电流按 UART 寄存器设置
    tmc.en_spreadCycle(true);
    tmc.rms_current(1200);
    tmc.microsteps(16);
    tmc.toff(3);  // 最后才使能斩波器
  }
}

// initTmcDriversWriteOnly：GPIO 位翻转模式的初始化：先查总线空闲电平，再盲写配置。
// 参数 rearLineHigh/frontLineHigh 回传 PA9/PA2 空闲是否为高（悬空/被下拉=接线或驱动异常）。
// 返回 true=电平正常且已盲写；false=电平异常（调用方 main setup() 会重试）。当前模式不编译。
bool initTmcDriversWriteOnly(bool & rearLineHigh, bool & frontLineHigh) {
  rearTmcUart.begin(115200);
  if (TMC_FRONT_ACTIVE) {
    frontTmcUart.begin(115200);
    delayMicroseconds(20);
  } else {
    frontLineHigh = true;  // 前轮为 DRV8825，无 PDN_UART，跳过前轮 UART
  }
  rearLineHigh = (GPIOA->IDR & (1UL << 9)) != 0;
  if (TMC_FRONT_ACTIVE) frontLineHigh = (GPIOA->IDR & (1UL << 2)) != 0;
  if (!rearLineHigh || (TMC_FRONT_ACTIVE && !frontLineHigh)) {
    GPIOA->MODER &= ~((3UL << (2 * 9)) | (3UL << (2 * 2)));
    return false;
  }
  configureTmcWriteOnly(rearTmc);
  if (TMC_FRONT_ACTIVE) configureTmcWriteOnly(frontTmc);
  return true;
}

#endif
#endif // !TMC_STANDALONE_STEP_DIR


}  // namespace control
// ============================================================
// 全局状态定义（core/state.h 里的 extern 在此落地，必须在全局作用域）
// ============================================================
// ============================================================
// 电机硬件定时器
// ============================================================
// TIM2：Motor1 STEP 边沿定时器 ｜ 32 位、1MHz、中断里翻 PA6；只做计数与中断，不占 PWM 输出。
HardwareTimer motor1Timer(TIM2);  // 32-bit STEP edge timer, PA6 unchanged
// TIM5：Motor2 STEP 边沿定时器 ｜ 32 位、1MHz、中断里翻 PA8。
HardwareTimer motor2Timer(TIM5);  // 32-bit STEP edge timer, PA8 unchanged
// motor1LastCommandHz：Motor1 实际下发命令 ｜ 单位 Hz ｜ 写：setMotor1/stopMotor1 ｜ 读：遥测第26列 ｜ 400Hz 写。
// 边界：停机路径写 0；数值已被 MAX_STEP_HZ 夹紧。
float motor1LastCommandHz = 0.0f;
// motor2LastCommandHz：Motor2 实际下发命令 ｜ 单位 Hz ｜ 写：setMotor2/stopMotor2 ｜ 读：遥测第27列 ｜ 400Hz 写。
float motor2LastCommandHz = 0.0f;
// 电机安装方向
// motor1Invert：Motor1 DIR 极性补偿 ｜ bool，false=前轮正装 ｜ 读：commandStepOutput 每拍 400Hz。
bool motor1Invert = false;
// motor2Invert：Motor2 DIR 极性补偿 ｜ bool，true=MKS TMC2209 V2.0 载板后轮与前轮方向相反（只翻后轮 DIR 极性）。
bool motor2Invert = true;  // MKS TMC2209 V2.0 后轮与前轮方向相反：只翻转后轮 DIR 极性
// 编码器方向
// encoder1Direction：编码器1 计数方向 ｜ ±1，当前 +1 ｜ 读：encoder1ISR 每 A 相边沿。
int encoder1Direction = 1;
// encoder2Direction：编码器2 计数方向 ｜ ±1，当前 -1（V10 实机 8/21 实测两路计数符号相反）｜ 读：encoder2ISR。
int encoder2Direction = - 1;  // V10 实机确认值：编码器2方向反（8/21 实测 enc1/enc2 计数符号相反）
// ============================================================
// 编码器 + 多源速度Kalman
// ============================================================
// encoder1Count：编码器1 累计计数 ｜ 单位 count（int32，回绕用无符号差）｜ 写：encoder1ISR ｜
// 读：tick100（关中断快照）/位置保持/main/遥测 ｜ 10.8 count = 1mm。
volatile int32_t encoder1Count = 0;
// encoder2Count：编码器2 累计计数 ｜ 单位 count ｜ 写：encoder2ISR ｜ 读：同上 ｜ 10.8 count = 1mm。
volatile int32_t encoder2Count = 0;

namespace control {

// StepOutput 定义在 control/motor.h；这里给出两个实例
// StepOutput 实例：step1 对应 Motor1（PA6=GPIOA 位6，定时器 TIM2）；频率状态由 commandStepOutput 维护。
StepOutput step1(& motor1Timer, 1UL << 6);
// StepOutput 实例：step2 对应 Motor2（PA8=GPIOA 位8，定时器 TIM5）。
StepOutput step2(& motor2Timer, 1UL << 8);

// ============================================================
// 电机停止
// 只pause，不修改STEP pinMode
// 后轮 PA9/USART1 等 TMC 细节保持不变
// ============================================================
// stopStepOutput：停一路 STEP 输出（关定时器计数、清标志、STEP 脚拉低）。
// 参数 out=要停的 StepOutput ｜ 无返回 ｜ 调用：commandStepOutput 的停止/换向/重启路径 + 停机入口。
// 边界：进临界区（关全局中断）保证 ISR 不会与关定时器竞争；若当前为高电平，先等 STEP_HIGH_US 让脉冲完整。
//   只 pause 定时器、不改 STEP 引脚 pinMode，避免毛刺。
void stopStepOutput(StepOutput & out) {
  uint32_t saved = __get_PRIMASK();
  __disable_irq();
  out.running = false;
  out.timer->getHandle()->Instance->CR1 &= ~TIM_CR1_CEN;
  if (out.high) delayMicroseconds(STEP_HIGH_US);
  out.timer->getHandle()->Instance->SR = 0;
  GPIOA->BSRR = out.mask << 16;
  out.high = false;
  __set_PRIMASK(saved);
}
// stepEdge：STEP 边沿中断服务：每进一次翻转一次电平，并重装 ARR。
// 参数 out=StepOutput ｜ 无返回 ｜ 调用：motor1EdgeISR/motor2EdgeISR（TIM2/TIM5 更新中断）。
// 时序：高电平段 ARR=STEP_HIGH_US-1（4µs）；低电平段 ARR=periodTicks-STEP_HIGH_US-1，
//   故一周期 = periodTicks µs ⇒ f_Hz = 1e6/periodTicks。边界：running=false 直接返回（不发脉冲）。
// 高频时 ARR 很小（MAX_STEP_HZ=20000 → periodTicks=50 ⇒ 低段 46 tick），ISR 负载由定时器频率决定。
void stepEdge(StepOutput & out) {
  TIM_TypeDef * tim = out.timer->getHandle()->Instance;
  if (!out.running) return;
  // 半周期切换：高电平段 ARR=STEP_HIGH_US-1=3（4µs）；低电平段 ARR=periodTicks-STEP_HIGH_US-1，
  //   故一整周期 = periodTicks µs ⇒ f=1e6/periodTicks Hz。
  if (out.high) {
    GPIOA->BSRR = out.mask << 16;
    out.high = false;
    tim->ARR = out.periodTicks - STEP_HIGH_US - 1;
  } else {
    GPIOA->BSRR = out.mask;
    out.high = true;
    out.risingEdges++;
    tim->ARR = STEP_HIGH_US - 1;
  }
  tim->CNT = 0;
}
// motor1EdgeISR：TIM2 更新中断入口，直接转 stepEdge(step1) ｜ 调用频率 = 2×STEP 频率。
void motor1EdgeISR() {
  stepEdge(step1);
}
// motor2EdgeISR：TIM5 更新中断入口，直接转 stepEdge(step2) ｜ 调用频率 = 2×STEP 频率。
void motor2EdgeISR() {
  stepEdge(step2);
}
// initMotors：初始化电机外设（调用：main setup() 一次）。
// 1) 4 脚设为输出、DIR 初值低、STEP 脚 BSRR 复位为低；
// 2) TIM2/TIM5 分频到 1MHz（prescale=clk/1e6）、ARR=1e6（=1Hz 周期）、关闭预装载（改 ARR 立即生效）；
// 3) 绑更新中断、resume 定时器，最后 stopStepOutput 确保两路处于停止态。
// 边界：板级时钟变化时 prescale 会自动算，不依赖 168MHz 写死。
void initMotors() {
  pinMode(MOTOR1_DIR_PIN, OUTPUT);
  pinMode(MOTOR2_DIR_PIN, OUTPUT);
  pinMode(MOTOR1_STEP_PIN, OUTPUT);
  pinMode(MOTOR2_STEP_PIN, OUTPUT);
  digitalWrite(MOTOR1_DIR_PIN, LOW);
  digitalWrite(MOTOR2_DIR_PIN, LOW);
  GPIOA->BSRR = (step1.mask | step2.mask) << 16;
  StepOutput * outputs[] = {
    & step1,
  & step2 };
  for (StepOutput * out: outputs) {
    out->timer->setPrescaleFactor(out->timer->getTimerClkFreq() / 1000000UL);
    out->timer->setOverflow(1000000UL, TICK_FORMAT);
    out->timer->setPreloadEnable(false);
  }
  motor1Timer.attachInterrupt(motor1EdgeISR);
  motor2Timer.attachInterrupt(motor2EdgeISR);
  motor1Timer.resume();
  motor2Timer.resume();
  stopStepOutput(step1);
  stopStepOutput(step2);
}
// commandStepOutput：把“带符号频率命令”落到一路 STEP 输出（单位 Hz，正负=方向）。
// 参数 out=输出通道；dirPin=DIR 引脚；invert=方向极性补偿；commandHz=命令频率（Hz）。无返回。
// 1) NaN 或 |Hz|<MOTOR_STOP_THRESHOLD_HZ → 停；|Hz|>MAX_STEP_HZ → 先夹紧；
// 2) 周期 period = round(1e6/|Hz|) µs（1 STEP=0.0625mm ⇒ 1Hz=0.0625mm/s，20000Hz≈1250mm/s 是硬上限）；
// 3) 首次启动/换向：先停→DIR 建立 4µs→改 DIR→ARR=1 让首个边沿 1µs 后到→使能；
// 4) 仅改频：不重启定时器，按“低电平已走比例”把 CNT 折算到新周期，保持相位连续（避免频率阶跃丢步）。
// 边界：临界区内改 ARR/CNT/CR1；dir 用 (commandHz>=0)!=invert 计算。
void commandStepOutput(StepOutput & out, uint32_t dirPin, bool invert, float commandHz) {
  if (!isfinite(commandHz) || fabsf(commandHz) < MOTOR_STOP_THRESHOLD_HZ) {
    stopStepOutput(out);
    return;
  }
  bool dir = (commandHz >= 0.0f) != invert;
  // 周期 µs 四舍五入：f=1e6/period；|Hz| 先夹到 MAX_STEP_HZ；1 STEP=0.0625mm ⇒ 1Hz=0.0625mm/s。
  uint32_t period = (uint32_t)(1000000.0f / fminf(fabsf(commandHz), MAX_STEP_HZ) + 0.5f);
  if (!out.running || dir != out.dir) {
    stopStepOutput(out);
    delayMicroseconds(DIR_SETUP_US);
    digitalWrite(dirPin, dir ? HIGH: LOW);
    out.dir = dir;
    delayMicroseconds(DIR_SETUP_US);
    uint32_t saved = __get_PRIMASK();
    __disable_irq();
    out.periodTicks = period;
    TIM_TypeDef * tim = out.timer->getHandle()->Instance;
    tim->ARR = 1;
    tim->CNT = 0;
    tim->SR = 0;
    out.running = true;
    tim->CR1 |= TIM_CR1_CEN;
    __set_PRIMASK(saved);
    return;
  }
  uint32_t saved = __get_PRIMASK();
  __disable_irq();
  TIM_TypeDef * tim = out.timer->getHandle()->Instance;
  uint32_t oldPeriod = out.periodTicks;
  out.periodTicks = period;
  if (!out.high && period != oldPeriod) {
    // Preserve fractional progress while retiming the LOW interval.
    tim->CR1 &= ~TIM_CR1_CEN;
    uint32_t oldLow = oldPeriod - STEP_HIGH_US;
    uint32_t newLow = period - STEP_HIGH_US;
    uint32_t elapsed = tim->CNT;
    if (elapsed >= oldLow) elapsed = oldLow - 1;
    uint32_t nextCount = (uint32_t)((uint64_t) elapsed * newLow / oldLow);
    tim->ARR = newLow - 1;
    tim->CNT = nextCount;
    tim->CR1 |= TIM_CR1_CEN;
  }
  __set_PRIMASK(saved);
}
// ============================================================
// 电机停止
// 只pause，不修改STEP pinMode
// ============================================================
// stopMotor1：停 Motor1 输出并清零 motor1LastCommandHz ｜ 无参/无返回 ｜ 调用：停机路径。
void stopMotor1() {
  stopStepOutput(step1);
  motor1LastCommandHz = 0.0f;
}
// stopMotor2：停 Motor2 输出并清零 motor2LastCommandHz ｜ 无参/无返回 ｜ 调用：停机路径。
void stopMotor2() {
  stopStepOutput(step2);
  motor2LastCommandHz = 0.0f;
}
// stopBothMotors：两路一起停（倒地/IMU 失效/掉拍/欠压等统一入口）｜ 无参/无返回 ｜ 400Hz 任一拍可调用。
void stopBothMotors() {
  stopMotor1();
  stopMotor2();

}
// ============================================================
// Motor1
// ============================================================
// setMotor1：下发 Motor1 频率命令（Hz）｜ 无返回 ｜ 400Hz 每拍由 mixMotors 调用。
// 内部先 commandStepOutput（停/限幅/换向），再记录 copysign(fmin(|Hz|,MAX_STEP_HZ),Hz) 或 0 到 motor1LastCommandHz。
void setMotor1(float commandHz) {
  commandStepOutput(step1, MOTOR1_DIR_PIN, motor1Invert, commandHz);
  motor1LastCommandHz = step1.running ? copysignf(fminf(fabsf(commandHz), MAX_STEP_HZ), commandHz): 0.0f;
}
// ============================================================
// Motor2
// ============================================================
// setMotor2：下发 Motor2 频率命令（Hz）｜ 无返回 ｜ 400Hz 每拍由 mixMotors 调用 ｜ 记录实际命令到 motor2LastCommandHz。
void setMotor2(float commandHz) {
  commandStepOutput(step2, MOTOR2_DIR_PIN, motor2Invert, commandHz);
  motor2LastCommandHz = step2.running ? copysignf(fminf(fabsf(commandHz), MAX_STEP_HZ), commandHz): 0.0f;
}

// ============================================================
// 电机合成：直立输出 ± 转向输出（原 loop 中的两行，算式不变）
// ============================================================
// mixMotors：直立共模 + 转向差模合成两轮命令 ｜ 参数 turnOffsetHz 单位 Hz（转向差动）｜ 无返回 ｜ 400Hz。
// Motor1 = finalBalanceOutput + turnOffsetHz；Motor2 = finalBalanceOutput - turnOffsetHz。
// 边界：任一轮 |命令|<1Hz 会被 setMotor 停；单轮超 MAX_STEP_HZ 会被夹紧（差动仍保留）。
void mixMotors(float turnOffsetHz) {
  float motor1Command = finalBalanceOutput + turnOffsetHz;
  float motor2Command = finalBalanceOutput - turnOffsetHz;
  setMotor1(motor1Command);
  setMotor2(motor2Command);
}

// ============================================================
// 编码器 ISR / 初始化
// ============================================================
// ============================================================
// 编码器ISR
// ============================================================
// encoder1ISR：编码器1 A 相 CHANGE 中断（上升沿+下降沿都进）。
// 计数方式：A==B → count += encoder1Direction，否则 -=；即每个 A 相边沿计 ±1。
// 分辨率：1024 线编码器只数 A 相双边沿 = 2048 count/圈（2 倍频，不是 4 倍频 AB 解码）；
//   实测 10.8 count = 1mm（1 count≈0.0926mm）。无返回；只做整数加减，实测 ISR 很轻。
void encoder1ISR() {
  if (digitalRead(ENCODER1_A_PIN) == digitalRead(ENCODER1_B_PIN)) {
    encoder1Count += encoder1Direction;
  } else {
    encoder1Count -= encoder1Direction;
  }

}
// encoder2ISR：编码器2 A 相 CHANGE 中断，规则同 encoder1ISR（方向由 encoder2Direction=-1 修正）｜ 2048 count/圈。
void encoder2ISR() {
  if (digitalRead(ENCODER2_A_PIN) == digitalRead(ENCODER2_B_PIN)) {
    encoder2Count += encoder2Direction;
  } else {
    encoder2Count -= encoder2Direction;
  }

}
// ============================================================
// 编码器初始化
// ============================================================
// initEncoders：编码器初始化（调用：main setup() 一次）：4 脚 INPUT_PULLUP，A 相挂 CHANGE 中断。
// 边界：B 相只被 digitalRead 读电平，不产生中断；上拉依赖编码器开漏输出，线长时注意干扰。
void initEncoders() {
  pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2ISR, CHANGE);

}

}  // namespace control
