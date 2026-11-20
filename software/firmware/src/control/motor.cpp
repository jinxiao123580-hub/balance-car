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
class HalfDuplexSerial: public Stream {
  protected:
  USART_TypeDef * uart;
  GPIO_TypeDef * gpio;
  uint8_t pinBit;
  public:
  uint32_t rxCount = 0;
  uint32_t txCount = 0;
  uint32_t rxErrors = 0;
  uint8_t lastRx = 0;
  uint8_t txFramePos = 0;
  uint8_t txFrameLen = 0;
  uint32_t bitTicks = 0;
  HalfDuplexSerial(USART_TypeDef * u, GPIO_TypeDef * g, uint8_t b): uart(u), gpio(g), pinBit(b) {
  }
  void begin(uint32_t baud) {
    rxCount = txCount = rxErrors = 0;
    lastRx = 0;
    txFramePos = txFrameLen = 0;
    // 这里使用真正的 GPIO 位号，不使用 Arduino 板级引脚编号。
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    (void) RCC->AHB1ENR;
#if TMC_GPIO_WRITE_ONLY
    // USART 半双工空闲时没有把该载板的 17.8 kΩ 下拉拉高。
    // 诊断只写帧改用普通 GPIO，确保空闲和每个停止位均为推挽高电平。
    gpio->BSRR = 1UL << pinBit;
    gpio->MODER = (gpio->MODER & ~(3UL << (2 * pinBit))) | (1UL << (2 * pinBit));
    gpio->OTYPER &= ~(1UL << pinBit);
    gpio->PUPDR = (gpio->PUPDR & ~(3UL << (2 * pinBit))) | (1UL << (2 * pinBit));
    gpio->OSPEEDR |= 3UL << (2 * pinBit);
    bitTicks = (HAL_RCC_GetHCLKFreq() + baud / 2) / baud;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    return;
#else
    gpio->MODER &= ~(3UL << (2 * pinBit));
    gpio->MODER |= (2UL << (2 * pinBit));
    gpio->OTYPER |= (1UL << pinBit);  // 单线收发：开漏并在回复时释放总线
    gpio->PUPDR &= ~(3UL << (2 * pinBit));
    gpio->PUPDR |= (1UL << (2 * pinBit));  // 空闲高电平；长线建议外部上拉
    gpio->OSPEEDR |= (3UL << (2 * pinBit));
    if (pinBit < 8) {
      gpio->AFR[0] &= ~(0xFUL << (4 * pinBit));
      gpio->AFR[0] |= (7UL << (4 * pinBit));
    } else {
      uint32_t b = pinBit - 8;
      gpio->AFR[1] &= ~(0xFUL << (4 * b));
      gpio->AFR[1] |= (7UL << (4 * b));
    }
    if (uart == USART1) RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    else if (uart == USART2) RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    uint32_t fck = (uart == USART1) ? HAL_RCC_GetPCLK2Freq(): HAL_RCC_GetPCLK1Freq();
    uart->CR1 = 0;
    uart->CR2 = 0;
    uart->CR3 = USART_CR3_HDSEL;
    uart->BRR = (fck + baud / 2) / baud;  // OVER8=0，16 倍过采样
    uart->CR1 = USART_CR1_UE | USART_CR1_RE;
#endif
  }
  int available(void) {
#if TMC_GPIO_WRITE_ONLY
    return 0;
#else
    return (uart->SR & USART_SR_RXNE) ? 1: 0;
#endif
  }
  int read(void) {
#if TMC_GPIO_WRITE_ONLY
    return - 1;
#else
    const uint32_t status = uart->SR;
    if (!(status & USART_SR_RXNE)) return - 1;
    rxErrors |= status & (USART_SR_FE | USART_SR_NE | USART_SR_ORE);
    lastRx = (uint8_t) uart->DR;
    rxCount++;
    return lastRx;
#endif
  }
  size_t write(uint8_t b) {
#if TMC_GPIO_WRITE_ONLY
    // 8N1，LSB first；单字节关中断以保持位时间准确。
    const uint32_t high = 1UL << pinBit;
    const uint32_t low = 1UL << (pinBit + 16);
    const uint32_t savedPrimask = __get_PRIMASK();
    __disable_irq();
    uint32_t edge = DWT->CYCCNT;
    gpio->BSRR = low;  // start bit
    edge += bitTicks;
    while ((int32_t)(DWT->CYCCNT - edge) < 0) {
    }
    for (uint8_t i = 0; i < 8;++i) {
      gpio->BSRR = (b & (1U << i)) ? high: low;
      edge += bitTicks;
      while ((int32_t)(DWT->CYCCNT - edge) < 0) {
      }
    }
    gpio->BSRR = high;  // stop bit and idle
    edge += bitTicks;
    while ((int32_t)(DWT->CYCCNT - edge) < 0) {
    }
    if (!savedPrimask) __enable_irq();
#else
    // TMCStepper 每字节调用 write()。保持 TE 覆盖整帧，避免字节间总线掉电平。
    // 读帧 4 字节，写帧 8 字节；第 3 字节的 bit7 是写标志。
    if (txFramePos == 0) {
      uart->CR1 &= ~USART_CR1_RE;
      uart->CR1 |= USART_CR1_TE;
    }
    if (txFramePos == 2) txFrameLen = (b & 0x80) ? 8: 4;
    while (!(uart->SR & USART_SR_TXE)) {
    }
    uart->DR = b;
    ++txFramePos;
    if (txFrameLen && txFramePos == txFrameLen) {
      while (!(uart->SR & USART_SR_TC)) {
      }
      uart->CR1 &= ~USART_CR1_TE;  // 回复阶段释放总线
      uart->CR1 |= USART_CR1_RE;
      txFramePos = txFrameLen = 0;
    }
#endif
    txCount++;
    return 1;
  }
  size_t write(const uint8_t * b, size_t n) {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
  int peek(void) {
    return - 1;
  }  // 当前仅供 TMCStepper 使用；它不调用 peek()
  void flush(void) {
#if !TMC_GPIO_WRITE_ONLY
    while (!(uart->SR & USART_SR_TC)) {
    }
#endif
  }
  using Print::write;
};

static HalfDuplexSerial rearTmcUart(USART1, GPIOA, 9);  // 后轮 PA9/USART1
static HalfDuplexSerial frontTmcUart(USART2, GPIOA, 2);  // 前轮 PA2/USART2
static TMC2209Stepper rearTmc[4] = {
  { & rearTmcUart, R_SENSE, 0 },
  { & rearTmcUart, R_SENSE, 1 },
  { & rearTmcUart, R_SENSE, 2 },
  { & rearTmcUart, R_SENSE, 3 }
};

static TMC2209Stepper frontTmc[4] = {
  { & frontTmcUart, R_SENSE, 0 },
  { & frontTmcUart, R_SENSE, 1 },
  { & frontTmcUart, R_SENSE, 2 },
  { & frontTmcUart, R_SENSE, 3 }
};

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
HardwareTimer motor1Timer(TIM2);  // 32-bit STEP edge timer, PA6 unchanged
HardwareTimer motor2Timer(TIM5);  // 32-bit STEP edge timer, PA8 unchanged
float motor1LastCommandHz = 0.0f;
float motor2LastCommandHz = 0.0f;
// 电机安装方向
bool motor1Invert = false;
bool motor2Invert = true;  // MKS TMC2209 V2.0 后轮与前轮方向相反：只翻转后轮 DIR 极性
// 编码器方向
int encoder1Direction = 1;
int encoder2Direction = - 1;  // V10 实机确认值：编码器2方向反（8/21 实测 enc1/enc2 计数符号相反）
// ============================================================
// 编码器 + 多源速度Kalman
// ============================================================
volatile int32_t encoder1Count = 0;
volatile int32_t encoder2Count = 0;

namespace control {

// StepOutput 定义在 control/motor.h；这里给出两个实例
StepOutput step1(& motor1Timer, 1UL << 6);
StepOutput step2(& motor2Timer, 1UL << 8);

// ============================================================
// 电机停止
// 只pause，不修改STEP pinMode
// 后轮 PA9/USART1 等 TMC 细节保持不变
// ============================================================
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
void stepEdge(StepOutput & out) {
  TIM_TypeDef * tim = out.timer->getHandle()->Instance;
  if (!out.running) return;
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
void motor1EdgeISR() {
  stepEdge(step1);
}
void motor2EdgeISR() {
  stepEdge(step2);
}
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
void commandStepOutput(StepOutput & out, uint32_t dirPin, bool invert, float commandHz) {
  if (!isfinite(commandHz) || fabsf(commandHz) < MOTOR_STOP_THRESHOLD_HZ) {
    stopStepOutput(out);
    return;
  }
  bool dir = (commandHz >= 0.0f) != invert;
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
void stopMotor1() {
  stopStepOutput(step1);
  motor1LastCommandHz = 0.0f;
}
void stopMotor2() {
  stopStepOutput(step2);
  motor2LastCommandHz = 0.0f;
}
void stopBothMotors() {
  stopMotor1();
  stopMotor2();

}
// ============================================================
// Motor1
// ============================================================
void setMotor1(float commandHz) {
  commandStepOutput(step1, MOTOR1_DIR_PIN, motor1Invert, commandHz);
  motor1LastCommandHz = step1.running ? copysignf(fminf(fabsf(commandHz), MAX_STEP_HZ), commandHz): 0.0f;
}
// ============================================================
// Motor2
// ============================================================
void setMotor2(float commandHz) {
  commandStepOutput(step2, MOTOR2_DIR_PIN, motor2Invert, commandHz);
  motor2LastCommandHz = step2.running ? copysignf(fminf(fabsf(commandHz), MAX_STEP_HZ), commandHz): 0.0f;
}

// ============================================================
// 电机合成：直立输出 ± 转向输出（原 loop 中的两行，算式不变）
// ============================================================
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
void encoder1ISR() {
  if (digitalRead(ENCODER1_A_PIN) == digitalRead(ENCODER1_B_PIN)) {
    encoder1Count += encoder1Direction;
  } else {
    encoder1Count -= encoder1Direction;
  }

}
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
void initEncoders() {
  pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER2_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2ISR, CHANGE);

}

}  // namespace control
