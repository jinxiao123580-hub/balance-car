#pragma once
// ============================================================
// control/motor.{h,cpp} — STEP 频率输出 / 零保持 / 换向软斜坡 / 编码器 ISR
// 以及 TMC2209 双板 UART 初始化（只在非独立 STEP/DIR 模式下编译）。
// ============================================================
#include <Arduino.h>
#include <HardwareTimer.h>

#include "core/cfg.h"
#include "core/state.h"
#include "core/util.h"

namespace control {

// 1MHz 32-bit timers schedule GPIO STEP edges; no PWM preload latency.
// StepOutput：一路 STEP 输出的运行时状态（一个实例 = 一个电机 + 一个 32 位定时器）。
// 定时器配成 1MHz 计数（1 tick = 1µs），中断里只翻转 GPIO，不占 PWM/比较通道，避免预装载延迟；
// 频率由 periodTicks（µs）决定：f_Hz = 1e6 / periodTicks。
// 边界：running=false 时 ISR 直接返回（不发脉冲）；periodTicks 最小 1（上限 MAX_STEP_HZ 已在 setMotor 夹紧）。
struct StepOutput {
// timer：底层定时器指针（step1→TIM2 / step2→TIM5，均 32 位）。
  HardwareTimer * timer;
// mask：STEP 引脚在 GPIOA->BSRR 中的位掩码（step1=1<<6/PA6，step2=1<<8/PA8）。
  uint32_t mask;
// periodTicks：STEP 周期 ｜ 单位 µs（1MHz tick）｜ 默认 1000000（=1Hz 空闲值）｜ 写：commandStepOutput 400Hz。
  volatile uint32_t periodTicks = 1000000;
// risingEdges：累计上升沿数（诊断量，只增不减；command ! 回显）｜ 单位 个。
  volatile uint32_t risingEdges = 0;
// running：定时器是否正在输出脉冲 ｜ volatile bool ｜ 写：commandStepOutput/stopStepOutput。
  volatile bool running = false;
// high：当前 STEP 电平（true=高）｜ volatile bool ｜ 由 stepEdge ISR 翻转、stopStepOutput 清。
  volatile bool high = false;
// dir：当前 DIR 逻辑电平（已含 invert 补偿）｜ bool ｜ 写：commandStepOutput，用于判断是否需要换向。
  bool dir = false;
// 构造：只保存定时器指针与 STEP 位掩码；定时器参数由 initMotors() 配置。
  StepOutput(HardwareTimer * t, uint32_t m): timer(t), mask(m) {
  }
};

// step1/step2：两路 STEP 输出实例（定义在 control/motor.cpp），分别对应 Motor1/Motor2。
// 读：panel/cmd.cpp 的 ! 诊断（risingEdges/periodTicks）。
extern StepOutput step1;
extern StepOutput step2;

// ---- 初始化 ---------------------------------------------------
// initMotors：初始化 4 个电机引脚（DIR/STEP 输出、初电平低）与 TIM2/TIM5：
//   分频到 1MHz、ARR=1e6、关闭预装载、绑定边沿中断，复位后先停输出。调用：main setup() 一次。
void initMotors();
// initEncoders：编码器 4 脚设内部上拉，把 A 相挂 CHANGE 中断（encoder1ISR/encoder2ISR）。
//   调用：main setup() 一次；边界：A 相边沿频率决定 ISR 负载，本车 <100kHz 无压力。
void initEncoders();

// ---- TMC2209 UART 配置（TMC_STANDALONE_STEP_DIR=1 时整段不编译）----
// ---- TMC2209 UART 配置：TMC_STANDALONE_STEP_DIR=1（当前）时整段不编译，不影响 Flash/RAM ----
#if !TMC_STANDALONE_STEP_DIR
// 轻量半双工 Stream：USART 半双工单线 (HDSEL=1)，TMC2209 PDN_UART 用
// HalfDuplexSerial：给 TMCStepper 用的轻量 Stream，实现 USART 半双工单线（HDSEL=1）收发。
// 作用：把 TMC2209 的 PDN_UART 当串口用；TMCStepper 每字节调 write()，读帧/写帧长度由第 3 字节 bit7 判定。
// 边界：rxErrors 记录 FE/NE/ORE；本工程 TMC_GPIO_WRITE_ONLY=1 时走 GPIO 位翻转只写分支，read() 恒 -1。
class HalfDuplexSerial: public Stream {
  protected:
// uart/gpio/pinBit：底层外设与引脚（构造时传入，begin() 按 TMC_GPIO_WRITE_ONLY 选 USART 或 GPIO 模式）。
  USART_TypeDef * uart;
  GPIO_TypeDef * gpio;
  uint8_t pinBit;
  public:
// rxCount/txCount/rxErrors：收发与错误计数（诊断用；initTmcDrivers 失败时打印）。
  uint32_t rxCount = 0;
  uint32_t txCount = 0;
  uint32_t rxErrors = 0;
// lastRx：最后一个收到的字节（诊断）。
  uint8_t lastRx = 0;
// txFramePos/txFrameLen：当前帧已发字节数/帧长（4=读帧，8=写帧）；写帧第 3 字节 bit7=1。
  uint8_t txFramePos = 0;
  uint8_t txFrameLen = 0;
// bitTicks：GPIO 位翻转模式下每比特的 CPU 周期数（=FCLK/baud，四舍五入）。
  uint32_t bitTicks = 0;
// 构造：只保存 USART/GPIO/引脚位号，不做硬件初始化；真正的引脚/波特率配置在 begin()。
// 边界：pinBit 用 GPIO 位号（0~15），不是 Arduino 板级编号。
  HalfDuplexSerial(USART_TypeDef * u, GPIO_TypeDef * g, uint8_t b): uart(u), gpio(g), pinBit(b) {
  }
// begin：初始化单线半双工 UART/GPIO。参数 baud=115200（TMC2209 固定）。返回 void。
// 两种实现：#if TMC_GPIO_WRITE_ONLY → 普通推挽 GPIO + DWT 循环计时只写；#else → USART HDSEL 开漏半双工。
// 边界：只开 RE 不开中断，读由 available()/read() 轮询；GPIO 模式下 DWT 周期计数器做位时间基准。
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
// available：返回可读字节数（0/1）。GPIO 只写模式下恒 0。
  int available(void) {
#if TMC_GPIO_WRITE_ONLY
    return 0;
#else
    return (uart->SR & USART_SR_RXNE) ? 1: 0;
#endif
  }
// read：读 1 字节；无数据返回 -1；GPIO 只写模式恒 -1。副作用：累加 rxErrors/rxCount。
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
// write(uint8_t)：发 1 字节，返回 1。
// GPIO 只写模式：8N1、LSB first，逐位 BSRR 翻转并用 DWT 忙等，单字节关中断保位时间准确。
// USART 模式：维护 TE/RE 切换，整帧发送完才释放总线（字节间不掉电平）。
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
// write(buf,n)：逐字节转发到 write(uint8_t)，返回 n（TMCStepper 用）。
  size_t write(const uint8_t * b, size_t n) {
    for (size_t i = 0; i < n; i++) write(b[i]);
    return n;
  }
// peek：恒返回 -1（TMCStepper 不调用；仅为实现 Stream 接口）。
  int peek(void) {
    return - 1;
  }  // 当前仅供 TMCStepper 使用；它不调用 peek()
// flush：等待 USART 发送完成（TC）；GPIO 模式无缓冲，直接返回。
  void flush(void) {
#if !TMC_GPIO_WRITE_ONLY
    while (!(uart->SR & USART_SR_TC)) {
    }
#endif
  }
// 引入 Print::write 重载，避免隐藏基类方法。
  using Print::write;
};

// 后轮/前轮 TMC UART 实例（定义在 control/motor.cpp）：rear=USART1/PA9，front=USART2/PA2。
extern HalfDuplexSerial rearTmcUart;   // 后轮 PA9/USART1
extern HalfDuplexSerial frontTmcUart;  // 前轮 PA2/USART2

// initTmcDrivers：两片 TMC2209 逐个探测地址 0~3、读 version==0x21 后写寄存器并回读校验。
// 返回 true=全部配置成功；false 时 main setup() 会打印诊断并重试（当前模式不编译本函数）。
bool initTmcDrivers();
// initTmcDriversWriteOnly：只写不读回的实验配置；rearLineHigh/frontLineHigh 回传总线空闲电平。
// 返回 true=写入完成（不代表配置生效，日志标记 UNVERIFIED）；当前模式不编译。
bool initTmcDriversWriteOnly(bool & rearLineHigh, bool & frontLineHigh);
#endif

// ---- 电机命令 -------------------------------------------------
// stopMotor1：停 Motor1 的 STEP 定时器并清零 motor1LastCommandHz ｜ 无参/无返回 ｜
//   调用：故障/倒地/IMU 失效/欠压等停机路径（400Hz 任一拍）。
void stopMotor1();
// stopMotor2：同上，作用对象 Motor2 ｜ 调用：同上停机路径；也有“只停单侧”的历史用法。
void stopMotor2();
// stopBothMotors：同时停两路（倒地/IMU 失效/掉拍等统一入口）｜ 无参/无返回。
void stopBothMotors();
// setMotor1：下发 Motor1 带符号 STEP 频率命令 ｜ 参数 commandHz 单位 Hz（正/负=转向）｜ 无返回 ｜
//   内部经 commandStepOutput：NaN/|Hz|<1 停、|Hz|>MAX_STEP_HZ 限幅；随后记录实际命令到 motor1LastCommandHz ｜ 400Hz。
void setMotor1(float commandHz);
// setMotor2：同上，作用对象 Motor2（motor2Invert 已补偿后轮方向）｜ 调用：mixMotors 每拍 400Hz。
void setMotor2(float commandHz);

// 直立输出 ± 转向输出 → 左右电机命令（原 loop 中的电机合成，逻辑逐字不变）
// mixMotors：把“直立共模 finalBalanceOutput”与“转向差模 turnOffsetHz”合成两轮命令：
//   Motor1 = final + turn，Motor2 = final - turn（单位均 Hz）。调用：main loop 每拍 400Hz。
// 边界：两轮各自会再经过 setMotor 的停止门限/限幅；转向使一侧反向属正常差动。
void mixMotors(float turnOffsetHz);

}  // namespace control
