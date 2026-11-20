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
struct StepOutput {
  HardwareTimer * timer;
  uint32_t mask;
  volatile uint32_t periodTicks = 1000000;
  volatile uint32_t risingEdges = 0;
  volatile bool running = false;
  volatile bool high = false;
  bool dir = false;
  StepOutput(HardwareTimer * t, uint32_t m): timer(t), mask(m) {
  }
};

extern StepOutput step1;
extern StepOutput step2;

// ---- 初始化 ---------------------------------------------------
void initMotors();
void initEncoders();

// ---- TMC2209 UART 配置（TMC_STANDALONE_STEP_DIR=1 时整段不编译）----
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

extern HalfDuplexSerial rearTmcUart;   // 后轮 PA9/USART1
extern HalfDuplexSerial frontTmcUart;  // 前轮 PA2/USART2

bool initTmcDrivers();
bool initTmcDriversWriteOnly(bool & rearLineHigh, bool & frontLineHigh);
#endif

// ---- 电机命令 -------------------------------------------------
void stopMotor1();
void stopMotor2();
void stopBothMotors();
void setMotor1(float commandHz);
void setMotor2(float commandHz);

// 直立输出 ± 转向输出 → 左右电机命令（原 loop 中的电机合成，逻辑逐字不变）
void mixMotors(float turnOffsetHz);

}  // namespace control
