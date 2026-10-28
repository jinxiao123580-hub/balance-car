// ============================================================
// TMC2209 spreadCycle(SC) 读回验证测试
// 目标: 配置 TMC 进 spreadCycle, 然后【读回】寄存器确认真正生效
// 关键: 不用 write-only, 走真正的 USART 半双工读回
//       version==0x21, IFCOUNT 递增, GCONF.en_spreadCycle==1
// 接线: 按编译开关选择测 PA2 或 PA9 (USART1/USART2)
// 结果: 经蓝牙 USART3 周期打印读回明细；本诊断不输出 STEP，防止车辆意外移动
// 依赖: TMCStepper 库
// ============================================================
#include <Arduino.h>
#include <TMCStepper.h>

// 选择测哪根线: 1=PA2(USART2, 前轮位), 0=PA9(USART1, 后轮位)
#define TEST_PA2 0

#if TEST_PA2
  #define TMC_UART_USART  USART2
  #define TMC_UART_GPIO   GPIOA
  #define TMC_UART_BIT    2   // PA2
  #define TMC_GPIO_PORT   GPIOA
#else
  #define TMC_UART_USART  USART1
  #define TMC_UART_GPIO   GPIOA
  #define TMC_UART_BIT    9   // PA9
  #define TMC_GPIO_PORT   GPIOA
#endif

#define R_SENSE 0.11f

// 模组式的半双工 Stream (HDSEL 读回版)
class HalfDuplexSerial : public Stream {
  protected:
    USART_TypeDef *uart;
    GPIO_TypeDef *gpio;
    uint8_t pinBit;
  public:
    uint32_t rxCount = 0, txCount = 0, rxErrors = 0;
    uint8_t txFramePos = 0, txFrameLen = 0;
    uint8_t lastRx = 0;
    HalfDuplexSerial(USART_TypeDef *u, GPIO_TypeDef *g, uint8_t b) : uart(u), gpio(g), pinBit(b) {}
    void begin(uint32_t baud) {
      rxCount = txCount = rxErrors = 0;
      txFramePos = txFrameLen = 0;
      RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
      (void)RCC->AHB1ENR;
      gpio->MODER &= ~(3UL << (2*pinBit));
      gpio->MODER |=  (2UL << (2*pinBit));        // AF
      gpio->OTYPER &= ~(1UL << pinBit);            // 发送时推挽高电平；TE 关闭后释放总线
      gpio->PUPDR  &= ~(3UL << (2*pinBit));
      gpio->PUPDR  |=  (1UL << (2*pinBit));       // 上拉空闲高
      gpio->OSPEEDR |= 3UL << (2*pinBit);
      if (pinBit < 8) { gpio->AFR[0] &= ~(0xFUL << (4*pinBit)); gpio->AFR[0] |= (7UL << (4*pinBit)); }
      else { uint8_t b=pinBit-8; gpio->AFR[1] &= ~(0xFUL << (4*b)); gpio->AFR[1] |= (7UL << (4*b)); }
      if (uart == USART1) RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
      else if (uart == USART2) RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
      uint32_t fck = (uart == USART1) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
      uart->CR1 = 0; uart->CR2 = 0;
      uart->CR3 = USART_CR3_HDSEL;
      uart->BRR = (fck + baud/2) / baud;
      uart->CR1 = USART_CR1_UE | USART_CR1_RE;
    }
    int available(void) { return (uart->SR & USART_SR_RXNE) ? 1 : 0; }
    int read(void) {
      uint32_t s = uart->SR;
      if (!(s & USART_SR_RXNE)) return -1;
      rxErrors |= s & (USART_SR_FE | USART_SR_NE | USART_SR_ORE);
      lastRx = (uint8_t)uart->DR; rxCount++;
      return lastRx;
    }
    size_t write(uint8_t b) {
      if (txFramePos == 0) { uart->CR1 &= ~USART_CR1_RE; uart->CR1 |= USART_CR1_TE; }
      if (txFramePos == 2) txFrameLen = (b & 0x80) ? 8 : 4;
      while (!(uart->SR & USART_SR_TXE)) {}
      uart->DR = b;
      ++txFramePos;
      ++txCount;
      if (txFrameLen && txFramePos == txFrameLen) {
        while (!(uart->SR & USART_SR_TC)) {}
        uart->CR1 &= ~USART_CR1_TE;
        uart->CR1 |= USART_CR1_RE;
        txFramePos = txFrameLen = 0;
      }
      return 1;
    }
    size_t write(const uint8_t *b, size_t n) { for (size_t i=0;i<n;i++) write(b[i]); return n; }
    int peek(void) { return (uart->SR & USART_SR_RXNE) ? (int)(uart->DR & 0xFF) : -1; }
    void flush(void) { while (!(uart->SR & USART_SR_TC)) {} }
    using Print::write;
};

static HalfDuplexSerial tmcUart(TMC_UART_USART, TMC_UART_GPIO, TMC_UART_BIT);
TMC2209Stepper drivers[4] = {
  {&tmcUart, R_SENSE, 0}, {&tmcUart, R_SENSE, 1},
  {&tmcUart, R_SENSE, 2}, {&tmcUart, R_SENSE, 3}
};
int detectedAddress = -1;
bool configured = false;

// 蓝牙串口 USART3 (PB10=TX / PB11=RX), 9600
#define BT_RX_PIN PB11
#define BT_TX_PIN PB10
HardwareSerial BTSerial(BT_RX_PIN, BT_TX_PIN);
const uint32_t BT_BAUD = 9600;

void setup() {
  // 本测试只访问 UART，不驱动电机 STEP；避免原测试上电自动转两圈。
  pinMode(PA6, OUTPUT);
  pinMode(PA8, OUTPUT);
  digitalWrite(PA6, LOW);
  digitalWrite(PA8, LOW);
  BTSerial.begin(BT_BAUD);
  delay(100);
  tmcUart.begin(115200);
}

void loop() {
  // 周期输出，蓝牙晚连接也能收到；未识别时不写寄存器。
  if (detectedAddress < 0) {
    for (int address = 0; address < 4; ++address) {
      if (drivers[address].version() == 0x21) {
        detectedAddress = address;
        break;
      }
    }
  }

#if TEST_PA2
  BTSerial.print("TMC READBACK PA2 ");
#else
  BTSerial.print("TMC READBACK PA9 ");
#endif
  BTSerial.print("address="); BTSerial.print(detectedAddress);
  BTSerial.print(" rx="); BTSerial.print(tmcUart.rxCount);
  BTSerial.print(" err=0x"); BTSerial.println(tmcUart.rxErrors, HEX);

  if (detectedAddress >= 0) {
    TMC2209Stepper &driver = drivers[detectedAddress];
    if (!configured) {
      uint8_t before = driver.IFCNT();
      BTSerial.print("BEFORE GCONF=0x"); BTSerial.print(driver.GCONF(), HEX);
      BTSerial.print(" CHOPCONF=0x"); BTSerial.print(driver.CHOPCONF(), HEX);
      BTSerial.print(" DRV_STATUS=0x"); BTSerial.println(driver.DRV_STATUS(), HEX);
      // 与平衡车固件相同的配置顺序。
      driver.toff(0);
      driver.begin();
      driver.I_scale_analog(false);
      driver.en_spreadCycle(true);
      driver.rms_current(1200);
      driver.microsteps(16);
      driver.toff(3);
      uint8_t after = driver.IFCNT();
      BTSerial.print("IFCNT before="); BTSerial.print(before);
      BTSerial.print(" after="); BTSerial.print(after);
      BTSerial.print(" delta="); BTSerial.println((uint8_t)(after - before));
      configured = true;
    }
    BTSerial.print("AFTER version=0x"); BTSerial.print(driver.version(), HEX);
    BTSerial.print(" GCONF.SC="); BTSerial.print(driver.en_spreadCycle());
    BTSerial.print(" MSTEP_REG="); BTSerial.print(driver.mstep_reg_select());
    BTSerial.print(" TOFF="); BTSerial.print(driver.toff());
    BTSerial.print(" microsteps="); BTSerial.print(driver.microsteps());
    BTSerial.print(" STATUS.stealth="); BTSerial.println(driver.stealth());
  }
  delay(2000);
}
