# 固件源码逐行注释版

> 说明：本文件把 `software/firmware/src/` 的代码按原文件顺序完整列出，每行代码后面跟一句中文解释。
> 空行和整行纯注释已省略（它们不是代码；其中说明性内容并入相邻行的解释）；行号保留原文件的物理行号，方便回查。
> 被大量空行拆开的语句仍按源码原样分行列出，因此会看到 `if (` / 条件 / `)` 这样分开的行。

## main.cpp（L1–L5838）

```cpp
L1  #include <Arduino.h>  // 引入 Arduino 基础框架，提供引脚、延时、millis 等接口
L3  #include <Wire.h>  // 引入 Wire 库，BMI270 通过 I2C 总线读写
L5  #include <SparkFun_BMI270_Arduino_Library.h>  // 引入 SparkFun BMI270 驱动库，读取加速度与陀螺原始数据
L7  #include <HardwareTimer.h>  // 引入硬件定时器库，用于 TIM2/TIM5 产生 STEP 脉冲
L9  #include <math.h>  // 引入数学库，后面要用到开方、取绝对值等函数
L10  #include "manual_motion.h"  // 引入手动运动模块头文件，提供手动通路控制类
L11  #include "position_jog.h"  // 引入位置点动模块头文件，提供位置点动的类定义
L12  #include "motion_drive.h"  // 引入运动驱动模块头文件，提供速度、位置、倾角外环类
L13  PositionOdometry positionOdom;  // 定义里程计对象，累计编码器里程供位置环使用
L14  PositionJog positionJog;  // 定义位置点动控制器实例，处理点动挪车动作
L16  MotionSpeedProfile motionSpeedProfile;  // 定义速度曲线对象，把操作量换算成目标速度
L17  MotionLeanControl motionLean;  // 定义前倾控制对象，把操作量换算成目标倾角
L18  MotionPositionPlan motionPlan;  // 定义预测位置环对象，X=0 时输出恒 0 行为不变
L19  NetSpeedEstimator motionNetSpeed;  // 定义 200ms 净速度估计器，供前后运动外环专用
L20  bool manualMotionWasBusy = false;  // 记录上一拍手动通路是否忙碌，用于判断状态切换
L21  PositionSpeedReference positionSpeedReference;  // 定义位置环速度参考对象，让目标速度平滑不突变
L22  PositionLeanControl positionLean;  // 定义位置环倾角控制实例，配合位置外环使用
L23  ManualTurnControl manualTurn;  // 定义手动转向控制器，负责处理转向命令
L24  ManualPitchControl manualForward;  // 定义手动前后控制器，负责处理前后移动命令
L27  #define TMC_STANDALONE_STEP_DIR 1  // 置 1 表示 TMC2209 走独立 STEP/DIR，不编译 UART 配置
L28  #if !TMC_STANDALONE_STEP_DIR  // TMC UART 配置代码块的总开关，当前条件为假不编译
L29  #include <TMCStepper.h>  // 仅 UART 模式下才引入 TMCStepper 驱动库
L30  #endif  // 结束 TMC UART 头文件的条件编译块
L77  #define MOTOR1_DIR_PIN   PA7  // 定义电机1 方向引脚为 PA7，本版固定不修改
L79  #define MOTOR1_STEP_PIN  PA6  // 定义电机1 脉冲引脚为 PA6，原设计对应 TIM3_CH1
L85  #define MOTOR2_DIR_PIN   PB0  // 定义电机2 方向引脚为 PB0，本版固定不修改
L87  #define MOTOR2_STEP_PIN  PA8  // 定义电机2 脉冲引脚为 PA8，原设计对应 TIM1_CH1
L93  #define BMI_SCL_PIN      PB8  // 定义 BMI270 的 I2C 时钟引脚为 PB8
L95  #define BMI_SDA_PIN      PB9  // 定义 BMI270 的 I2C 数据引脚为 PB9
L101  #define OLED_SDA_PIN     PF7  // 保留 OLED 数据脚定义，本版完全不用 OLED
L103  #define OLED_SCL_PIN     PF9  // 保留 OLED 时钟脚定义，本版完全不用 OLED
L105  #define OLED_VCC_PIN     PC0  // 保留 OLED 电源脚定义，本版完全不用 OLED
L107  #define OLED_GND_PIN     PC2  // 保留 OLED 地脚定义，本版完全不用 OLED
L113  #define ENCODER1_A_PIN   PA0  // 定义编码器1 的 A 相输入脚为 PA0
L115  #define ENCODER1_B_PIN   PA1  // 定义编码器1 的 B 相输入脚为 PA1
L121  #define ENCODER2_A_PIN   PB6  // 定义编码器2 的 A 相输入脚为 PB6
L123  #define ENCODER2_B_PIN   PB7  // 定义编码器2 的 B 相输入脚为 PB7
L142  #define BT_RX_PIN PB11  // 定义蓝牙串口接收脚 PB11，对应 USART3_RX
L144  #define BT_TX_PIN PB10  // 定义蓝牙串口发送脚 PB10，对应 USART3_TX
L148  HardwareSerial BTSerial(  // 开始构造蓝牙串口对象，构造参数分多行书写
L150  BT_RX_PIN,  // 构造参数一：接收脚 PB11 放在前
L152  BT_TX_PIN  // 构造参数二：发送脚 PB10
L154  );  // 多行括号收尾，蓝牙串口对象构造到此结束
L158  const uint32_t BT_BAUD = 9600;  // 定义蓝牙波特率 9600，与 JDY-23 出厂默认一致
L167  #if !TMC_STANDALONE_STEP_DIR  // 开始 TMC UART 专属代码区，当前整段不参与编译
L168  #define R_SENSE 0.11f  // 定义 TMC 载板采样电阻 0.11Ω，用于电流换算
L169  #define TMC_GPIO_WRITE_ONLY 1  // 置 1 走只写模式：不发读帧，只单向写寄存器
L172  const bool TMC_FRONT_ACTIVE = false;  // 前轮当前为 DRV8825，不参与 TMC UART 配置
L175  class HalfDuplexSerial : public Stream {  // 定义半双工单线串口类，继承 Arduino 的 Stream 基类
L176  protected:  // 以下为受保护成员，仅供本类内部及其派生类使用
L177  USART_TypeDef *uart;  // 保存底层 USART 寄存器基址，便于直接操作外设
L178  GPIO_TypeDef *gpio;  // 保存 PDN_UART 所在的 GPIO 端口指针
L179  uint8_t pinBit;  // 保存该引脚在端口内的位号，用于位操作
L180  public:  // 以下为公开成员，供外部读取状态和收发数据
L181  uint32_t rxCount = 0;  // 统计接收字节数，用于判断串口是否真的收到回复
L182  uint32_t txCount = 0;  // 统计发送字节数，确认配置帧是否真的发出
L183  uint32_t rxErrors = 0;  // 累计接收错误标志，区分帧错、噪声与溢出
L184  uint8_t lastRx = 0;  // 记录最后收到的字节，便于判断地址是否匹配
L185  uint8_t txFramePos = 0;  // 记录当前帧内字节位置，用于识别帧首帧尾
L186  uint8_t txFrameLen = 0;  // 记录本帧总长度，读帧 4 字节写帧 8 字节
L187  uint32_t bitTicks = 0;  // 记录每位占用的 CPU 周期数，供软件模拟时序
L188  HalfDuplexSerial(USART_TypeDef *u, GPIO_TypeDef *g, uint8_t b)  // 构造函数声明，接收 USART 外设、GPIO 端口与位号
L189  : uart(u), gpio(g), pinBit(b) {}  // 初始化列表把 USART、GPIO、位号三个成员直接赋初值
L190  void begin(uint32_t baud) {  // 配置半双工串口的引脚复用与波特率，函数体开始
L191  rxCount = txCount = rxErrors = 0;  // 清零收发计数与错误计数，保证每次初始化从零统计
L192  lastRx = 0;  // 清零最后接收字节，避免沿用上次的旧值
L193  txFramePos = txFrameLen = 0;  // 清零帧内位置与帧长状态，准备接收新的帧
L195  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // 打开 GPIOA 时钟，因为后面要配置 PA9 或 PA2
L196  (void)RCC->AHB1ENR;  // 回读寄存器一次，确保时钟使能真正生效
L197  #if TMC_GPIO_WRITE_ONLY  // 只写模式下走普通 GPIO 加软件时序，不用 USART
L200  gpio->BSRR = 1UL << pinBit;  // 先把引脚拉高，模拟串口空闲时的高电平状态
L201  gpio->MODER = (gpio->MODER & ~(3UL << (2*pinBit))) | (1UL << (2*pinBit));  // 把引脚模式设为通用输出，准备软件翻转电平
L202  gpio->OTYPER &= ~(1UL << pinBit);  // 设为推挽输出而非开漏，保证高电平有力
L203  gpio->PUPDR = (gpio->PUPDR & ~(3UL << (2*pinBit))) | (1UL << (2*pinBit));  // 使能内部上拉，空闲时把线路维持在高电平
L204  gpio->OSPEEDR |= 3UL << (2*pinBit);  // 把引脚翻转速度设到最高，保证位时序精度
L205  bitTicks = (HAL_RCC_GetHCLKFreq() + baud / 2) / baud;  // 按主频和波特率算出每一位占用的周期数
L206  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // 使能 DWT 跟踪单元，后面用周期计数器定位时
L207  DWT->CYCCNT = 0;  // 清零周期计数器，把它当作位定时的零点
L208  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;  // 启动周期计数器，软件模拟发送才能开始计时
L209  return;  // 只写模式配置到此结束，直接返回跳过 USART 配置
L210  #else  // 否则走真正的 USART 半双工配置分支
L211  gpio->MODER   &= ~(3UL << (2*pinBit));  // 先清掉该引脚的模式位，准备重新设置
L212  gpio->MODER   |=  (2UL << (2*pinBit));  // 把引脚设为复用功能模式，交给 USART 使用
L213  gpio->OTYPER  |=  (1UL << pinBit);  // 单线收发用开漏并配合上下拉，回复时释放总线
L214  gpio->PUPDR   &= ~(3UL << (2*pinBit));  // 先清掉该引脚的上下拉配置位
L215  gpio->PUPDR   |=  (1UL << (2*pinBit));  // 设成上拉，空闲为高电平，长线建议外部再加拉
L216  gpio->OSPEEDR |=  (3UL << (2*pinBit));  // 把引脚翻转速度设到最高档，减小边沿延迟
L217  if (pinBit < 8) { gpio->AFR[0] &= ~(0xFUL << (4*pinBit)); gpio->AFR[0] |= (7UL << (4*pinBit)); }  // 位号小于 8 时把 AFR[0] 对应四位设成 AF7
L218  else         { uint32_t b=pinBit-8; gpio->AFR[1] &= ~(0xFUL << (4*b)); gpio->AFR[1] |= (7UL << (4*b)); }  // 位号大于等于 8 时用 AFR[1] 的对应四位设 AF7
L219  if (uart == USART1) RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // 若使用 USART1 则打开其挂载的 APB2 时钟
L220  else if (uart == USART2) RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // 若使用 USART2 则打开其挂载的 APB1 时钟
L221  uint32_t fck = (uart == USART1) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();  // 读取该 USART 所在总线的实际时钟频率供算分频
L222  uart->CR1 = 0;  // 先清 CR1，把串口关掉再重新配置
L223  uart->CR2 = 0;  // 清 CR2，停止位等参数保持默认设置
L224  uart->CR3 = USART_CR3_HDSEL;  // 置半双工单线模式位，让 TX 与 RX 共用一根线
L225  uart->BRR = (fck + baud / 2) / baud;  // 按十六倍过采样算出波特率分频值写入 BRR
L226  uart->CR1 = USART_CR1_UE | USART_CR1_RE;  // 使能串口并打开接收，准备接收驱动回复
L227  #endif  // 结束只写与半双工两种配置分支的条件编译
L228  }  // begin 配置函数体结束
L229  int available(void) {  // 查询是否有可读字节，兼容 Stream 的虚接口
L230  #if TMC_GPIO_WRITE_ONLY  // 只写模式没有任何接收通路，直接返回 0
L231  return 0;  // 只写模式报可用字节为 0，上层不会来读
L232  #else  // 否则去查 USART 状态寄存器判断是否收到字节
L233  return (uart->SR & USART_SR_RXNE) ? 1 : 0;  // RXNE 置位说明有字节待读，返回 1 否则 0
L234  #endif  // 结束可用字节查询的条件编译
L235  }  // available 查询函数结束
L236  int read(void) {  // 读一个字节，供 TMCStepper 解析驱动回复
L237  #if TMC_GPIO_WRITE_ONLY  // 只写模式下读操作永远失败
L238  return -1;  // 无接收通路，返回 -1 表示当前无数据可读
L239  #else  // 否则走 USART 硬件接收分支
L240  const uint32_t status = uart->SR;  // 先读一次状态寄存器，避免读数据后标志被清掉
L241  if (!(status & USART_SR_RXNE)) return -1;  // 没有 RXNE 就直接返回 -1 告诉上层无数据
L242  rxErrors |= status & (USART_SR_FE | USART_SR_NE | USART_SR_ORE);  // 把帧错、噪声、溢出标志累计进错误计数
L243  lastRx = (uint8_t)uart->DR;  // 读数据寄存器得到字节，并记录为最后接收值
L244  rxCount++;  // 接收字节计数自增，便于诊断实际通信量
L245  return lastRx;  // 返回刚读到的那个字节
L246  #endif  // 结束读字节函数的条件编译
L247  }  // read 读字节函数结束
L248  size_t write(uint8_t b) {  // 发送一个字节，TMCStepper 是逐字节调用它的
L249  #if TMC_GPIO_WRITE_ONLY  // 只写模式下用 GPIO 软件模拟 8N1 时序
L251  const uint32_t high = 1UL << pinBit;  // BSRR 置位掩码，写它把线路拉成高电平
L252  const uint32_t low = 1UL << (pinBit + 16);  // BSRR 复位掩码，写它把线路拉成低电平
L253  const uint32_t savedPrimask = __get_PRIMASK();  // 保存当前中断开关状态，发送完要恢复回去
L254  __disable_irq();  // 关中断，保证位时间不被其他中断打断
L255  uint32_t edge = DWT->CYCCNT;  // 取当前周期数作为这一位的边界时间基准
L256  gpio->BSRR = low;  // 拉低线路发出起始位，串口帧从这里开始
L257  edge += bitTicks;  // 算出起始位结束的时间边界
L258  while ((int32_t)(DWT->CYCCNT - edge) < 0) {}  // 空等循环，一直等到起始位时间走完
L259  for (uint8_t i = 0; i < 8; ++i) {  // 依次发送八个数据位，串口协议是低位在先
L260  gpio->BSRR = (b & (1U << i)) ? high : low;  // 按该位取值把线路拉高或拉低，输出数据位
L261  edge += bitTicks;  // 把边界时间累加一位的时长
L262  while ((int32_t)(DWT->CYCCNT - edge) < 0) {}  // 空等循环，等到这一位的时间结束
L263  }  // 八位数据位循环发送结束
L264  gpio->BSRR = high;  // 把线路拉高，发出停止位并保持空闲高电平
L265  edge += bitTicks;  // 算出停止位结束的时间边界
L266  while ((int32_t)(DWT->CYCCNT - edge) < 0) {}  // 空等循环，等停止位时间走完再收尾
L267  if (!savedPrimask) __enable_irq();  // 原来开着中断的话现在恢复开中断
L268  #else  // 否则走 USART 硬件发送分支
L271  if (txFramePos == 0) {  // 帧首字节时把串口从接收状态切到发送
L272  uart->CR1 &= ~USART_CR1_RE;  // 先关掉接收使能，避免自己发的内容被自己收回
L273  uart->CR1 |= USART_CR1_TE;  // 打开发送使能，开始往总线上发帧
L274  }  // 帧首字节的处理到此结束
L275  if (txFramePos == 2) txFrameLen = (b & 0x80) ? 8 : 4;  // 第三个字节的 bit7 是写标志，据此决定 8 或 4 字节
L276  while (!(uart->SR & USART_SR_TXE)) {}  // 等发送寄存器空出来，否则写 DR 会丢数据
L277  uart->DR = b;  // 把字节写进数据寄存器，由硬件移出到线上
L278  ++txFramePos;  // 帧内字节位置加一，跟踪当前发到第几个字节
L279  if (txFrameLen && txFramePos == txFrameLen) {  // 本帧长度已知且已发到帧尾时做收尾处理
L280  while (!(uart->SR & USART_SR_TC)) {}  // 最后一个字节还没完全移出时继续等
L281  uart->CR1 &= ~USART_CR1_TE;  // 关闭发送使能，进入回复阶段时释放总线
L282  uart->CR1 |= USART_CR1_RE;  // 重新打开接收，等待驱动回传应答数据
L283  txFramePos = txFrameLen = 0;  // 帧位置与帧长清零，准备接收下一帧
L284  }  // 整帧发送收尾分支结束
L285  #endif  // 结束软件模拟与硬件发送两种分支的条件编译
L286  txCount++;  // 发送字节计数自增，用于统计实际通信量
L287  return 1;  // 报告成功发出一个字节
L288  }  // write 发送字节函数结束
L289  size_t write(const uint8_t *b, size_t n) { for (size_t i=0;i<n;i++) write(b[i]); return n; }  // 批量写接口：逐字节调用单字节写，返回写入字节数
L290  int peek(void) { return -1; }  // 不实现预读，返回 -1，TMCStepper 实际不会调用它
L291  void flush(void) {  // 冲刷发送缓冲，保证已写字节真正移出到线上
L292  #if !TMC_GPIO_WRITE_ONLY  // 软件模拟模式下没有硬件缓冲，不需要冲刷
L293  while (!(uart->SR & USART_SR_TC)) {}  // 等发送完成标志置位，说明最后一帧已彻底发完
L294  #endif  // 结束冲刷函数的条件编译
L295  }  // flush 冲刷函数结束
L296  using Print::write;  // 合并基类的 write 重载，避免被子类同名函数隐藏
L297  };  // 半双工串口类定义到此结束
L299  static HalfDuplexSerial rearTmcUart(USART1, GPIOA, 9);  // 后轮 PDN_UART 走 USART1 的 PA9，引脚位号 9
L300  static HalfDuplexSerial frontTmcUart(USART2, GPIOA, 2);  // 前轮 PDN_UART 走 USART2 的 PA2，引脚位号 2
L301  static TMC2209Stepper rearTmc[4] = {  // 后轮竖四个候选驱动对象，逐个试地址以确定真实地址
L302  {&rearTmcUart, R_SENSE, 0}, {&rearTmcUart, R_SENSE, 1},  // 地址 0 与地址 1 的两个候选驱动对象
L303  {&rearTmcUart, R_SENSE, 2}, {&rearTmcUart, R_SENSE, 3}  // 地址 2 与地址 3 的两个候选驱动对象
L304  };  // 后轮候选驱动数组初始化到此结束
L305  static TMC2209Stepper frontTmc[4] = {  // 前轮竖四个候选驱动对象，逐个试地址寻找真实地址
L306  {&frontTmcUart, R_SENSE, 0}, {&frontTmcUart, R_SENSE, 1},  // 前轮地址 0 与地址 1 的两个候选驱动对象
L307  {&frontTmcUart, R_SENSE, 2}, {&frontTmcUart, R_SENSE, 3}  // 前轮地址 2 与地址 3 的两个候选驱动对象
L308  };  // 前轮候选驱动数组初始化到此结束
L310  bool configureTmcDriver(TMC2209Stepper &tmc) {  // 配置一个 TMC2209 并逐项校验寄存器写入是否生效
L311  if (tmc.version() != 0x21) return false;  // 版本号不是 0x21 说明不是 TMC2209，直接放弃
L312  const uint8_t ifcntBefore = tmc.IFCNT();  // 记下写之前的接口计数器，用来确认写入确实生效
L313  tmc.begin();  // 初始化驱动，置 pdn_disable 与寄存器微步选择
L314  tmc.toff(3);  // 打开斩波器并设置 toff 为 3
L315  tmc.en_spreadCycle(true);  // 强制使用 spreadCycle，显式关掉静音 stealthChop
L316  tmc.rms_current(1200);  // 设运行电流 1200mA，前提是载板采样电阻 0.11Ω
L317  tmc.microsteps(16);  // 设 16 微步，UART 寄存器设置优先于 MS1/MS2
L318  const uint8_t ifcntAfter = tmc.IFCNT();  // 记下写之后的接口计数器，用于比对增长量
L319  return (uint8_t)(ifcntAfter - ifcntBefore) >= 5 &&  // 返回条件开头：接口计数器至少涨 5 才算写入生效
L320  tmc.version() == 0x21 && tmc.en_spreadCycle() &&  // 继续核对版本号正确且 spreadCycle 确实打开
L321  tmc.mstep_reg_select() && tmc.toff() == 3 &&  // 继续核对微步来自寄存器且 toff 确实为 3
L322  tmc.microsteps() == 16;  // 最后确认微步数确实等于 16，全部满足才通过
L323  }  // 配置校验函数体结束，调用方据此判断成功或失败
L325  bool findAndConfigureTmc(TMC2209Stepper (&candidates)[4], uint8_t (&versions)[4], uint8_t &address) {  // 在四个候选地址中找出真实地址并完成配置
L326  address = 0xFF;  // 默认地址 0xFF 表示无效，找到后再改写
L327  for (uint8_t i = 0; i < 4; ++i) {  // 依次试地址 0 到 3 四个候选
L328  versions[i] = candidates[i].version();  // 记录该地址读回的版本号，供诊断打印用
L329  if (versions[i] == 0x21 && configureTmcDriver(candidates[i])) {  // 版本号对且配置校验通过，说明这个地址就是它
L330  address = i;  // 记下命中的地址号，后续读写寄存器都用这个地址
L331  return true;  // 找到即返回成功，不再继续试后面的地址
L332  }  // 命中分支的右花括号，结束找到后的处理流程
L333  }  // 地址遍历循环的右花括号，四个候选扫描结束
L334  return false;  // 四个地址都没有命中，返回失败
L335  }  // 寻址配置函数体结束，返回 false 表示四个地址都没命中
L337  bool initTmcDrivers() {  // 上电初始化后轮与可选前轮的 TMC UART 通信
L338  rearTmcUart.begin(115200);  // 后轮 PDN_UART 以 115200 波特率建链
L339  if (TMC_FRONT_ACTIVE) frontTmcUart.begin(115200);  // 前轮确实是 TMC 时才初始化它的 UART
L340  delay(50);  // 等 50ms 让驱动上电稳定再进行通信
L341  uint8_t rearVersions[4], frontVersions[4];  // 暂存两路各四个地址读回的版本号
L342  uint8_t rearAddress, frontAddress;  // 暂存两路最终命中的地址号
L343  const bool rearOk = findAndConfigureTmc(rearTmc, rearVersions, rearAddress);  // 后轮寻址并配置，结果存进后轮成功标志
L344  const bool frontOk = TMC_FRONT_ACTIVE  // 前轮结果用条件表达式，先判断前轮是否为 TMC
L345  ? findAndConfigureTmc(frontTmc, frontVersions, frontAddress)  // 前轮是 TMC 才真正去寻址配置
L346  : true;  // 前轮非 TMC 时直接视为成功，跳过配置
L347  if (!rearOk || !frontOk) {  // 任一路失败就打印一串诊断信息
L348  BTSerial.print("TMC UART FAIL rear=");  // 先把失败标记发到蓝牙，说明哪一路起头
L349  BTSerial.print(rearOk);  // 打印后轮是否配置成功的布尔值
L350  BTSerial.print(" front=");  // 打印前轮结果前的标签
L351  BTSerial.println(frontOk);  // 打印前轮结果并换行，第一条诊断结束
L352  BTSerial.print("TMC DIAG R[0..3]=");  // 打印后轮四个地址版本号的诊断标题
L353  for (uint8_t i = 0; i < 4; ++i) { BTSerial.print(rearVersions[i], HEX); BTSerial.print(' '); }  // 逐个打印后轮四个地址读回的版本号十六进制值
L354  BTSerial.print("F[0..3]=");  // 打印前轮四个地址版本的诊断标题
L355  for (uint8_t i = 0; i < 4; ++i) { BTSerial.print(frontVersions[i], HEX); BTSerial.print(' '); }  // 逐个打印前轮四个地址读回的版本号十六进制值
L356  BTSerial.println();  // 输出一个换行，结束这一行诊断文本
L357  BTSerial.print("TMC RX rear="); BTSerial.print(rearTmcUart.rxCount);  // 打印后轮累计接收字节数，判断是否有回传
L358  BTSerial.print(" front="); BTSerial.print(frontTmcUart.rxCount);  // 打印前轮累计接收字节数
L359  BTSerial.print(" err="); BTSerial.print(rearTmcUart.rxErrors, HEX);  // 打印后轮累计接收错误标志，用十六进制
L360  BTSerial.print('/'); BTSerial.print(frontTmcUart.rxErrors, HEX);  // 斜杠分隔后打印前轮累计接收错误标志
L361  BTSerial.print(" last="); BTSerial.print(rearTmcUart.lastRx, HEX);  // 打印后轮最后收到的字节，看地址是否对
L362  BTSerial.print('/'); BTSerial.print(frontTmcUart.lastRx, HEX);  // 斜杠分隔后打印前轮最后收到的字节
L363  BTSerial.print(" PA9/PA2="); BTSerial.print((GPIOA->IDR >> 9) & 1);  // 直接读 GPIOA 输入寄存器第 9 位看后轮线电平
L364  BTSerial.print('/'); BTSerial.println((GPIOA->IDR >> 2) & 1);  // 斜杠分隔后打印前轮 PA2 线电平并换行
L365  } else {  // 两路都成功时走正常分支，只报命中的地址
L366  BTSerial.print("TMC UART address rear="); BTSerial.print(rearAddress);  // 打印后轮命中的 UART 地址
L367  BTSerial.print(" front="); BTSerial.println(frontAddress);  // 打印前轮命中的地址并换行
L368  }  // 诊断输出分支到此结束
L369  return rearOk && frontOk;  // 只有两路都成功才算初始化通过
L370  }  // TMC 初始化函数结束
L372  #if TMC_GPIO_WRITE_ONLY  // 只写模式专用配置函数区开始，本段当前整体不编译
L373  void configureTmcWriteOnly(TMC2209Stepper (&candidates)[4]) {  // 无读回时对四个候选地址盲写配置，谁匹配谁生效
L375  for (auto &tmc : candidates) {  // 四个可能地址逐个写入配置，实际只有一个会响应
L376  tmc.toff(0);  // 配置阶段先把斩波器关掉，避免中途乱发脉冲
L377  tmc.begin();  // 初始化驱动，置 pdn_disable 与寄存器微步选择
L378  tmc.I_scale_analog(false);  // 运行电流改由 UART 寄存器决定，不用模拟电压设定
L379  tmc.en_spreadCycle(true);  // 打开 spreadCycle 斩波模式，获得更大的低速扭矩
L380  tmc.rms_current(1200);  // 设运行电流 1200mA
L381  tmc.microsteps(16);  // 把微步细分设成 16，寄存器设置优先于硬件引脚
L382  tmc.toff(3);  // 最后一步才打开斩波器，让前面配置全部生效
L383  }  // 候选地址循环体结束，四个地址都已写过一遍配置
L384  }  // 盲写配置函数体结束，实际只有地址匹配的驱动会生效
L386  bool initTmcDriversWriteOnly(bool &rearLineHigh, bool &frontLineHigh) {  // 只写模式初始化，同时回传两路线电平是否正常
L387  rearTmcUart.begin(115200);  // 后轮先按 115200 波特率建链
L388  if (TMC_FRONT_ACTIVE) {  // 前轮是 TMC 时才配置它的 UART
L389  frontTmcUart.begin(115200);  // 前轮 UART 建链
L390  delayMicroseconds(20);  // 稍等片刻让线路电平稳定下来
L391  } else {  // 否则前轮不是 TMC，跳过 UART 初始化
L392  frontLineHigh = true;  // 前轮没有 PDN_UART，线电平直接当作正常
L393  }  // 前轮分支的右花括号，回到主流程继续后面的检查
L394  rearLineHigh = (GPIOA->IDR & (1UL << 9)) != 0;  // 读 PA9 电平判断后轮线是否被驱动拉高
L395  if (TMC_FRONT_ACTIVE) frontLineHigh = (GPIOA->IDR & (1UL << 2)) != 0;  // 前轮是 TMC 时同样读 PA2 电平做判断
L396  if (!rearLineHigh || (TMC_FRONT_ACTIVE && !frontLineHigh)) {  // 任一路线电平不对说明接线或驱动异常
L397  GPIOA->MODER &= ~((3UL << (2*9)) | (3UL << (2*2)));  // 恢复引脚模式，放弃后续 UART 配置
L398  return false;  // 返回失败，调用方将报告 UART 配置异常
L399  }  // 线电平检查分支结束，两路线都正常才继续配置
L400  configureTmcWriteOnly(rearTmc);  // 对后轮四个候选地址盲写配置
L401  if (TMC_FRONT_ACTIVE) configureTmcWriteOnly(frontTmc);  // 前轮是 TMC 时同样盲写配置
L402  return true;  // 配置流程全部完成，报告成功
L403  }  // 只写模式初始化函数结束
L404  #endif  // 只写模式配置函数区结束
L405  #endif  // TMC UART 专属代码区结束，本段当前不参与编译
L420  const float GY_DIRECTION = 1.0f;  // 陀螺方向系数取 1，表示保持原符号不翻转
L430  const float IMU_FORWARD_ACCEL_DIRECTION = -1.0f;  // 前向线加速度方向系数取 -1，实测该方向为负
L436  const float TURN_GYRO_DIRECTION = 1.0f;  // 转向角速度方向系数取 1，保持原符号
L442  int balanceDirection = -1;  // 直立环整体方向取 -1，与电机安装方向配套
L448  bool motor1Invert = false;  // 电机1 不翻转安装方向
L450  bool motor2Invert = true;  // 只翻转后轮 DIR 极性，因为前后轮安装方向相反
L456  int encoder1Direction = 1;  // 编码器1 计数方向取正
L458  int encoder2Direction = -1;  // 编码器2 计数方向取反，实机确认两路符号相反
L471  float balanceAngleOffset = 2.99f;  // 参数 O：上电默认平衡目标角 2.99 度
L495  float balanceKp = 1000.0f;  // 直立环 P：角度误差 1 度对应的原始输出量
L497  float balanceKd = 100.0f;  // 直立环 D：角速度阻尼增益
L501  float BALANCE_OUTPUT_LIMIT_HZ = 1750.0f;  // 参数 L：直立环最终 STEP 频率上限，防超频失步
L508  float BALANCE_OUTPUT_SLEW_HZ = 100.0f;  // 参数 S：100Hz 端点每次允许的最大频率增量
L509  float BALANCE_ACCEL_SCALE = 0.10f;  // 参数 R：直立 PD 输出到每次频率增量的比例
L513  float HIGH_SPEED_ACCEL_START_HZ = 1000.0f;  // 参数 B：高速加速保护的启动频率
L514  float HIGH_SPEED_ACCEL_LIMIT_HZ = 40.0f;  // 参数 H：高速继续加速时每次允许的增量上限
L515  float lastFinalBalanceOutput = 0.0f;  // 记录上一拍最终频率，用于斜坡限速比较
L522  const uint16_t REVERSAL_SOFT_TICKS = 25;  // 换向软窗口 25 个端点周期，约合 250ms
L523  const float REVERSAL_SOFT_LIMIT_SCALE = 0.25f;  // 软窗口内增量限幅压到 0.25 倍，减缓换向爬升
L527  const float SPEED_MEAS_TAU_S = 0.25f;  // 速度环测量低通时间常数 0.25s，压掉车体摆动
L531  float BALANCE_ANGLE_DEADBAND_DEG = 0.0f;  // 参数 Z：角度死区，当前为 0 即死区关闭
L578  float SPEED_ERROR_DEADBAND = 2.0f;  // 速度误差死区，小于该值不积分，避免零附近积累
L589  float MAX_TARGET_FORWARD_SPEED = 30.0f;  // 参数 MS：目标速度上限，可运行时用命令修改
L608  int speedLoopDirection = -1;  // 速度外环整体方向取 -1，架空实测定反后翻号
L609  float speedDirectKp = 0.0f;  // 参数 Q：速度直投增益，第一版关闭避免与速度环打架
L611  float speedPredictHorizon = 0.0f;  // 参数 A：STEP 速度预测时间，0 表示关闭预测
L612  const float SPEED_ACCEL_FILTER_ALPHA = 0.80f;  // 20Hz 速度差分低通系数，越大越平滑但响应变慢
L615  float speedKp = -0.10f;  // 参数 K 即 SV：速度环 P，负号方向已由实车确认
L619  float speedKi = 0.0f;  // 参数 I：速度环积分增益，先用纯 P 验证方向再启用
L620  float speedKd = 0.0f;  // 速度环微分增益，当前关闭为 0
L621  float MAX_SPEED_ANGLE = 0.8f;  // 参数 N：速度修正角上限 0.8 度，再大容易把车拽翻
L628  const float SPEED_ANGLE_SLEW_DEG_PER_UPDATE = 0.10f;  // 每次外环更新目标角修正最多变化 0.1 度，防跳变
L629  float SPEED_INTEGRAL_LIMIT = 50.0f;  // 速度积分累积上限，用来做积分抗饱和
L633  const float WHEEL_TO_HZ_SCALE = 4.0f;  // count/50ms 到 STEP Hz 的量纲标定放大系数
L639  const float SPEED_DIRECT_FULL_ANGLE = 2.0f;  // 姿态偏差小于等于 2 度时速度直投百分之百参与
L641  const float SPEED_DIRECT_DISABLE_ANGLE = 4.0f;  // 姿态偏差大于等于 4 度时速度直投降为 0
L647  const float SPEED_DIRECT_MAX_GYRO_DPS = 80.0f;  // 角速度超过 80 度每秒时关掉速度直投，优先救姿态
L667  float positionKp = 0.02f;  // 参数 X：位置环 P，实测安全值，过大放大静止漂移
L668  float MAX_POSITION_RETURN_SPEED = 0.30f;  // 参数 Y：回位目标速度上限，单位 count/50ms
L669  float POSITION_DEADBAND_COUNT = 12.0f;  // 参数 M：位置死区，按平均编码器 count 计
L684  float turnKp = 0.0f;  // 转向环 P，当前为 0 即转向环关闭
L686  float turnKd = 0.0f;  // 转向环 D，当前为 0 即转向环关闭
L699  float targetForwardSpeed = 0.0f;  // 目标前进速度，当前通常为 0，主要为遥控预留
L701  float speedCmd = 0.0f;  // 参数 SP：命令设定的目标速度，与手动通路互斥
L703  float targetTurnRate = 0.0f;  // 目标转向角速度，转向环关闭时保持 0
L748  float speedError = 0.0f;  // 速度误差，等于目标速度减融合速度，单位 count/50ms
L749  float speedPContribution = 0.0f;  // 速度环比例项对输出目标倾角的贡献
L750  float speedIContribution = 0.0f;  // 速度环积分项对输出目标倾角的贡献
L751  float speedAngleUnclamped = 0.0f;  // 限幅之前的速度修正角，仅用于观察诊断
L752  float speedAngleLimited = 0.0f;  // 限幅之后的速度修正角，实际进入目标角
L758  float speedIntegral = 0.0f;  // 经典速度环的积分累积，会通过目标角起作用
L759  float speedAngleCorrection = 0.0f;  // 参数 SC：速度环输出的倾角修正，叠加到目标平衡角
L760  float speedErrorPrev = 0.0f;  // 上一拍速度误差，供微分阻尼项计算变化率
L761  float speedErrorDotFilt = 0.0f;  // 低通后的速度误差变化率，作为微分阻尼使用
L777  const float MOTOR_STOP_THRESHOLD_HZ = 1.0f;  // 低于这个频率视为停止，不再给电机发脉冲
L779  const float PWM_UPDATE_MIN_DELTA_HZ = 5.0f;  // 频率变化小于该值就不重配定时器，减少开销
L791  const bool HOLD_LAST_OUTPUT_ON_ZERO = false;  // 输出为 0 时不保持上次频率，直接立即停止
L799  const uint8_t ZERO_HOLD_MAX_CYCLES = 3;  // 最多保持 3 个控制拍，约 7.5ms 就强制归零
L801  uint8_t motor1ZeroHoldCount = 0;  // 电机1 过零保持计数，记录已保持多少拍
L803  uint8_t motor2ZeroHoldCount = 0;  // 电机2 过零保持计数，记录已保持多少拍
L809  const uint8_t IMU_FAIL_CONFIRM_COUNT = 8;  // IMU 连续失败 8 拍约 20ms 才确认故障
L811  uint8_t imuFailCount = 0;  // IMU 连续读取失败计数，用于去抖确认
L815  const uint8_t FALL_CONFIRM_COUNT = 8;  // 倒地需连续 8 拍约 20ms 确认，防瞬间晃动误触发
L817  uint8_t fallConfirmCount = 0;  // 倒地确认计数，累计连续满足条件的拍数
L823  const float EMERGENCY_FALL_ANGLE_DEG = 75.0f;  // 倾斜达到 75 度立即紧急停机，不等去抖确认
L844  float OUTER_FULL_ANGLE_DEG = 5.0f;  // 参数 F：偏差小于等于 5 度时外环保持全权限
L845  float OUTER_DISABLE_ANGLE_DEG = 10.0f;  // 参数 G：偏差大于等于 10 度时外环完全退出
L850  float verticalLoopGain = 0.70f;  // 直立环输出衰减系数 0.7，削弱直立环给外环让权重
L855  const float SPEED_INTEGRAL_DECAY_BIG_ANGLE = 0.95f;  // 大角度时速度积分每拍乘该系数缓慢释放，防旧积分推车
L862  float outerLoopAuthority = 1.0f;  // 外环当前权限，0 到 1 连续变化，1 为全权限
L875  const float FALL_ANGLE_DEG = 75.0f;  // 倒地判定角 75 度，连续 8 帧触发停机并清状态
L888  const uint32_t CONTROL_PERIOD_US = 2500;  // 控制周期 2500 微秒，即主节拍 400Hz
L890  const float CONTROL_DT = 0.0025f;  // 控制周期换算成秒，供积分与微分计算使用
L900  TwoWire BMIWire(  // 定义 BMI270 专用的 I2C 总线对象，参数分多行
L901  BMI_SDA_PIN,  // I2C 数据脚配 PB9
L902  BMI_SCL_PIN  // I2C 时钟脚配 PB8
L903  );  // 多行括号收尾，I2C 对象构造结束
L906  BMI270 imu;  // 定义 BMI270 驱动实例，后续读写都经由它
L909  const uint8_t BMI270_ADDRESS = BMI2_I2C_PRIM_ADDR;  // SDO 接地用主地址 0x68，若拉高需改成 0x69
L917  const uint8_t BMI270_REG_ACC_CONF = 0x40;  // BMI270 加速度配置寄存器地址 0x40
L918  const uint8_t BMI270_REG_GYR_CONF = 0x42;  // BMI270 陀螺配置寄存器地址 0x42
L919  const uint8_t BMI270_REG_CMD = 0x7E;  // BMI270 命令寄存器地址 0x7E
L920  const uint8_t BMI270_SOFT_RESET_CMD = 0xB6;  // 软复位命令值 0xB6，上电时先复位芯片
L921  const uint8_t BMI270_ACC_CONF_400HZ = 0xAA;  // 加速度 400Hz 输出、normal 滤波对应的配置值
L922  const uint8_t BMI270_GYR_CONF_400HZ = 0xAA;  // 陀螺 400Hz 输出对应的配置值
L934  HardwareTimer motor1Timer(TIM2);  // 电机1 的 STEP 脉冲改用 TIM2 边沿定时器产生
L936  HardwareTimer motor2Timer(TIM5);  // 电机2 的 STEP 脉冲改用 TIM5 边沿定时器产生
L940  float motor1LastCommandHz = 0.0f;  // 电机1 当前命令频率，用于判断是否需要重配定时器
L942  float motor2LastCommandHz = 0.0f;  // 电机2 当前命令频率，用于判断是否需要重配定时器
L955  volatile int32_t encoder1Count = 0;  // 编码器1 累加计数，中断里修改所以要 volatile
L957  volatile int32_t encoder2Count = 0;  // 编码器2 累加计数，中断里修改所以要 volatile
L961  int32_t lastEncoder1Count = 0;  // 上一个速度窗口时编码器1 的计数快照
L963  int32_t lastEncoder2Count = 0;  // 上一个速度窗口时编码器2 的计数快照
L967  float encoderSpeed1 = 0.0f;  // 编码器1 单独测得的速度，单位 count/50ms
L969  float encoderSpeed2 = 0.0f;  // 编码器2 单独测得的速度，单位 count/50ms
L975  float rawVehicleSpeed = 0.0f;  // 编码器直接测得的车速，单位 count/50ms
L979  bool encoder1Trusted = true;  // 编码器1 信任门状态，默认信任该路数据
L980  bool encoder2Trusted = true;  // 编码器2 信任门状态，默认信任该路数据
L981  uint16_t encoder1MotionTicks = 0;  // 信任判定窗内编码器1 有脉冲的拍数
L982  uint16_t encoder2MotionTicks = 0;  // 信任判定窗内编码器2 有脉冲的拍数
L983  uint16_t encoderTrustWindowTicks = 0;  // 信任判定窗当前已累积的拍数
L984  uint32_t fallStopCount = 0;  // 倒地保护累计触发次数，只增不减供遥测
L985  int rawSpeedEncoderMode = 0;  // 编码器取速模式，0 双路平均 1 仅一路 2 仅另一路
L989  float telemGyroAbsMax = 0.0f;  // 遥测窗口内陀螺俯仰角速度绝对值的最大值
L990  float telemAccPitchDevMax = 0.0f;  // 窗口内加速度角与陀螺积分角偏差的最大值
L991  float telemBalHzAbsMax = 0.0f;  // 窗口内直立环输出绝对值的最大值，看饱和深度
L992  float telemKalmanBiasMin = 1e9f;  // 窗口内陀螺零偏估计的最小值初值
L993  float telemKalmanBiasMax = -1e9f;  // 窗口内陀螺零偏估计的最大值初值
L996  int32_t prevIntegratedSpeedCmdSign = 0;  // 上一拍速度命令的符号，用于识别换向时刻
L997  uint16_t reversalSoftTicksRemaining = 0;  // 换向后剩余软过渡端点周期数，用完即恢复正常限幅
L1003  float vehicleSpeed = 0.0f;  // 多源 Kalman 融合后的车速，单位 count/50ms
L1004  float vehicleSpeedFiltered = 0.0f;  // 速度环实际使用的低通速度，时间常数 0.25s
L1010  float vehiclePosition = 0.0f;  // 相对开机位置的累计位移，单位平均编码器 count
L1016  int32_t positionZeroCount1 = 0;  // 位置零点对应的编码器1 累计计数
L1018  int32_t positionZeroCount2 = 0;  // 位置零点对应的编码器2 累计计数
L1024  float targetVehiclePosition = 0.0f;  // 目标位置固定为 0，即尽量回到启动位置
L1027  float positionError = 0.0f;  // 位置误差，等于目标位置减当前位移
L1028  float positionSpeedTarget = 0.0f;  // 位置环生成的目标速度，交给速度环执行
L1029  bool positionHoldWasEnabled = false;  // 记录位置保持上一拍是否已启用，供状态切换判断
L1031  bool positionHoldSuspended = false;  // 位置保持挂起标志，供手动命令与位置环仲裁
L1032  bool manualMotionIsActive() {  // 判断手动通路是否正在执行，供互斥判断使用
L1033  uint32_t now = millis();  // 只取一次当前时间，供各模块统一判断超时
L1034  return manualTurn.busy(now) || manualForward.busy(now);  // 转向或前后任一路忙，就认为手动通路活跃
L1035  }  // 手动通路活跃判断函数结束，返回布尔结果
L1036  void captureCurrentPositionTarget();  // 前置声明：把当前编码器位置抓成新的目标位置
L1037  void stopPositionJog(int reason);  // 前置声明：按原因停止位置点动
L1038  void updatePositionHold();  // 前置声明：位置保持外环的周期更新
L1039  void paramStorageBoot();  // 前置声明：开机从 flash 读回 22 项参数
L1058  float imuForwardAccelG = 0.0f;  // 前向线加速度估计值，单位 g
L1060  float imuForwardAccelFilteredG = 0.0f;  // 低通后的前向线加速度，抑制步进电机高频振动
L1066  const float IMU_ACCEL_LPF_ALPHA = 0.05f;  // 前向加速度在 400Hz 下的一阶低通系数
L1071  const float ENCODER_SPEED_LPF_ALPHA = 0.4f;  // 编码器速度一阶低通系数，越小越平滑但越慢
L1080  const uint16_t ENCODER_TRUST_WINDOW_TICKS = 50;  // 编码器信任判定窗 50 拍，约 500ms
L1081  const uint16_t ENCODER_TRUST_ACTIVE_TICKS = 5;  // 窗内至少 5 拍有脉冲就判该编码器活跃
L1082  const int32_t  ENCODER_TRUST_MIN_COUNTS = 1;  // 单拍至少 1 个 count 才算这一拍有脉冲
L1088  const float IMU_ACCEL_CLAMP_G = 0.60f;  // 前向加速度钳位上限，防止振动尖峰把速度带飞
L1096  const float IMU_SPEED_ACCEL_DEADBAND_G = 0.03f;  // 小加速度死区，静止时的零偏振动不再积分成假速度
L1104  const float MAX_IMU_SPEED_DELTA_PER_CYCLE = 0.02f;  // 每个控制拍 IMU 最多能改变的速度估计量，防冲击
L1135  float IMU_ACCEL_TO_SPEED_GAIN = 3.0f;  // 把 g 换算成 count/50ms 每秒变化率的经验系数
L1141  float SPEED_KALMAN_Q_BASE = 0.20f;  // 速度 Kalman 的基础过程噪声，表示模型不确定度
L1147  float SPEED_KALMAN_Q_MOTOR_GAIN = 0.0008f;  // 电机命令越大说明速度越可能快变，用该系数增大过程噪声
L1153  float SPEED_KALMAN_R_LOW_SPEED = 0.20f;  // 低速时编码器量化严重，测量噪声 R 取较大值
L1159  float SPEED_KALMAN_R_NORMAL = 0.08f;  // 中高速时编码器更连续，测量噪声 R 可以减小
L1165  float speedKalmanP = 1.0f;  // 速度 Kalman 的协方差状态初值
L1171  float speedKalmanQ = SPEED_KALMAN_Q_BASE;  // 当前正在使用的过程噪声 Q，仅用于调试观察
L1173  float speedKalmanR = SPEED_KALMAN_R_LOW_SPEED;  // 当前正在使用的测量噪声 R，仅用于调试观察
L1181  float imuSpeedPredictionTrust = 1.0f;  // IMU 预测可信度，振动或冲击大时把它的作用降低
L1196  float ax = 0.0f;  // 加速度计 X 轴读数，单位 g
L1198  float ay = 0.0f;  // 加速度计 Y 轴读数，单位 g
L1200  float az = 0.0f;  // 加速度计 Z 轴读数，单位 g
L1204  float gx = 0.0f;  // 陀螺 X 轴角速度，单位度每秒
L1206  float gy = 0.0f;  // 陀螺 Y 轴角速度，即俯仰轴，单位度每秒
L1208  float gz = 0.0f;  // 陀螺 Z 轴角速度，单位度每秒
L1223  const float PITCH_MOUNT_OFFSET_DEG = 0.0f;  // 竖装 BMI270 的固定安装误差补偿，不当作开机零点
L1224  float pitchZero = PITCH_MOUNT_OFFSET_DEG;  // 俯仰零点直接取固定安装补偿值
L1226  float gyroYZero = 0.0f;  // 俯仰轴陀螺零偏估计，开机标定得到
L1228  float gyroXZero = 0.0f;  // X 轴陀螺零偏，本版主要用俯仰轴
L1241  float accPitch = 0.0f;  // 加速度计解算出的俯仰角，动态时会受线性加速度影响
L1243  float pitch = 0.0f;  // 融合后的俯仰角，是直立环与保护逻辑的主输入
L1247  float gyroPitchRate = 0.0f;  // 俯仰角速度，单位度每秒，供直立环 D 项使用
L1249  float gyroTurnRate = 0.0f;  // 转向角速度，单位度每秒，供转向环使用
L1251  float gyroIntegAngle = 0.0f;  // 去掉零偏后的陀螺积分角，400Hz 逐拍累计
L1266  float kalmanAngle = 0.0f;  // Kalman 角度状态，现已由互补滤波取代
L1268  float compAngle = 0.0f;  // 互补滤波角度状态，当前实际使用的姿态角
L1273  const float PITCH_GLITCH_DEG = 5.0f;  // 单拍角度突变超过 5 度就判为毛刺，直接丢掉
L1274  float pitchPrev = 0.0f;  // 上一拍有效俯仰角，毛刺时用它替代当前值
L1276  float kalmanBias = 0.0f;  // 陀螺零偏估计值，Kalman 状态之一
L1280  float P00 = 1.0f;  // 角度与角度之间协方差状态 P00
L1282  float P01 = 0.0f;  // 角度与零偏之间协方差状态 P01
L1284  float P10 = 0.0f;  // 零偏与角度之间协方差状态 P10
L1286  float P11 = 1.0f;  // 零偏与零偏之间协方差状态 P11
L1292  float Q_angle = 0.01f;  // 角度过程噪声，表示角度模型的不确定度
L1294  float Q_bias = 0.03f;  // 零偏过程噪声，表示零偏漂移的允许速率
L1298  const float COMP_FILTER_INIT = 0.95f;  // 互补滤波出厂的陀螺权重，蓝牙可调
L1299  float COMP_ALPHA = 0.99f;  // 当前互补滤波权重，越接近 1 越信陀螺更平滑
L1305  float R_measure = 0.03f;  // 基础测量噪声，现改用动态 R，此变量已不使用
L1329  float accelMagnitude = 1.0f;  // 加速度模长，接近 1g 说明主要测到的是重力
L1331  float accelMagnitudeError = 0.0f;  // 加速度模长与 1g 的偏差，反映动态程度
L1333  float dynamicR = 0.03f;  // 动态测量噪声，越大越不信加速度角而更信陀螺
L1339  const float ACC_ERR_LOW  = 0.04f;  // 模长偏差低阈值，低于它认为动态很小
L1341  const float ACC_ERR_HIGH = 0.10f;  // 模长偏差高阈值，超过它认为动态很大
L1345  const float ACC_INNOV_HIGH = 20.0f;  // 角度分歧超过 20 度判为加速度计毛刺，只信陀螺
L1349  const float R_LOW_DYNAMIC  = 0.02f;  // 低动态时使用的测量噪声 R 值
L1351  const float R_MID_DYNAMIC  = 0.20f;  // 中动态 R 值，改连续插值后已不再使用
L1353  const float R_HIGH_DYNAMIC = 1.20f;  // 高动态时使用的测量噪声 R 值，几乎只信陀螺
L1366  float targetBalanceAngle = 4.0f;  // 目标平衡角，初值与参数 O 保持一致
L1370  float balanceOutput = 0.0f;  // 直立环 PD 合计输出，进入后续混合与限幅
L1371  float balancePOutput = 0.0f;  // 直立环比例项单独输出，供诊断查看
L1372  float balanceDOutput = 0.0f;  // 直立环微分项单独输出，供诊断查看
L1374  float turnOutput = 0.0f;  // 转向环输出，当前参数为 0 所以恒为 0
L1380  float speedDirectOutput = 0.0f;  // 速度直投到 STEP 频率的修正量
L1381  float speedDirectUnclampedOutput = 0.0f;  // 未限幅的速度直投量，仅用于诊断观察
L1382  float speedAccelerationFiltered = 0.0f;  // 低通后的速度差分，抑制高频抖动
L1383  float speedPredictionDelta = 0.0f;  // 速度预测给输出带来的增量
L1384  float speedPredicted = 0.0f;  // 预测得到的速度估计值
L1385  float speedPredictPrevSpeed = 0.0f;  // 上一拍的预测速度，用于计算差分
L1391  float speedDirectAuthority = 0.0f;  // 速度直投权限，跟随外环权限在 0 到 1 之间变化
L1392  float speedDirectAppliedOutput = 0.0f;  // 实际叠加进输出去的速度直投量
L1398  float finalBalanceOutput = 0.0f;  // 最终输出的 STEP 频率，单位 Hz
L1399  float mixedBalanceOutput = 0.0f;  // 直立环与外环混合后的输出，尚未限幅
L1400  float limitedBalanceOutput = 0.0f;  // 限幅之后的混合输出，留给下一步做斜坡与钳位
L1401  float balanceAccelRawOutput = 0.0f;  // 100Hz 本拍未限幅的 STEP 频率增量，单位 Hz 每次
L1402  float balanceAccelLimitedOutput = 0.0f;  // 100Hz 增量限幅后的结果
L1403  float balanceAccelEffectiveLimit = 0.0f;  // 本拍实际使用的增量限幅值，随保护状态变化
L1404  bool highSpeedAccelProtectionActive = false;  // 高速加速保护是否正在生效的标志
L1405  float balanceAccelSubstepOutput = 0.0f;  // 把一个 100Hz 增量均分到 4 个 400Hz 小步
L1406  float integratedMotorSpeedCommand = 0.0f;  // 对增量积分得到的 STEP 目标频率
L1407  uint8_t balanceAccelDivider = 0;  // 400Hz 到 100Hz 的分频计数
L1408  const uint8_t BALANCE_ACCEL_UPDATE_DIVIDER = 4;  // 400Hz 分频 4 得到 100Hz 端点更新
L1412  float lastTurnError = 0.0f;  // 上一拍转向误差，供转向环微分使用
L1414  void clearControlOutputDiagnostics()  // 定义清空所有控制输出与诊断量的函数
L1415  {  // 函数体开始，下面逐项把输出量与本拍状态清零
L1416  manualTurn.clear();  // 清掉手动转向的状态与计时
L1417  manualForward.clear();  // 清掉手动前后移动的状态与计时
L1418  positionJog.cancel(PositionJog::FAULT);  // 以故障原因取消位置点动，防止残留动作
L1419  positionLean.reset();  // 复位位置环的倾角控制状态
L1420  positionSpeedReference.reset();  // 复位速度参考，但编码器里程计仍保持连续
L1421  positionHoldSuspended = false;  // 解除位置保持挂起，允许位置环重新介入
L1422  positionHoldWasEnabled = false;  // 清掉位置保持历史状态，下次重新判断切换
L1423  balancePOutput = 0.0f;  // 清直立环比例项输出为零
L1424  balanceDOutput = 0.0f;  // 清直立环微分项输出为零
L1425  balanceOutput = 0.0f;  // 清直立环合计输出为零
L1426  turnOutput = 0.0f;  // 清转向环输出为零，防止残留转向量继续作用
L1427  speedDirectOutput = 0.0f;  // 清速度直投输出为零，避免残留叠加量带偏车轮
L1428  speedDirectUnclampedOutput = 0.0f;  // 清未限幅速度直投为零
L1429  speedAccelerationFiltered = 0.0f;  // 清速度差分滤波状态为零
L1430  speedPredictionDelta = 0.0f;  // 清速度预测增量为零，预测链从零重新开始
L1431  speedPredicted = 0.0f;  // 清预测速度为零，等下一拍重新给出预测值
L1432  speedPredictPrevSpeed = vehicleSpeed;  // 预测起点直接取当前融合速度，避免下一拍跳变
L1433  speedDirectAppliedOutput = 0.0f;  // 清实际叠加的速度直投量为零
L1434  speedDirectAuthority = 0.0f;  // 清速度直投权限为零，等外环重新给出
L1435  positionSpeedTarget = 0.0f;  // 清位置环目标速度为零
L1436  positionError = 0.0f;  // 清位置误差为零，位置环下一拍重新计算
L1437  mixedBalanceOutput = 0.0f;  // 清混合输出为零，直立环与外环混合从零起算
L1438  limitedBalanceOutput = 0.0f;  // 清限幅输出为零，防止旧限幅值继续生效
L1439  finalBalanceOutput = 0.0f;  // 清最终 STEP 频率为零，确保电机停住
L1440  balanceAccelRawOutput = 0.0f;  // 清未限幅增量输出为零
L1441  balanceAccelLimitedOutput = 0.0f;  // 清限幅后增量输出为零
L1442  balanceAccelEffectiveLimit = 0.0f;  // 清本拍实际增量限幅值
L1443  highSpeedAccelProtectionActive = false;  // 清高速加速保护标志，解除继续加速的限制
L1444  balanceAccelSubstepOutput = 0.0f;  // 清 400Hz 子步增量
L1445  integratedMotorSpeedCommand = 0.0f;  // 清积分得到的目标频率为零
L1446  prevIntegratedSpeedCmdSign = 0;  // 清换向检测用的命令符号
L1447  reversalSoftTicksRemaining = 0;  // 清换向软窗口剩余计数，恢复正常限幅
L1448  balanceAccelDivider = 0;  // 清 400Hz 到 100Hz 的分频计数
L1449  lastFinalBalanceOutput = 0.0f;  // 清斜坡限速用的上一拍频率
L1450  }  // 清空函数体结束，所有控制量与标志已归零
L1463  uint32_t lastControlUs = 0;  // 上一控制拍的时间戳，用于计算调度间隔
L1464  uint32_t controlOverrunCount = 0;  // 控制超时次数，反映 400Hz 节拍是否被拖慢
L1465  uint32_t controlMaxGapUs = 0;  // 控制拍之间的最大间隔，诊断调度抖动
L1466  uint32_t controlMaxWorkUs = 0;  // 单个控制拍内最大耗时，诊断运算是否超时
L1468  uint32_t outerLoopDivider = 0;  // 400Hz 到 100Hz 外环更新的分频计数
L1487  const uint32_t TELEMETRY_DIVIDER = 400;  // 400Hz 除以 400 等于 1Hz，完整 D 帧遥测频率
L1489  uint32_t telemetryDivider = 0;  // 完整 D 帧遥测的分频计数
L1491  const uint32_t FAST_TELEMETRY_DIVIDER = 100;  // 400Hz 除以 100 等于 4Hz，快速 G 帧遥测频率
L1492  uint32_t fastTelemetryDivider = 0;  // 快速 G 帧遥测的分频计数
L1494  uint32_t telemetryMuteUntilMs = 0;  // 收到命令后暂停遥测到该时刻，把带宽让给命令回显
L1496  uint32_t btRxBytes = 0;  // 蓝牙累计接收字节数，诊断命令通道是否通
L1497  uint32_t btCmdCount = 0;  // 已解析的蓝牙命令条数，与收字节数互相印证
L1505  static const uint16_t TX_BUF_SIZE = 4096;  // 遥测发送环形缓冲区总容量 4096 字节，用来缓存待发的整行文本
L1507  static char txBuffer[TX_BUF_SIZE];  // 环形缓冲的实际字符数组，按 head/tail 索引循环入队出队
L1509  static volatile uint16_t txHead = 0;  // 环形缓冲写指针，主循环与中断都可能访问，故用 volatile
L1511  static volatile uint16_t txTail = 0;  // 环形缓冲读指针，由 flush 函数负责向前推进
L1515  static const float TX_BUDGET_PER_CYCLE = 2.0f;  // 每个 2.5ms 控制周期应累加的发送预算 2 字节，对应 800B/s
L1517  static float txBudgetBytes = 0.0f;  // 当前累计可用发送预算，按时间增量累加、发送后扣减
L1519  static const float TX_BUDGET_MAX = 40.0f;  // 预算上限 40 字节，防止长时间积压后一次暴写阻塞主循环
L1522  static bool txPushLine(const char* line, uint16_t len)  // 整行入队函数：传入行指针与长度，返回是否成功排队
L1523  {  // 整行入队函数体开始
L1524  if (len == 0 || len >= TX_BUF_SIZE)  // 长度为 0 或超过缓冲容量时直接判为非法行
L1525  return false;  // 非法行返回失败，由调用方丢弃整行
L1527  uint16_t space;  // 声明变量存放缓冲中可用的连续空间字节数
L1528  if (txHead >= txTail)  // 写指针不落后于读指针时，可用空间落在第一段连续区
L1529  space = (TX_BUF_SIZE - 1) - (txHead - txTail);  // 已用长度即 head-tail，用总容量减一后扣除它算剩余
L1530  else  // 否则环形已回绕，可用空间被分成两段
L1531  space = (txTail - txHead) - 1;  // 回绕时第二段连续空间为 tail-head 再减一
L1532  if (space < len + 1)  // 空间放不下整行加一个换行时放弃入队
L1533  return false;  // 空间不足返回失败，保证每次都是整行不拆分
L1534  for (uint16_t i = 0; i < len; i++)  // 逐字节把整行内容拷进环形缓冲
L1535  {  // 逐字节拷贝循环体开始
L1536  txBuffer[txHead] = line[i];  // 把当前字节写到写指针所在位置
L1537  txHead = (txHead + 1) % TX_BUF_SIZE;  // 写指针环形前移一位，越界回绕到 0
L1538  }  // 逐字节拷贝循环结束
L1539  txBuffer[txHead] = '\n';  // 行尾补一个换行符，接收端才能按行解析
L1540  txHead = (txHead + 1) % TX_BUF_SIZE;  // 写指针再前移一格，为下一行留出位置
L1541  return true;  // 整行入队成功返回 true
L1542  }  // 整行入队函数结束
L1545  void flushTelemetryTx()  // 按预算把环形缓冲里的数据非阻塞地刷到 BLE 串口
L1546  {  // flush 函数体开始
L1547  static uint32_t lastBudgetUs = micros();  // 记录上次结算预算的时刻，静态变量跨调用保持
L1548  uint32_t budgetNowUs = micros();  // 读取当前微秒时刻，用于计算经过的时间增量
L1549  txBudgetBytes += (uint32_t)(budgetNowUs - lastBudgetUs) * 0.0008f;  // 按 800B/s 把经过的微秒数换算成新增的发送预算
L1550  lastBudgetUs = budgetNowUs;  // 更新结算时刻，避免下一拍重复累计同一段时间
L1551  if (txBudgetBytes > TX_BUDGET_MAX)  // 预算超过上限时准备截断
L1552  txBudgetBytes = TX_BUDGET_MAX;  // 截断到 40 字节上限，防止积压后一次性暴写
L1554  while (txHead != txTail && txBudgetBytes >= 1.0f)  // 缓冲有数据且预算够发一字节时进入发送循环
L1555  {  // 外层按预算发送循环体开始
L1556  int avail = BTSerial.availableForWrite();  // 查询串口发送缓冲当前还能容纳多少字节
L1557  if (avail <= 0)  // 串口缓冲已满时不再写入
L1558  break;  // 直接跳出，等下一周期再发，保持不阻塞
L1559  int toSend = (int)txBudgetBytes;  // 本周期预算够发多少字节就取多少
L1560  if (toSend > avail)  // 预算超过串口可用空间时以可用空间为准
L1561  toSend = avail;  // 实际发送量压到串口此刻能容纳的字节数
L1562  if (toSend <= 0)  // 算出仍不足一字节则本周期放弃发送
L1563  break;  // 跳出循环，保证 flush 始终非阻塞
L1564  uint16_t n = 0;  // 本周期已发送字节计数清零
L1565  while (n < toSend && txHead != txTail)  // 逐字节发送，直到发够额度或缓冲被取空
L1566  {  // 内层逐字节发送循环体开始
L1567  BTSerial.write((uint8_t)txBuffer[txTail]);  // 取出 tail 处一个字节写入 BLE 串口
L1568  txTail = (txTail + 1) % TX_BUF_SIZE;  // tail 环形前移一位，越界回绕到 0
L1569  n++;  // 本次已发送字节数加一
L1570  }  // 内层发送循环结束
L1571  txBudgetBytes -= (float)n;  // 从预算里扣掉本周期实际发出的字节数
L1572  }  // 外层循环结束，没发完的数据留到下周期
L1573  }  // flush 函数结束
L1576  void prepareCommandResponse() {}  // 参数回显前的占位实现：保留接口，当前不做任何事
L1577  class QueuedBtReply : public Print {  // 自定义 Print 子类，把回显内容先攒进缓冲再整行入队
L1578  public:  // 公有成员区开始，暴露 write 接口
L1579  using Print::write;  // 引入基类 write 重载，避免重写后其他重载被隐藏
L1580  size_t write(uint8_t c) override {  // 重写单字节写接口，把输出逐字节收进本地缓冲
L1581  if (used >= sizeof(buffer) - 1) { overflow = true; return 0; }  // 缓冲将满时置溢出标志并拒收，防止越界写
L1582  buffer[used++] = (char)c;  // 把字节存入缓冲并推进已用长度
L1583  return 1;  // 成功接收一个字节返回 1
L1584  }  // write 重写函数体结束
L1585  ~QueuedBtReply() { if (!overflow && used) txPushLine(buffer, used); }  // 析构时若未溢出且有内容，整行推入发送环形缓冲
L1586  private:  // 私有成员区开始，藏住缓冲与状态
L1587  char buffer[768];  // 回显缓冲 768 字节，足够容纳一条长参数回显
L1588  uint16_t used = 0;  // 已用字节数，初值 0
L1589  bool overflow = false;  // 溢出标志，初值 false，溢出则整行丢弃
L1590  };  // QueuedBtReply 类定义结束
L1603  void encoder1ISR()  // 编码器 1 的 A 相中断服务函数，边沿触发计数
L1605  {  // 编码器 1 ISR 函数体开始
L1607  if (  // if 条件开始：判断 A、B 两相电平是否相同
L1609  digitalRead(ENCODER1_A_PIN)  // 读取编码器 1 A 相当前电平
L1611  ==  // 与下一行读到的 B 相电平比较是否相等
L1613  digitalRead(ENCODER1_B_PIN)  // 读取编码器 1 B 相当前电平
L1615  )  // if 条件括号闭合，进入分支
L1617  {  // A、B 电平相同的分支开始
L1619  encoder1Count += encoder1Direction;  // 按方向系数累加编码器 1 计数
L1621  }  // 两相电平相同分支结束
L1623  else  // 电平不同时走 else 分支
L1625  {  // else 分支体开始
L1627  encoder1Count -= encoder1Direction;  // 按方向系数反向累减编码器 1 计数
L1629  }  // 两相电平不同分支结束
L1631  }  // 编码器 1 ISR 结束
L1636  void encoder2ISR()  // 编码器 2 的 A 相中断服务函数，边沿触发计数
L1638  {  // 编码器 2 ISR 函数体开始
L1640  if (  // if 条件开始：判断 A、B 两相电平是否相同
L1642  digitalRead(ENCODER2_A_PIN)  // 读取编码器 2 A 相当前电平
L1644  ==  // 与下一行读到的 B 相电平比较是否相等
L1646  digitalRead(ENCODER2_B_PIN)  // 读取编码器 2 B 相当前电平
L1648  )  // if 条件括号闭合，进入分支
L1650  {  // A、B 电平相同的分支开始
L1652  encoder2Count += encoder2Direction;  // 按方向系数累加编码器 2 计数
L1654  }  // 两相电平相同分支结束
L1656  else  // 电平不同时走 else 分支
L1658  {  // else 分支体开始
L1660  encoder2Count -= encoder2Direction;  // 按方向系数反向累减编码器 2 计数
L1662  }  // 两相电平不同分支结束
L1664  }  // 编码器 2 ISR 结束
L1677  bool bmiWriteRegister(  // BMI270 单寄存器写函数声明，只用于 400Hz 配置回读
L1679  uint8_t reg,  // 参数：要写入的寄存器地址
L1681  uint8_t value  // 参数：要写进该寄存器的一个字节数值
L1683  )  // 写寄存器函数参数列表闭合
L1685  {  // 写寄存器函数体开始
L1687  BMIWire.beginTransmission(  // 发起一次 I2C 写传输，先指定从机地址
L1689  BMI270_ADDRESS  // BMI270 的 I2C 从机地址常量
L1691  );  // beginTransmission 参数列表闭合
L1695  BMIWire.write(reg);  // 写出要操作的目标寄存器地址
L1697  BMIWire.write(value);  // 写出真正要写进寄存器的数据字节
L1701  uint8_t error =  // 声明变量接收结束传输的返回码
L1703  BMIWire.endTransmission();  // 结束传输，把缓存的字节真正发上总线
L1707  delayMicroseconds(500);  // 写后延时 500 微秒，等传感器内部完成写入
L1711  return  // 返回语句开始：用错误码比较结果作返回值
L1713  error == 0;  // 返回码为 0 表示写入成功
L1715  }  // 写寄存器函数结束
L1728  bool bmiReadRegister(  // BMI270 单寄存器读函数声明，读出值用引用回传
L1730  uint8_t reg,  // 参数：要读取的寄存器地址
L1732  uint8_t &value  // 引用参数，用于把读到的字节回传给调用方
L1734  )  // 读寄存器函数参数列表闭合
L1736  {  // 读寄存器函数体开始
L1738  BMIWire.beginTransmission(  // 发起 I2C 传输，先指定从机地址
L1740  BMI270_ADDRESS  // BMI270 的 I2C 从机地址常量
L1742  );  // beginTransmission 参数列表闭合
L1746  BMIWire.write(reg);  // 先写出要读取的寄存器地址
L1750  if (  // if 条件开始：检查写地址阶段是否失败
L1752  BMIWire.endTransmission(false)  // 结束传输但保留总线占用，false 表示不发停止位
L1754  != 0  // 返回码非 0 说明写地址失败
L1756  )  // if 条件括号闭合，继续读数据
L1758  {  // 写地址失败分支体开始
L1760  return false;  // 写地址失败直接返回 false
L1762  }  // 写地址失败分支结束
L1767  uint8_t count =  // 声明变量接收实际请求到的字节数
L1769  BMIWire.requestFrom(  // 从 BMI270 发起读数据请求
L1771  BMI270_ADDRESS,  // BMI270 的 I2C 从机地址常量
L1773  (uint8_t)1  // 本次请求读取 1 个字节
L1775  );  // requestFrom 参数列表闭合
L1780  if (  // if 条件开始：校验返回值与驱动可读字节
L1782  count != 1  // 实际读到字节数不等于 1 判为失败
L1784  ||  // 该条件与后面的 available 检查是逻辑或关系
L1786  !BMIWire.available()  // 驱动内部没有可读字节也算失败
L1788  )  // if 条件括号闭合，开始取数据
L1790  {  // 读失败分支体开始
L1792  return false;  // 读失败返回 false，交由调用方处理
L1794  }  // 读失败分支结束
L1799  value =  // 把读到的字节赋给引用参数回传
L1801  BMIWire.read();  // 从 I2C 接收缓冲取出一个字节
L1806  return true;  // 读寄存器成功返回 true
L1808  }  // 读寄存器函数结束
L1821  bool configureBMI270_400Hz()  // 把 BMI270 加速度计与陀螺仪都配成 400Hz 并回读校验
L1822  {  // 400Hz 配置函数体开始
L1824  if (!bmiWriteRegister(BMI270_REG_ACC_CONF, BMI270_ACC_CONF_400HZ))  // 写加速度计配置寄存器为 400Hz 配置值，失败即返回
L1825  {  // 加速度计写失败分支体开始
L1826  return false;  // 加速度计配置写失败直接返回 false
L1827  }  // 加速度计失败分支结束
L1828  delay(5);  // 延时 5ms 等加速度计配置生效
L1830  if (!bmiWriteRegister(BMI270_REG_GYR_CONF, BMI270_GYR_CONF_400HZ))  // 写陀螺仪配置寄存器为 400Hz 配置值
L1831  {  // 陀螺仪写失败分支体开始
L1832  return false;  // 陀螺仪配置写失败直接返回 false
L1833  }  // 陀螺仪失败分支结束
L1834  delay(10);  // 延时 10ms 等陀螺仪配置生效
L1836  uint8_t accConf = 0;  // 声明变量存放回读的加速度计配置值
L1837  uint8_t gyrConf = 0;  // 声明变量存放回读的陀螺仪配置值
L1838  return bmiReadRegister(BMI270_REG_ACC_CONF, accConf)  // 回读加速度计配置，作为首个校验条件
L1839  && bmiReadRegister(BMI270_REG_GYR_CONF, gyrConf)  // 并且回读陀螺仪配置也要成功
L1840  && accConf == BMI270_ACC_CONF_400HZ  // 并且加速度计配置确实等于 400Hz 配置值
L1841  && gyrConf == BMI270_GYR_CONF_400HZ;  // 并且陀螺仪配置确实等于 400Hz 配置值
L1842  }  // 配置函数结束，返回是否所有校验都通过
L1854  bool readBMI270()  // 读一次 BMI270 数据并做轴向符号修正
L1855  {  // BMI270 读取函数体开始
L1856  if (imu.getSensorData() != BMI2_OK)  // 调用库取传感器数据，返回码不是 OK 即失败
L1857  return false;  // 读取失败返回 false，本拍不做姿态更新
L1861  ax = imu.data.accelX;  // 加速度 X 轴直通，保持原符号
L1862  ay = -imu.data.accelY;  // 加速度 Y 轴取反，抵消绕 X 轴翻转 180° 的安装
L1863  az = -imu.data.accelZ;  // 加速度 Z 轴取反，与 Y 轴一起翻转
L1864  gx = imu.data.gyroX;  // 陀螺仪 X 轴直通，保持原符号
L1865  gy = -imu.data.gyroY;  // 陀螺仪 Y 轴取反，使角速度符号与加速度角一致
L1866  gz = -imu.data.gyroZ;  // 陀螺仪 Z 轴取反，与 Y 轴一起翻转
L1868  return true;  // 读取并换算成功返回 true
L1869  }  // BMI270 读取函数结束
L1881  bool initBMI270()  // 初始化 BMI270：识别芯片、装载配置并启用传感器
L1882  {  // BMI270 初始化函数体开始
L1885  delay(1500);  // 先等 1500ms，让传感器电源晚于 MCU 稳定起来
L1887  BMIWire.begin();  // 初始化 BMI270 专用的 I2C 总线
L1888  BMIWire.setClock(100000);  // 初始化阶段先用 100kHz 低速，提高首次通信成功率
L1889  delay(300);  // 起总线后再等 300ms 让器件就绪
L1891  for (int i = 0; i < 12; i++)  // 最多重试 12 次，覆盖冷上电时序不稳的情况
L1892  {  // 初始化重试循环体开始
L1893  BMIWire.beginTransmission(BMI270_ADDRESS);  // 向 BMI270 地址发起一次空传输做探测
L1894  uint8_t probeError = BMIWire.endTransmission();  // 结束传输取返回码，非 0 表示器件没应答
L1895  if (probeError != 0)  // 探测失败时进入重试分支
L1896  {  // 探测失败分支体开始
L1897  delay(500);  // 等 500ms 再试，给电源和上电复位留时间
L1898  continue;  // 跳过本次后续步骤，直接进入下一次重试
L1899  }  // 探测失败分支结束
L1902  bmiWriteRegister(BMI270_REG_CMD, BMI270_SOFT_RESET_CMD);  // 先软复位，清除上次未完成的配置装载状态
L1903  delay(50);  // 软复位后等 50ms 让芯片恢复
L1905  int8_t result = imu.beginI2C(BMI270_ADDRESS, BMIWire);  // 调用库初始化：装载配置文件并使能传感器
L1906  if (result == BMI2_OK)  // 库初始化返回 OK 时再进一步验证
L1907  {  // 初始化成功分支体开始
L1908  delay(200);  // 等 200ms 让传感器输出稳定
L1909  if (imu.getSensorData() == BMI2_OK)  // 再读一次数据确认真正可用
L1910  return true;  // 数据可读即初始化成功，返回 true
L1911  }  // 初始化成功分支结束
L1913  BTSerial.print("BMI270 init retry ");  // 通过 BLE 打出重试提示的前半段
L1914  BTSerial.print(i + 1);  // 打印当前重试序号（从 1 开始）
L1915  BTSerial.print(" err=");  // 打印错误码前缀 err=
L1916  BTSerial.println((int)result);  // 打印库初始化返回码，便于现场定位
L1917  delay(500);  // 本次重试失败后再等 500ms
L1918  }  // 初始化重试循环结束
L1920  return false;  // 12 次都失败，返回 false 表示 IMU 不可用
L1921  }  // BMI270 初始化函数结束
L1937  float calculateAccelPitch()  // 用加速度重力分量算绝对俯仰角，单位度
L1939  {  // 加速度角函数体开始
L1941  return  // 返回表达式开始：反三角函数结果
L1943  atan2f(  // 用 atan2 由两轴分量算角度，保留象限信息
L1945  -az,  // 纵轴取 -az，与前面 Y/Z 翻转后的轴向约定一致
L1947  -ax  // 横轴取 -ax 作为 atan2 的第二个参数
L1949  )  // atan2f 参数列表闭合
L1951  *  // 弧度结果乘以换算系数
L1953  180.0f  // 每弧度对应 180 度
L1955  /  // 除以圆周率 PI，完成弧度转度
L1957  PI;  // PI 作为除数，整个返回表达式结束
L1959  }  // 加速度角函数结束
L1974  bool calibrateIMU()  // 开机静止采样，估计陀螺零偏和初始绝对倾角
L1975  {  // IMU 校准函数体开始
L1979  const float CAL_STATIC_GYRO_DPS = 4.0f;  // 静止判据：角速度小于 4 度每秒才计入有效样本
L1980  const float CAL_GRAV_MAG_TOL = 0.25f;  // 静止判据：加速度模长与 1g 之差小于 0.25g
L1981  const int SAMPLE_TARGET = 400;  // 需要累计 400 个有效静止样本才算校准成功
L1982  const uint32_t CAL_TIMEOUT_MS = 20000;  // 总等待上限 20 秒，超时放弃，防止开机卡死
L1984  delay(300);  // 上电后先等 300ms 让传感器输出稳定
L1985  float pitchSum = 0.0f, gyroYSum = 0.0f, gyroXSum = 0.0f;  // 累计俯仰角与两轴陀螺读数，初值全部清零
L1986  int valid = 0;  // 已累计的有效样本计数
L1987  uint32_t tStart = millis();  // 记录采样起始时刻，用于超时判断
L1988  while (valid < SAMPLE_TARGET && (millis() - tStart) < CAL_TIMEOUT_MS)  // 样本不够且未超时就继续采样
L1989  {  // 静止采样循环体开始
L1990  if (readBMI270())  // 只有本拍成功读到 IMU 才参与静止判据
L1991  {  // 本拍读取成功分支体开始
L1992  float mag = sqrtf(ax*ax + ay*ay + az*az);  // 计算三轴加速度模长，静止时应接近 1g
L1993  bool still = (fabsf(gy) < CAL_STATIC_GYRO_DPS)  // 静止第一判据：俯仰轴角速度足够小
L1994  && (fabsf(mag - 1.0f) < CAL_GRAV_MAG_TOL);  // 并且加速度模长接近 1g，排除线性加速度干扰
L1995  if (still)  // 判定为静止时才累计样本
L1996  {  // 判定为静止的分支体开始
L1997  pitchSum += calculateAccelPitch();  // 累加本拍由加速度算出的绝对俯仰角
L1998  gyroYSum += gy;  // 累加陀螺 Y 轴读数，用于求俯仰轴零偏
L1999  gyroXSum += gx;  // 累加陀螺 X 轴读数，用于求零偏
L2000  valid++;  // 有效样本数加一
L2001  }  // 静止分支结束
L2002  }  // 读取成功分支结束
L2003  delay(3);  // 每轮采样间隔 3ms，约合 300Hz 采样率
L2004  }  // 静止采样循环结束
L2005  if (valid < SAMPLE_TARGET / 2)  // 有效样本不足目标一半则认为校准不可靠
L2006  {  // 校准失败分支体开始
L2007  return false;  // 样本太少返回 false，提示校准失败
L2008  }  // 校准失败分支结束
L2009  pitchZero = PITCH_MOUNT_OFFSET_DEG;  // 静态倾角零点取固定安装偏置 PITCH_MOUNT_OFFSET_DEG
L2010  gyroYZero = gyroYSum / (float)valid;  // 陀螺 Y 零偏取所有有效样本的均值
L2011  gyroXZero = gyroXSum / (float)valid;  // 陀螺 X 零偏同样取所有有效样本的均值
L2012  const float initialPitch = pitchSum / (float)valid - pitchZero;  // 初始俯仰角为加速度角均值减去安装零点
L2013  accPitch  = initialPitch;  // 用初始角初始化加速度角状态变量
L2014  pitch     = initialPitch;  // 同样初始化融合后的俯仰角 pitch
L2015  kalmanAngle = initialPitch;  // 初始化 Kalman 角度状态，避免从 0 慢慢收敛
L2016  kalmanBias  = 0.0f;  // 零偏状态从 0 开始，零偏已单独估好
L2017  compAngle   = initialPitch;  // 初始化互补滤波角度状态
L2018  pitchPrev   = initialPitch;  // 初始化上一拍角度，供角速度积分使用
L2019  gyroIntegAngle = initialPitch;  // 初始化陀螺积分角，保证动态 R 判据一致
L2020  P00 = 1.0f; P01 = 0.0f; P10 = 0.0f; P11 = 1.0f;  // 协方差矩阵复位为单位阵，重新开始收敛
L2021  return true;  // 校准成功返回 true
L2022  }  // IMU 校准函数结束
L2035  float updateKalman(  // 两状态 Kalman 滤波：状态为角度与陀螺零偏
L2037  float measuredAngle,  // 入参：加速度测得的绝对角度，单位度
L2039  float measuredRate,  // 入参：陀螺测得的角速度，单位度每秒
L2041  float dt  // 入参：本拍时间步长，单位秒
L2043  )  // 参数列表闭合，函数声明结束
L2045  {  // Kalman 函数体开始
L2055  float rate =  // 先算出扣除零偏后的真实角速度
L2057  measuredRate  // 陀螺原始角速度读数，先取出来
L2059  -  // 减去零偏，得到净角速度
L2061  kalmanBias;  // 估计的零偏，得到净角速度
L2066  kalmanAngle +=  // 状态预测：角度按净角速度积分
L2068  dt  // 状态预测：乘以时间步长 dt
L2070  *  // 乘号片段：乘以时间步长 dt
L2072  rate;  // 净角速度，角度预测完成
L2085  P00 +=  // 协方差预测第一项 P00 递推
L2087  dt  // 协方差预测：乘以时间步长 dt
L2089  *  // 再乘以后面括号内的组合项
L2091  (  // 括号开始，内容是过程噪声与耦合项
L2093  dt * P11  // 角度方差经 dt 传播后的一项
L2095  -  // 减去（协方差耦合项 P01）
L2097  P01  // 协方差耦合项 P01，作为被减项
L2099  -  // 再减去（协方差耦合项 P10）
L2101  P10  // 协方差耦合项 P10，作为第二个被减项
L2103  +  // 加上（角度过程噪声 Q_angle）
L2105  Q_angle  // 角度过程噪声 Q_angle，为积分留不确定度
L2107  );  // 括号闭合，P00 预测完成
L2112  P01 -=  // 耦合项 P01 的状态预测递推
L2114  dt  // 耦合项预测：乘以时间步长 dt
L2116  *  // 乘号片段：接着乘时间步长 dt
L2118  P11;  // 减去 P11 与 dt 的乘积，完成 P01 预测
L2123  P10 -=  // 耦合项 P10 的状态预测递推
L2125  dt  // 耦合项预测：乘以时间步长 dt
L2127  *  // 乘号片段：接着乘时间步长 dt
L2129  P11;  // 减去 P11 与 dt 的乘积，完成 P10 预测
L2134  P11 +=  // 零偏方差 P11 预测，叠加零偏过程噪声
L2136  Q_bias  // 零偏过程噪声系数 Q_bias
L2138  *  // 乘号片段：接着乘时间步长
L2140  dt;  // 乘以时间步长，P11 预测完成
L2153  float innovation =  // 计算测量创新：测量角与预测角之差
L2155  measuredAngle  // 加速度测得的绝对角度
L2157  -  // 减去预测角，得到测量创新
L2159  kalmanAngle;  // Kalman 预测角，作为被减项
L2180  float S =  // 计算创新协方差 S，供随后求增益
L2182  P00  // 角度预测方差 P00，创新方差主体
L2184  +  // 加上动态测量噪声 R
L2186  dynamicR;  // 动态测量噪声 R，振动越大 R 越大
L2199  float K0 =  // 角度通道的 Kalman 增益
L2201  P00  // 角度预测方差 P00，作为分子
L2203  /  // 除以创新协方差 S，得到增益
L2205  S;  // 创新协方差，得到角度增益 K0
L2210  float K1 =  // 零偏通道的 Kalman 增益
L2212  P10  // 角度与零偏的协方差 P10，作分子
L2214  /  // 除以创新协方差 S，得到增益
L2216  S;  // 创新协方差，得到零偏增益 K1
L2229  kalmanAngle +=  // 用角度增益修正预测角
L2231  K0  // 角度增益 K0，作为修正权重
L2233  *  // 乘以测量创新，放大角度修正
L2235  innovation;  // 测量创新，完成角度状态修正
L2240  kalmanBias +=  // 用零偏增益修正零偏估计
L2242  K1  // 零偏增益 K1，作为修正权重
L2244  *  // 乘以测量创新，修正零偏估计
L2246  innovation;  // 测量创新，完成零偏状态修正
L2259  float oldP00 = P00;  // 暂存旧的 P00，供协方差修正使用
L2261  float oldP01 = P01;  // 暂存旧的 P01，避免修正时相互覆盖
L2266  P00 -=  // 协方差修正：P00 减去增益带来的方差
L2268  K0  // 角度增益 K0，作为修正权重
L2270  *  // 乘以旧 P00，完成 P00 修正
L2272  oldP00;  // 旧 P00，修正 P00 时被减的基准
L2277  P01 -=  // 协方差修正：P01
L2279  K0  // 角度增益 K0，作为修正权重
L2281  *  // 乘以旧 P01，完成 P01 修正
L2283  oldP01;  // 旧 P01，修正 P01 时被减的基准
L2288  P10 -=  // 协方差修正：P10
L2290  K1  // 零偏增益 K1，作为修正权重
L2292  *  // 乘以旧 P00，完成 P10 修正
L2294  oldP00;  // 旧 P00，修正 P10 时被减的基准
L2299  P11 -=  // 协方差修正：P11
L2301  K1  // 零偏增益 K1，作为修正权重
L2303  *  // 乘以旧 P01，完成 P11 修正
L2305  oldP01;  // 旧 P01，修正 P11 时被减的基准
L2310  return  // 返回融合后的角度估计
L2312  kalmanAngle;  // 即 Kalman 角度状态值
L2314  }  // Kalman 函数结束
L2323  float updateComplementary(  // 互补滤波姿态更新函数，取代 Kalman 的简化方案
L2325  float measuredAngle,  // 入参：加速度测得的绝对角，本式里作为修正项
L2327  float measuredRate,  // 入参：陀螺测得的角速度，单位度每秒
L2329  float dt  // 入参：本拍时间步长，单位秒
L2331  )  // 参数列表闭合，函数声明结束
L2333  {  // 互补滤波函数体开始
L2335  compAngle =  // 互补滤波核心：陀螺积分与加速度角加权融合
L2337  COMP_ALPHA  // 权重 COMP_ALPHA，越接近 1 越信陀螺积分
L2339  *  // 乘号片段：权重乘以陀螺预测角
L2341  (compAngle + measuredRate * dt)  // 上一拍角度加本拍陀螺积分增量
L2343  +  // 加上加速度通道的加权修正
L2345  (1.0f - COMP_ALPHA)  // 加速度通道权重 1-COMP_ALPHA
L2347  *  // 乘号片段：权重乘以加速度角
L2349  measuredAngle;  // 加速度测得的绝对角，完成加权融合
L2351  return  // 返回融合后的角度值
L2353  compAngle;  // 即互补滤波角度 compAngle
L2355  }  // 互补滤波函数结束
L2368  void updateAttitude()  // 每拍姿态更新：算加速度角、角速度并做融合
L2370  {  // 姿态更新函数体开始
L2380  accPitch =  // 加速度算出的绝对角，先减去安装零点
L2382  calculateAccelPitch()  // 调用重力分量反算绝对俯仰角
L2384  -  // 减去固定安装零点 pitchZero
L2386  pitchZero;  // 固定安装零点 pitchZero
L2399  gyroPitchRate =  // 俯仰轴角速度，先扣掉陀螺零偏
L2401  GY_DIRECTION  // 符号方向系数 GY_DIRECTION，统一正方向
L2403  *  // 乘号片段：系数乘以净角速度
L2405  (  // 括号开始，准备扣零偏
L2407  gy  // 陀螺 Y 轴原始读数
L2409  -  // 减去，扣掉陀螺零偏
L2411  gyroYZero  // 陀螺零偏 gyroYZero
L2413  );  // 括号闭合，得到扣偏后的角速度
L2432  accelMagnitude =  // 三轴加速度模长，用于判断动态程度
L2434  sqrtf(  // 对三轴平方和开平方
L2436  ax * ax  // X 轴分量平方，用于求模长
L2438  +  // 加，累加 Y 轴分量平方
L2440  ay * ay  // Y 轴分量平方，用于求模长
L2442  +  // 加，累加 Z 轴分量平方
L2444  az * az  // Z 轴分量平方，用于求模长
L2446  );  // 开方调用闭合，得到加速度模长
L2451  accelMagnitudeError =  // 模长与 1g 的偏差，反映线性加速度大小
L2453  fabsf(  // 对模长偏差取绝对值
L2455  accelMagnitude  // 上一步算出的加速度模长
L2457  -  // 减去理论静止加速度 1g
L2459  1.0f  // 理论静止值 1g，作为比较基准
L2461  );  // 取绝对值调用闭合，得到模长误差
L2486  float errRatio = accelMagnitudeError / ACC_ERR_HIGH;  // 模长误差按 ACC_ERR_HIGH 归一，得到 0~1 比例
L2487  if (errRatio > 1.0f) { errRatio = 1.0f; }  // 比例超过 1 截断为 1，防止 R 无上限
L2488  float innovRatio = fabsf(accPitch - gyroIntegAngle) / ACC_INNOV_HIGH;  // 加速度角与陀螺积分角的分歧比例，抓单拍毛刺
L2489  if (innovRatio > 1.0f) { innovRatio = 1.0f; }  // 分歧比例同样截断到 1
L2490  float combinedRatio = (errRatio > innovRatio) ? errRatio : innovRatio;  // 取两个比例较大者，任一异常都抬高噪声
L2491  dynamicR = R_LOW_DYNAMIC + (R_HIGH_DYNAMIC - R_LOW_DYNAMIC) * combinedRatio;  // 用较大比例在 R 低值到高值之间插值出 dynamicR
L2504  pitch =  // 姿态角 pitch 赋值语句开头，结果来自下一行的 Kalman 融合
L2506  updateKalman(  // 调用角度 Kalman 融合，输入加速度角、陀螺角速度与采样周期
L2508  accPitch,  // 加速度计解算出的倾角，作为融合的观测量
L2510  gyroPitchRate,  // 陀螺仪俯仰角速度，作为预测与积分输入
L2512  CONTROL_DT  // 400Hz 主拍对应的采样周期，兼作积分步长
L2514  );  // 结束函数调用，融合结果回写 pitch
L2519  {  // 进入毛刺抑制代码块，限定中间变量作用域
L2520  float dPitch = pitch - pitchPrev;  // 计算本拍 pitch 相对上一有效值的单步变化量
L2521  if (dPitch > PITCH_GLITCH_DEG) pitch = pitchPrev;  // 单步上跳超过阈值判为毛刺，沿用上一有效角度
L2522  else if (dPitch < -PITCH_GLITCH_DEG) pitch = pitchPrev;  // 单步下跳超过阈值同样判为毛刺，回退上一有效角度
L2523  else pitchPrev = pitch;  // 变化在阈值内视为真实角度，更新上一有效值
L2524  }  // 结束毛刺抑制代码块，恢复正常角度使用
L2528  gyroIntegAngle += (gyroPitchRate - kalmanBias) * CONTROL_DT;  // 去掉 Kalman 零偏后按 400Hz 积分，得到供遥测的陀螺积分角
L2557  float pitchRad =  // 先把角度制的 pitch 换算成弧度，供三角运算
L2559  pitch  // 待转换的 pitch 数值，单位为度
L2561  *  // 乘号，连接角度与弧度换算系数
L2563  DEG_TO_RAD;  // 度转弧度系数，完成换算并结束语句
L2568  imuForwardAccelG =  // 计算 IMU 前向线加速度（单位 g），供速度 Kalman 预测
L2570  IMU_FORWARD_ACCEL_DIRECTION  // 方向系数，把前向加速度统一为向前为正
L2572  *  // 乘号，连接方向系数与括号内的加速度
L2574  (  // 括号开始，先算去掉重力后的加速度
L2576  az  // 加速度计 Z 轴读数，其中含重力投影
L2578  +  // 加号：用重力投影把 az 里的重力分量抵消
L2580  sinf(pitchRad)  // 当前倾角对应的重力在 Z 轴上的分量
L2582  );  // 结束括号与表达式，得到前向加速度
L2589  if (  // 开始前向加速度上限幅判断，防尖峰污染预测量
L2591  imuForwardAccelG  // 待检查的前向加速度值
L2593  >  // 大于比较：判断是否超过加速度上限
L2595  IMU_ACCEL_CLAMP_G  // 允许的前向加速度上限
L2597  )  // 结束判断条件，进入限幅处理
L2599  {  // 进入上限幅处理，把尖峰压到上限
L2601  imuForwardAccelG =  // 超过上限则改写下限幅值
L2603  IMU_ACCEL_CLAMP_G;  // 赋值为加速度上限，抑制正向尖峰
L2605  }  // 结束上限幅分支，继续负向检查
L2610  if (  // 开始负向限幅判断，处理反向尖峰
L2612  imuForwardAccelG  // 待检查的前向加速度值
L2614  <  // 小于比较：判断是否低于负向下限
L2616  -IMU_ACCEL_CLAMP_G  // 负方向上的加速度下限
L2618  )  // 结束负向限幅的判断条件
L2620  {  // 进入负向限幅处理，压掉反向尖峰
L2622  imuForwardAccelG =  // 低于下限则拉回下限值
L2624  -IMU_ACCEL_CLAMP_G;  // 赋值为负下限幅，压住反向尖峰
L2626  }  // 结束负向限幅分支，进入低通环节
L2633  imuForwardAccelFilteredG =  // 对前向加速度做一阶低通，得到平滑值
L2635  (1.0f - IMU_ACCEL_LPF_ALPHA)  // 旧值保留系数，与采样周期配套
L2637  *  // 乘号，旧值乘以保留系数
L2639  imuForwardAccelFilteredG  // 上一拍的滤波结果，作为低通旧值
L2641  +  // 加上本拍新值的加权项
L2643  IMU_ACCEL_LPF_ALPHA  // 新值权重，越小滤波越重
L2645  *  // 乘号，把新值乘以权重后相加
L2647  imuForwardAccelG;  // 本拍原始加速度，完成低通更新
L2654  if (  // 开始按加速度模长误差分级降低 IMU 预测可信度
L2656  accelMagnitudeError  // 加速度模长偏离 1g 的误差量
L2658  <  // 小于比较：判断误差是否很小
L2660  ACC_ERR_LOW  // 低误差门限，视为准静态
L2662  )  // 结束低误差区间的判断
L2664  {  // 进入高可信分支，满权重使用 IMU
L2666  imuSpeedPredictionTrust =  // 误差很小，完全信任 IMU 的速度预测
L2668  1.0f;  // 可信度取 1.0，完全采用预测
L2670  }  // 结束高可信分支，转入下一区间
L2672  else if (  // 中等误差区间，可信度打折
L2674  accelMagnitudeError  // 加速度模长误差量，与高门限比较
L2676  <  // 小于比较：判断是否在中误差区间
L2678  ACC_ERR_HIGH  // 高误差门限，超过则判为剧烈振动
L2680  )  // 结束中误差区间的判断
L2682  {  // 进入中等可信分支，预测打对折
L2684  imuSpeedPredictionTrust =  // 有振动或线性加速度，降低预测权重
L2686  0.45f;  // 可信度取 0.45，抑制振动下的预测
L2688  }  // 结束中等分支，进入最低档
L2690  else  // 误差超过高门限，可信度最低
L2692  {  // 进入最低可信分支，几乎不用预测
L2694  imuSpeedPredictionTrust =  // 剧烈振动时几乎不用 IMU 预测
L2696  0.12f;  // 可信度取 0.12，仅留极小预测量
L2698  }  // 结束可信度分级，回到主流程
L2711  gyroTurnRate =  // 计算转向角速度，供差速与转向控制使用
L2713  TURN_GYRO_DIRECTION  // 转向方向系数，统一左右转符号
L2715  *  // 乘号，把方向系数乘到角速度上
L2717  (  // 括号开始，先减去 X 轴零偏
L2719  gx  // 陀螺仪 X 轴原始角速度
L2721  -  // 减去 X 轴静止零偏
L2723  gyroXZero  // 开机标定得到的 X 轴零偏
L2725  );  // 结束表达式，得到转向角速度
L2727  }  // 结束姿态更新函数，回到调用处
L2740  void initEncoders()  // 编码器初始化入口，配置两路正交编码器输入
L2742  {  // 函数体开始，依次配置四根编码器引脚
L2744  pinMode(  // 把 1 号编码器 A 相引脚设为带内部上拉输入
L2746  ENCODER1_A_PIN,  // 1 号编码器 A 相引脚号
L2748  INPUT_PULLUP  // 输入模式并启用内部上拉，适配开漏输出
L2750  );  // 结束该引脚设置，继续下一根
L2754  pinMode(  // 设置 1 号编码器 B 相引脚
L2756  ENCODER1_B_PIN,  // 1 号编码器 B 相引脚号
L2758  INPUT_PULLUP  // 输入模式并启用内部上拉电阻
L2760  );  // 结束该引脚设置，继续下一根
L2764  pinMode(  // 设置 2 号编码器 A 相引脚
L2766  ENCODER2_A_PIN,  // 2 号编码器 A 相引脚号
L2768  INPUT_PULLUP  // 输入模式并启用内部上拉电阻
L2770  );  // 结束该引脚设置，继续下一根
L2774  pinMode(  // 设置 2 号编码器 B 相引脚
L2776  ENCODER2_B_PIN,  // 2 号编码器 B 相引脚号
L2778  INPUT_PULLUP  // 输入模式并启用内部上拉电阻
L2780  );  // 结束该引脚设置，准备挂中断
L2785  attachInterrupt(  // 为 1 号编码器 A 相挂上双边沿中断
L2787  digitalPinToInterrupt(  // 把引脚号转成中断号，兼容不同封装
L2789  ENCODER1_A_PIN  // 1 号编码器 A 相引脚
L2791  ),  // 结束引脚转换，作为中断源
L2793  encoder1ISR,  // 中断服务函数，内部按 B 相电平判方向计数
L2795  CHANGE  // 双边沿触发，A 相每次跳变都计数以提高分辨率
L2797  );  // 结束 1 号编码器的中断注册
L2802  attachInterrupt(  // 为 2 号编码器 A 相挂中断
L2804  digitalPinToInterrupt(  // 把 2 号 A 相引脚号转成中断号
L2806  ENCODER2_A_PIN  // 2 号编码器 A 相引脚
L2808  ),  // 结束引脚转换，作为中断源参数
L2810  encoder2ISR,  // 2 号编码器中断服务函数
L2812  CHANGE  // 双边沿触发计数，与 1 号编码器一致
L2814  );  // 结束 2 号编码器的中断注册
L2816  }  // 结束编码器初始化，返回主流程
L2830  struct StepOutput {  // 定义一路 STEP 输出的运行状态结构体
L2831  HardwareTimer* timer;  // 该路使用的硬件定时器指针，负责产生 STEP 节拍
L2832  uint32_t mask;  // GPIOA 上对应 STEP 引脚的位掩码，供 BSRR 快速置位
L2833  volatile uint32_t periodTicks = 1000000;  // 一个完整 STEP 周期的 tick 数，约等于微秒
L2834  volatile uint32_t risingEdges = 0;  // 累计上升沿数，可用于核对实际脉冲数
L2835  volatile bool running = false;  // 输出是否正在运行的标志
L2836  volatile bool high = false;  // 当前 STEP 引脚是否处于高电平
L2837  bool dir = false;  // 当前方向引脚的电平状态
L2838  StepOutput(HardwareTimer* t, uint32_t m) : timer(t), mask(m) {}  // 构造函数，绑定定时器并记录引脚掩码
L2839  };  // 结束结构体定义，随后建立两个实例
L2840  StepOutput step1(&motor1Timer, 1UL << 6);  // 1 号电机 STEP 状态，绑定 motor1 定时器与 PA6 位
L2841  StepOutput step2(&motor2Timer, 1UL << 8);  // 2 号电机 STEP 状态，绑定 motor2 定时器与 PA8 位
L2842  const uint32_t STEP_HIGH_US = 4;  // STEP 高电平保持 4us，满足驱动器最小脉宽
L2843  const uint32_t DIR_SETUP_US = 4;  // 方向电平到首个脉冲的建立时间 4us
L2844  const float MAX_STEP_HZ = 20000.0f;  // STEP 频率上限 20kHz，避免超出电机能力
L2845  void stepEdge(StepOutput& out) {  // STEP 节拍中断处理，每次翻转一次引脚
L2846  TIM_TypeDef* tim = out.timer->getHandle()->Instance;  // 取底层 TIM 寄存器基址，便于直接写 ARR 与 CNT
L2847  if (!out.running) return;  // 输出已停止则直接返回，不产生脉冲
L2848  if (out.high) {  // 当前为高电平，本拍要拉低形成下降沿
L2849  GPIOA->BSRR = out.mask << 16;  // BSRR 高 16 位写 1 复位对应引脚，输出低
L2850  out.high = false;  // 记录当前已处于低电平
L2851  tim->ARR = out.periodTicks - STEP_HIGH_US - 1;  // 装载低电平段时长，与高电平段凑满整个周期
L2852  } else {  // 当前为低电平，本拍拉高形成上升沿
L2853  GPIOA->BSRR = out.mask;  // BSRR 低 16 位写 1 置位引脚，输出高
L2854  out.high = true;  // 记录当前已处于高电平
L2855  out.risingEdges++;  // 上升沿计数加一，完成一次完整 STEP
L2856  tim->ARR = STEP_HIGH_US - 1;  // 装载高电平段 4us 对应的计数值
L2857  }  // 结束高电平与低电平的分支判断
L2858  tim->CNT = 0;  // 计数器清零，让下一段从零开始计时
L2859  }  // 结束翻转处理函数，退出中断
L2860  void motor1EdgeISR() { stepEdge(step1); }  // 1 号定时器中断入口，转调通用翻转逻辑
L2861  void motor2EdgeISR() { stepEdge(step2); }  // 2 号定时器中断入口，转调通用翻转逻辑
L2862  void stopStepOutput(StepOutput& out) {  // 停止一路 STEP 输出并确保引脚回落到低电平
L2863  uint32_t saved = __get_PRIMASK();  // 保存当前中断屏蔽状态，便于临界区后恢复
L2864  __disable_irq();  // 关总中断，避免停机过程被打断
L2865  out.running = false;  // 先清运行标志，中断进来也不会再翻转
L2866  out.timer->getHandle()->Instance->CR1 &= ~TIM_CR1_CEN;  // 关闭定时器计数使能，彻底停止产生中断
L2867  if (out.high) delayMicroseconds(STEP_HIGH_US);  // 若正处在高电平，等够脉宽再拉低
L2868  out.timer->getHandle()->Instance->SR = 0;  // 清状态寄存器，丢弃残留的中断标志
L2869  GPIOA->BSRR = out.mask << 16;  // 把 STEP 引脚强制拉低，保持静止电平
L2870  out.high = false;  // 更新高电平状态标志，与引脚一致
L2871  __set_PRIMASK(saved);  // 恢复先前的中断屏蔽状态
L2872  }  // 结束停止函数，引脚保持低电平
L2873  void initMotors() {  // 电机初始化：配置 DIR/STEP 引脚与 1MHz 定时器
L2874  pinMode(MOTOR1_DIR_PIN, OUTPUT);  // 1 号电机方向引脚设为输出
L2875  pinMode(MOTOR2_DIR_PIN, OUTPUT);  // 2 号电机方向引脚设为输出
L2876  pinMode(MOTOR1_STEP_PIN, OUTPUT);  // 1 号电机 STEP 引脚设为输出
L2877  pinMode(MOTOR2_STEP_PIN, OUTPUT);  // 2 号电机 STEP 引脚设为输出
L2878  digitalWrite(MOTOR1_DIR_PIN, LOW);  // 1 号方向初值置低，对应默认前进方向
L2879  digitalWrite(MOTOR2_DIR_PIN, LOW);  // 2 号方向初值也置低，两轮同为前进
L2880  GPIOA->BSRR = (step1.mask | step2.mask) << 16;  // 两路 STEP 引脚一起拉低，确保上电不输出脉冲
L2881  StepOutput* outputs[] = {&step1, &step2};  // 把两路输出放进数组，便于统一配置
L2882  for (StepOutput* out : outputs) {  // 遍历两路定时器做相同初始化
L2883  out->timer->setPrescaleFactor(out->timer->getTimerClkFreq() / 1000000UL);  // 分频到 1MHz，使 tick 与微秒一一对应
L2884  out->timer->setOverflow(1000000UL, TICK_FORMAT);  // 溢出值设 1e6，即最长 1 秒周期
L2885  out->timer->setPreloadEnable(false);  // 关闭预装载，中断里写 ARR 立即生效
L2886  }  // 结束定时器遍历，两路分频就绪
L2887  motor1Timer.attachInterrupt(motor1EdgeISR);  // 注册 1 号定时器更新中断，产出 STEP 边沿
L2888  motor2Timer.attachInterrupt(motor2EdgeISR);  // 注册 2 号定时器更新中断
L2889  motor1Timer.resume();  // 启动 1 号定时器计数
L2890  motor2Timer.resume();  // 启动 2 号定时器计数
L2891  stopStepOutput(step1);  // 立即停 1 号输出，确保上电时电机不动
L2892  stopStepOutput(step2);  // 立即停 2 号输出，确保上电不抖动
L2893  }  // 结束电机初始化，返回主流程
L2894  void commandStepOutput(StepOutput& out, uint32_t dirPin, bool invert, float commandHz) {  // 按目标频率驱动一路 STEP，含换向与相位保持
L2895  if (!isfinite(commandHz) || fabsf(commandHz) < MOTOR_STOP_THRESHOLD_HZ) {  // 指令非有限值或低于停止阈值则停机
L2896  stopStepOutput(out);  // 停止输出并拉低 STEP
L2897  return;  // 停机后直接返回，不再配置定时器
L2898  }  // 结束停机判断，进入正常驱动流程
L2899  bool dir = (commandHz >= 0.0f) != invert;  // 由指令符号与安装方向取反得到实际方向电平
L2900  uint32_t period = (uint32_t)(1000000.0f / fminf(fabsf(commandHz), MAX_STEP_HZ) + 0.5f);  // 频率换周期 tick，四舍五入并限到最大频率
L2901  if (!out.running || dir != out.dir) {  // 未运行或方向改变，需要停机重新配置
L2902  stopStepOutput(out);  // 先停输出，避免带电换向造成丢步
L2903  delayMicroseconds(DIR_SETUP_US);  // 等方向引脚电平稳定后再发脉冲
L2904  digitalWrite(dirPin, dir ? HIGH : LOW);  // 写入新的方向电平，决定正反转
L2905  out.dir = dir;  // 记录当前方向，供下拍比较
L2906  delayMicroseconds(DIR_SETUP_US);  // 再等建立时间，保证首个脉冲方向正确
L2907  uint32_t saved = __get_PRIMASK();  // 保存中断状态，准备进入临界区
L2908  __disable_irq();  // 关中断，防止写寄存器时被 STEP 中断打断
L2909  out.periodTicks = period;  // 更新周期，中断按此值计时
L2910  TIM_TypeDef* tim = out.timer->getHandle()->Instance;  // 取底层 TIM 寄存器指针
L2911  tim->ARR = 1;  // 先装 1，使计数快速溢出产生首个边沿
L2912  tim->CNT = 0;  // 计数器清零，从零开始
L2913  tim->SR = 0;  // 清中断标志，避免立即误触发
L2914  out.running = true;  // 置运行标志，允许中断翻转引脚
L2915  tim->CR1 |= TIM_CR1_CEN;  // 开定时器计数，开始输出脉冲
L2916  __set_PRIMASK(saved);  // 恢复中断屏蔽，退出临界区
L2917  return;  // 启动流程结束，返回调用处
L2918  }  // 结束启动分支，进入同向调速逻辑
L2919  uint32_t saved = __get_PRIMASK();  // 同方向调速，进入临界区保护
L2920  __disable_irq();  // 关中断，保证周期改写是原子的
L2921  TIM_TypeDef* tim = out.timer->getHandle()->Instance;  // 取底层 TIM 寄存器指针
L2922  uint32_t oldPeriod = out.periodTicks;  // 记下旧周期，用于折算已走过的进度
L2923  out.periodTicks = period;  // 写入新周期，中断后续按此计时
L2924  if (!out.high && period != oldPeriod) {  // 只在低电平段且周期变化时做无缝重定时
L2926  tim->CR1 &= ~TIM_CR1_CEN;  // 暂停计数，便于安全改写 ARR 与 CNT
L2927  uint32_t oldLow = oldPeriod - STEP_HIGH_US;  // 旧周期里的低电平段时长
L2928  uint32_t newLow = period - STEP_HIGH_US;  // 新周期里的低电平段时长
L2929  uint32_t elapsed = tim->CNT;  // 已走过的计数，代表低电平段进度
L2930  if (elapsed >= oldLow) elapsed = oldLow - 1;  // 越界保护，避免折算时出现除零或超界
L2931  uint32_t nextCount = (uint32_t)((uint64_t)elapsed * newLow / oldLow);  // 按比例把进度折算到新周期，保持相位连续
L2932  tim->ARR = newLow - 1;  // 装载新的低电平段长度
L2933  tim->CNT = nextCount;  // 写回折算后的计数，接着走完剩余时间
L2934  tim->CR1 |= TIM_CR1_CEN;  // 恢复计数，使周期切换不产生抖动
L2935  }  // 结束重定时分支，恢复中断后返回
L2936  __set_PRIMASK(saved);  // 恢复中断屏蔽状态，退出临界区
L2937  }  // 结束频率指令函数，回到调用处
L2952  void stopMotor1() { stopStepOutput(step1); motor1LastCommandHz = 0.0f; }  // 停 1 号电机并清零其最近指令频率
L2957  void stopMotor2() { stopStepOutput(step2); motor2LastCommandHz = 0.0f; }  // 停 2 号电机并清零其最近指令频率
L2962  void stopBothMotors()  // 两路同时停止的封装函数入口
L2964  {  // 函数体开始，依次停两路电机
L2966  stopMotor1();  // 停 1 号电机并清零指令频率
L2968  stopMotor2();  // 停 2 号电机并清零指令频率
L2970  }  // 结束两路停止函数，回到调用处
L2983  void setMotor1(float commandHz) {  // 设置 1 号电机目标 STEP 频率
L2984  commandStepOutput(step1, MOTOR1_DIR_PIN, motor1Invert, commandHz);  // 调用底层驱动，带上本机方向取反标志
L2985  motor1LastCommandHz = step1.running ? copysignf(fminf(fabsf(commandHz), MAX_STEP_HZ), commandHz) : 0.0f;  // 记录实际生效频率，停机则记 0，供遥测显示
L2986  }  // 结束 setMotor1
L2999  void setMotor2(float commandHz) {  // 设置 2 号电机目标 STEP 频率
L3000  commandStepOutput(step2, MOTOR2_DIR_PIN, motor2Invert, commandHz);  // 调用底层驱动，带 2 号方向取反标志
L3001  motor2LastCommandHz = step2.running ? copysignf(fminf(fabsf(commandHz), MAX_STEP_HZ), commandHz) : 0.0f;  // 记录 2 号实际生效频率，停机记 0
L3002  }  // 结束 setMotor2
L3027  void predictVehicleSpeedKalman()  // 速度 Kalman 预测步入口，400Hz 每拍调用
L3029  {  // 函数体开始，先取预测用加速度
L3045  float accelForSpeed =  // 取滤波后的前向加速度作为速度预测输入
L3047  imuForwardAccelFilteredG;  // 复用前面算出的低通加速度
L3056  if (  // 开始小加速度死区判断
L3058  fabsf(accelForSpeed)  // 当前加速度的绝对值，用于死区判断
L3060  <  // 小于比较：判断是否落在死区内
L3062  IMU_SPEED_ACCEL_DEADBAND_G  // 死区门限，滤掉零偏级别的微小加速度
L3064  )  // 结束死区判断，死区内直接归零
L3066  {  // 进入死区处理，把微小加速度置零
L3068  accelForSpeed =  // 死区内把加速度视为 0，防止积分成假速度
L3070  0.0f;  // 赋零，防止零偏被积分成假速度
L3072  }  // 结束死区处理，继续算速度增量
L3077  float imuDeltaSpeed =  // 本拍由 IMU 推算出的速度增量
L3079  accelForSpeed  // 去死区后的前向加速度
L3081  *  // 乘可信度，振动时减小预测权重
L3083  imuSpeedPredictionTrust  // 按误差分级得到的预测可信度
L3085  *  // 乘物理换算系数，把 g 折成速度
L3087  IMU_ACCEL_TO_SPEED_GAIN  // g 到速度单位的换算增益
L3089  *  // 乘采样周期得到速度增量
L3091  CONTROL_DT;  // 400Hz 采样周期，完成增量计算
L3098  if (  // 开始单周期增量上限幅
L3100  imuDeltaSpeed  // 本拍速度增量，检查是否超上限
L3102  >  // 大于比较：判断增量是否超上限
L3104  MAX_IMU_SPEED_DELTA_PER_CYCLE  // 单拍允许的最大增量，防预测暴走
L3106  )  // 结束上限判断，进入限幅
L3108  {  // 进入上限幅，把增量压到允许值
L3110  imuDeltaSpeed =  // 把过大的正向预测压回上限
L3112  MAX_IMU_SPEED_DELTA_PER_CYCLE;  // 取上限值，结束正向限幅
L3114  }  // 结束正向限幅，继续负向检查
L3119  if (  // 开始负向限幅，处理反向尖峰
L3121  imuDeltaSpeed  // 本拍速度增量，检查是否低于下限
L3123  <  // 小于比较：判断是否低于下限
L3125  -MAX_IMU_SPEED_DELTA_PER_CYCLE  // 负向增量的下限，防止反向暴走
L3127  )  // 结束负向判断，进入限幅
L3129  {  // 进入负向限幅，把增量拉回下限
L3131  imuDeltaSpeed =  // 把反向尖峰同样限到下限
L3133  -MAX_IMU_SPEED_DELTA_PER_CYCLE;  // 取负向下限，结束负向限幅
L3135  }  // 结束负向限幅，准备累加到速度
L3140  vehicleSpeed +=  // 把 IMU 增量累加进融合速度，完成预测步
L3142  imuDeltaSpeed;  // 本次速度增量，累加进融合速度
L3165  speedKalmanQ =  // 按电机输出幅值动态估计过程噪声 Q
L3167  SPEED_KALMAN_Q_BASE  // 基础 Q，静止时也保留少量不确定性
L3169  +  // 加上与输出相关的部分
L3171  SPEED_KALMAN_Q_MOTOR_GAIN  // 输出折算到 Q 的增益系数
L3173  *  // 乘输出绝对值，输出越大 Q 越大
L3175  fabsf(balanceOutput);  // 平衡环输出大小，代表速度可能变化得快
L3180  speedKalmanP +=  // 协方差按 Q 与周期预测增长
L3182  speedKalmanQ  // 本拍过程噪声，随电机输出变化
L3184  *  // 乘采样周期，得到协方差增量
L3186  CONTROL_DT;  // 完成协方差预测，退出预测步
L3188  }  // 结束预测函数，等待编码器修正
L3193  void correctVehicleSpeedKalman(  // 速度 Kalman 修正步，用编码器速度作观测
L3195  float encoderMeasurement  // 编码器测得的速度，单位 counts/50ms
L3197  )  // 结束参数列表，函数体开始
L3199  {  // 函数体开始，先按测量选 R
L3215  if (  // 按测量大小选择观测噪声 R
L3217  fabsf(encoderMeasurement)  // 编码器速度的绝对值，用于分档
L3219  <  // 小于比较：判断是否属于低速
L3221  2.0f  // 2 以下算低速，量化误差明显
L3223  )  // 结束低速判断，进入 R 赋值分支
L3225  {  // 进入低速分支，R 取较小值
L3227  speedKalmanR =  // 低速时 R 取小值，仍以编码器为准
L3229  SPEED_KALMAN_R_LOW_SPEED;  // 低速专用观测噪声，抑制量化误差
L3231  }  // 结束低速分支，走常规分支
L3233  else  // 中高速分支，测量更可信
L3235  {  // 进入中高速分支，采用常规 R
L3237  speedKalmanR =  // 中高速测量更可信，用常规 R
L3239  SPEED_KALMAN_R_NORMAL;  // 常规观测噪声，正常速度下使用
L3241  }  // 结束 R 选择，开始计算新息
L3246  float innovation =  // 计算新息：实测速度减预测速度
L3248  encoderMeasurement  // 编码器测量值，作为观测基准
L3250  -  // 减去预测速度，得到偏差
L3252  vehicleSpeed;  // 当前 Kalman 预测速度
L3257  float innovationCovariance =  // 新息协方差，等于状态协方差加观测噪声
L3259  speedKalmanP  // 状态协方差，代表预测的不确定度
L3261  +  // 加上观测噪声，得到新息协方差
L3263  speedKalmanR;  // 观测噪声，即 R 的当前值
L3268  float K =  // 计算 Kalman 增益
L3270  speedKalmanP  // 状态不确定性，增益的分子部分
L3272  /  // 除以新息协方差，得到增益
L3274  innovationCovariance;  // 新息协方差，得到 0~1 的增益
L3279  vehicleSpeed +=  // 用增益乘新息修正速度
L3281  K  // Kalman 增益，决定信测量还是信预测
L3283  *  // 乘增益，得到本次修正幅度
L3285  innovation;  // 新息量，即测量与预测的偏差
L3290  speedKalmanP =  // 协方差按 1-K 收缩
L3292  (1.0f - K)  // 未被观测解释的比例，用于收缩 P
L3294  *  // 乘旧协方差，得到更新后的 P
L3296  speedKalmanP;  // 旧协方差，到此完成 P 更新
L3303  if (  // 协方差下限保护，防止修正越来越迟钝
L3305  speedKalmanP  // 更新后的协方差，检查是否过小
L3307  <  // 小于比较：判断是否低于下限
L3309  0.02f  // 最小协方差门限，防止修正过慢
L3311  )  // 结束下限判断，进入保护
L3313  {  // 进入下限保护，抬升协方差
L3315  speedKalmanP =  // 强制抬到下限，保持修正灵敏度
L3317  0.02f;  // 取下限值，结束保护流程
L3319  }  // 结束协方差保护，退出修正步
L3321  }  // 结束修正函数，回到外环继续
L3337  void updateOuterLoop()  // 漂移拉回外环入口，100Hz 每 10ms 调用一次
L3339  {  // 函数体开始，先取编码器快照
L3341  int32_t count1;  // 暂存 1 号编码器计数快照
L3343  int32_t count2;  // 暂存 2 号编码器计数快照
L3347  noInterrupts();  // 关中断，保证两路计数被同一时刻读取
L3351  count1 =  // 读取 1 号编码器累计计数
L3353  encoder1Count;  // 中断里维护的全局计数
L3357  count2 =  // 读取 2 号编码器累计计数
L3359  encoder2Count;  // 中断里维护的全局计数
L3363  interrupts();  // 尽快恢复中断，避免丢掉编码器边沿
L3376  int32_t delta1 =  // 算 10ms 内 1 号编码器的增量
L3378  count1  // 本次计数快照，作为被减数
L3380  -  // 减去上次快照，得到本拍增量
L3382  lastEncoder1Count;  // 上次快照，差值为 10ms 增量
L3387  int32_t delta2 =  // 算 2 号编码器 10ms 增量
L3389  count2  // 本次计数快照，作为被减数
L3391  -  // 减去上次快照，得到本拍增量
L3393  lastEncoder2Count;  // 上次快照，差值即 2 号增量
L3398  lastEncoder1Count =  // 更新 1 号快照供下次使用
L3400  count1;  // 本次计数，留待下次相减
L3404  lastEncoder2Count =  // 更新 2 号快照，供下一次求增量
L3406  count2;  // 本次计数，留待下次相减
L3411  encoderSpeed1 =  // 把 10ms 增量乘 5 换成 counts/50ms 并低通
L3413  (1.0f - ENCODER_SPEED_LPF_ALPHA) * encoderSpeed1 + ENCODER_SPEED_LPF_ALPHA * ((float)delta1 * 5.0f);  // 一阶低通滤掉量化跳变，保持原有速度量纲
L3418  encoderSpeed2 =  // 2 号编码器速度同样换算并低通
L3420  (1.0f - ENCODER_SPEED_LPF_ALPHA) * encoderSpeed2 + ENCODER_SPEED_LPF_ALPHA * ((float)delta2 * 5.0f);  // 得到 2 号的 counts/50ms 速度
L3430  if (delta1 >= ENCODER_TRUST_MIN_COUNTS || delta1 <= -ENCODER_TRUST_MIN_COUNTS)  // 1 号增量够大即记一次运动，用于判活
L3431  encoder1MotionTicks++;  // 1 号编码器活跃计数加一
L3433  if (delta2 >= ENCODER_TRUST_MIN_COUNTS || delta2 <= -ENCODER_TRUST_MIN_COUNTS)  // 2 号编码器增量够大同样记一次运动
L3434  encoder2MotionTicks++;  // 2 号编码器活跃计数加一
L3436  encoderTrustWindowTicks++;  // 信任判定窗口计时加一
L3438  if (encoderTrustWindowTicks >= ENCODER_TRUST_WINDOW_TICKS)  // 窗口满则结算一次两路的可信状态
L3439  {  // 进入结算分支，刷新两路信任标志
L3440  const bool encoder1Active = (encoder1MotionTicks >= ENCODER_TRUST_ACTIVE_TICKS);  // 窗口内 1 号活跃拍数达标则判为正常
L3442  const bool encoder2Active = (encoder2MotionTicks >= ENCODER_TRUST_ACTIVE_TICKS);  // 同样判定 2 号编码器是否正常
L3446  if (encoder1Active || encoder2Active)  // 至少一路在动才更新结论，全静默说明车停着
L3447  {  // 进入信任更新分支，改写可信状态
L3448  encoder1Trusted = encoder1Active;  // 用本窗口结论刷新 1 号信任标志
L3450  encoder2Trusted = encoder2Active;  // 刷新 2 号信任标志
L3451  }  // 结束信任更新，随后清零计数
L3453  encoder1MotionTicks = 0;  // 清零 1 号活跃计数，开始下个窗口
L3455  encoder2MotionTicks = 0;  // 清零 2 号活跃计数
L3457  encoderTrustWindowTicks = 0;  // 清零窗口计时，开始下一个窗口
L3458  }  // 结束窗口结算，回到速度合成
L3460  rawSpeedEncoderMode =  // 记录当前使用的编码器组合，供遥测排查
L3461  (encoder1Trusted && encoder2Trusted) ? 0 : (encoder1Trusted ? 1 : 2);  // 0 双路、1 仅 1 号、2 仅 2 号三档
L3482  if (encoder1Trusted && encoder2Trusted)  // 两路都可信时取平均速度
L3484  rawVehicleSpeed = -0.5f * (encoderSpeed1 + encoderSpeed2);  // 两路取均值并取负，使向前为正
L3486  else if (encoder1Trusted)  // 只有 1 号可信时单独用它
L3488  rawVehicleSpeed = -encoderSpeed1;  // 1 号速度取负作为车速
L3490  else  // 否则只用 2 号一路，降级运行
L3492  rawVehicleSpeed = -encoderSpeed2;  // 2 号速度取负作为车速
L3513  correctVehicleSpeedKalman(  // 调用速度卡尔曼修正，把融合速度收敛到编码器观测
L3515  rawVehicleSpeed  // 传入编码器原始速度作为观测量
L3517  );  // 卡尔曼修正调用结束，vehicleSpeed 已更新
L3526  bool jogPositionControl=false;  // 本拍是否由位置点动接管，默认否
L3527  if(positionJog.active){  // 点动会话处于活动状态时进入处理
L3528  int mode=(encoder1Trusted && encoder2Trusted)?0:(encoder1Trusted?1:(encoder2Trusted?2:3));  // 由两路编码器可信状态得出里程计模式 0/1/2/3
L3529  positionJog.update(vehiclePosition,vehicleSpeed,mode,millis(),.01f);  // 以 100Hz 推进点动轨迹与到位判定
L3530  if(!positionJog.active){  // 点动刚结束（到位或停止或故障）时进入收尾
L3531  captureCurrentPositionTarget();  // 用当前位置重置驻车目标，避免结束时回拉
L3532  if(positionJog.reason==PositionJog::ARRIVED){  // 若因到位而结束，则采用到位统计结果
L3533  targetVehiclePosition=positionJog.arrival.center;  // 驻车目标取 1s 窗口平均位置，抵消统计噪声
L3534  positionError=targetVehiclePosition-vehiclePosition;  // 立即算出位置误差，供驻车环本拍使用
L3535  }  // 点动到位处理分支结束
L3536  }  // 点动结束收尾分支结束
L3537  else{  // 点动仍在进行中的分支
L3538  jogPositionControl=true;  // 本拍交由点动位置控制，速度通路让位
L3539  targetVehiclePosition=positionJog.reference;  // 驻车目标跟随点动参考轨迹
L3540  positionError=targetVehiclePosition-vehiclePosition;  // 用目标减位置更新点动位置误差
L3541  positionSpeedTarget=targetForwardSpeed=0;  // 点动期间速度目标清零，倾角另由位置倾角给
L3542  }  // 点动进行中的分支结束
L3543  }  // 位置点动处理整体结束
L3545  if (manualForward.busy(millis())) {  // 手动前后通路忙（命令有效或斜坡未完）时进入
L3548  if (!manualMotionWasBusy) {  // 本次刚进入运动，做无缝衔接初始化
L3550  motionLean.seed(speedAngleCorrection);  // 用当前速度环倾角作初值，入口不跳变
L3551  motionSpeedProfile.reset();  // 速度斜坡从零重新起算
L3552  motionNetSpeed.reset(vehiclePosition);  // 200ms 净速度估计器以当前位置重新初始化
L3553  motionPlan.start(vehiclePosition);  // 预测位置环从当前位置开始规划
L3554  manualMotionWasBusy = true;  // 标记已进入手动运动，供退出与互斥判断
L3555  }  // 手动入口初始化分支结束
L3556  float normalized = ManualTurnControl::clamp(  // 把操作量归一化到正负 1，开始限幅计算
L3557  manualForward.planned / ManualPitchControl::maxAngle, -1.0f, 1.0f);  // 计划倾角除以 2 度上限得档位，再限在正负 1
L3558  float motionSpeedReference =  // 计算手动通路的速度参考
L3559  motionSpeedProfile.update(normalized * MotionSpeedProfile::maxSpeed, .01f);  // 档位乘满档 5 counts/50ms，再按加速率斜升
L3561  float netSpeed = motionNetSpeed.update(vehiclePosition);  // 200ms 窗口净速度，滤掉平衡点高频摆动
L3564  float posCorrection = motionPlan.update(motionSpeedReference, vehiclePosition, netSpeed,  // 调预测位置环，得到速度增量式修正
L3565  true, .01f, positionKp,  // 使能位置环，100Hz，用驻车位置增益
L3566  MAX_POSITION_RETURN_SPEED,  // 修正量受回位速度上限约束
L3567  POSITION_DEADBAND_COUNT);  // 带死区，抑制静止附近的抖动
L3568  motionLean.update(motionSpeedReference + posCorrection, netSpeed, true, .01f);  // 速度误差经 EG 增益生成运动倾角
L3569  positionSpeedTarget = targetForwardSpeed = 0.0f;  // 手动通路不用速度环，目标速度清零
L3570  speedError = speedIntegral = speedErrorPrev = speedErrorDotFilt = 0.0f;  // 清速度误差与微分滤波历史
L3571  speedPContribution = speedIContribution = 0.0f;  // 清 P 分量与 I 分量，避免残留进下一拍
L3572  speedAngleCorrection = speedAngleUnclamped = speedAngleLimited = 0.0f;  // 清速度环倾角的三个中间量
L3573  return;  // 本拍由手动通路接管，跳过驻车与速度 PI
L3574  }  // 手动前后运动分支结束
L3576  manualMotionWasBusy = false;  // 退出手动通路，标记释放段开始
L3577  motionPlan.stop(vehiclePosition);  // 位置计划停止并吸附到当前位置
L3578  if (motionLean.busy() || fabsf(motionSpeedProfile.applied) > 0.0001f) {  // 仍有残留倾角或速度，需要平滑释放
L3579  motionSpeedProfile.update(0.0f, .01f);  // 速度参考按斜坡滑回零
L3580  motionLean.update(0.0f, vehicleSpeed, false, .01f);  // 倾角按释放速率回零，避免角度阶跃
L3581  }  // 运动结束释放段分支结束
L3583  bool positionHoldEnabled = positionKp > 0.000001f;  // 位置增益非零才启用驻车环
L3584  if(!jogPositionControl){  // 点动期间不走常规驻车逻辑
L3585  if (positionHoldEnabled && !positionHoldWasEnabled && !positionHoldSuspended)  // 位置环刚被启用且未被挂起时
L3586  {  // 位置环上升沿处理分支开始
L3587  captureCurrentPositionTarget();  // 把当前位置设为驻车目标，避免起步回拉
L3588  positionError = 0.0f;  // 位置误差清零，从零开始
L3589  }  // 位置环上升沿处理分支结束
L3590  positionHoldWasEnabled = positionHoldEnabled;  // 记录本拍使能状态，供下拍判上升沿
L3592  if (positionHoldEnabled && !positionHoldSuspended)  // 位置环有效且未挂起时执行位置 P 外环
L3593  {  // 位置 P 外环分支开始
L3594  float effectivePositionError = positionError;  // 取位置误差，接着做死区处理
L3595  if (effectivePositionError > POSITION_DEADBAND_COUNT)  // 正向误差超过死区宽度
L3596  effectivePositionError -= POSITION_DEADBAND_COUNT;  // 减去死区宽度，只留净误差
L3597  else if (effectivePositionError < -POSITION_DEADBAND_COUNT)  // 负向误差超过死区宽度
L3598  effectivePositionError += POSITION_DEADBAND_COUNT;  // 负向同样扣除死区宽度
L3599  else  // 位置误差落在死区范围之内
L3600  effectivePositionError = 0.0f;  // 死区内视为零误差，不再修正
L3602  positionSpeedTarget = positionKp * effectivePositionError;  // 位置 P 外环：误差乘 X 得速度目标
L3603  if (positionSpeedTarget > MAX_POSITION_RETURN_SPEED)  // 正向回位速度超过允许上限
L3604  positionSpeedTarget = MAX_POSITION_RETURN_SPEED;  // 钳到回位速度正向上限
L3605  else if (positionSpeedTarget < -MAX_POSITION_RETURN_SPEED)  // 负回位速度低于负上限
L3606  positionSpeedTarget = -MAX_POSITION_RETURN_SPEED;  // 钳到回位速度负向上限
L3607  targetForwardSpeed = positionSpeedTarget;  // 位置环输出成为总目标速度
L3608  }  // 位置 P 外环分支结束
L3609  else  // 位置环未启用或被挂起的分支
L3610  {  // 位置环无效时的分支开始
L3611  positionSpeedTarget = 0.0f;  // 位置环速度目标直接清零
L3612  targetForwardSpeed = 0.0f;  // 总目标速度清零，交给手动或 SP 通路
L3613  }  // 位置环无效时的分支结束
L3615  }  // 结束 !jogPositionControl 条件块
L3616  bool jogAngleControl=jogPositionControl || positionLean.busy();  // 点动中或位置倾角未释放则走角度通路
L3617  if(jogAngleControl)positionSpeedTarget=targetForwardSpeed=0;  // 角度通路下速度目标全清，避免叠加
L3618  float jogDirection=speedLoopDirection*speedKp>=0 ? 1.0f : -1.0f;  // 按速度环符号取向，保证误差正负拉到正确方向
L3619  positionLean.update(jogPositionControl ? positionError : 0,jogPositionControl,.01f,jogDirection);  // 位置误差经 Y 限幅生成点动倾角
L3620  targetForwardSpeed=positionSpeedReference.update(targetForwardSpeed,.01f);  // 目标速度经每秒 3 counts/50ms 的斜率限制
L3623  if (manualMotionWasBusy) {  // 与手动  通路互斥检查
L3624  speedCmd = 0.0f;  // 手动通路动作时清掉 SP，防两套通路同写目标速度
L3625  } else if (speedCmd != 0.0f) {  // 手动通路空闲且有 SP 指令时
L3626  targetForwardSpeed = speedCmd;  // SP 直接作为总目标速度进入速度 PI
L3627  }  // SP 速度指令互斥分支结束
L3628  if(jogPositionControl)positionSpeedTarget=targetForwardSpeed;  // 点动时把总目标速度回写位置速度目标
L3636  if (  // 开始对总目标速度做正向安全限幅
L3638  targetForwardSpeed  // 读取总目标速度参与正向比较
L3640  >  // 判据：目标速度大于正向上限
L3642  MAX_TARGET_FORWARD_SPEED  // 总目标速度的正向安全上限常数
L3644  )  // 条件表达式结束，进入分支
L3646  {  // 正向上限限幅分支开始
L3648  targetForwardSpeed =  // 准备把目标速度改写为上限值
L3650  MAX_TARGET_FORWARD_SPEED;  // 赋值为正向上限，防止指令超速
L3652  }  // 正向上限限幅分支结束
L3657  if (  // 开始对总目标速度做负向安全限幅
L3659  targetForwardSpeed  // 读取总目标速度参与负向比较
L3661  <  // 判据：目标速度小于负向下限
L3663  -MAX_TARGET_FORWARD_SPEED  // 总目标速度的负向安全下限常数
L3665  )  // 条件表达式结束，进入分支
L3667  {  // 负向下限限幅分支开始
L3669  targetForwardSpeed =  // 准备把目标速度改写为下限值
L3671  -MAX_TARGET_FORWARD_SPEED;  // 赋值为负向下限，防止反向超速
L3673  }  // 负向下限限幅分支结束
L3688  vehicleSpeedFiltered = vehicleSpeed;  // 回退后不做低通，直接用瞬时速度
L3690  speedError =  // 计算目标速度与实测速度之差
L3692  targetForwardSpeed  // 目标速度（counts/50ms）
L3694  -  // 减去实测速度，得到速度误差
L3696  vehicleSpeed;  // 实测速度项，误差赋值结束
L3714  float speedErrDot = (speedError - speedErrorPrev) * 100.0f;  // 100Hz 差分求误差变化率
L3715  speedErrorDotFilt = 0.7f * speedErrorDotFilt + 0.3f * speedErrDot;  // 一阶低通滤掉微分噪声
L3716  speedErrorPrev = speedError;  // 保存本拍误差供下拍差分
L3717  speedPContribution = speedKp * speedError;  // P 项：速度误差乘速度环 P
L3718  float speedDContribution = speedKd * speedErrorDotFilt;  // D 项：滤波后的误差变化率乘 D 增益
L3721  if (jogAngleControl || manualForward.busy(millis()) || fabsf(speedKi) < 0.000001f)  // 角度通路或手动通路或 I 增益为零时不积分
L3722  {  // 禁止积分时的清零分支开始
L3723  speedIntegral = 0.0f;  // 积分器清零，防止历史积分污染输出
L3724  }  // 禁止积分时的清零分支结束
L3725  else if (outerLoopAuthority >= 0.2f && fabsf(speedError) > SPEED_ERROR_DEADBAND)  // 有外环权限且误差超死区才允许积分
L3726  {  // 条件积分更新分支开始
L3727  float candidateIntegral = speedIntegral + speedError * 0.01f;  // 候选积分：误差乘 100Hz 步长累加
L3728  if (candidateIntegral > SPEED_INTEGRAL_LIMIT) candidateIntegral = SPEED_INTEGRAL_LIMIT;  // 候选积分按正向积分上限限幅
L3729  else if (candidateIntegral < -SPEED_INTEGRAL_LIMIT) candidateIntegral = -SPEED_INTEGRAL_LIMIT;  // 候选积分按负向积分上限限幅
L3731  float currentUnclamped = speedLoopDirection *  // 算当前未限幅倾角，供抗饱和比较
L3732  (speedPContribution + speedKi * speedIntegral + speedDContribution);  // 当前 PI 输出（未含位置倾角）
L3733  float candidateUnclamped = speedLoopDirection *  // 算采纳候选积分后的未限幅倾角
L3734  (speedPContribution + speedKi * candidateIntegral + speedDContribution);  // 候选 PI 倾角输出结果
L3736  if (fabsf(candidateUnclamped) <= MAX_SPEED_ANGLE ||  // 候选输出未撞到 N 限幅
L3737  fabsf(candidateUnclamped) < fabsf(currentUnclamped))  // 或其绝对值正在变小，才允许积分推进
L3738  {  // 允许积分推进分支开始
L3739  speedIntegral = candidateIntegral;  // 采纳候选积分，实现条件积分抗饱和
L3740  }  // 允许积分推进的分支结束
L3741  }  // 条件积分更新分支结束
L3743  speedIContribution = speedKi * speedIntegral;  // I 项：积分量乘速度环 K
L3745  speedAngleUnclamped = speedLoopDirection *  // 合成速度环倾角，先按环方向取符号
L3746  (speedPContribution + speedIContribution + speedDContribution) + positionLean.applied;  // P 加 I 加 D 后再叠加位置倾角
L3747  float desiredSpeedAngle = speedAngleUnclamped;  // 得到待限幅的目标倾角
L3750  speedDirectUnclampedOutput = 0.0f;  // 旧直投通道未限幅输出置零，仅留遥测
L3751  speedDirectOutput = 0.0f;  // 直投输出置零，该通道已退出控制
L3752  speedAccelerationFiltered = 0.0f;  // 加速度滤波中间量置零
L3753  speedPredictionDelta = 0.0f;  // 旧速度预测修正量置零
L3754  speedPredicted = vehicleSpeed;  // 预测速度直接取实测值
L3755  speedPredictPrevSpeed = vehicleSpeed;  // 预测历史速度同步实测，保持遥测连续
L3760  if (desiredSpeedAngle > MAX_SPEED_ANGLE)  // 目标倾角正向超过 N 上限
L3761  desiredSpeedAngle = MAX_SPEED_ANGLE;  // 钳到正向 N 限幅上限
L3766  if (desiredSpeedAngle < -MAX_SPEED_ANGLE)  // 目标倾角负向超过 N 上限
L3767  desiredSpeedAngle = -MAX_SPEED_ANGLE;  // 钳到负向 N 限幅上限
L3768  speedAngleLimited = desiredSpeedAngle;  // 记录限幅后的速度倾角
L3771  if (fabsf(speedKp) < 0.000001f && fabsf(speedKi) < 0.000001f && fabsf(speedKd) < 0.000001f)  // 速度环 P I D 全为零时的纯直立测试分支
L3772  {  // 速度环全关时的清零分支开始
L3773  positionLean.reset();  // 位置倾角控制器复位清零
L3774  speedAngleCorrection = 0.0f;  // 速度倾角修正清零，禁止残留污染直立环
L3775  speedIntegral = 0.0f;  // 速度环积分量直接清零
L3776  speedPContribution = 0.0f;  // 速度环 P 分量清零
L3777  speedIContribution = 0.0f;  // 速度环 I 分量清零
L3778  speedAngleUnclamped = 0.0f;  // 未限幅的速度环倾角清零
L3779  speedAngleLimited = 0.0f;  // 限幅后的速度倾角清零
L3780  speedErrorDotFilt = 0.0f;  // 微分项滤波状态量清零
L3781  speedErrorPrev = speedError;  // 误差历史与当前对齐，避免下拍假微分
L3782  speedDirectOutput = 0.0f;  // 旧直投通道输出量清零
L3783  speedDirectUnclampedOutput = 0.0f;  // 未限幅的直投输出清零
L3784  speedAccelerationFiltered = 0.0f;  // 加速度滤波中间量清零
L3785  speedPredictionDelta = 0.0f;  // 旧速度预测增量值清零
L3786  speedPredicted = vehicleSpeed;  // 预测速度直接取实测值
L3787  speedPredictPrevSpeed = vehicleSpeed;  // 预测历史速度同步实测
L3788  speedDirectAuthority = 0.0f;  // 直投通道的权限量清零
L3789  positionSpeedTarget = 0.0f;  // 位置环速度目标量清零
L3790  targetForwardSpeed = 0.0f;  // 总目标速度也随之清零
L3791  }  // 速度环全关清零分支结束
L3792  else  // 速度环增益有效，正常生成倾角
L3793  {  // 速度环正常输出分支开始
L3794  float speedAngleDelta = desiredSpeedAngle - speedAngleCorrection;  // 算本拍需要的倾角增量
L3796  if (speedAngleDelta > SPEED_ANGLE_SLEW_DEG_PER_UPDATE)  // 正增量超过单拍速率上限
L3797  speedAngleDelta = SPEED_ANGLE_SLEW_DEG_PER_UPDATE;  // 钳到单拍允许的最大增量
L3798  else if (speedAngleDelta < -SPEED_ANGLE_SLEW_DEG_PER_UPDATE)  // 负增量超过单拍速率下限
L3799  speedAngleDelta = -SPEED_ANGLE_SLEW_DEG_PER_UPDATE;  // 钳到负向单拍最大增量
L3801  speedAngleCorrection += speedAngleDelta;  // 按斜率累积成平滑的速度倾角修正
L3802  }  // 速度环正常输出分支结束
L3809  }  // updateOuterLoop 函数结束
L3828  void updateOuterLoopAuthority()  // 定义外环权限管理函数，按车身偏离平衡角给权限
L3830  {  // 外环权限管理函数体开始
L3832  float uprightError =  // 计算相对机械平衡角的直立误差
L3834  pitch  // 当前俯仰角参与误差计算
L3836  -  // 用俯仰角减去机械平衡角 O
L3838  balanceAngleOffset;  // O 机械平衡角，误差不含速度环偏移
L3842  float absUprightError =  // 取误差绝对值用于分级判定
L3844  fabsf(  // 对误差调用绝对值函数
L3846  uprightError  // 把直立误差传入绝对值函数
L3848  );  // 取绝对值的语句到此结束
L3853  if (  // 开始判定是否给满权限
L3855  absUprightError  // 直立误差的绝对值参与判定
L3857  <=  // 误差不超过全权限角阈值
L3859  OUTER_FULL_ANGLE_DEG  // 全权限阈值：该角内权限为 1
L3861  )  // 条件表达式结束，进入分支
L3863  {  // 外环满权限的处理分支开始
L3865  outerLoopAuthority =  // 给外环权限变量赋满值
L3867  1.0f;  // 权限为 1，外环满额生效
L3869  }  // 外环满权限的处理分支结束
L3871  else if (  // 进入权限线性过渡区判定
L3873  absUprightError  // 直立误差绝对值参与比较
L3875  <  // 误差尚未超过禁用角范围
L3877  OUTER_DISABLE_ANGLE_DEG  // 禁用角：超过它外环权限为零
L3879  )  // 条件表达式结束，进入分支
L3881  {  // 权限线性过渡分支开始
L3883  outerLoopAuthority =  // 按剩余区间算过渡权限
L3885  (  // 过渡权限分子括号开始
L3887  OUTER_DISABLE_ANGLE_DEG  // 禁用角作为分子被减数
L3889  -  // 减去当前直立误差绝对值
L3891  absUprightError  // 当前误差，得剩余可用量
L3893  )  // 过渡权限分子括号结束
L3895  /  // 除以过渡区宽度得比例
L3897  (  // 过渡权限分母括号开始
L3899  OUTER_DISABLE_ANGLE_DEG  // 禁用角作为分母被减数
L3901  -  // 再减去全权限角得区间宽度
L3903  OUTER_FULL_ANGLE_DEG  // 全权限角，得过渡区宽度
L3905  );  // 权限计算结果落在 0 到 1 之间
L3907  }  // 权限线性过渡分支结束
L3909  else  // 误差超过禁用角的分支
L3911  {  // 外环权限禁用分支开始
L3913  outerLoopAuthority =  // 外环权限变量直接置零
L3915  0.0f;  // 权限为 0，直立 PD 全力救车
L3917  }  // 外环权限禁用分支结束
L3929  float pitchOffset = manualForward.normalAuthority()*speedAngleCorrection + motionLean.applied;  // 速度倾角乘手动权限，再加运动倾角
L3930  targetBalanceAngle = balanceAngleOffset + outerLoopAuthority * pitchOffset;  // 目标角等于 O 加权限乘偏移，大角度自动回落
L3937  if (  // 开始判断权限是否过低
L3939  outerLoopAuthority  // 当前外环权限值参与比较
L3941  <  // 判据：权限小于 0.2
L3943  0.2f  // 过大角度判定用的权限阈值 0.2
L3945  )  // 条件表达式结束，进入分支
L3947  {  // 大角度积分衰减分支开始
L3949  speedIntegral *= SPEED_INTEGRAL_DECAY_BIG_ANGLE;  // 大角度时积分快速衰减，恢复后不推车
L3951  }  // 大角度积分衰减分支结束
L3953  }  // updateOuterLoopAuthority 函数结束
L3972  void updatePositionHold()  // 定义驻车位置快速拉回函数，400Hz 读编码器
L3973  {  // 驻车位置拉回函数体开始
L3974  int32_t a,b;noInterrupts();a=encoder1Count;b=encoder2Count;interrupts();  // 关中断原子读取两路编码器计数再开中断
L3975  int mode=(encoder1Trusted && encoder2Trusted)?0:(encoder1Trusted?1:(encoder2Trusted?2:3));  // 由编码器可信状态得出里程计模式
L3976  vehiclePosition=positionOdom.update(a,b,mode);  // 累加码盘位移得车体位置，单位 count
L3977  if(positionJog.active && (fabsf(pitch-balanceAngleOffset)>5 || fabsf(speedKp)<.000001f)){  // 点动车时车身偏 O 超 5 度或速度环关闭即故障
L3978  stopPositionJog(PositionJog::FAULT);return;  // 中止点动并返回，防止失控
L3979  }  // 点动故障中止分支结束
L3980  if(positionJog.active && mode!=positionJog.source){  // 点动中编码器可信模式变了
L3981  stopPositionJog(PositionJog::SOURCE_CHANGED);return;  // 因数据源切换中止点动并返回
L3982  }  // 数据源切换中止分支结束
L3985  bool followPosition=manualMotionIsActive() || manualForward.normalAuthority()<1.0f;  // 手动运动或权限过渡期，驻车环应跟随实际位置
L3986  bool resume=positionHoldSuspended && !followPosition;  // 本拍是否刚从挂起恢复
L3987  positionHoldSuspended=followPosition;  // 保存挂起状态供后续拍使用
L3988  if(followPosition){  // 处于位置跟随期的分支
L3989  if(positionJog.active)stopPositionJog(PositionJog::STOP);  // 跟随时停止点动，避免目标冲突
L3990  targetVehiclePosition=vehiclePosition;  // 驻车目标跟随当前位置，不积累旧误差
L3991  positionSpeedReference.reset();  // 速度参考斜率器复位，防阶跃
L3992  positionError=positionSpeedTarget=targetForwardSpeed=0;return;  // 清零误差与速度目标后返回
L3993  }  // 位置跟随期的分支结束
L3994  if(resume && !positionJog.active)targetVehiclePosition=vehiclePosition;  // 恢复瞬间以当前位置为目标，不拽回
L3995  if(positionJog.active)targetVehiclePosition=positionJog.reference;  // 点动中驻车目标跟随其参考轨迹
L3996  positionError=targetVehiclePosition-vehiclePosition;  // 算驻车位置误差供外环 P 使用
L3997  }  // updatePositionHold 函数结束
L4000  void captureCurrentPositionTarget()  // 定义把当前位置存为驻车目标的函数
L4001  {  // 目标捕获函数的左花括号，函数体从这里开始
L4002  if(positionJog.active)positionJog.cancel(PositionJog::STOP);  // 若点动仍在进行则先按STOP原因取消，避免带着点动目标去捕获
L4003  int32_t a,b;noInterrupts();a=encoder1Count;b=encoder2Count;interrupts();  // 关中断后原子读取两路编码器计数，防止读一半被中断改写
L4004  int mode=(encoder1Trusted && encoder2Trusted)?0:(encoder1Trusted?1:(encoder2Trusted?2:3));  // 由两路编码器可信标志算出0双路、1仅enc1、2仅enc2、3全失效
L4005  vehiclePosition=positionOdom.update(a,b,mode);  // 用本次计数和来源模式更新里程计，得到最新车体位置
L4007  targetVehiclePosition=vehiclePosition;  // 把当前车体位置写成新的位置保持目标，这就是目标捕获
L4008  positionError=positionSpeedTarget=0;  // 位置误差与位置环目标速度清零，捕获后不立刻产生输出
L4009  targetForwardSpeed=positionSpeedReference.applied;  // 目标前进速度退回速度环当前生效值，保持无跳变
L4010  speedIntegral=0;  // 速度积分清零，防止旧积分在下一拍继续累积
L4011  speedErrorPrev=targetForwardSpeed-vehicleSpeed;  // 用新目标速度初始化上一拍速度误差，抑制D项首次尖峰
L4012  speedErrorDotFilt=0;  // 速度误差微分滤波值清零，让滤波器重新起步
L4013  }  // 目标捕获函数在此结束，位置环从新目标重新开始
L4015  void stopPositionJog(int reason)  // 定义停止点动的接口，参数reason记录本次停止原因
L4016  {  // 停止点动函数的左花括号
L4017  positionJog.cancel(reason);  // 先按传入原因取消点动，让点动状态机退出运行态
L4018  captureCurrentPositionTarget();  // 再把当前位置重新捕获为目标，原地驻车不再继续漂移
L4019  }  // 停止点动函数到此结束，点动状态已复位
L4029  void updateBalanceLoop()  // 直立环更新函数，被400Hz主拍每拍调用一次
L4031  {  // 直立环函数体的左花括号
L4033  float angleError =  // 开始计算平衡角误差：目标角减当前倾角，分多行书写
L4035  targetBalanceAngle  // 误差表达式续行，这里是目标平衡角，即O所在的位置
L4037  -  // 误差表达式续行，这里是减号，表示目标角减倾角
L4039  pitch;  // 误差表达式收尾，减去当前俯仰角得到角度误差
L4052  float effectiveAngleError = 0.0f;  // 先把有效角度误差置0，再由下面的死区规则赋值
L4054  if (angleError > BALANCE_ANGLE_DEADBAND_DEG)  // 角度误差超过正死区宽度时进入正向分支
L4056  {  // 正向死区分支的左花括号
L4058  effectiveAngleError = angleError - BALANCE_ANGLE_DEADBAND_DEG;  // 正误差减去死区宽度，使P项从死区边界由0连续增长
L4060  }  // 正向死区分支结束，P项误差已连续化
L4062  else if (angleError < -BALANCE_ANGLE_DEADBAND_DEG)  // 角度误差低于负死区宽度时进入负向分支
L4064  {  // 负向死区分支的左花括号
L4066  effectiveAngleError = angleError + BALANCE_ANGLE_DEADBAND_DEG;  // 负误差加上死区宽度，负侧同样实现连续无突跳
L4068  }  // 负向死区分支结束，负侧误差同样连续
L4070  balancePOutput = balanceDirection * balanceKp * effectiveAngleError;  // 直立P项：方向系数乘P增益乘有效角度误差，得增量Hz
L4071  balanceDOutput = balanceDirection * (-balanceKd * gyroPitchRate);  // 直立D项：取陀螺角速度乘D增益的负值做阻尼，抑制晃动
L4072  balanceOutput = balancePOutput + balanceDOutput;  // P项加D项得到直立PD合计，作为后续积分的输入
L4091  speedDirectAuthority = 0.0f;  // 本版不用直接速度修正，权限系数固定为0
L4104  balanceAccelDivider++;  // 100Hz分频计数器自增，每4个400Hz拍进一次
L4105  if (balanceAccelDivider >= BALANCE_ACCEL_UPDATE_DIVIDER)  // 计满4拍即10ms时，才执行一次加速度式直立计算
L4106  {  // 100Hz更新分支的左花括号
L4107  balanceAccelDivider = 0;  // 分频计数器清零，重新开始下一个10ms窗口
L4108  balanceAccelRawOutput = BALANCE_ACCEL_SCALE * balanceOutput;  // 直立PD输出乘缩放系数，换算成本拍频率增量Hz
L4113  balanceAccelEffectiveLimit = BALANCE_OUTPUT_SLEW_HZ;  // 默认使用S限幅，即每100Hz端点频率增量上限
L4114  highSpeedAccelProtectionActive = false;  // 高速保护标志先置false，下面满足条件才置位
L4115  const bool acceleratingFurther =  // 判断当前是否仍在朝同一方向继续加速，供高速保护用
L4116  integratedMotorSpeedCommand * balanceAccelRawOutput > 0.0f;  // 速度命令与增量同号即仍在同向加速，条件是乘积大于0
L4117  if (fabsf(integratedMotorSpeedCommand) >= HIGH_SPEED_ACCEL_START_HZ &&  // 已达高速起点且仍在同向加速时进入收紧分支
L4118  acceleratingFurther)  // 上一行条件收尾：确认为继续加速
L4119  {  // 高速收紧分支的左花括号
L4120  balanceAccelEffectiveLimit = HIGH_SPEED_ACCEL_LIMIT_HZ;  // 高速段改用更小的增量上限，避免频率陡升导致丢步
L4121  if (balanceAccelEffectiveLimit > BALANCE_OUTPUT_SLEW_HZ)  // 若该上限反而比S大，则不允许它放宽S
L4122  balanceAccelEffectiveLimit = BALANCE_OUTPUT_SLEW_HZ;  // 收回到S，保证这层保护只会更严不会更松
L4123  highSpeedAccelProtectionActive = true;  // 记录本拍处于高速加速保护，供遥测观察
L4124  }  // 高速收紧分支结束，本拍限幅值已确定
L4127  if (reversalSoftTicksRemaining > 0)  // 换向软窗口还有剩余拍数时进入软过渡
L4128  {  // 换向软窗口分支的左花括号
L4129  reversalSoftTicksRemaining--;  // 窗口剩余拍数递减一格，走完自动关闭窗口
L4130  balanceAccelEffectiveLimit *= REVERSAL_SOFT_LIMIT_SCALE;  // 增量限幅再乘缩小系数，换向瞬间更柔和
L4131  }  // 换向软窗口分支结束，窗口剩余拍数已更新
L4133  balanceAccelLimitedOutput = balanceAccelRawOutput;  // 先把原始增量当作限幅结果初值
L4135  if (balanceAccelLimitedOutput > balanceAccelEffectiveLimit)  // 正向越限时按本拍有效上限截断
L4136  balanceAccelLimitedOutput = balanceAccelEffectiveLimit;  // 正向截断到有效上限，增量不再继续增大
L4137  else if (balanceAccelLimitedOutput < -balanceAccelEffectiveLimit)  // 负向越限时按负的有效上限截断
L4138  balanceAccelLimitedOutput = -balanceAccelEffectiveLimit;  // 负向截断到负有效上限
L4140  balanceAccelSubstepOutput =  // 把本拍增量平摊到4个400Hz子拍，消除频率阶跃
L4141  balanceAccelLimitedOutput / (float)BALANCE_ACCEL_UPDATE_DIVIDER;  // 除以分频数，得到每个2.5ms子拍执行的增量
L4142  }  // 100Hz更新分支结束
L4146  float stepIncrement = balanceAccelSubstepOutput;  // 取本子拍要执行的频率增量，即端点增量的四分之一
L4147  float candidateSpeed = integratedMotorSpeedCommand + stepIncrement;  // 预演积分后的候选速度，仅用于换向判断
L4148  int nextSign = candidateSpeed > 1.0f ? 1 : (candidateSpeed < -1.0f ? -1 : 0);  // 按±1Hz门限给出候选速度符号，0表示接近零速
L4149  if (nextSign != 0 && prevIntegratedSpeedCmdSign != 0 &&  // 候选符号与前次符号都非零且相反，说明正在换向
L4150  nextSign != prevIntegratedSpeedCmdSign) {  // 换向条件的第二部分与分支左花括号
L4151  if (reversalSoftTicksRemaining == 0) {  // 仅在本窗口尚未开启时收紧一次，避免重复缩放
L4152  balanceAccelSubstepOutput *= REVERSAL_SOFT_LIMIT_SCALE;  // 子拍增量乘缩小系数，换向瞬间放慢频率变化
L4153  balanceAccelLimitedOutput *= REVERSAL_SOFT_LIMIT_SCALE;  // 已限幅增量同步缩小，保持遥测量之间一致
L4154  balanceAccelEffectiveLimit *= REVERSAL_SOFT_LIMIT_SCALE;  // 有效限幅同步缩小，本拍后续都用收紧后的值
L4155  stepIncrement = balanceAccelSubstepOutput;  // 用缩小后的子拍增量覆盖待执行增量
L4156  }  // 首次收紧分支结束，随后打开软窗口
L4157  reversalSoftTicksRemaining = REVERSAL_SOFT_TICKS;  // 打开换向软窗口，接下来若干拍继续用收紧限幅
L4158  }  // 换向判断分支结束，下一步做增量积分
L4159  integratedMotorSpeedCommand += stepIncrement;  // 把本子拍增量累加进频率积分，得到新的STEP频率
L4160  int actualSign = integratedMotorSpeedCommand > 1.0f ? 1 :  // 按±1Hz门限给出积分后速度命令的符号，分两行写
L4161  (integratedMotorSpeedCommand < -1.0f ? -1 : 0);  // 符号表达式的续行：低于-1Hz为负向，其余为0
L4162  if (actualSign != 0) prevIntegratedSpeedCmdSign = actualSign;  // 符号非零时记录下来，供下一拍判断是否发生换向
L4163  if (integratedMotorSpeedCommand > BALANCE_OUTPUT_LIMIT_HZ)  // 频率积分结果正向超过输出限幅L时截断
L4164  integratedMotorSpeedCommand = BALANCE_OUTPUT_LIMIT_HZ;  // 正向截到+L，频率不会超过输出限幅
L4165  else if (integratedMotorSpeedCommand < -BALANCE_OUTPUT_LIMIT_HZ)  // 频率积分结果负向超过-L时截断
L4166  integratedMotorSpeedCommand = -BALANCE_OUTPUT_LIMIT_HZ;  // 负向截到-L，频率不会低于负限幅
L4168  speedDirectAppliedOutput = 0.0f;  // 直接速度修正不参与，应用值固定为0
L4169  mixedBalanceOutput = integratedMotorSpeedCommand;  // 混合输出就等于积分后的STEP频率
L4171  if (mixedBalanceOutput > BALANCE_OUTPUT_LIMIT_HZ)  // 混合结果正向超L时按L限幅
L4172  limitedBalanceOutput = BALANCE_OUTPUT_LIMIT_HZ;  // 正向限幅到L，混合输出不越上限
L4173  else if (mixedBalanceOutput < -BALANCE_OUTPUT_LIMIT_HZ)  // 混合结果负向超-L时按-L限幅
L4174  limitedBalanceOutput = -BALANCE_OUTPUT_LIMIT_HZ;  // 负向限幅到-L，混合输出不越下限
L4175  else  // 两侧都未越限时走不截断分支
L4176  limitedBalanceOutput = mixedBalanceOutput;  // 未越限则原样保留混合结果
L4179  finalBalanceOutput = limitedBalanceOutput;  // 积分已平滑，这里不再二次slew，直接作为最终前后输出
L4180  lastFinalBalanceOutput = finalBalanceOutput;  // 缓存上一拍最终输出，供遥测与前后对比
L4182  }  // 直立环更新函数到此结束，最终输出已就绪
L4199  void updateTurnLoop()  // 转向环更新函数定义，与直立环同拍调用
L4200  {  // 转向环函数体的左花括号
L4201  float turnError = targetTurnRate - gyroTurnRate;  // 转向误差等于目标转向角速度减陀螺实测转向角速度
L4202  float derivative = (turnError-lastTurnError) / CONTROL_DT;  // 误差差分除控制周期得到转向微分项
L4203  lastTurnError = turnError;  // 保存本拍误差，供下一拍做差分
L4204  float legacy = turnKp*turnError + turnKd*derivative;  // 传统PD转向输出：P乘误差加D乘微分，仅作参考量
L4205  turnOutput = manualTurn.update(millis(), CONTROL_DT,  // 交给手动转向状态机：传入时间、周期与倾角等信息
L4206  pitch-balanceAngleOffset, BALANCE_OUTPUT_LIMIT_HZ,  // 第二个参数是倾角减平衡角偏移O，即相对平衡角的倾角
L4207  finalBalanceOutput, legacy);  // 再传输出限幅L、当前直立最终输出和传统PD结果
L4208  }  // 转向环更新函数到此结束，转向输出已就绪
L4225  void sendTelemetry()  // 全量遥测发送函数定义，按D帧输出几十个字段
L4226  {  // 全量遥测函数体的左花括号
L4229  char line[896];  // 896字节行缓冲，注释说明要装74个数值字段并追加新字段
L4230  int n = snprintf(line, sizeof(line),  // 用snprintf把整行格式化进缓冲，并返回写入长度
L4231  "D,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%d,%d,%d,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",  // D帧格式串，按字段顺序排好74个%d与%ld占位
L4232  (unsigned long)millis(),  // 第0列：开机以来的毫秒时间戳
L4233  (int)(pitch * 100.0f),  // 1列：卡尔曼倾角乘100，单位0.01度
L4234  (int)(gyroPitchRate * 100.0f),  // 2列：陀螺俯仰角速度乘100
L4235  (int)(targetBalanceAngle * 100.0f),  // 3列：目标平衡角乘100
L4236  (int)balancePOutput,  // 4列：直立P项输出，单位Hz
L4237  (int)balanceDOutput,  // 5列：直立D项输出，单位Hz
L4238  (int)balanceOutput,  // 6列：直立PD合计，单位Hz
L4239  (int)(speedAngleCorrection * 100.0f),  // 7列：速度环给出的目标角修正，乘100
L4240  (int)(outerLoopAuthority * 1000.0f),  // 8列：外环权限乘1000，0表示外环完全退出
L4241  (int)speedDirectOutput,  // 9列：直接制动输出，独立H限幅后、权限前
L4242  (int)speedDirectAppliedOutput,  // 10列：乘过外环权限后的直接制动输出
L4243  (int)(positionSpeedTarget * 1000.0f),  // 11列：位置环生成的目标速度乘1000
L4244  (int)mixedBalanceOutput,  // 12列：直立与直接制动混合、总限幅前频率
L4245  (int)limitedBalanceOutput,  // 13列：总限幅L之后、平滑之前的输出
L4246  (int)finalBalanceOutput,  // 14列：平滑后最终前后输出频率
L4247  (int)turnOutput,  // 15列：转向环输出频率
L4248  (int)(vehicleSpeed * 1000.0f),  // 16列：融合后的车速乘1000
L4249  (int)(rawVehicleSpeed * 1000.0f),  // 17列：编码器原始速度乘1000，尚未融合
L4250  (int)(speedAccelerationFiltered * 1000.0f),  // 18列：滤波后的速度加速度乘1000
L4251  (int)(speedPredicted * 1000.0f),  // 19列：预测速度乘1000
L4252  (int)(speedPredictionDelta * 1000.0f),  // 20列：预测增量乘1000
L4253  (int)speedDirectUnclampedOutput,  // 21列：STEP独立H限幅之前的原始输出
L4254  (int)(vehiclePosition * 100.0f),  // 22列：车体位置乘100，由编码器计数换算
L4255  (int)(positionError * 100.0f),  // 23列：位置误差乘100
L4256  (long)encoder1Count,  // 24列：编码器1累计计数
L4257  (long)encoder2Count,  // 25列：编码器2累计计数
L4258  (int)motor1LastCommandHz,  // 26列：电机1最后下发的命令频率
L4259  (int)motor2LastCommandHz,  // 27列：电机2最后下发的命令频率
L4260  (int)(accPitch * 100.0f),  // 28列：加速度计算出的倾角乘100
L4261  (int)(kalmanBias * 1000.0f),  // 29列：卡尔曼偏置乘1000
L4262  (int)(gyroIntegAngle * 100.0f),  // 30列：陀螺积分角乘100
L4263  (int)balanceAccelRawOutput,  // 31列：直立PD换算的原始频率增量，每10ms一次
L4264  (int)balanceAccelLimitedOutput,  // 32列：经S限幅后的频率增量
L4265  (int)integratedMotorSpeedCommand,  // 33列：增量积分后的STEP频率
L4266  (int)(speedError * 1000.0f),  // 34列：速度误差乘1000
L4267  (int)(speedPContribution * 100.0f),  // 35列：速度环P项给出的角度贡献乘100
L4268  (int)(speedIContribution * 100.0f),  // 36列：速度环I项给出的角度贡献乘100
L4269  (int)(speedAngleUnclamped * 100.0f),  // 37列：N限幅前的速度修正目标角乘100
L4270  (int)(speedAngleLimited * 100.0f),  // 38列：N限幅后、平滑前的目标角乘100
L4271  (int)balanceAccelEffectiveLimit,  // 39列：本拍实际生效的增量限幅，可看高速与换向收紧
L4272  (int)rawSpeedEncoderMode,  // 40列：编码器来源0双路 1仅enc1 2仅enc2 3全失效
L4273  (int)(dynamicR * 1000.0f),  // 41列：动态R乘1000，越大越不信加速度计角度
L4274  (int)(accelMagnitudeError * 1000.0f),  // 42列：加速度模长误差乘1000，是动态R的驱动量
L4275  (unsigned long)fallStopCount,  // 43列：倒地保护累计触发次数，能抓到短促误触发
L4276  (int)(telemGyroAbsMax * 100.0f),  // 44列：窗口内陀螺绝对值峰值乘100，看有无尖峰
L4277  (int)(telemAccPitchDevMax * 100.0f),  // 45列：窗口内加速度角与陀螺积分角最大偏差乘100
L4278  (int)telemBalHzAbsMax,  // 46列：窗口内直立输出峰值，反映饱和深度
L4279  (int)(telemKalmanBiasMin * 1000.0f),  // 47列：窗口内卡尔曼偏置最小值乘1000
L4280  (int)(telemKalmanBiasMax * 1000.0f),  // 48列：窗口内卡尔曼偏置最大值乘1000
L4281  (int)(manualForward.requested*1000.0f),  // 49列：手动前进请求值乘1000
L4282  (int)(manualForward.applied*1000.0f),  // 50列：手动前进实际生效值乘1000
L4283  (int)(0.0f),  // 51列：手动I项占位，自学习已移除故固定0以保持列数
L4284  (int)manualTurn.requested,  // 52列：手动转向请求值
L4285  (int)positionHoldSuspended,  // 53列：位置保持是否被挂起
L4286  (manualForward.needsZero ? 1 : 0) | (manualTurn.needsZero ? 2 : 0),  // 54列：两位标志，bit0前进需归零、bit1转向需归零
L4287  (unsigned long)(millis()-manualForward.receivedMs),  // 55列：距上次收到手动前进指令的毫秒数
L4288  5,  // 56列：手动控制模式，5表示±2度固定平滑保持
L4289  (int)(targetVehiclePosition*100), (int)positionJog.active,  // 57、58列：位置保持目标乘100与点动是否激活
L4290  (unsigned long)positionJog.sequence, positionJog.reason,  // 59、60列：点动序号与最近一次停止原因
L4291  (encoder1Trusted && encoder2Trusted)?0:(encoder1Trusted?1:(encoder2Trusted?2:3)),  // 61列：编码器来源模式，与第40列同源复算
L4292  (int)(positionJog.speed*1000),  // 62列：点动速度乘1000
L4293  (int)(positionJog.goal*100),(int)(positionJog.referenceSpeed*1000),  // 63、64列：点动目标位置乘100与参考速度乘1000
L4294  (int)(positionLean.requested*1000),(int)(positionLean.applied*1000),  // 65、66列：位置倾斜请求值与生效值乘1000
L4295  (int)(positionJog.arrival.error*100),positionJog.arrival.goodWindows,  // 67、68列：点动到位误差乘100与连续合格窗口数
L4296  (int)(positionLean.kp*1000000),(int)(positionLean.limit*1000),  // 69、70列：位置倾斜P增益乘1e6与限幅乘1000
L4297  (int)(manualForward.rate*1000),manualForward.phase,(int)(manualForward.normalAuthority()*1000));  // 71至74列：手动前进速率、相位与外环权限乘1000
L4298  if (n > 0 && (size_t)n < sizeof(line))  // 只在写入长度有效且未超缓冲时才发送，防止越界
L4299  txPushLine(line, (uint16_t)n);  // 把整行压入发送环形缓冲，由后台异步发出
L4302  telemGyroAbsMax = 0.0f;  // 清空陀螺峰值，开始累积下一个250ms窗口
L4303  telemAccPitchDevMax = 0.0f;  // 清空倾角偏差峰值，供下个窗口统计
L4304  telemBalHzAbsMax = 0.0f;  // 清空直立输出峰值，供下个窗口统计
L4305  telemKalmanBiasMin = kalmanBias;  // 卡尔曼偏置最小值用当前值复位
L4306  telemKalmanBiasMax = kalmanBias;  // 卡尔曼偏置最大值也用当前值复位
L4307  }  // 全量遥测函数到此结束，窗口极值已复位
L4320  void sendFastTelemetry()  // 精简遥测函数定义，G帧只发少量关键量
L4321  {  // 精简遥测函数体的左花括号
L4322  char line[96];  // 96字节缓冲足够放下G帧的几十个字符
L4323  int n = snprintf(line, sizeof(line), "G,%lu,%d,%d,%d,%d,%d,%d,%d",  // G帧格式串：时间戳加7个整数字段
L4324  (unsigned long)millis(),  // 第1个字段：开机毫秒时间戳
L4325  (int)(vehiclePosition * 100.0f),  // 第2个字段：车体位置乘100
L4326  (int)(vehicleSpeed * 1000.0f),  // 第3个字段：融合速度乘1000
L4327  (int)(pitch * 100.0f),  // 第4个字段：卡尔曼倾角乘100
L4328  (int)(targetBalanceAngle * 100.0f),  // 第5个字段：目标平衡角乘100
L4329  (int)finalBalanceOutput,  // 第6个字段：平滑后最终前后输出频率
L4330  (int)(manualForward.requested * 1000.0f),  // 第7个字段：手动前进请求乘1000
L4331  (int)btRxBytes);  // 第8个字段：蓝牙累计接收字节数，判断链路是否通
L4332  if (n > 0)  // 只在格式化成功时才发送
L4333  txPushLine(line, (uint16_t)n);  // 压入发送缓冲，由BLE异步发出
L4334  }  // 精简遥测函数到此结束，G帧已发出
L4336  void setup()  // setup函数定义，上电只跑一次，负责全部硬件初始化
L4338  {  // setup函数体的左花括号
L4340  initMotors();  // 初始化电机STEP/DIR引脚与定时器，先把电机置为停止
L4344  initEncoders();  // 初始化两路编码器输入，建立计数基准
L4356  BTSerial.begin(  // 打开蓝牙串口，波特率由BT_BAUD决定，分三行书写
L4358  BT_BAUD  // 波特率参数用宏BT_BAUD，这里是9600
L4360  );  // 串口初始化调用的右括号与分号，结束该语句
L4364  delay(100);  // 先等100ms让蓝牙模块串口稳定，再继续后面初始化
L4366  #if TMC_STANDALONE_STEP_DIR  // 预处理分支：TMC走独立STEP/DIR模式时编译下面这段
L4369  BTSerial.println("TMC STEP/DIR standalone: UART disabled; mode/current/microsteps hardware-selected");  // 独立模式由硬件跳线定模式电流细分，串口打印告知
L4370  #elif TMC_GPIO_WRITE_ONLY  // 否则若为只写GPIO配置模式，就编译这一段
L4371  stopBothMotors();  // 初始化前先让两个电机停下，确保安全
L4372  bool rearLineHigh = false, frontLineHigh = false;  // 准备记录两路TMC的UART线电平状态
L4373  while (!initTmcDriversWriteOnly(rearLineHigh, frontLineHigh)) {  // 循环尝试用只写方式配置TMC，失败就重试直到成功
L4374  stopBothMotors();  // 每次重试前都先停电机，避免误动作
L4375  BTSerial.print("TMC WRITE-ONLY WAIT rearLineHigh="); BTSerial.print(rearLineHigh);  // 打印等待提示与后轮线路电平，便于排查接线
L4376  BTSerial.print(" frontLineHigh="); BTSerial.println(frontLineHigh);  // 打印前轮线路电平并换行，完成一次状态输出
L4377  delay(1000);  // 等1秒再重试，避免刷屏和总线冲突
L4378  }  // 只写配置的重试循环结束
L4379  BTSerial.print("TMC WRITE-ONLY SENT rearBytes="); BTSerial.print(rearTmcUart.txCount);  // 打印已写出的后轮字节数，作为成功率参考
L4380  BTSerial.print(" frontBytes="); BTSerial.print(frontTmcUart.txCount);  // 打印前轮已写字节数，与后轮一起核对写入量
L4381  BTSerial.println(" spreadCycle=REQUESTED current=1200mA microsteps=16 UNVERIFIED");  // 声明请求的斩波模式电流细分，并标注未被回读验证
L4382  #else  // 否则走UART可回读的TMC配置分支
L4384  while (!initTmcDrivers()) {  // 循环尝试用UART配置TMC2209，失败重试
L4385  stopBothMotors();  // 重试期间保持电机停止
L4386  delay(1000);  // 每次失败等1秒再试，等驱动器上电稳定
L4387  }  // UART配置重试循环结束
L4388  BTSerial.println("TMC UART OK: both spreadCycle, 1/16 step");  // 两片TMC都配好，打印spreadCycle与16细分确认
L4389  #endif  // 结束TMC初始化的条件编译分支
L4403  paramStorageBoot();  // 从flash读回22项参数覆盖编译默认值，实现参数持久化
L4405  while (!initBMI270())  // 循环尝试初始化BMI270，失败则重试
L4406  {  // BMI270重试循环的左花括号
L4407  stopBothMotors();  // 重试前先停电机，保证车体静止
L4408  clearControlOutputDiagnostics();  // 清空控制输出诊断量，避免带旧值进入控制
L4409  BTSerial.println("BOOT RETRY: BMI270 init failed");  // 打印初始化失败提示，方便看串口定位问题
L4410  delay(1000);  // 等1秒再重试，给BMI270上电留出时间
L4411  }  // BMI270重试循环结束
L4413  BMIWire.setClock(400000);  // 把BMI270的I2C时钟提到400kHz，满足400Hz采样带宽
L4414  delay(20);  // 等20ms让I2C总线稳定
L4427  while (!configureBMI270_400Hz())  // 循环把BMI270配成400Hz输出，失败重试
L4428  {  // 400Hz配置重试循环的左花括号
L4429  stopBothMotors();  // 重试时保持电机停止，避免上电瞬间误动作
L4430  clearControlOutputDiagnostics();  // 清空控制输出诊断量，重试前不留旧数据
L4431  BTSerial.println("BOOT RETRY: BMI270 400Hz config failed");  // 打印400Hz配置失败提示
L4432  delay(500);  // 等500ms再重试，让400Hz配置稳定生效
L4433  }  // 400Hz配置重试循环结束
L4446  while (!calibrateIMU())  // 循环做IMU零偏校准，失败重试
L4447  {  // 校准重试循环的左花括号
L4448  stopBothMotors();  // 校准时必须停电机，否则零偏会被振动污染
L4449  clearControlOutputDiagnostics();  // 清空控制输出诊断量，校准前不留旧数据
L4450  BTSerial.println("BOOT RETRY: IMU calibration failed; keep car still");  // 打印校准失败提示，并提醒保持车体静止
L4451  delay(1000);  // 等1秒再重试，等车体完全静止后再校准
L4452  }  // 校准重试循环结束，IMU零偏已经可用
L4454  BTSerial.println("BOOT OK: BMI270 ready");  // 打印BMI270就绪，初始化阶段到此完成
L4467  noInterrupts();  // 关中断，下面要一次性读走四个计数器做基准
L4471  lastEncoder1Count =  // 记录编码器1初值作为上一拍计数基准，分两行写
L4473  encoder1Count;  // 续行：赋值为encoder1Count
L4477  lastEncoder2Count =  // 记录编码器2初值作为上一拍计数基准，分两行写
L4479  encoder2Count;  // 续行：赋值为encoder2Count
L4483  positionZeroCount1 =  // 记录编码器1的位置零点，用于把计数换算成位置
L4485  encoder1Count;  // 续行：位置零点取当前encoder1Count
L4489  positionZeroCount2 =  // 记录编码器2的位置零点，用于位置换算
L4491  encoder2Count;  // 续行：位置零点取当前encoder2Count
L4495  interrupts();  // 恢复中断，这段临界区结束
L4499  vehiclePosition =  // 位置初始化语句开始，其余部分在下一分片继续
L4501  0.0f;  // 上一条 vehiclePosition 赋值的数值部分，把位置计数整体清零
L4505  targetVehiclePosition =  // 开始给目标车辆位置 targetVehiclePosition 赋值
L4507  0.0f;  // targetVehiclePosition 取 0，位置环目标点先定在原点
L4511  positionError = 0.0f;  // 位置误差清零，位置环上电不应带历史误差
L4512  positionSpeedTarget = 0.0f;  // 位置环输出的速度目标清零，避免开机立刻朝某一方向跑
L4513  positionHoldWasEnabled = false;  // 清位置保持标志，位置保持必须重新使能才生效
L4526  vehicleSpeed =  // 开始给 Kalman 融合车速 vehicleSpeed 赋值
L4528  0.0f;  // vehicleSpeed 取 0，融合车速从静止状态开始估计
L4532  rawVehicleSpeed =  // 开始给未滤波的原始车速 rawVehicleSpeed 赋值
L4534  0.0f;  // rawVehicleSpeed 取 0，编码器差分的车速初值为静止
L4538  imuForwardAccelG =  // 开始给 IMU 前向加速度 imuForwardAccelG 赋值
L4540  0.0f;  // imuForwardAccelG 取 0，开机认为车身没有前向加速度
L4544  imuForwardAccelFilteredG =  // 开始给滤波后的前向加速度 imuForwardAccelFilteredG 赋值
L4546  0.0f;  // 滤波加速度取 0，低通状态从零开始收敛
L4550  imuSpeedPredictionTrust =  // 开始给 imuSpeedPredictionTrust 赋值
L4552  1.0f;  // 可信度初值取 1，开机完全信任 IMU 加速度外推
L4556  speedKalmanP =  // 开始给速度 Kalman 协方差 speedKalmanP 赋值
L4558  1.0f;  // 协方差初值 1，表示初始速度估计的不确定性中等
L4562  speedKalmanQ =  // 开始给 Kalman 过程噪声 speedKalmanQ 赋值
L4564  SPEED_KALMAN_Q_BASE;  // Q 取基础常量，速度模型的过程噪声用编译期基准值
L4568  speedKalmanR =  // 开始给 Kalman 观测噪声 speedKalmanR 赋值
L4570  SPEED_KALMAN_R_LOW_SPEED;  // R 取低速档常量，静止附近更信任编码器速度观测
L4574  speedError =  // 开始给速度环误差 speedError 赋值
L4576  0.0f;  // speedError 取 0，速度 PI 第一拍从零误差算起
L4584  speedDirectOutput =  // 开始给速度直通输出 speedDirectOutput 赋值
L4586  0.0f;  // 直通输出取 0，开机直通通路不预先出力
L4590  speedDirectAuthority =  // 开始给直通通路权限 speedDirectAuthority 赋值
L4592  0.0f;  // 直通权限取 0，开机默认不启用速度直通混合
L4596  finalBalanceOutput =  // 开始给最终直立输出 finalBalanceOutput 赋值
L4598  0.0f;  // finalBalanceOutput 取 0，电机拿到 0Hz 即保持不转
L4602  targetForwardSpeed =  // 开始给目标前进速度 targetForwardSpeed 赋值
L4604  0.0f;  // 目标速度取 0，开机不让车自己往前走
L4609  targetBalanceAngle =  // 开始给目标平衡角 targetBalanceAngle 赋值
L4611  balanceAngleOffset;  // 目标平衡角取 O（balanceAngleOffset），直立环围绕平衡点工作
L4615  outerLoopDivider =  // 开始给外环分频计数 outerLoopDivider 赋值
L4617  0;  // 分频计数取 0，400Hz 主拍累加到 4 才跑一次 100Hz 外环
L4621  telemetryDivider =  // 开始给遥测分频计数 telemetryDivider 赋值
L4623  0;  // 遥测分频计数取 0，先发第一帧再按分频间隔上传
L4627  imuFailCount =  // 开始给 IMU 连续失败计数 imuFailCount 赋值
L4629  0;  // 失败计数取 0，开机视为 IMU 工作正常
L4633  fallConfirmCount =  // 开始给摔倒确认计数 fallConfirmCount 赋值
L4635  0;  // 摔倒确认计数取 0，避免开机瞬间被误判为摔倒
L4639  motor1ZeroHoldCount =  // 开始给电机1零位保持计数 motor1ZeroHoldCount 赋值
L4641  0;  // motor1ZeroHoldCount 取 0，清掉开机前的保持计时
L4645  motor2ZeroHoldCount =  // 开始给电机2零位保持计数 motor2ZeroHoldCount 赋值
L4647  0;  // motor2ZeroHoldCount 取 0，与电机1对称清零
L4651  outerLoopAuthority =  // 开始给外环权限 outerLoopAuthority 赋值
L4653  1.0f;  // 外环权限取 1，角度在安全范围内时外环全权修正
L4659  BTSerial.println(  // 通过蓝牙串口输出 CSV 表头，先打印 println 的函数名与左括号
L4661  "time_ms,fused_speed,speed_direct_hz"  // 表头字符串：时间、融合车速、直通输出三列，网页端按此解析
L4663  );  // println 的右括号，表头输出到此结束并换行
L4668  lastControlUs =  // 开始给控制周期时间戳 lastControlUs 赋值
L4670  micros();  // 取 micros() 作主拍基准，后面按 2500us 定出 400Hz 节拍
L4672  }  // setup() 函数到此结束，下面的代码都属于主循环
L4707  #include "param_store.h"  // 引入参数持久化头文件，拿到 ParamStore 的槽位表与读写接口
L4708  #ifdef STM32F4xx  // 仅当目标芯片定义了 STM32F4xx 时才编译下面这段 flash 端口代码
L4709  #include "param_flash_f4.h"  // 引入 F4 内部 flash 扇区读写实现，参数落到 sector 10/11
L4710  #endif  // 条件编译结束，非 F4 平台没有真实 flash 端口
L4713  static const struct ParamSlotDef {  // 定义参数槽位结构体 ParamSlotDef：一个字母键配一个 float 指针
L4714  char key;  // 结构体成员1：key 是面板与 BLE 共用的单字母参数名
L4715  float* ptr;  // 结构体成员2：ptr 指向真正的全局参数变量，读写都走它
L4716  } paramSlots[ParamStore::PARAM_COUNT] = {  // 声明 22 项槽位表 paramSlots，长度由 PARAM_COUNT 决定
L4717  {'P', &balanceKp},                  {'D', &balanceKd},  // 槽位前两项：P 绑 balanceKp，D 绑 balanceKd
L4718  {'O', &balanceAngleOffset},         {'Z', &BALANCE_ANGLE_DEADBAND_DEG},  // 槽位第3、4项：O 绑平衡角偏移，Z 绑平衡角死区（度）
L4719  {'L', &BALANCE_OUTPUT_LIMIT_HZ},    {'S', &BALANCE_OUTPUT_SLEW_HZ},  // 槽位第5、6项：L 绑输出限幅 Hz，S 绑每周期最大变化 Hz
L4720  {'R', &BALANCE_ACCEL_SCALE},        {'B', &HIGH_SPEED_ACCEL_START_HZ},  // 槽位第7、8项：R 绑加速度缩放，B 绑高速加速度补偿起点 Hz
L4721  {'H', &HIGH_SPEED_ACCEL_LIMIT_HZ},  {'K', &speedKp},  // 槽位第9、10项：H 绑高速补偿上限 Hz，K 绑速度环 P
L4722  {'I', &speedKi},                    {'N', &MAX_SPEED_ANGLE},  // 槽位第11、12项：I 绑速度环 I，N 绑速度修正角上限（度）
L4723  {'W', &SPEED_INTEGRAL_LIMIT},       {'J', &SPEED_ERROR_DEADBAND},  // 槽位第13、14项：W 绑速度积分限幅，J 绑速度误差死区
L4724  {'E', &speedKd},                    {'X', &positionKp},  // 槽位第15、16项：E 绑速度环 D，X 绑位置环 P
L4725  {'Y', &MAX_POSITION_RETURN_SPEED},  {'M', &POSITION_DEADBAND_COUNT},  // 槽位第17、18项：Y 绑位置回位速度上限，M 绑位置死区 count
L4726  {'F', &OUTER_FULL_ANGLE_DEG},       {'G', &OUTER_DISABLE_ANGLE_DEG},  // 槽位第19、20项：F 绑外环全权角，G 绑外环失效角（度）
L4727  {'T', &turnKp},                     {'U', &turnKd},  // 槽位第21、22项：T 绑转向环 P，U 绑转向环 D
L4728  };  // 槽位表初始化结束，顺序必须与 PAR 回显、面板 22 项一致
L4730  static float paramDefaults[ParamStore::PARAM_COUNT];  // 开机把 22 个编译默认值抄进 paramDefaults，供 DEF 恢复
L4731  static bool  paramDirty = true;  // 脏标志初值 true，表示当前 RAM 值尚未与 flash 记录核对一致
L4732  static bool  paramSaved = false;  // paramSaved 初值 false，表示还没确认 flash 有记录
L4733  static ParamStore::Store::Scan paramScan;  // 保存开机扫描结果：记录序号、所在扇区、是否找到有效记录
L4735  #ifdef STM32F4xx  // 同样只在 STM32F4xx 下编译下面这段 flash 端口实例
L4736  static Stm32F4FlashPort paramFlashPort;  // 实例化 F4 flash 端口对象，内部封装扇区擦除与写入
L4737  static ParamStore::FlashPort& paramPort = paramFlashPort;  // 用引用 paramPort 绑定端口，上层代码不直接依赖具体芯片
L4738  #endif  // 条件编译结束，非 F4 平台不提供 paramPort
L4740  static int paramSlotIndexOf(char key) {  // 定义按单字母查槽位下标的函数 paramSlotIndexOf
L4741  char upper = (char)toupper((unsigned char)key);  // 先把入参字母转成大写，命令大小写都接受
L4742  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT; ++i) {  // 遍历整张 22 项槽位表
L4743  if (paramSlots[i].key == upper) return (int)i;  // 字母命中就返回下标，调用方据此知道该改哪个参数
L4744  }  // 查表循环体到此结束，下面处理未命中
L4745  return -1;  // 遍历完仍未命中返回 -1，表示不是已知参数键
L4746  }  // 查表函数到此结束，返回 -1 表示未命中
L4748  static void paramCollect(float* values) {  // 定义 paramCollect：把 22 个全局参数收集成数组准备写盘
L4749  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT; ++i) values[i] = *paramSlots[i].ptr;  // 逐项解引用槽位指针取出当前值，顺序与槽位表一致
L4750  }  // 收集函数到此结束，values 装满 22 项
L4752  static void paramApply(const float* values) {  // 定义 paramApply：把数组里的值写回 22 个全局参数
L4753  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT; ++i) {  // 逐项遍历 22 个参数槽位
L4754  if (isfinite(values[i])) *paramSlots[i].ptr = values[i];  // 只在 isfinite 成立时写入，挡住 flash 里的 NaN/Inf 脏值
L4755  }  // 遍历循环体到此结束，继续下一个参数
L4756  }  // 回写函数到此结束，非法值保持原样
L4759  static bool paramWriteAllowed() {  // 定义 paramWriteAllowed：判断此刻是否允许擦写 flash
L4760  return !manualMotionIsActive() && !positionJog.active && imuFailCount == 0;  // 手动运动、位置点动、IMU 失败任一成立就禁写，避免车动时卡拍
L4761  }  // 写盘许可判断到此结束
L4764  void paramStorageBoot() {  // 定义 paramStorageBoot：开机读回参数并预擦备用扇区
L4765  for (uint8_t i = 0; i < ParamStore::PARAM_COUNT; ++i) {  // 先遍历全部 22 个槽位
L4766  paramDefaults[i] = *paramSlots[i].ptr;  // 把当前编译值抄进 paramDefaults，作为 DEF 的出厂值
L4767  }  // 抄写循环体到此结束，默认值全部就位
L4768  float values[ParamStore::PARAM_COUNT];  // 准备 22 项临时数组，用来接收 flash 读回的参数
L4769  paramScan  = ParamStore::Store().scan(paramPort, values);  // 扫描扇区找最新有效记录，同时把记录里的参数读进 values
L4770  paramSaved = paramScan.found;  // paramSaved 记为是否找到有效记录，决定 SAVED 回显
L4771  paramDirty = !paramScan.found;  // 没找到记录就标脏，提示当前值还没有落盘
L4772  if (paramScan.found) {  // 找到有效记录时进入应用分支
L4773  paramApply(values);  // 把 flash 读回的值写回全局参数
L4774  BTSerial.print("PARAMS: loaded from flash seq=");  // 串口回显已从 flash 载入参数，并打印记录序号
L4775  BTSerial.print((unsigned long)paramScan.seq);  // 打印记录序号，便于确认载入的是最新那一份
L4776  BTSerial.print(" sector=");  // 先打印扇区号字段的前缀文字
L4777  BTSerial.println((int)paramScan.sector);  // 打印记录所在的扇区号
L4779  ParamStore::Store().prepare(paramPort, paramScan);  // 当前扇区写满就先擦好另一块，这 1.1s 放在开机执行更安全
L4780  } else {  // 没有有效记录时走 else 分支
L4781  BTSerial.println("PARAMS: compiled defaults");  // 串口提示本次使用编译默认值，flash 里没有历史参数
L4782  }  // else 分支到此结束
L4783  }  // 开机载入函数到此结束
L4786  enum ParamWordCommand { PARAM_WORD_NONE = 0, PARAM_WORD_SAVE, PARAM_WORD_LOAD, PARAM_WORD_DEF };  // 定义整词命令枚举：无命中、SAVE、LOAD、DEF 四种取值
L4788  static ParamWordCommand paramWordCommand(const char* cmd, uint8_t len) {  // 定义 paramWordCommand：把整条命令归一成一个整词再比对
L4789  char word[8];  // 词缓冲 8 字节，够放 SAVE/LOAD/DEF 加结束符
L4790  uint8_t n = 0;  // 有效字符计数初值 0
L4791  bool truncated = false;  // 超长标记，词超过 7 个字符就判为非法命令
L4792  while (len > 0 && (*cmd == ' ' || *cmd == '\t')) { ++cmd; --len; }  // 跳过命令前面的空格和制表符
L4793  while (len > 0 && *cmd != ' ' && *cmd != '\t' && *cmd != '\r' && *cmd != '\n') {  // 连续读取非空白字符，直到遇到空白或换行
L4794  if (n + 1u < sizeof(word)) word[n++] = (char)toupper((unsigned char)*cmd);  // 缓冲未满就存字符并转大写，SAVE 和 save 都认
L4795  else truncated = true;  // 缓冲放不下就置超长标记，稍后返回无命中
L4796  ++cmd; --len;  // 指针前移一位，剩余长度减一
L4797  }  // 取词循环体到此结束，word 里是完整词
L4798  word[n] = '\0';  // 补字符串结束符，word 现在是一个完整词
L4799  while (len > 0 && (*cmd == ' ' || *cmd == '\t' || *cmd == '\r' || *cmd == '\n')) {  // 跳过词后面的所有空白与换行
L4800  ++cmd; --len;  // 继续前移指针并缩短剩余长度
L4801  }  // 尾部空白跳过到此结束
L4802  if (truncated || n == 0 || len != 0) return PARAM_WORD_NONE;  // 超长、空词或尾部还有残留字符都判为无命中
L4803  if (!strcmp(word, "SAVE")) return PARAM_WORD_SAVE;  // 词等于 SAVE 就返回保存命令
L4804  if (!strcmp(word, "LOAD")) return PARAM_WORD_LOAD;  // 词等于 LOAD 就返回载入命令
L4805  if (!strcmp(word, "DEF"))  return PARAM_WORD_DEF;  // 词等于 DEF 就返回恢复出厂命令
L4806  return PARAM_WORD_NONE;  // 都不匹配返回无命中，交给后面的单字母参数解析
L4807  }  // 整词判定函数到此结束
L4809  static char btCmdBuf[48];static uint8_t btCmdLen = 0;  // BT 命令行缓冲 48 字节，同时声明已收字符数 btCmdLen
L4810  static bool btCmdOverflow = false;  // 溢出标志：一行太长时置位，该行稍后整条丢弃
L4812  void handleBtCommand(char* cmd, uint8_t len);  // 前置声明 handleBtCommand，供上面的轮询函数调用
L4822  static bool handleTunableCommand(const char* cmd, uint8_t len, QueuedBtReply& reply)  // 定义 handleTunableCommand：拦截字母加数值的运行时可调命令
L4823  {  // 函数体开始，后面的代码是命令解析主体
L4824  uint8_t lead = 0;  // 前导空格计数初值 0
L4825  while (lead < len && cmd[lead] == ' ') ++lead;  // 跳过命令开头的空格，让 lead 指向第一个字母
L4826  if (len < lead + 2) return false;  // 连两个字母都凑不齐就直接放行给后面的单字母解析
L4827  if (lead + 2 < len && cmd[lead + 2] != ' ' && cmd[lead + 2] != '\t') return false;  // 第 3 个字符必须是空白，否则是 P8 这类命令不能按两字母解析
L4828  const char a = (char)toupper((unsigned char)cmd[lead]);  // 取第一个字母并转成大写
L4829  const char b = (char)toupper((unsigned char)cmd[lead + 1]);  // 取第二个字母并转成大写
L4830  const bool isErrorGain    = (a == 'E' && b == 'G');  // 判断是否为 EG：手动通路的误差增益命令
L4831  const bool isSpeedCmd     = (a == 'S' && b == 'P');  // 判断是否为 SP：目标速度设定命令
L4832  const bool isSpeedKp      = (a == 'S' && b == 'V');  // 判断是否为 SV：速度环 P 增益在线可调（2026-11-19 新增）
L4833  const bool isMaxSpeed     = (a == 'M' && b == 'S');  // 判断是否为 MS：速度范围上限，只改 RAM 不写 flash
L4834  if (!isErrorGain && !isSpeedCmd && !isSpeedKp && !isMaxSpeed) return false;  // 四个两字母命令都不是就返回 false，交回单字母流程
L4836  uint8_t vstart = lead + 2;  // 数值起点定在第二个字母后面一位
L4837  while (vstart < len && (cmd[vstart] == ' ' || cmd[vstart] == '\t')) ++vstart;  // 跳过字母与数值之间的空格和制表符
L4838  if (vstart >= len) { reply.println("ERR:SP/SV/EG/MS need a number"); return true; }  // 数值缺失时回一条错误提示并结束处理
L4839  char vbuf[16];  // 数值文本缓冲 16 字节，足够放一般浮点写法
L4840  uint8_t vlen = 0;  // 数值有效字符计数初值 0
L4841  for (uint8_t i = vstart; i < len && vlen + 1u < sizeof(vbuf); ++i) {  // 逐字符收集数值，末尾留一位给结束符
L4842  const char c = cmd[i];  // 取出当前字符准备判定
L4843  if (c == ' ' || c == '\t' || c == '\r' || c == '\n') continue;  // 空白与换行直接跳过，只保留数字和符号
L4844  vbuf[vlen++] = c;  // 有效字符存入缓冲并递增计数
L4845  }  // 收集循环体到此结束，vbuf 待补结束符
L4846  vbuf[vlen] = '\0';  // 补结束符，vbuf 成为一个可解析的数字串
L4847  char* end = nullptr;  // end 指针用来检查 strtof 是否把整串都解析完
L4848  const float value = strtof(vbuf, &end);  // 把 vbuf 文本转成 float 数值
L4850  if (end == vbuf || *end != '\0' || !(value > -1.0e6f && value < 1.0e6f)) {  // 没解析出字符、有残留字符或数值越界都判为非法输入
L4851  reply.println("ERR:SP/SV/EG/MS need a number");  // 回同一条数值错误提示
L4852  return true;  // 返回 true，表示这条命令已被消费
L4853  }  // 数值校验分支到此结束
L4855  if (isSpeedCmd) {  // 处理 SP 目标速度命令
L4856  speedCmd = ManualTurnControl::clamp(value, -MAX_TARGET_FORWARD_SPEED, MAX_TARGET_FORWARD_SPEED);  // 把数值钳到 ±MS 后写入 speedCmd，这是唯一的速度设定入口
L4857  reply.print("SP ok ");  // 回显前缀 SP ok
L4858  reply.println(speedCmd, 3);  // 回显写入后的速度指令，保留 3 位小数
L4859  return true;  // 返回 true，表示命令已处理
L4860  }  // SP 处理分支到此结束
L4861  if (isMaxSpeed) {  // 处理 MS 速度上限命令
L4863  MAX_TARGET_FORWARD_SPEED = ManualTurnControl::clamp(value, 0.5f, 60.0f);  // 上限钳在 0.5~60 counts/50ms，防止设成停车或远超步进能力
L4865  speedCmd = ManualTurnControl::clamp(speedCmd,  // 上限改小后原 speedCmd 可能越界，立即重新钳位
L4866  -MAX_TARGET_FORWARD_SPEED,  // 钳位下界取负的速度上限
L4867  MAX_TARGET_FORWARD_SPEED);  // 钳位上界取正的速度上限
L4868  reply.print("MS ok ");  // 回显前缀 MS ok
L4869  reply.print(MAX_TARGET_FORWARD_SPEED, 3);  // 回显新的速度上限，保留 3 位小数
L4870  reply.print(" SP=");  // 回显当前速度指令的字段名
L4871  reply.println(speedCmd, 3);  // 回显钳位后的 SP 值
L4872  return true;  // 返回 true，表示命令已处理
L4873  }  // MS 处理分支到此结束
L4874  if (isSpeedKp) {  // 处理 SV 速度环 P 增益命令
L4876  speedKp = ManualTurnControl::clamp(value, -0.5f, 0.5f);  // 增益钳在 ±0.5，极性与大小都允许现场试
L4877  reply.print("SV ok ");  // 回显前缀 SV ok
L4878  reply.println(speedKp, 4);  // 回显 speedKp，保留 4 位小数
L4879  return true;  // 返回 true，表示命令已处理
L4880  }  // SV 处理分支到此结束
L4881  {  // 剩下的只可能是 EG，单独开一个作用域
L4883  motionLean.errorGain = ManualTurnControl::clamp(value, 0.0f, 0.15f);  // 误差增益钳在 0~0.15，开太大一步就会打满角度上限
L4884  }  // EG 处理作用域到此结束
L4885  reply.print(" EG ");  // 回显字段名前缀 EG
L4886  reply.print(motionLean.errorGain, 3);  // 回显新的误差增益，保留 3 位小数
L4887  reply.println();  // 输出换行，结束这条回包
L4888  return true;  // 返回 true，表示命令已处理
L4889  }  // 可调命令处理函数到此结束
L4891  void pollBtCommand()  // 定义 pollBtCommand：每圈从蓝牙串口抽取完整命令行
L4892  {  // 函数体开始，后面是逐字节取数与拼行逻辑
L4893  uint8_t readBudget = 32;  // 每圈最多读 32 字节，防止 BLE 流量把 400Hz 主拍拖慢
L4894  while (readBudget-- && BTSerial.available() > 0) {  // 预算没用完且串口有数据就继续读
L4895  char c = (char)BTSerial.read();  // 从蓝牙串口取一个字节字符
L4896  btRxBytes++;  // 累计接收字节数，用于链路流量统计
L4897  if (c == '\n' || c == '\r') {  // 遇到回车或换行表示一行命令结束
L4898  if (!btCmdOverflow) handleBtCommand(btCmdBuf, btCmdLen);  // 没溢出就把这一行交给 handleBtCommand 解析
L4899  btCmdOverflow = false;  // 无论是否溢出都清掉溢出标志
L4900  btCmdLen = 0;  // 行缓冲长度清零，准备接收下一行
L4901  } else if (btCmdLen < sizeof(btCmdBuf) - 1) {  // 未满一行时继续收字符，末尾留一位给结束符
L4902  btCmdBuf[btCmdLen++] = c;  // 字符入缓冲并递增行长度
L4903  } else {  // 缓冲已满，走溢出标记分支
L4904  btCmdOverflow = true;  // 置溢出标志，这一行稍后整条丢弃
L4905  }  // 溢出分支到此结束，继续读下一个字节
L4906  }  // 读取循环体到此结束，本圈轮询完成
L4907  }  // 轮询函数到此结束，等下一圈再读
L4909  void handleBtCommand(char* cmd, uint8_t len)  // 定义 handleBtCommand：解析一整行蓝牙命令
L4910  {  // 函数体开始，后面是各类命令的解析主体
L4911  QueuedBtReply reply;  // 建回包队列，本次解析的回复都排进它稍后统一发出
L4913  if (handleTunableCommand(cmd, len, reply)) return;  // 先拦 SP/SV/EG/MS 两字母可调命令，命中就直接返回
L4916  switch (paramWordCommand(cmd, len)) {  // 判断是否为 SAVE/LOAD/DEF 这类整词命令
L4917  case PARAM_WORD_SAVE: {  // SAVE 分支：把当前参数写进 flash
L4918  if (!paramWriteAllowed()) { reply.println("ERR:SAVE needs idle vehicle"); return; }  // 车在动或 IMU 异常时拒绝保存，避免擦写打断控制
L4919  float values[ParamStore::PARAM_COUNT];  // 准备 22 项数组，装当前参数值
L4920  paramCollect(values);  // 把全局参数收集进数组
L4921  bool erasedSector = false;  // 擦除标记初值 false，由 save 回写是否真的擦过扇区
L4922  bool ok = ParamStore::Store().save(paramPort, values, paramScan, erasedSector);  // 调用 ParamStore::save 写盘，一次写入 22 项参数
L4923  paramSaved = ok;  // 写成功才认为 flash 里有有效记录
L4924  paramDirty = !ok;  // 写失败则保持脏标志，提示参数仍未落盘
L4925  if (!ok) { reply.println("ERR:save failed"); return; }  // 写失败回错误并结束，不再回成功提示
L4926  reply.print("SAVED OK seq=");  // 回显 SAVED OK 与记录序号前缀
L4927  reply.print((unsigned long)paramScan.seq);  // 打印本次写入的记录序号
L4928  if (erasedSector) reply.print(" sectorErased=1");  // 本次若擦过扇区就额外标注 sectorErased=1
L4929  reply.println();  // 输出换行，结束这条回包
L4930  return;  // SAVE 分支返回上层
L4931  }  // SAVE 分支到此结束
L4932  case PARAM_WORD_LOAD: {  // LOAD 分支：从 flash 重新读回参数
L4933  if (!paramWriteAllowed()) { reply.println("ERR:LOAD needs idle vehicle"); return; }  // 同样要求车静止、IMU 正常才允许读回
L4934  float values[ParamStore::PARAM_COUNT];  // 准备数组接收读回的参数
L4935  ParamStore::Store::Scan sc = ParamStore::Store().scan(paramPort, values);  // 重新扫描 flash，取最新有效记录并读出参数
L4936  paramScan = sc;  // 把本次扫描结果记到 paramScan，供后续 SAVE 续写
L4937  if (!sc.found) { reply.println("ERR:no saved params"); return; }  // 没有有效记录就回一条错误
L4938  paramApply(values);  // 把读回的值写回全局参数
L4939  paramSaved = true;  // 标记 flash 里有有效记录可再次 LOAD
L4940  paramDirty = false;  // 值已与 flash 一致，清掉脏标志
L4941  reply.print("LOAD OK seq=");  // 回显 LOAD OK 与序列号前缀
L4942  reply.println((unsigned long)sc.seq);  // 打印载入记录的序号并换行
L4943  return;  // LOAD 分支返回上层
L4944  }  // LOAD 分支到此结束
L4945  case PARAM_WORD_DEF: {  // DEF 分支：恢复编译默认值，只改 RAM
L4948  if (!paramWriteAllowed()) { reply.println("ERR:DEF needs idle vehicle"); return; }  // 车未静止时拒绝执行，与 SAVE/LOAD 同一门槛
L4949  paramApply(paramDefaults);  // 把开机抄下的默认值写回全局参数
L4950  paramDirty = true;  // 默认值没有落盘，标脏提示后续要 SAVE
L4951  reply.println("DEF OK ram-only (SAVE to persist) SAVED=0");  // 回显 DEF 成功且仅作用于 RAM，并报告 SAVED=0
L4952  return;  // DEF 分支返回上层
L4953  }  // DEF 分支到此结束
L4954  default: break;  // 其他整词不命中，落到下面的单字母参数解析
L4955  }  // 整词命令分支到此结束
L4957  while (len > 0 && (*cmd == ' ' || *cmd == '\t')) { cmd++; len--; }  // 跳过命令前面的空格与制表符
L4958  if (len == 0) return;  // 整行只有空白就直接丢弃
L4959  char key = *cmd; cmd++; len--;  // 取第一个字符当参数键，指针后移一位
L4960  while (len > 0 && (*cmd == ' ' || *cmd == '\t')) { cmd++; len--; }  // 跳过键与数值之间的空格与制表符
L4961  uint8_t vlen = len;  // vlen 初值取剩余长度，后面再去掉尾部空白
L4962  while (vlen > 0 && (cmd[vlen-1] == ' ' || cmd[vlen-1] == '\r' || cmd[vlen-1] == '\n')) vlen--;  // 扣掉尾部空格与回车换行等空白字符
L4963  if (key == '?') {  // 键是 ? 时回显当前全部参数
L4964  prepareCommandResponse();  // 准备回包缓冲区，避免一行太长被截断
L4965  reply.print("PAR: TURN=2 MOVE=5 SMOOTH=1 MOTION=2 FW=20261112F TILTMAX=");  // 回显固件标识、运动模式编号与 TILTMAX 字段名
L4966  reply.print(ManualPitchControl::maxAngle, 2);  // 回显手动倾角最大值 ManualPitchControl::maxAngle
L4967  reply.print(" ");  // 参数之间补一个空格做分隔
L4968  reply.print("P="); reply.print(balanceKp, 2);  // 回显直立环 P（balanceKp），保留 2 位小数
L4969  reply.print(" D="); reply.print(balanceKd, 2);  // 回显直立环 D（balanceKd）
L4970  reply.print(" O="); reply.print(balanceAngleOffset, 2);  // 回显平衡角 O（balanceAngleOffset）
L4971  reply.print(" Z="); reply.print(BALANCE_ANGLE_DEADBAND_DEG, 2);  // 回显平衡角死区 Z（度）
L4972  reply.print(" L="); reply.print(BALANCE_OUTPUT_LIMIT_HZ, 1);  // 回显输出限幅 L（Hz），1 位小数
L4973  reply.print(" S="); reply.print(BALANCE_OUTPUT_SLEW_HZ, 1);  // 回显每周期最大变化 S（Hz）
L4974  reply.print(" R="); reply.print(BALANCE_ACCEL_SCALE, 4);  // 回显加速度缩放 R，保留 4 位小数
L4975  reply.print(" F="); reply.print(OUTER_FULL_ANGLE_DEG, 2);  // 回显外环全权角 F（度）
L4976  reply.print(" G="); reply.print(OUTER_DISABLE_ANGLE_DEG, 2);  // 回显外环失效角 G（度）
L4977  reply.print(" T="); reply.print(turnKp, 2);  // 回显转向环 P（turnKp）
L4978  reply.print(" U="); reply.print(turnKd, 2);  // 回显转向环 D（turnKd）
L4979  reply.print(" K="); reply.print(speedKp, 2);  // 回显速度环 P（K），与 SV 写的是同一个变量
L4980  reply.print(" I="); reply.print(speedKi, 2);  // 回显速度环 I（speedKi）
L4981  reply.print(" N="); reply.print(MAX_SPEED_ANGLE, 2);  // 回显速度修正角上限 N（度）
L4982  reply.print(" W="); reply.print(SPEED_INTEGRAL_LIMIT, 1);  // 回显速度积分限幅 W
L4983  reply.print(" J="); reply.print(SPEED_ERROR_DEADBAND, 2);  // 回显速度误差死区 J
L4984  reply.print(" E="); reply.print(speedKd, 3);  // 回显速度环 D（speedKd），保留 3 位小数
L4985  reply.print(" Q="); reply.print(speedDirectKp, 3);  // 回显速度直通增益 Q（speedDirectKp）
L4986  reply.print(" A="); reply.print(speedPredictHorizon, 3);  // 回显速度预测时域 A（speedPredictHorizon）
L4987  reply.print(" B="); reply.print(HIGH_SPEED_ACCEL_START_HZ, 1);  // 回显高速加速度补偿起点 B（Hz）
L4988  reply.print(" H="); reply.print(HIGH_SPEED_ACCEL_LIMIT_HZ, 1);  // 回显高速加速度补偿上限 H（Hz）
L4989  reply.print(" X="); reply.print(positionKp, 4);  // 回显位置环 P（X），保留 4 位小数
L4990  reply.print(" Y="); reply.print(MAX_POSITION_RETURN_SPEED, 2);  // 回显位置回位速度上限 Y
L4991  reply.print(" M="); reply.print(POSITION_DEADBAND_COUNT, 2);  // 回显位置死区 M（count）
L4993  reply.print(" EG="); reply.print(motionLean.errorGain, 3);  // 回显手动通路误差增益 EG
L4995  reply.print(" SAVED="); reply.print(paramDirty ? 0 : (paramSaved ? 1 : 0));  // 回显 SAVED：1 表示与 flash 一致，0 表示改过未存
L4997  reply.print(" SP="); reply.print(speedCmd, 3);  // PAR 行末尾追加当前速度指令 SP（counts/50ms）
L4999  reply.print(" SV="); reply.print(speedKp, 4);  // 回显速度环 P 的 SV 别名，与 K 同值
L5000  reply.print(" MS="); reply.print(MAX_TARGET_FORWARD_SPEED, 3);  // 回显速度上限 MS（counts/50ms）
L5001  reply.println();  // 上一条应答结束后补一个空行，方便手机端分行阅读
L5002  reply.print("JOGCFG ");reply.print(positionLean.kp,4);reply.print(' ');reply.println(positionLean.limit,3);  // 回读点动倾斜环增益与限幅，按4位和3位小数打印便于核对
L5003  return;  // 该分支处理完毕直接返回，本次命令结束
L5004  }  // 结束这段命令处理分支的复合语句块
L5006  if (key == '!' && vlen == 0) {  // 单字母感叹号且没有参数时，进入诊断查询分支
L5007  prepareCommandResponse();  // 准备应答缓冲，随后所有打印都写进这块缓冲
L5008  #if TMC_STANDALONE_STEP_DIR  // 编译期判据：驱动器工作在硬件STEP/DIR独立模式
L5009  reply.println("TMC STEP/DIR standalone: hardware-selected mode");  // 先说明驱动模式，硬件选档不需要软件配置
L5010  reply.print("STEP edges="); reply.print(step1.risingEdges);  // 打印电机1累计上升沿数，反映STEP脉冲总量
L5011  reply.print('/'); reply.print(step2.risingEdges);  // 用斜杠分隔再打印电机2脉冲数，便于对比两轮
L5012  reply.print(" periodUs="); reply.print(step1.periodTicks);  // 打印电机1当前脉冲周期计数值，看转速是否异常
L5013  reply.print('/'); reply.print(step2.periodTicks);  // 分隔后打印电机2周期，比较两侧脉冲间隔
L5014  reply.print(" overrun="); reply.print(controlOverrunCount);  // 打印主拍掉拍次数，判断400Hz时序是否被拖垮
L5015  reply.print(" maxGapUs="); reply.print(controlMaxGapUs);  // 打印最大拍间隔，暴露最长的一次调度延迟
L5016  reply.print(" maxWorkUs="); reply.println(controlMaxWorkUs);  // 打印单拍最大耗时，评估控制计算的时序裕量
L5017  reply.print("encoderMode="); reply.print(rawSpeedEncoderMode);  // 打印编码器测速模式，确认当前用哪套计数来源
L5018  reply.print(" softTicks="); reply.println(reversalSoftTicksRemaining);  // 打印换向软计数剩余拍数，看是否还在保护窗口内
L5019  #elif TMC_GPIO_WRITE_ONLY  // 编译期切换：驱动器为只写GPIO模式，不能回读
L5020  reply.print("TMC WRITE-ONLY SENT rearBytes="); reply.print(rearTmcUart.txCount);  // 打印后驱动器已发字节数，确认配置确实写过
L5021  reply.print(" frontBytes="); reply.print(frontTmcUart.txCount);  // 打印前驱动器发送字节数，对比两路都下发过
L5022  reply.println(" spreadCycle=REQUESTED microsteps=16 UNVERIFIED");  // 声明请求spreadCycle与16细分，但无法回读故未验证
L5023  #else  // 编译期兜底分支：驱动芯片支持寄存器回读
L5024  reply.println("TMC READBACK MODE");  // 提示当前处于可回读校验的工作模式
L5025  #endif  // 驱动模式的条件编译到此结束
L5026  return;  // 诊断信息输出完毕，返回不再走参数解析
L5027  }  // 结束感叹号诊断查询分支
L5029  if (vlen == 0) return;  // 没有参数值就直接返回，单字母查询命令不赋参
L5030  char vbuf[32];  // 在栈上准备32字节缓冲，用来存放参数副本
L5031  if (vlen >= sizeof(vbuf)) { reply.println("ERR:value too long"); return; }  // 参数超长就报错返回，防止缓冲区被写越界
L5032  for (uint8_t i = 0; i < vlen; i++) vbuf[i] = cmd[i];  // 逐字节把命令参数拷进本地缓冲区
L5033  vbuf[vlen] = '\0';  // 补上字符串结束符，便于后续解析函数使用
L5035  if(key=='!'){  // 带参数的感叹号命令：写入点动倾斜环参数
L5036  float kp=0,limit=0;  // 先把增益与限幅赋零，等待解析结果覆盖
L5037  if(!parseJogConfig(vbuf,kp,limit)){reply.println("ERR:invalid jog config");return;}  // 解析两个浮点参数，格式不对就报错返回
L5038  if(positionJog.active || positionLean.busy()){reply.println("ERR:stop jog before config");return;}  // 点动进行中不许改参数，避免中途变增益失控
L5039  positionLean.kp=kp;positionLean.limit=limit;  // 把解析结果写入点动倾斜环控制器
L5040  reply.print("JOGCFG ");reply.print(kp,4);reply.print(' ');reply.println(limit,3);  // 回显设置后的增益与限幅值，供上位机核对
L5041  return;  // 参数写入完成，结束本次命令处理
L5042  }  // 结束感叹号参数写入分支的处理
L5044  if(key=='~' || key=='&'){  // 波浪号发起点动，与号维持点动心跳，共用分支
L5045  uint32_t id=0;float delta=0;const char* rest=nullptr;  // 声明序号、位移和剩余串指针三个解析输出变量
L5046  if(key=='&'){  // 心跳命令只续期已有会话，不新建动作
L5047  if(parseJogId(vbuf,id,rest) && *rest=='\0')positionJog.heartbeat(id,millis());  // 解析出序号且无多余字符时，刷新该会话心跳时间
L5048  return;  // 心跳处理完即返回，不做任何运动控制
L5049  }  // 结束点动心跳命令的独立分支
L5050  if(!parseJogFrame(vbuf,id,delta)){reply.println("ERR:invalid jog");return;}  // 解析点动帧，格式错误就报错返回
L5051  if(delta!=0){reply.println("ERR:use fixed-angle hold mode");return;}  // 现在只允许零位移，位置变化走固定角保持通路
L5052  if(positionJog.newer(id)){  // 只有序号比上次新才接受，用来丢弃重复帧
L5053  if(delta!=0 && (manualMotionIsActive() || imuFailCount!=0 ||  // 需要位移时的安全检查第一段：手动通路忙或IMU异常
L5054  fabsf(pitch-balanceAngleOffset)>5 || fabsf(speedKp)<.000001f || positionLean.kp<=0 || !(encoder1Trusted || encoder2Trusted))){  // 倾角超5度、速度环增益为零或编码器不可信也拒绝
L5055  reply.println("ERR:jog needs upright idle vehicle");return;  // 车辆未扶正或未就绪时拒绝点动请求
L5056  }  // 结束点动位移前的安全检查块
L5057  if(delta==0){positionJog.start(id,0,vehiclePosition,rawSpeedEncoderMode,millis());captureCurrentPositionTarget();}  // 零位移点动：记录起始位置并锁定当前目标
L5058  else{  // 否则是带位移的点动帧，走另一条设置路径
L5059  updatePositionHold();  // 先按当前状态更新位置环输出，保证基线一致
L5060  positionJog.start(id,delta,vehiclePosition,rawSpeedEncoderMode,millis());  // 以当前车辆位置为基准启动一次位移点动
L5061  targetVehiclePosition=positionJog.reference;  // 把点动给出的设定点作为位置环目标
L5062  positionHoldWasEnabled=positionKp>0.000001f;  // 记录点动之前位置环是否开启，便于结束后恢复
L5063  }  // 结束带位移点动的设置分支
L5064  }  // 结束新序号才处理的判断块
L5065  if(id==positionJog.sequence)positionJog.heartbeat(id,millis());  // 序号匹配当前会话时补一次心跳，避免超时退出
L5066  reply.print("JOG ");reply.print((unsigned long)positionJog.sequence);reply.print(' ');  // 应答点动会话序号，供上位机对账
L5067  reply.print((int)positionJog.active);reply.print(' ');reply.println(positionJog.reason);  // 追加活动标志与原因码，说明接受还是拒绝
L5068  return;  // 点动应答写完，返回主循环
L5069  }  // 结束点动与心跳命令分支
L5071  if (key == '^' || key == '@') {  // 尖号是固定角手动命令，与号是其兼容写法
L5072  float turn, forward;  // 声明转向量与前进档位两个解析结果
L5073  if (!parseManualMotion(vbuf, turn, forward)) {  // 解析手动运动参数，失败进入错误分支
L5074  reply.println("ERR:invalid motion");  // 参数格式不对，回告无效运动命令
L5075  return;  // 放弃本次手动运动命令
L5076  }  // 结束解析失败的错误分支
L5077  if(key=='@' && (turn!=0 || forward!=0)){reply.println("ERR:use fixed-angle ^ command");return;}  // 与号只接受零值，非零运动必须用尖号命令
L5079  if(forward!=0 && fabsf(forward)>ManualPitchControl::maxAngle){reply.println("ERR:forward gear out of range");return;}  // 前进档位超上限即拒绝，防止倾角目标过大
L5080  if(turn!=0 && positionJog.active)stopPositionJog(PositionJog::STOP);  // 要转向时先停掉位置点动，避免两套目标打架
L5081  if(forward!=0 && !manualForward.busy(millis())){  // 仅在新一次前进指令且通路空闲时做起点初始化
L5082  float currentOffset=targetBalanceAngle-balanceAngleOffset;  // 取当前平衡目标相对平衡角O的偏角，作手动起点
L5083  if(manualForward.needsZero || imuFailCount!=0 || fabsf(pitch-balanceAngleOffset)>5 ||  // 安全检查第一段：需归零、IMU异常或车身未扶正
L5084  !isfinite(currentOffset) || fabsf(currentOffset)>ManualPitchControl::maxAngle){  // 起点非有限或超出上限也拒绝，避免切入时跳变
L5085  reply.println("ERR:angle hold needs upright ready vehicle");return;  // 通知握角命令需要车辆已扶正就绪才能执行
L5086  }  // 结束手动前进的安全检查块
L5087  manualForward.seed(currentOffset);  // 把当前目标作为起点，切入时不产生角度跳变
L5088  }  // 结束前进起点初始化分支
L5089  uint32_t now = millis();  // 取一次毫秒时基，供两个手动通路共用
L5090  manualTurn.command(turn, now);  // 下发转向指令并记录时间戳，内部按斜坡限速
L5091  manualForward.command(forward, now);  // 下发前进档位指令并记录时间戳
L5092  reply.println("MOTION OK");  // 回告手动运动命令已受理
L5093  return;  // 手动分支处理完毕，返回主循环
L5094  }  // 结束手动运动命令分支
L5097  {  // 开一个局部作用域，只为校验参数首字符
L5098  char c0 = vbuf[0];  // 取出参数首字符，判断是否可能是数字
L5099  if (!((c0>='0'&&c0<='9')||c0=='-'||c0=='.')) return;  // 首字符非数字或符号就静默忽略，防误设参数
L5100  }  // 结束首字符校验的局部作用域
L5102  char* valueEnd = nullptr;  // 准备接收strtof解析停止的位置指针
L5103  float val = strtof(vbuf, &valueEnd);  // 把参数字符串转换成浮点数值
L5104  if (valueEnd == vbuf || *valueEnd != '\0' || !isfinite(val)) {  // 一个字符没解析、有残留或非有限值都算非法
L5105  reply.println("ERR:invalid value");  // 回告数值非法，参数保持原值不变
L5106  return;  // 放弃本次参数设置并返回
L5107  }  // 结束数值合法性判断分支
L5108  switch (key) {  // 按单字母命令字分发到各个可调参数
L5109  case 'P': case 'p': balanceKp = val; break;  // 字母P设置直立环比例增益
L5110  case 'D': case 'd': balanceKd = val; break;  // 字母D设置直立环微分增益
L5111  case 'O': case 'o': balanceAngleOffset = val; break;  // 字母O设置机械平衡角，单位度
L5112  case 'Z': case 'z': BALANCE_ANGLE_DEADBAND_DEG = val; break;  // 字母Z设置直立环角度死区，抑制站立抖动
L5113  case 'L': case 'l': BALANCE_OUTPUT_LIMIT_HZ = fabsf(val); break;  // 字母L设置输出限幅Hz，取绝对值防负限幅
L5114  case 'S': case 's': BALANCE_OUTPUT_SLEW_HZ = fabsf(val); break;  // 字母S设置每100Hz端点频率增量上限
L5115  case 'R': case 'r': BALANCE_ACCEL_SCALE = fabsf(val); break;  // 字母R设置加速度前馈的缩放系数
L5116  case 'F': case 'f': OUTER_FULL_ANGLE_DEG = val; break;  // 字母F设置外环全权限角度，小于此角给全权
L5117  case 'G': case 'g': OUTER_DISABLE_ANGLE_DEG = val; break;  // 字母G设置外环完全退出的角度门限
L5118  case 'V': case 'v':  // 字母V设置转向量，走手动转向通路
L5119  if(val!=0 && positionJog.active)stopPositionJog(PositionJog::STOP);  // 有转向输出时先终止位置点动
L5120  manualTurn.command(val, millis());  // 下发转向指令，内部按斜坡限制变化速度
L5121  val = manualTurn.requested;  // 用实际受理值回显，让上位机知道是否被限幅
L5122  break;  // 结束转向参数分支，转到统一回显流程
L5123  case 'T': case 't': turnKp = val; break;  // 字母T设置转向环比例增益
L5124  case 'U': case 'u': turnKd = val; break;  // 字母U设置转向环微分增益
L5125  case 'K': case 'k': speedKp = val; break;  // 字母K设置速度环比例增益
L5126  case 'I': case 'i': speedKi = val; break;  // 字母I设置速度环积分增益
L5127  case 'N': case 'n': MAX_SPEED_ANGLE = val; break;  // 字母N设置速度修正角上限，单位度
L5128  case 'W': case 'w': SPEED_INTEGRAL_LIMIT = val; break;  // 字母W设置速度积分限幅，防止积分饱和
L5129  case 'J': case 'j': SPEED_ERROR_DEADBAND = val; break;  // 字母J设置速度误差死区，减少静态抖动
L5130  case 'E': case 'e': speedKd = val; break;  // 字母E设置速度环微分增益
L5131  case 'Q': case 'q': speedDirectKp = fabsf(val); break;  // 字母Q设置速度直通修正增益，取绝对值
L5132  case 'H': case 'h': HIGH_SPEED_ACCEL_LIMIT_HZ = fabsf(val); break;  // 字母H设置高速段的加速度限幅
L5133  case 'A': case 'a': speedPredictHorizon = fabsf(val); break;  // 字母A设置速度预测时域，影响超前量
L5134  case 'B': case 'b': HIGH_SPEED_ACCEL_START_HZ = fabsf(val); break;  // 字母B设置高速加速度限制的起始频率
L5135  case 'X': case 'x': {  // 字母X开关位置环：增益大于零即视为启用
L5136  bool wasEnabled = positionKp > 0.000001f;  // 先记住修改之前位置环是否处于开启状态
L5137  positionKp = fabsf(val);  // 写入新的位置环增益，负值按零处理后等于关闭
L5138  bool nowEnabled = positionKp > 0.000001f;  // 判断修改之后位置环是否处于启用状态
L5139  if (!wasEnabled && nowEnabled && !positionHoldSuspended) captureCurrentPositionTarget();  // 由关到开时把当前位置锁为目标，避免猛回零
L5140  if (!nowEnabled) {  // 关闭位置环时需要清掉残留的目标值
L5141  positionSpeedTarget = 0.0f;  // 清掉位置环给出的速度指令
L5142  targetForwardSpeed = 0.0f;  // 清目标前进速度，防止继续按旧值行驶
L5143  }  // 结束关闭位置环时的目标清理
L5144  positionHoldWasEnabled = nowEnabled;  // 记录最新的使能状态，供其他模块判断
L5145  break;  // 结束位置环开关分支，转到统一回显
L5146  }  // 结束位置环开关分支的作用域
L5147  case 'Y': case 'y': MAX_POSITION_RETURN_SPEED = fabsf(val); break;  // 字母Y设置位置环的回位速度上限
L5148  case 'M': case 'm': POSITION_DEADBAND_COUNT = fabsf(val); break;  // 字母M设置位置死区，单位是编码器计数
L5149  case 'C': case 'c': captureCurrentPositionTarget(); break;  // 字母C把当前位置重新锁定为位置环目标
L5150  default:  // 其余字母一律视为未知命令
L5151  prepareCommandResponse();  // 准备应答缓冲，用于输出错误提示
L5152  reply.println("ERR:unknown cmd");  // 回告未知命令，不修改任何参数
L5153  return;  // 直接返回，跳过成功回显流程
L5154  }  // 结束单字母参数写入的分发开关
L5155  if (paramSlotIndexOf(key) >= 0) paramDirty = true;  // 改的是可存参数就置脏位，提示尚未保存
L5156  prepareCommandResponse();  // 开始组织成功应答的内容
L5157  reply.print(key);  // 回显命令字母，让上位机确认改的是哪项
L5158  reply.print("=");  // 输出等号作为键值之间的分隔符
L5159  reply.print(val, 3);  // 回显实际生效值，保留三位小数
L5160  reply.println(" OK");  // 追加OK并换行，完成整条应答
L5161  }  // 结束蓝牙命令处理函数
L5163  void loop()  // Arduino主循环入口，400Hz控制流水都在这里
L5165  {  // loop函数体开始，下面按固定节拍执行
L5167  uint32_t nowUs =  // 声明微秒时间戳变量，先赋一次初值
L5169  micros();  // 读微秒计数器，作为本拍的起始时间
L5175  pollBtCommand();  // 每圈都轮询蓝牙，命令处理不受2.5ms节拍限制
L5176  flushTelemetryTx();  // 顺手把待发遥测推出去，避免发送缓冲积压
L5177  nowUs = micros();  // 通信处理完重新取时，避免把耗时算进控制节拍
L5190  if (  // 开始2.5ms节拍闸门判断
L5192  (uint32_t)(  // 转成无符号整型，微秒计数器回绕时差值仍正确
L5194  nowUs  // 当前时刻的微秒计数，作为被减数
L5196  -  // 减去上次控制时刻，得到本拍间隔
L5198  lastControlUs  // 上一次控制拍的基准时刻
L5200  )  // 结束差值表达式的括号
L5202  <  // 与主拍周期比较，小于说明还没到点
L5204  CONTROL_PERIOD_US  // 主拍周期2500微秒，对应400Hz
L5206  )  // 结束整个节拍判断条件
L5208  {  // 未到点就提前返回的分支开始
L5210  return;  // 提前返回，等下一圈loop再来
L5212  }  // 结束节拍闸门，未到点的路径到此为止
L5225  uint32_t controlGapUs = nowUs - lastControlUs;  // 算实际拍间隔，用于诊断与掉拍判断
L5226  if (controlGapUs > controlMaxGapUs) controlMaxGapUs = controlGapUs;  // 记录最大拍间隔，暴露调度延迟峰值
L5227  if (controlGapUs >= CONTROL_PERIOD_US * 2UL) {  // 间隔超过两拍即认定掉拍，需重新对齐时基
L5228  controlOverrunCount++;  // 掉拍计数加一，供诊断查询使用
L5229  lastControlUs = nowUs;  // 把基准直接拉到现在，不做补拍
L5230  stopBothMotors();  // 掉拍时立即停机，防止用过期状态驱动电机
L5231  clearControlOutputDiagnostics();  // 清掉本拍的输出诊断量
L5232  outerLoopDivider = 0;  // 重置外环分频，让100Hz相位重新对齐
L5233  } else {  // 否则是正常未掉拍的分支
L5234  lastControlUs += CONTROL_PERIOD_US;  // 基准加一个周期，保持长期频率精度不累积漂移
L5235  }  // 结束掉拍与正常对齐的分支
L5236  uint32_t controlWorkStartUs = micros();  // 记下控制计算起点，用于统计单拍耗时
L5249  if (  // 开始判断IMU读取是否成功
L5251  !readBMI270()  // 读BMI270失败时该表达式为真，进入故障处理
L5253  )  // 结束IMU读取结果的判据
L5255  {  // IMU读取失败处理块开始
L5257  imuFailCount++;  // 连续读失败计数加一，用于确认停机
L5261  if (  // 判断连续失败是否还没达到确认次数
L5263  imuFailCount  // 当前连续读失败的次数
L5265  <  // 小于确认阈值就先不动作，继续观察
L5267  IMU_FAIL_CONFIRM_COUNT  // 确认阈值，避免单次读失败就停机
L5269  )  // 结束失败次数的阈值比较
L5271  {  // 未达阈值就继续沿用上一拍输出
L5275  return;  // 保持上一拍电机输出继续跑，等下次读成功
L5277  }  // 结束未达确认次数的提前返回分支
L5281  stopBothMotors();  // 连续失败确认后立即停两路电机
L5282  clearControlOutputDiagnostics();  // 清掉输出诊断量，避免带故障数据上报
L5286  balanceOutput =  // 开始把直立环输出清零
L5288  0.0f;  // 给上面的直立环输出赋零值
L5292  finalBalanceOutput =  // 开始把最终平衡输出清零
L5294  0.0f;  // 给最终平衡输出变量赋零值
L5298  speedDirectOutput =  // 开始把速度直通修正量清零
L5300  0.0f;  // 给速度直通修正量赋零值
L5304  speedDirectAuthority =  // 开始把速度直通权限清零
L5306  0.0f;  // 给速度直通权限赋零值，彻底退出修正
L5308  speedIntegral = 0.0f;  // 清速度积分，避免故障恢复时带着旧积分
L5309  speedAngleCorrection = 0.0f;  // 清速度修正角，防止继续给出倾角补偿
L5310  speedErrorPrev = 0.0f;  // 清上一拍速度误差，防止微分项产生冲击
L5311  speedErrorDotFilt = 0.0f;  // 清滤波后的速度误差变化率
L5312  positionSpeedTarget = 0.0f;  // 清位置环给出的速度指令
L5313  targetForwardSpeed = 0.0f;  // 清目标前进速度，避免故障后继续行驶
L5317  return;  // IMU故障处理后返回，本拍不再做控制
L5319  }  // 结束IMU读取失败处理块
L5323  imuFailCount =  // 读取成功后开始把连续失败计数复位
L5325  0;  // 把连续失败计数清零，恢复正常判定
L5338  updateAttitude();  // 用陀螺与加速度计更新卡尔曼姿态角
L5357  predictVehicleSpeedKalman();  // 400Hz用IMU预测车速，编码器稍后做绝对修正
L5374  telemetryDivider++;  // 常规遥测分频计数值加一
L5378  if (  // 开始常规遥测的分频判断
L5380  telemetryDivider  // 常规遥测的分频计数值
L5382  >=  // 达到分频数就发送一帧遥测
L5384  TELEMETRY_DIVIDER  // 分频系数，决定D帧1Hz与G帧4Hz
L5386  )  // 结束遥测分频判断条件
L5388  {  // 常规遥测到点发送分支开始
L5390  telemetryDivider =  // 常规遥测分频计数准备复位
L5392  0;  // 把常规遥测分频计数清零，重新计数
L5396  sendTelemetry();  // 发送低速遥测帧，例如1Hz的D帧
L5397  }  // 结束常规遥测发送分支
L5399  fastTelemetryDivider++;  // 快速遥测分频计数加一
L5400  if (fastTelemetryDivider >= FAST_TELEMETRY_DIVIDER)  // 快速遥测也按分频判断，约4Hz发一帧
L5401  {  // 快速遥测到点发送分支开始
L5402  fastTelemetryDivider = 0;  // 快速遥测分频计数复位
L5403  sendFastTelemetry();  // 发送G帧等高速遥测数据
L5404  }  // 结束快速遥测发送分支
L5417  float fallAngleError =  // 开始算车身偏离平衡角的角度误差
L5419  fabsf(  // 取绝对值，左右两侧倒都算
L5421  pitch  // 当前俯仰角，单位度，来自姿态解算
L5423  -  // 减去平衡角O得到倾角偏差
L5425  balanceAngleOffset  // 机械平衡角O，可在线标定
L5427  );  // 结束绝对值函数的调用
L5431  if (  // 开始对比倾角偏差与倒地角度门限
L5433  fallAngleError  // 当前倾角相对平衡角的偏差
L5435  >=  // 达到或超过倒地角度门限
L5437  FALL_ANGLE_DEG  // 倒地角度阈值，单位度
L5439  )  // 结束倒地判据的条件表达式
L5441  {  // 倾角超门限的连续计数分支
L5443  fallConfirmCount++;  // 连续超限计数加一，等待确认倒地
L5445  }  // 结束连续超限计数递增分支
L5447  else  // 回到门限以内则走清零分支
L5449  {  // 回到门限内的清零分支开始
L5451  fallConfirmCount =  // 回到门限内，开始把连续计数清零
L5453  0;  // 赋零，连续超限计数归零
L5455  }  // 结束连续计数清零分支
L5459  bool confirmedFall =  // 判断是否已连续确认倒地
L5461  fallConfirmCount  // 连续超限的拍数计数值
L5463  >=  // 达到确认拍数即认定倒地
L5465  FALL_CONFIRM_COUNT;  // 确认拍数阈值，约对应20ms
L5469  bool emergencyFall =  // 另设一条更急的单拍倒地判据
L5471  fallAngleError  // 当前倾角偏差，用于紧急判据
L5473  >=  // 达到紧急倒地角度门限
L5475  EMERGENCY_FALL_ANGLE_DEG;  // 紧急倒地角度，单拍即可触发停机
L5479  if (  // 开始倒地保护的触发判断
L5481  confirmedFall  // 连续确认后的倒地标志
L5483  ||  // 或者满足紧急倒地条件之一
L5485  emergencyFall  // 单拍超限的紧急倒地标志
L5487  )  // 结束倒地保护条件表达式
L5489  {  // 倒地保护动作执行块开始
L5491  stopBothMotors();  // 立即停下两路电机，切断动力输出
L5492  clearControlOutputDiagnostics();  // 清掉输出诊断量，避免故障数据上报
L5493  fallStopCount++;  // 倒地停机计数加一，弥补遥测窗看不清短触发
L5497  balanceOutput =  // 开始把直立环输出清零
L5499  0.0f;  // 把直立环输出变量赋成零值
L5503  turnOutput =  // 开始把转向环输出清零
L5505  0.0f;  // 把转向环输出变量赋成零值
L5509  speedError =  // 开始把速度误差变量清零
L5511  0.0f;  // 把速度误差变量赋成零值
L5517  speedDirectOutput =  // 开始把速度直通输出清零
L5519  0.0f;  // 把速度直通输出变量赋成零值
L5523  speedDirectAuthority =  // 开始把速度直通权限清零
L5525  0.0f;  // 把速度直通权限变量赋成零值
L5527  speedIntegral = 0.0f;  // 清速度积分，倒地后不再累积
L5528  speedAngleCorrection = 0.0f;  // 清速度修正角，倒地后不给倾角补偿
L5529  speedErrorPrev = 0.0f;  // 清上一拍速度误差，防止微分冲击
L5530  speedErrorDotFilt = 0.0f;  // 清滤波后的速度误差变化率
L5534  finalBalanceOutput =  // 开始把最终平衡输出清零
L5536  0.0f;  // 把最终平衡输出变量赋成零值
L5540  outerLoopAuthority =  // 外环权限清零，彻底退出速度与位置修正
L5542  0.0f;  // 把外环权限变量赋成零值
L5546  motor1ZeroHoldCount =  // 电机1零保持计数准备清零
L5548  0;  // 把电机1零保持计数清零
L5552  motor2ZeroHoldCount =  // 电机2零保持计数准备清零
L5554  0;  // 把电机2零保持计数清零
L5558  targetForwardSpeed =  // 开始把目标前进速度清零
L5560  0.0f;  // 把目标前进速度变量赋成零值
L5564  noInterrupts();  // 关中断，保证读取编码器计数时不被打断
L5568  int32_t resetCount1 =  // 声明电机1复位基准计数值
L5570  encoder1Count;  // 取关中断期间读到的编码器1累计计数
L5574  int32_t resetCount2 =  // 声明电机2复位基准计数值
L5576  encoder2Count;  // 取关中断期间读到的编码器2累计计数
L5580  interrupts();  // 重新开中断，恢复编码器增量采集
L5584  lastEncoder1Count =  // 把电机1上一拍计数对齐到新基准
L5586  resetCount1;  // 用刚才的原子快照给该变量赋值
L5590  lastEncoder2Count =  // 把电机2上一拍计数对齐到新基准
L5592  resetCount2;  // 用原子快照的电机2计数给该变量赋值
L5596  positionZeroCount1 =  // 位置环零点基准同步到电机1当前计数
L5598  resetCount1;  // 用电机1原子计数值作为位置零点
L5602  positionZeroCount2 =  // 位置环零点基准同步到电机2当前计数
L5604  resetCount2;  // 用电机2原子计数值作为位置零点
L5608  vehiclePosition =  // 开始把车辆累计位置清零
L5610  0.0f;  // 把车辆累计位置变量赋成零值
L5614  positionError = 0.0f;  // 位置误差清零，位置环从零开始
L5615  positionSpeedTarget = 0.0f;  // 把位置环给出的速度指令清零
L5616  positionHoldWasEnabled = false;  // 位置保持使能标记复位为假
L5620  vehicleSpeed =  // 开始把卡尔曼车速清零
L5622  0.0f;  // 把卡尔曼车速变量赋成零值
L5626  rawVehicleSpeed =  // 开始把编码器原始车速清零
L5628  0.0f;  // 把编码器原始车速变量赋成零值
L5632  imuForwardAccelG =  // 开始把IMU前进加速度清零
L5634  0.0f;  // 把IMU前进加速度变量赋成零值
L5638  imuForwardAccelFilteredG =  // 开始把滤波后的前进加速度清零
L5640  0.0f;  // 把滤波后的前进加速度变量清零
L5644  speedKalmanP =  // 速度卡尔曼协方差复位到初始值
L5646  1.0f;  // 协方差取初始不确定度1.0
L5650  targetBalanceAngle =  // 平衡目标角复位为机械平衡角O
L5652  balanceAngleOffset;  // 取机械平衡角O作为新的平衡目标
L5656  return;  // 倒地保护处理完毕，本拍直接返回
L5658  }  // 结束倒地保护动作执行块
L5666  manualForward.update(millis(), CONTROL_DT, pitch-balanceAngleOffset);  // 400Hz推进手动倾角通路，传入相对O的偏角
L5667  updatePositionHold();  // 用最新编码器累计位置更新位置环
L5679  outerLoopDivider++;  // 外环分频计数值加一，每拍递增
L5684  if (  // 开始100Hz外环的分频判断
L5686  outerLoopDivider  // 外环分频计数值，每拍加一
L5688  >=  // 达到4次即满10ms周期
L5690  4  // 400Hz下每4拍执行一次外环
L5692  )  // 结束外环触发判断条件
L5694  {  // 100Hz外环执行分支开始
L5696  outerLoopDivider =  // 外环分频计数准备复位
L5698  0;  // 把外环分频计数清零，下拍重新计
L5702  updateOuterLoop();  // 跑速度PI与位置外环，给出平衡目标修正
L5704  }  // 结束100Hz外环执行分支
L5719  updateOuterLoopAuthority();  // 400Hz按倾角平滑调整外环权限
L5745  updateBalanceLoop();  // 400Hz直立PD，算出平衡输出
L5758  updateTurnLoop();  // 400Hz转向环，算出转向输出
L5771  float motor1Command =  // 电机1指令为平衡与转向之和
L5773  finalBalanceOutput  // 平衡输出，两轮共模分量
L5775  +  // 加上转向输出，作为差模分量
L5777  turnOutput;  // 加上转向输出，形成右轮共模加差模
L5782  float motor2Command =  // 电机2指令为平衡减转向
L5784  finalBalanceOutput  // 平衡输出，两轮共模分量
L5786  -  // 减去转向输出，两轮形成差速
L5788  turnOutput;  // 减去转向输出，形成左轮反向差模
L5793  setMotor1(  // 把电机1指令下发给脉冲发生器
L5795  motor1Command  // 待下发的指令值，单位Hz
L5797  );  // 结束电机1指令下发函数调用
L5802  setMotor2(  // 把电机2指令下发给脉冲发生器
L5804  motor2Command  // 待下发的指令值，单位Hz
L5806  );  // 结束电机2指令下发函数调用
L5809  {  // 250ms遥测窗口极值统计块开始
L5810  float aGyro = fabsf(gyroPitchRate);  // 取俯仰角速度的绝对值
L5811  if (aGyro > telemGyroAbsMax) telemGyroAbsMax = aGyro;  // 刷新窗口内角速度的峰值
L5812  float aDev = fabsf(accPitch - gyroIntegAngle);  // 算加速度角与陀螺积分角的偏差绝对值
L5813  if (aDev > telemAccPitchDevMax) telemAccPitchDevMax = aDev;  // 记录窗口内姿态融合偏差的峰值
L5814  float aBal = fabsf(balanceOutput);  // 取直立输出的绝对值，单位Hz
L5815  if (aBal > telemBalHzAbsMax) telemBalHzAbsMax = aBal;  // 记录窗口内平衡输出峰值，单位Hz
L5816  if (kalmanBias < telemKalmanBiasMin) telemKalmanBiasMin = kalmanBias;  // 记录窗口内卡尔曼零偏的最小值
L5817  if (kalmanBias > telemKalmanBiasMax) telemKalmanBiasMax = kalmanBias;  // 记录窗口内卡尔曼零偏的最大值
L5818  }  // 结束遥测窗口极值统计块
L5822  flushTelemetryTx();  // 按发送预算把缓冲遥测推出去，不阻塞主拍
L5823  uint32_t workUs = micros() - controlWorkStartUs;  // 算本拍控制计算的实际耗时
L5824  if (workUs > controlMaxWorkUs) controlMaxWorkUs = workUs;  // 刷新单拍最大耗时，评估时序裕量
L5825  }  // loop函数体到此结束
```

## motion_drive.h

```cpp
L1  #pragma once  // 头文件只被包含一次的保护
L2  #include <cmath>  // 引入数学库，用于有限性判断与绝对值、开方运算
L3  #include "manual_turn.h"  // 复用其中的 clamp 限幅与转向控制声明
L16  struct MotionSpeedProfile {  // 速度目标斜坡：操作量只决定速度目标，倾角改由速度反馈闭环生成
L20  static constexpr float maxSpeed = 5.0f;  // 满档速度目标上限 5 counts/50ms，约合 100 counts/s
L21  static constexpr float accel    = 20.0f;  // 加速度限 20 counts/50ms 每秒，0 到满速约 0.5 秒
L22  float applied = 0.0f;  // 当前实际输出的速度目标，单位 counts/50ms
L23  void reset() { applied = 0.0f; }  // 清空斜坡状态，使下一次进入从零速起步
L24  float update(float target, float dt) {  // 按目标速度与控制周期做一阶限加速度推进
L25  if (!std::isfinite(target) || !std::isfinite(dt) || dt <= 0.0f) return applied;  // 目标或周期非法时保持原输出，避免 NaN 污染控制环
L26  float step = ManualTurnControl::clamp(target - applied, -accel * dt, accel * dt);  // 把这一拍的增量限制在正负 accel 乘 dt 内，防止目标跳变
L27  applied += step;  // 累加限加速度后的增量，得到本拍速度目标
L28  if (std::fabs(applied) < 0.0001f) applied = 0.0f;  // 接近零时归零，消除浮点残差造成的抖动量
L29  return applied;  // 把平滑后的速度目标交给速度环使用
L30  }  // 速度斜坡更新函数结束
L31  };  // 速度斜坡结构体定义结束
L36  struct NetSpeedEstimator {  // 200ms 净速度估计器：用长窗口位置差分求净速度
L37  static constexpr int windowSamples = 20;  // 100Hz 下 20 个样本正好覆盖 200ms 的时间窗口
L38  static constexpr float windowSeconds = 0.2f;  // 窗口长度 0.2 秒，用于把位置差换算成速度
L39  float history[windowSamples + 1] = {};  // 环形缓冲存 21 个历史位置，比窗口多留一格取最老值
L40  int head = 0;  // 环形缓冲的写入位置，逐格前进并回绕
L41  bool seeded = false;  // 首次调用前未播种，收到第一个位置先对齐
L42  float last1 = 0.0f, last2 = 0.0f;  // 前两拍位置，供三点中值滤波使用
L43  static float median3(float a, float b, float c) {  // 三点中值滤波，抑制单拍的位置跳变
L44  float lo = (a < b) ? a : b;  // 取 a、b 中较小者作为区间下界
L45  float hi = (a < b) ? b : a;  // 取 a、b 中较大者作为区间上界
L46  if (c < lo) return lo;  // c 偏小则取中值 lo，把跳变压回区间内
L47  if (c > hi) return hi;  // c 偏大则取中值 hi，同样削掉单拍尖峰
L48  return c;  // c 在区间内直接采用，保留真实运动信息
L49  }  // 三点中值滤波函数到此结束
L50  void reset(float position = 0.0f) {  // 把窗口全部填成同一位置，避免起步时算出巨大速度
L51  for (int i = 0; i <= windowSamples; ++i) history[i] = position;  // 用当前位置填满整个环形缓冲
L52  head = 0;  // 写指针归零，从窗口头部重新开始
L53  last1 = last2 = position;  // 前两拍位置也置成当前值
L54  seeded = true;  // 标记已播种，后续更新可以正常差分
L55  }  // 把窗口状态整体重置的复位函数到此结束
L56  float update(float position) {  // 每拍输入累计位置，输出 counts/50ms 净速度
L57  if (!std::isfinite(position)) return 0.0f;  // 位置非法时返回零速，不污染外环
L58  if (!seeded) { reset(position); return 0.0f; }  // 首拍只播种并返回零速，避免差值虚高
L59  float filtered = median3(last2, last1, position);  // 对最近三点取中值，抑制单拍跳变
L60  last2 = last1;  // 历史位置向后滚动一拍
L61  last1 = position;  // 记住本拍原始位置供下拍中值使用
L62  history[head] = filtered;  // 把滤波后的位置写入环形缓冲
L63  head = (head + 1) % (windowSamples + 1);  // 写指针环形前进一格并回绕
L64  float oldest = history[head];  // 取出 200ms 之前最老的那个位置点
L66  return (filtered - oldest) / windowSeconds * 0.05f;  // 位置差除以 0.2 秒再乘 0.05，换成 counts/50ms
L67  }  // 每拍更新净速度的函数到此结束
L68  };  // 净速度估计器定义结束
L74  struct MotionPositionPlan {  // 预测位置环：位置计划积分加前视补偿，输出只是速度增量
L75  static constexpr float horizon = 0.15f;  // 前视时间 0.15 秒，用来抵消净速度窗口的滞后
L76  float plan = 0.0f;  // 位置计划值，单位是编码器计数 count
L77  bool active = false;  // 是否处于已启动的位置运动状态
L78  void start(float position) { plan = position; active = true; }  // 启动位置运动，计划从当前位置开始
L79  void stop(float position) { plan = position; active = false; }  // 停止位置运动，计划跟随当前位置不回起点
L80  float update(float speedRef, float position, float netSpeed, bool engaged, float dt,  // 输入速度参考、位置、净速度等状态，输出速度增量
L81  float gain, float limit, float deadband) {  // 增益、输出限幅与死区宽度参数
L82  if (!std::isfinite(position) || !std::isfinite(netSpeed) ||  // 位置或净速度非法时先拦掉
L83  !std::isfinite(dt) || dt <= 0.0f) {  // 周期非法时同样走保护分支
L84  return 0.0f;  // 无效输入下不干预速度环
L85  }  // 非法输入保护分支到此结束
L86  if (engaged && active) {  // 处于运动且已启动时才积分位置计划
L87  plan += speedRef * (dt / 0.05f);  // 计划按参考速度积分，1 count/50ms 等于 20 counts/s
L88  } else {  // 其余情况让计划跟随实际位置
L89  plan = position;  // 不回起点，避免停车后自己往回跑
L90  }  // 计划跟随实际位置这一分支到此结束
L91  float predicted = position + netSpeed * (horizon / 0.05f);  // 用当前位置加净速度乘前视时间，预测未来位置
L92  float error = plan - predicted;  // 计划与预测位置之差即位置误差
L93  if (std::fabs(error) <= deadband) return 0.0f;  // 误差落在死区内不输出，防止原地来回修
L94  error -= (error > 0.0f) ? deadband : -deadband;  // 扣掉死区宽度，做成连续过渡而不是硬跳
L95  return ManualTurnControl::clamp(gain * error, -limit, limit);  // 按增益放大并限幅，得到速度环的修正增量
L96  }  // 位置环每拍更新的函数到此结束
L97  };  // 预测位置环结构体定义到此结束
L99  struct MotionLeanControl {  // 把速度误差变成倾角指令的倾角控制（EG 增益）
L103  static constexpr float feedForwardGain = 0.06f;  // 前馈 0.06 度每 counts/50ms，用来补静摩擦
L104  static constexpr float speedErrorGain  = 0.05f;  // 误差增益 0.05 度每 counts/50ms，是主调节项
L107  float errorGain = speedErrorGain;  // 运行时可调的误差增益，可由蓝牙 EG 改写
L108  static constexpr float limit           = 0.90f;  // 倾角绝对上限 0.90 度，贴边会进入摇不动的极限环
L109  static constexpr float engageRate      = 2.50f;  // 进入运动时倾角变化率上限 2.5 度每秒
L110  static constexpr float releaseRate     = 3.50f;  // 释放段允许 3.5 度每秒，可以降得快一些
L111  float requested = 0.0f;  // 本拍请求的目标倾角，单位度
L112  float applied = 0.0f;  // 经速率限制后实际施加的倾角
L113  void reset() { requested = applied = 0.0f; }  // 把请求倾角与实际输出一起清零
L115  void seed(float offset) {  // 进入运动时用现有倾角播种，避免入口跳变
L116  if (!std::isfinite(offset)) offset = 0.0f;  // 非法初值一律当作零来处理
L117  applied = ManualTurnControl::clamp(offset, -limit, limit);  // 把播种值限到倾角上限以内
L118  requested = applied;  // 请求值同步，避免立刻产生斜坡
L119  }  // 用现有倾角播种的初始化函数到此结束
L120  bool busy() const { return std::fabs(applied) > 0.0001f; }  // 倾角不为零即认为占用中，供状态机判断
L122  void update(float speedReference, float measuredSpeed, bool engaged, float dt) {  // 按速度参考与实测净速度计算倾角
L123  if (!std::isfinite(speedReference) || !std::isfinite(measuredSpeed) ||  // 速度参考或实测速度非法时
L124  !std::isfinite(dt) || dt <= 0.0f) {  // 控制周期非法时走复位分支
L125  reset();  // 非法输入直接复位，输出回到零
L126  return;  // 非法输入时本拍不再继续往下算
L127  }  // 非法输入保护分支到此结束
L128  if (engaged) {  // 运动时按前馈加误差增益算目标倾角
L129  float speedError = speedReference - measuredSpeed;  // 参考速度减实测净速度得到速度误差
L130  float base = feedForwardGain * speedReference + errorGain * speedError;  // 前馈补静摩擦，误差项主调节倾角大小
L132  requested = ManualTurnControl::clamp(base, -limit, limit);  // 把合成倾角限到正负 0.90 度以内
L133  } else {  // 未运动时目标倾角归零
L134  requested = 0.0f;  // 从当前倾角按释放斜率平滑回零
L135  }  // 未运动时目标倾角归零分支到此结束
L136  float rate = engaged ? engageRate : releaseRate;  // 按进入或释放选择不同的斜坡速率
L137  applied += ManualTurnControl::clamp(requested - applied, -rate * dt, rate * dt);  // 输出按速率限幅逼近请求值
L138  if (std::fabs(applied) < 0.0001f) applied = 0.0f;  // 接近零时归零，避免残留小倾角
L139  }  // 倾角每拍更新的函数到此结束
L140  };  // 倾角控制结构体定义结束
```

## position_jog.h

```cpp
L1  #pragma once  // 头文件只被包含一次的保护
L2  #include "manual_turn.h"  // 复用 clamp 限幅与转向控制声明
L3  #include <cstdlib>  // 解析命令帧要用 strtoul 与 strtof 两个函数
L4  #include <cerrno>  // 溢出检查依赖 errno 与 ERANGE 两个标准定义
L6  struct PositionOdometry {  // 位置里程计：由双轮编码器累计位置，单位 count
L7  int32_t last1=0,last2=0;  // 上一拍左右轮的原始编码器计数
L8  float position=0;  // 累计位置，单位是编码器计数 count
L9  bool initialized=false;  // 是否已完成首拍对齐的标志位
L10  void reset(int32_t a,int32_t b){last1=a;last2=b;position=0;initialized=true;}  // 用当前计数对齐，并把位置清零
L11  float update(int32_t a,int32_t b,int mode){  // 输入双轮计数与位置模式，返回累计位置
L12  if(!initialized){reset(a,b);return position;}  // 首拍只对齐不积分，避免起步跳变
L14  float d1=(int32_t)((uint32_t)a-(uint32_t)last1);  // 无符号相减再转回有符号，兼容编码器计数回绕
L15  float d2=(int32_t)((uint32_t)b-(uint32_t)last2);  // 右轮同样做回绕安全的增量计算
L16  last1=a;last2=b;  // 保存本拍计数供下拍差分
L17  position -= mode==0 ? .5f*(d1+d2) : (mode==1 ? d1 : (mode==2 ? d2 : 0));  // 按模式取双轮均值或单轮增量，正方向取负
L18  return position;  // 返回更新后的累计位置
L19  }  // 里程计每拍更新的函数到此结束
L20  };  // 里程计结构体定义结束
L22  struct PositionSpeedReference {  // 位置环速度参考斜坡：模式切换时不让速度请求跳步
L23  float applied=0;  // 当前斜坡输出值，单位是 counts/50ms
L24  void reset(){applied=0;}  // 清零速度参考斜坡的当前输出
L25  float update(float target,float dt){  // 按目标做每秒 3.0 的限斜率推进
L26  if(!std::isfinite(target)||!std::isfinite(dt)||dt<=0)return applied;  // 非法输入时保持上次输出
L27  applied += ManualTurnControl::clamp(target-applied,-3.0f*dt,3.0f*dt);  // 每拍变化不超过 3.0 乘 dt，避免请求跳变
L28  return applied;  // 返回平滑后的速度参考
L29  }  // 速度参考更新函数结束
L30  };  // 速度参考结构体定义结束
L32  struct PositionLeanControl {  // 位置环独立的 P 倾角控制，输出叠加给平衡环
L33  float kp=.005f,limit=.30f,requested=0,applied=0;  // 位置环默认 kp 为 0.005，倾角上限 0.30 度
L34  static bool validConfig(float p,float cap){  // 校验 kp 与上限范围，供蓝牙配置使用
L35  return std::isfinite(p)&&std::isfinite(cap)&&p>=0&&p<=.02f&&cap>=.05f&&cap<=.50f;  // kp 必须落在 0 到 0.02，倾角上限落在 0.05 到 0.50 度
L36  }  // 配置合法性校验函数到此结束
L37  bool busy()const{return std::fabs(applied)>.00001f;}  // 倾角未归零即认为位置环仍在动作
L38  void reset(){requested=applied=0;}  // 把请求倾角与实际输出一起清零
L39  void update(float error,bool moving,float dt,float direction){  // 由位置误差、是否运动与方向算目标倾角
L40  if(!std::isfinite(error)||!std::isfinite(dt)||dt<=0||!std::isfinite(direction)){reset();return;}  // 输入非法时复位并退出
L41  requested=moving ? ManualTurnControl::clamp(direction*kp*error,-limit,limit) : 0;  // 运动时按 kp 乘误差乘方向算倾角，否则归零
L42  bool building=moving && requested*applied>=0 && std::fabs(requested)>std::fabs(applied);  // 同向且误差在扩大时视为倾角建立段
L43  float rate=building ? .10f : .30f;  // 建立段慢速率 0.10，其余段 0.30 度每秒
L44  applied += ManualTurnControl::clamp(requested-applied,-rate*dt,rate*dt);  // 输出按速率限幅逼近目标倾角
L45  if(!moving && std::fabs(applied)<.00001f)applied=0;  // 不动且已接近零时彻底清零
L46  }  // 位置倾角更新函数结束
L47  };  // 位置倾角控制定义结束
L49  struct JogArrivalWindow {  // 到点统计窗口：连续 1 秒平稳才认定到达
L50  float elapsed=0,sumPosition=0,sumSpeed=0,first=0,low=0,high=0,center=0,error=0;  // 窗口内累计位置、速度以及首末与极值统计量
L51  int goodWindows=0;  // 连续合格窗口计数，达到 2 个才确认到达
L52  void reset(float position=0){elapsed=sumPosition=sumSpeed=0;first=low=high=center=position;error=0;goodWindows=0;}  // 清空统计并把极值对齐到当前位置
L53  bool update(float goal,float reference,float referenceSpeed,float position,float speed,float dt){  // 每拍喂入计划、实测位置与速度做统计
L54  if(std::fabs(goal-reference)>2 || std::fabs(referenceSpeed)>.2f){reset(position);return false;}  // 计划与参考偏差超 2 counts 或参考速度过大就重开窗口
L55  if(elapsed==0){first=low=high=position;}  // 窗口首拍记录起点位置
L56  low=std::fmin(low,position);high=std::fmax(high,position);  // 跟踪窗口内位置的最小值与最大值
L57  elapsed+=dt;sumPosition+=position*dt;sumSpeed+=speed*dt;  // 累计时间并做位置、速度的时间加权积分
L58  if(elapsed<.9999f)return false;  // 未满 1 秒不判定，继续统计
L59  center=sumPosition/elapsed;error=goal-center;  // 用时间加权均值算中心位置与到点误差
L60  bool good=std::fabs(error)<=5 && std::fabs(sumSpeed/elapsed)<=1 &&  // 误差不超 5 counts 且平均速度不超 1 counts/50ms
L61  std::fabs(position-first)<=10 && high-low<=60;  // 窗口内位移不超 10、峰峰值不超 60 counts
L62  goodWindows=good ? goodWindows+1 : 0;  // 合格则累加计数，否则清零重来
L63  elapsed=sumPosition=sumSpeed=0;  // 单窗统计量清零，开始下一个窗口
L64  return goodWindows>=2;  // 连续两个合格窗口即判定到达
L65  }  // 到达统计更新函数结束
L66  };  // 到达统计窗口定义结束
L67  struct PositionJog {  // 位置点动状态机：按增量走位并处理到达与异常
L68  enum Reason {NONE=0,RUNNING=1,ARRIVED=2,STOP=3,LINK_LOST=4,SOURCE_CHANGED=5,FAULT=6,TIMEOUT=7};  // 结束原因编码，0 空闲 1 运行 2 到达 3 停止等
L69  bool active=false,seen=false;  // 是否运动中、是否已收到过命令序号
L70  uint32_t sequence=0,started=0,lastHeartbeat=0;  // 命令序号、启动时刻与最近心跳时刻
L71  int source=0,reason=NONE;  // 来源控制模式与当前结束原因
L72  float goal=0,reference=0,referenceSpeed=0,speed=0,settled=0;  // 目标、参考位置、参考速度、实测速度与稳定量
L73  JogArrivalWindow arrival;  // 到点统计窗口的一个实例
L74  bool newer(uint32_t id)const{return !seen || (int32_t)(id-sequence)>0;}  // 序号用有符号比较，兼容 32 位自然回绕
L75  void cancel(int why){active=false;referenceSpeed=speed=settled=0;reason=why;}  // 取消运动并清零速度状态，记录原因
L76  bool start(uint32_t id,float delta,float position,int mode,uint32_t now){  // 接收序号、增量、当前位置、模式与时刻
L77  if(!newer(id))return false;  // 旧序号直接拒绝，防止重复执行同一条命令
L78  seen=true;sequence=id;  // 记录序号，后续心跳与更新都靠它校验
L79  if(delta==0){cancel(STOP);return true;}  // 增量为零即立即停止，仍返回成功
L81  if(!active){reference=position;referenceSpeed=0;speed=0;}  // 从静止启动时参考对齐当前位置，速度从零起
L82  goal=position+ManualTurnControl::clamp(delta,-200,200);  // 目标位置等于当前位置加增量，增量限正负 200 counts
L83  source=mode;started=lastHeartbeat=now;  // 记录来源模式与启动时刻，用于超时判断
L84  settled=0;arrival.reset(position);active=true;reason=RUNNING;  // 清稳定量、重开到达窗口并置为运行态
L85  return true;  // 命令被接受，启动流程成功返回
L86  }  // 点动启动函数到此结束
L87  void heartbeat(uint32_t id,uint32_t now){if(active && id==sequence)lastHeartbeat=now;}  // 序号匹配才刷新心跳，防止旧心跳续命
L88  void update(float position,float measuredSpeed,int mode,uint32_t now,float dt){  // 每拍推进参考轨迹并检查异常与到达
L89  if(!active)return;  // 未处于运动状态时直接返回不做处理
L90  if(!std::isfinite(position)||!std::isfinite(measuredSpeed)||!std::isfinite(dt)||dt<=0){  // 位置、速度或周期非法时判为故障
L91  cancel(FAULT);return;  // 按故障原因取消本次点动运动
L92  }  // 非法输入保护分支到此结束
L93  if(mode!=source){cancel(SOURCE_CHANGED);return;}  // 控制模式变了就取消，避免跨界继续走位
L94  if(uint32_t(now-lastHeartbeat)>500){cancel(LINK_LOST);return;}  // 超过 500ms 没有心跳判为链路丢失
L95  if(uint32_t(now-started)>12000){cancel(TIMEOUT);return;}  // 单次点动超过 12 秒判超时并取消
L96  float remaining=goal-reference;  // 剩余距离等于目标位置减参考位置
L97  float direction=remaining>=0 ? 1.0f : -1.0f;  // 按剩余方向取正负一作为运动方向
L99  float leadRoom=std::fmax(0.0f,50.0f-direction*(reference-position));  // 前视余量：参考超前的位置越多，可用余量越小
L100  float leadCap=std::fmin(.3f*leadRoom,std::sqrt(.3f*leadRoom));  // 前视限速取线性与开方两者中的较小值
L101  float wanted=direction*std::fmin(3.0f,std::fmin(.08f*std::fabs(remaining),leadCap));  // 期望参考速度，同时受剩余距离与前视余量限制
L102  float oldSpeed=referenceSpeed;  // 保存上一拍参考速度用于梯形积分
L103  referenceSpeed += ManualTurnControl::clamp(wanted-referenceSpeed,-3.0f*dt,3.0f*dt);  // 参考速度按每秒 3.0 限斜率逼近期望值
L104  reference += .5f*(oldSpeed+referenceSpeed)*20.0f*dt;  // 参考位置按平均速度积分，乘 20 换成 counts/s
L105  speed=0;  // 点动模式只给零速阻尼，不下发速度命令
L106  if(arrival.update(goal,reference,referenceSpeed,position,measuredSpeed,dt))cancel(ARRIVED);  // 到点窗口确认后按到达原因结束
L107  }  // 点动状态机每拍更新的函数到此结束
L108  };  // 位置点动状态机定义结束
L109  inline bool parseJogId(const char* text,uint32_t& id,const char*& rest){  // 解析点动帧开头的命令序号部分
L110  if(*text<'0'||*text>'9')return false;  // 必须数字开头，否则报文不合法
L111  errno=0;char* end=nullptr;unsigned long value=std::strtoul(text,&end,10);  // 把序号解析成无符号长整型
L112  if(end==text || errno==ERANGE || value>UINT32_MAX)return false;  // 没有数字、溢出或超 32 位都判失败
L113  id=(uint32_t)value;rest=end;return true;  // 写出序号与剩余文本指针
L114  }  // 点动序号解析函数到此结束
L115  inline bool parseJogFrame(const char* text,uint32_t& id,float& delta){  // 解析完整点动帧：序号加逗号再加增量
L116  const char* rest=nullptr;if(!parseJogId(text,id,rest)||*rest!=',')return false;  // 序号后面必须紧跟逗号
L117  const char* value=rest+1;char* end=nullptr;float v=std::strtof(value,&end);  // 解析逗号后面的增量浮点数
L118  if(end==value||*end!='\0'||!std::isfinite(v)||std::fabs(v)>200||(v!=0 && std::fabs(v)<5))return false;  // 必须整串结束，幅度不超 200 且非零时至少 5 counts
L119  delta=v;return true;  // 写出解析得到的增量并返回成功
L120  }  // 完整点动帧解析函数到此结束
L122  inline bool parseJogConfig(const char* text,float& kp,float& limit){  // 解析位置环配置帧：kp 加逗号再加上限
L123  char* end=nullptr;float p=std::strtof(text,&end);  // 解析逗号前面的第一段 kp 数值
L124  if(end==text||*end!=',')return false;  // 第一段 kp 解析完后必须紧跟逗号
L125  const char* capText=end+1;float cap=std::strtof(capText,&end);  // 解析逗号后面的第二段倾角上限
L126  if(end==capText||*end!='\0'||!PositionLeanControl::validConfig(p,cap))return false;  // 整串结束且配置合法才接受
L127  kp=p;limit=cap;return true;  // 把校验通过的 kp 与上限写回调用方
L128  }  // 位置环配置帧解析函数到此结束
```

## param_store.h

```cpp
L1  #pragma once  // 头文件只被包含一次的保护
L2  #include <cstddef>  // 引入 size_t 等标准库基础类型定义
L3  #include <cstdint>  // 引入定宽整数类型，保证记录字段宽度固定
L4  #include <cstring>  // 用到 memcpy 与 memcmp 两个内存操作函数
L23  namespace ParamStore {  // 参数持久化纯逻辑层所在命名空间
L25  constexpr uint8_t  PARAM_COUNT    = 22;  // 参数共 22 项，与面板项数和 PAR 回显顺序一致
L26  constexpr uint16_t FORMAT_VERSION = 3;  // 记录格式版本，改参数顺序或语义必须加一作废旧记录
L27  constexpr uint32_t RECORD_MAGIC   = 0x50434833u;  // 记录魔数 PCH3，用来识别本格式的记录
L28  constexpr uint32_t ERASED_WORD    = 0xFFFFFFFFu;  // 擦除后的 flash 字内容，即全一
L30  struct Record {  // 一条参数记录，长度固定 104 字节
L31  uint32_t magic;  // 魔数，占记录的第一个字
L32  uint16_t version;  // 记录格式版本号，占一个 16 位字段
L33  uint16_t count;  // 参数项数，与版本号共用一个字
L34  uint32_t seq;  // 递增序号，扫描时取最大者为准
L35  float    values[PARAM_COUNT];  // 存放 22 个 float 参数值，合计 88 字节
L36  uint32_t crc;  // CRC32，覆盖本字段之前的全部字节
L37  };  // 参数记录结构体定义到此结束
L39  static_assert(sizeof(float) == 4, "记录格式假定 float 为 4 字节");  // 编译期确认 float 为 4 字节，否则记录布局失效
L40  static_assert(sizeof(Record) == 104, "记录必须是 104 B（4 字节对齐，无填充）");  // 编译期确认整条记录恰为 104 字节且无填充
L41  constexpr uint32_t RECORD_WORDS = sizeof(Record) / 4;  // 一条记录按 32 位字计为 26 个字
L44  inline uint32_t crc32(const void* data, size_t len, uint32_t crc = 0xFFFFFFFFu) {  // 逐位实现的 CRC32，用 IEEE 802.3 反射多项式
L45  const uint8_t* p = static_cast<const uint8_t*>(data);  // 转成字节指针逐字节喂入
L46  for (size_t i = 0; i < len; ++i) {  // 遍历待校验的每个字节
L47  crc ^= p[i];  // 把当前字节的值异或进 CRC 结果
L48  for (int b = 0; b < 8; ++b) {  // 每个字节要做 8 次移位处理
L49  uint32_t mask = static_cast<uint32_t>(-static_cast<int32_t>(crc & 1u));  // 按最低位生成全零或全一的掩码
L50  crc = (crc >> 1) ^ (0xEDB88320u & mask);  // 右移一位并按掩码条件异或反射多项式
L51  }  // 内层八次位移的循环到此结束
L52  }  // 逐字节累加校验的外层循环结束
L53  return ~crc;  // 按标准做法取反，得到最终 CRC
L54  }  // 逐位实现的 CRC32 函数到此结束
L57  constexpr uint32_t RECORD_PAYLOAD_BYTES = sizeof(Record) - sizeof(uint32_t);  // 记录除 crc 之外的字节数，也就是被校验的部分
L60  class FlashPort {  // 平台适配接口：扇区大小、读取、擦除与逐字编程
L61  public:  // 以下为子类必须实现的纯虚接口
L62  virtual ~FlashPort() = default;  // 虚析构，允许通过基类指针释放派生对象
L63  virtual uint32_t sectorSizeBytes() const = 0;  // 子类需返回单个扇区的字节数
L64  virtual uint32_t sectorCount() const = 0;  // 子类需返回可用的扇区总数量
L66  virtual const uint32_t* readWords(uint32_t sector) const = 0;  // 取扇区首字的只读视图，供扫描与回读
L67  virtual bool eraseSector(uint32_t sector) = 0;  // 擦除指定编号的整个 flash 扇区
L68  virtual bool programWord(uint32_t sector, uint32_t wordIndex, uint32_t value) = 0;  // 按字序号向扇区编程一个字
L69  };  // 平台适配虚接口定义到此结束
L71  class Store {  // 扫描、预清与追加写的实现类
L72  public:  // 以下是 Store 对外公开的接口方法
L73  static constexpr uint32_t MAX_SECTORS = 4;  // 扫描与校验最多只处理 4 个扇区
L75  struct Scan {  // 扫描结果：最新记录位置与各扇区追加点
L76  bool     found  = false;  // 是否已经找到一条有效记录
L77  uint32_t seq    = 0;  // 有效记录中的最大序号
L78  uint8_t  sector = 0;  // 该记录所在扇区，新记录优先追加到这里
L79  uint32_t nextSlot[MAX_SECTORS] = {};  // 每个扇区下一个可写槽位
L80  };  // 扫描结果结构定义结束
L84  Scan scan(const FlashPort& port, float* values) const {  // 扫描全部扇区，取序号最大且校验通过的记录
L85  Scan out;  // 先构造一个空的扫描结果作为返回值
L86  const uint32_t count = port.sectorCount();  // 先问适配层一共有几个扇区
L87  if (count == 0 || count > MAX_SECTORS) return out;  // 扇区数为零或超上限直接返回空结果
L88  for (uint32_t s = 0; s < count; ++s) {  // 对每个扇区依次做一遍完整扫描
L89  const uint32_t* base = port.readWords(s);  // 取该扇区首字地址当作基址
L90  const uint32_t slots = port.sectorSizeBytes() / sizeof(Record);  // 每个扇区可放下的槽位数
L91  if (base == nullptr) { out.nextSlot[s] = 0; continue; }  // 指针为空时该扇区从第 0 槽起写
L92  uint32_t slot = 0;  // 从第一个槽位开始向后找
L93  for (; slot < slots; ++slot) {  // 遍历槽位，遇到空槽就停
L94  const uint32_t* w = base + slot * RECORD_WORDS;  // 指向本槽位记录的首字地址
L95  if (w[0] == ERASED_WORD) break;  // 首字仍是全一说明这里是空槽，往后的都不必看
L96  if (recordValid(w)) {  // 校验通过的记录才参与比较
L97  uint32_t seq = w[2];  // 第三个字存序号，读出来比较大小
L98  if (!out.found || seq > out.seq) {  // 序号更大时才更新扫描结果
L99  out.found  = true;  // 标记已经找到有效记录
L100  out.seq    = seq;  // 记下目前找到的最大序号
L101  out.sector = static_cast<uint8_t>(s);  // 记下这条记录所在的扇区
L102  if (values != nullptr) {  // 调用方需要参数值时才拷贝
L103  std::memcpy(values,  // 从记录偏移 12 字节处开始拷贝参数值
L104  reinterpret_cast<const uint8_t*>(w) + 12,  // 魔数、版本计数、序号共占前 12 字节
L105  PARAM_COUNT * sizeof(float));  // 一次拷满 22 个 float 参数，共 88 字节
L106  }  // 参数值拷贝分支到此结束
L107  }  // 更大序号比较分支到此结束
L108  }  // 有效记录处理分支到此结束
L109  }  // 单扇区内的槽位循环到此结束
L110  out.nextSlot[s] = slot;  // 记录本扇区第一个空槽的位置
L111  }  // 所有扇区的扫描循环到此结束
L112  return out;  // 把汇总好的扫描结果返回调用方
L113  }  // 整个扫描函数到此结束
L117  bool prepare(FlashPort& port, Scan& sc) {  // 开机预清：当前扇区写满就趁开机先擦好另一个扇区
L118  const uint32_t sectors = port.sectorCount();  // 先读出适配层的扇区总数备用
L119  if (!sc.found || sectors < 2 || sectors > MAX_SECTORS) return true;  // 无有效记录或扇区不足两个时无需预清
L120  const uint32_t slotsPerSector = port.sectorSizeBytes() / sizeof(Record);  // 算出每个扇区能放多少条记录
L121  if (sc.nextSlot[sc.sector] < slotsPerSector) return true;  // 当前扇区还有空槽，直接返回不必预清
L122  const uint32_t other = (sc.sector + 1u) % sectors;  // 取另一个扇区作为轮换目标
L123  if (!port.eraseSector(other)) return false;  // 擦除另一个扇区，失败则立即返回
L124  sc.nextSlot[other] = 0;  // 把该扇区的追加点清零
L125  sc.sector = static_cast<uint8_t>(other);  // 后续追加落到刚擦好的扇区
L126  return true;  // 预清完成，可安全继续开机流程
L127  }  // 开机预清函数到此结束
L131  bool save(FlashPort& port, const float* values, Scan& sc, bool& erasedAny) {  // 追加一条记录并就地更新扫描结果
L132  erasedAny = false;  // 默认本次没有发生擦除
L133  const uint32_t sectors = port.sectorCount();  // 读出扇区总数用于后续判断
L134  if (sectors == 0 || sectors > MAX_SECTORS || values == nullptr) return false;  // 扇区数或参数指针非法时直接判失败
L136  uint32_t sector;  // 声明待写入记录所在的扇区号
L137  uint32_t slot;  // 声明待写入记录所在的槽位号
L138  uint32_t seq;  // 声明本条记录要用的新序号
L139  if (!sc.found) {  // 没有有效记录时从零开始写
L142  if (!port.eraseSector(0)) return false;  // 先擦 0 号扇区，flash 只能把 1 写成 0 不能直接覆盖
L143  erasedAny = true;  // 标记本次发生了擦除，耗时可到秒级
L144  sector = 0;  // 没有旧记录时从 0 号扇区开头写
L145  slot   = 0;  // 从该扇区第 0 个槽位开始写
L146  seq    = 1;  // 首条记录的序号从 1 开始
L147  } else {  // 已有有效记录时按追加处理
L148  sector = sc.sector;  // 优先追加到最新记录所在的扇区
L149  slot   = sc.nextSlot[sector];  // 追加到该扇区的下一个空槽
L150  seq    = sc.seq + 1;  // 序号在前一条基础上加一
L151  if (slot >= port.sectorSizeBytes() / sizeof(Record)) {  // 槽位用尽说明本扇区已经写满
L153  uint32_t other = (sector + 1u) % sectors;  // 换到另一个扇区继续写
L154  if (!port.eraseSector(other)) return false;  // 先把另一个扇区整块擦干净
L155  erasedAny = true;  // 标记本次发生了扇区擦除
L156  sector = other;  // 后续记录写到刚擦好的扇区
L157  slot   = 0;  // 从该扇区的第 0 个槽位开始
L158  sc.nextSlot[other] = 0;  // 同步更新扫描结果里的追加点
L159  }  // 写满换扇区的分支到此结束
L160  }  // 已有记录时的追加分支到此结束
L162  Record rec = {};  // 先在内存里组好整条记录
L163  rec.magic   = RECORD_MAGIC;  // 把记录魔数 PCH3 写进第一个字
L164  rec.version = FORMAT_VERSION;  // 写入当前的记录格式版本号
L165  rec.count   = PARAM_COUNT;  // 写入参数项数，与版本共用一个字
L166  rec.seq     = seq;  // 写入本条记录的递增序号
L167  std::memcpy(rec.values, values, PARAM_COUNT * sizeof(float));  // 把 22 个参数值一次拷进记录
L168  rec.crc = crc32(&rec, RECORD_PAYLOAD_BYTES);  // 对 crc 之前的字节计算校验值
L170  const uint32_t* words = reinterpret_cast<const uint32_t*>(&rec);  // 把记录按 32 位字视图逐字编程
L171  for (uint32_t i = 0; i < RECORD_WORDS; ++i) {  // 把整条记录拆成 26 个 32 位字依次编程
L172  if (!port.programWord(sector, slot * RECORD_WORDS + i, words[i])) return false;  // 任一个字编程失败即整体判失败
L173  }  // 逐字编程的循环到此结束
L176  const uint32_t* base = port.readWords(sector) + slot * RECORD_WORDS;  // 回读刚写入的那个槽位
L177  if (base == nullptr || std::memcmp(base, &rec, sizeof(rec)) != 0) return false;  // 回读内容与内存记录不一致则判失败
L179  sc.found  = true;  // 成功后更新扫描结果的找到标记
L180  sc.seq    = seq;  // 把扫描结果里的最新序号更新一下
L181  sc.sector = static_cast<uint8_t>(sector);  // 更新扫描结果里的扇区号
L182  sc.nextSlot[sector] = slot + 1;  // 把该扇区的追加点前移一个槽
L183  return true;  // 整条记录写成功并返回真
L184  }  // 追加保存函数到此结束
L186  private:  // 以下为内部使用的静态辅助函数
L187  static bool recordValid(const uint32_t* w) {  // 校验一条记录：CRC、魔数、版本、项数与数值有限性
L188  uint32_t crc = crc32(w, RECORD_PAYLOAD_BYTES);  // 对记录负载部分重新算一遍 CRC
L189  if (crc != w[RECORD_WORDS - 1]) return false;  // CRC 与记录尾部不符即无效
L190  if (w[0] != RECORD_MAGIC) return false;  // 魔数不是 PCH3 就直接判无效
L192  if (static_cast<uint16_t>(w[1] & 0xFFFFu) != FORMAT_VERSION) return false;  // 低 16 位记录的版本号必须匹配
L193  if (static_cast<uint16_t>(w[1] >> 16) != PARAM_COUNT) return false;  // 高 16 位参数项数必须匹配
L195  const float* v = reinterpret_cast<const float*>(reinterpret_cast<const uint8_t*>(w) + 12);  // 参数值的起始地址是记录偏移 12 字节处
L196  for (uint8_t i = 0; i < PARAM_COUNT; ++i) {  // 逐个检查参数值是否有限
L197  if (!(v[i] > -1.0e9f && v[i] < 1.0e9f)) return false;  // 超出正负十亿或为 NaN 都判无效，避免整车失控
L198  }  // 参数值有限性检查循环到此结束
L199  return true;  // 所有校验都通过，记录可用
L200  }  // 记录校验函数到此结束
L201  };  // 参数存储类 Store 的定义到此结束
L203  }  // 参数持久化命名空间结束
```

## manual_motion.h

```cpp
L1  #pragma once  // 头文件只被包含一次的保护
L2  #include "manual_turn.h"  // 复用限幅函数与转向控制声明
L3  #include <cstdlib>  // 解析手柄帧时要用到 strtof 函数
L6  struct ManualPitchControl {  // 固定相对俯仰控制：角度与角速度连续过渡，含中途释放
L7  enum Phase {IDLE=0, MOVE=1, BRAKE=2};  // 三个相位：空闲、运动、制动
L8  static constexpr float maxAngle=2.0f;  // 固定俯仰角的绝对上限为 2.0 度
L9  static constexpr float maxRate=2.0f;  // 角速度的绝对上限为 2.0 度每秒
L10  static constexpr float maxAcceleration=4.0f;  // 角加速度上限 4.0 度每二次方秒
L11  float requested=0,applied=0,rate=0;  // 请求角、实际角与当前角速度
L12  uint32_t receivedMs=0;  // 最近一次命令的到达时刻，用于 350ms 超时
L13  bool needsZero=false;  // 必须先收到零位才接受非零命令
L14  int phase=IDLE;  // 当前所处相位，默认是空闲
L15  float planned=0,elapsed=0,duration=0,start=0,delta=0,brakeRate=0,resumeTime=1;  // 计划角、已过时间、段时长、起点、增量、制动员速率与恢复度
L16  void clear(){requested=applied=rate=planned=0;phase=IDLE;elapsed=duration=0;resumeTime=0;needsZero=true;}  // 整体复位，并把零位锁定重新置上
L17  void seed(float offset){  // 用当前实际倾角播种，避免入口跳变
L18  applied=ManualTurnControl::clamp(offset,-maxAngle,maxAngle);rate=0;  // 播种值限幅到正负 2 度并清零角速度
L19  planned=applied;phase=IDLE;elapsed=duration=0;resumeTime=0;  // 计划对齐输出，相位回到空闲并清时间
L20  }  // 用当前角播种的初始化函数到此结束
L21  void command(float angle,uint32_t now){  // 接收新的目标俯仰角与时刻
L22  if(angle==0)needsZero=false;  // 收到零位即解除零位锁定
L23  if(needsZero)return;  // 尚未收到零位前忽略非零命令
L24  requested=std::isfinite(angle) ? ManualTurnControl::clamp(angle,-maxAngle,maxAngle) : 0;  // 目标角限幅到正负 2 度，非法值按零处理
L25  receivedMs=now;  // 记录命令时刻供超时判断
L26  }  // 接收新命令的函数到此结束
L27  bool busy(uint32_t now)const{  // 是否仍在跟踪命令：请求有效、相位非空闲或未归零
L28  return (requested!=0 && uint32_t(now-receivedMs)<=350) || phase!=IDLE || std::fabs(applied)>.000001f;  // 350ms 内的非零命令或未回零都算忙
L29  }  // 运动忙判断函数到此结束
L30  float normalAuthority()const{  // 恢复度用 smoothstep 从零平滑升到一
L31  float u=ManualTurnControl::clamp(resumeTime,0,1);  // 把恢复度限制在 0 到 1 之间
L32  return u*u*(3-2*u);  // 三次 smoothstep 曲线，两端斜率为零
L33  }  // 恢复度计算函数到此结束
L34  void beginMove(){  // 从当前角出发规划到计划角的三次过渡段
L35  start=applied;delta=planned-start;elapsed=0;  // 记录起点与增量，时间清零
L36  if(std::fabs(delta)<.000001f){applied=planned;rate=0;phase=IDLE;return;}  // 增量近零说明已在目标位置，直接回空闲
L38  duration=std::fmax(1.5f*std::fabs(delta)/maxRate,std::sqrt(6*std::fabs(delta)/maxAcceleration));  // 段时长取速度与加速度两个下界中的较大者
L39  phase=MOVE;  // 进入运动相位，开始插值过渡
L40  }  // 开始运动规划的函数到此结束
L41  void update(uint32_t now,float dt,float angleError){  // 每拍推进相位并输出当前俯仰角
L42  if(uint32_t(now-receivedMs)>350)requested=0;  // 超过 350ms 没有新命令则目标归零
L43  if(!std::isfinite(angleError)||!std::isfinite(dt)||dt<=0||std::fabs(angleError)>=10){clear();return;}  // 输入非法或倾角误差不小于 10 度则整体复位
L44  if(requested!=planned){  // 目标发生变化时重新规划
L45  planned=requested;  // 把计划目标更新为新的请求值
L46  if(phase!=BRAKE){  // 制动相位中不重复规划
L47  if(std::fabs(rate)>.000001f){  // 还有角速度就先制动到零
L48  start=applied;brakeRate=rate;elapsed=0;duration=std::fabs(rate)/maxAcceleration;phase=BRAKE;  // 按当前角速度算制动时长并进入制动相位
L49  }else beginMove();  // 角速度已为零则直接开始新过渡段
L50  }  // 制动相位处理分支到此结束
L51  }  // 目标变化处理分支到此结束
L52  float left=dt;  // 本拍剩余可用的推进时间
L53  for(int transition=0;transition<3 && left>0 && phase!=IDLE;transition++){  // 一拍内最多推进三段相位切换
L54  float step=std::fmin(left,duration-elapsed);elapsed+=step;left-=step;  // 取本段剩余时间与本拍剩余时间中的较小者
L55  if(phase==BRAKE){  // 制动相位按匀减速推进
L56  float sign=brakeRate>0 ? 1.0f : -1.0f;  // 按制动员速率的方向取符号
L57  applied=start+brakeRate*elapsed-.5f*sign*maxAcceleration*elapsed*elapsed;  // 位置按匀减速运动学公式计算
L58  rate=brakeRate-sign*maxAcceleration*elapsed;  // 角速度随时间线性减小
L59  if(elapsed>=duration){rate=0;beginMove();}  // 制动到零速后接着规划新段
L60  }else{  // 运动相位按 smoothstep 曲线插值推进
L61  float u=ManualTurnControl::clamp(elapsed/duration,0,1);  // 把已过时间归一化到 0 到 1
L62  applied=start+delta*u*u*(3-2*u);  // 位置沿 smoothstep 曲线从起点过渡到终点
L63  rate=delta*6*u*(1-u)/duration;  // 角速度等于 smoothstep 曲线的导数
L64  if(elapsed>=duration){applied=planned;rate=0;phase=IDLE;}  // 到达终点后对齐计划并回到空闲
L65  }  // 运动相位处理分支到此结束
L66  }  // 相位推进循环到此结束
L67  applied=ManualTurnControl::clamp(applied,-maxAngle,maxAngle);  // 输出再限幅到正负 2 度，防止数值越界
L68  resumeTime=busy(now) ? 0 : std::fmin(1.0f,resumeTime+dt);  // 忙则恢复度清零，闲则每拍累加直到一
L69  }  // 俯仰每拍更新的函数到此结束
L70  };  // 俯仰控制结构体定义结束
L73  inline bool parseManualMotion(const char* value, float& turn, float& forward) {  // 解析整帧转向频率加逗号加俯仰角，全合法才提交
L74  char* end=nullptr;  // 声明解析结束位置的指针
L75  float t=std::strtof(value,&end);  // 解析逗号前面的转向频率
L76  if (end==value || *end!=',' || !std::isfinite(t)) return false;  // 必须有逗号且为有限数
L77  const char* second=end+1;  // 逗号后面的俯仰角文本
L78  float f=std::strtof(second,&end);  // 解析逗号后面的俯仰角分量
L79  if (end==second || *end!='\0' || !std::isfinite(f)) return false;  // 必须整串结束且为有限数
L80  turn=t; forward=f;  // 两段都合法后一起提交，避免半帧生效
L81  return true;  // 解析成功，可以提交这一帧
L82  }  // 手柄帧解析函数到此结束
```

## manual_turn.h

```cpp
L1  #pragma once  // 头文件只被包含一次的保护
L2  #include <cmath>  // 引入数学库，用到有限性判断与绝对值
L3  #include <cstdint>  // 引入定宽整数，用于毫秒时间戳
L6  struct ManualTurnControl {  // 转向控制：输入是左右轮 STEP 频率差（Hz），不是标定偏航率
L7  float requested = 0, applied = 0;  // 请求频率与经限速后的实际频率
L8  uint32_t receivedMs = 0;  // 最近命令时刻，用于超时归零
L9  bool needsZero = false;  // 必须先收到零位才接受非零命令
L10  static constexpr uint32_t timeoutMs = 350;  // 命令 350ms 未刷新即超时归零
L11  static constexpr float maxHz = 300.0f;  // 频率差请求的绝对上限为 300Hz
L12  static constexpr float slewHzPerSecond = 300.0f;  // 频率变化率上限为每秒 300Hz
L13  static float clamp(float x, float lo, float hi) {  // 通用限幅函数，被多个模块复用
L14  return x < lo ? lo : (x > hi ? hi : x);  // 低于下限取下限，高于上限取上限
L15  }  // 通用限幅函数到此结束
L16  void clear() { requested = applied = 0; needsZero = true; }  // 清空请求与输出并复位零位锁定
L17  void command(float hz, uint32_t now) {  // 接收新的左右轮频率差命令
L18  if (hz == 0) needsZero = false;  // 收到零位即解除零位锁定
L19  if (needsZero) return;  // 未收到零位前忽略非零命令
L20  requested = std::isfinite(hz) ? clamp(hz, -maxHz, maxHz) : 0;  // 限幅到正负 300Hz，非法值按零处理
L21  receivedMs = now;  // 记录这次命令的毫秒时刻
L22  }  // 接收频率命令的函数到此结束
L23  bool busy(uint32_t now) const {  // 命令有效或输出未归零即认为还忙
L24  return (requested != 0 && uint32_t(now-receivedMs) <= timeoutMs)  // 350ms 内的非零请求算还在忙
L25  || std::fabs(applied) > 0.01f;  // 实际输出仍大于 0.01Hz 也算忙
L26  }  // 转向忙判断函数到此结束
L27  float update(uint32_t now, float dt, float angleError,  // 每拍把频率差请求转成可下发的频率
L28  float limit, float balance, float legacy) {  // 倾角误差、输出上限、平衡共模量与旧偏置
L29  if (uint32_t(now-receivedMs) > timeoutMs) requested = 0;  // 命令超时后把请求值归零
L30  if (!std::isfinite(angleError) || !std::isfinite(limit) ||  // 任一路输入参数非法时
L31  !std::isfinite(balance) || !std::isfinite(legacy) ||  // 都走清零保护分支返回
L32  !std::isfinite(dt) || dt <= 0) { clear(); return 0; }  // 周期非法时清空并返回零
L34  float room = std::fmax(0.0f, limit-std::fabs(balance));  // 扣掉平衡共模输出后的剩余空间，避免单轮削顶
L35  float cap = std::fmin(maxHz, std::fmax(0.0f, limit)*0.25f);  // 本次可用上限，最多为上限的四分之一且不超 300Hz
L36  float authority = clamp((10.0f-std::fabs(angleError))/5.0f, 0, 1);  // 倾角误差 5 到 10 度之间线性降低权限
L37  cap = std::fmin(room, cap*authority);  // 上限同时受平衡余量与权限约束
L38  if (authority == 0) { requested = 0; needsZero = true; }  // 权限归零即锁零位，禁止继续转向
L39  float target = clamp(requested+legacy, -cap, cap);  // 请求值叠加旧偏置后再限幅
L40  applied += clamp(target-applied, -slewHzPerSecond*dt, slewHzPerSecond*dt);  // 按每秒 300Hz 的斜率逼近目标
L42  applied = clamp(applied, -cap, cap);  // 失权时立即压回上限，不等斜坡
L43  return applied;  // 返回本拍应下发的频率差
L44  }  // 转向每拍更新的函数到此结束
L45  };  // 转向控制结构体定义结束
```

## param_flash_f4.h

```cpp
L1  #pragma once  // 头文件只被包含一次的保护
L2  #include "param_store.h"  // 复用参数记录格式与 FlashPort 抽象接口
L14  #ifdef STM32F4xx  // 仅在带 F4 HAL 宏时才编译本适配层
L15  #include <stm32f4xx_hal.h>  // 需要 HAL 的 flash 擦写接口与标志位宏
L17  class Stm32F4FlashPort : public ParamStore::FlashPort {  // 用 F4 内部 flash 的 sector10 与 11 做双备份端口
L18  public:  // 以下是重写基类接口的公开段
L19  static constexpr uint32_t SECTOR_ADDRESS[2] = {0x080C0000u, 0x080E0000u};  // 两个扇区首址分别是 0x080C0000 与 0x080E0000
L20  static constexpr uint32_t HAL_SECTOR_ID[2]  = {FLASH_SECTOR_10, FLASH_SECTOR_11};  // 对应的 HAL 扇区编号是扇区 10 与 11
L21  static constexpr uint32_t SECTOR_BYTES      = 128u * 1024u;  // F407 上每个扇区固定为 128KB 大小
L23  uint32_t sectorSizeBytes() const override { return SECTOR_BYTES; }  // 向存储层报告单个扇区的字节数
L24  uint32_t sectorCount() const override { return 2; }  // 向存储层报告本端口只有两个扇区
L26  const uint32_t* readWords(uint32_t sector) const override {  // 取扇区首字的只读视图
L27  if (sector > 1u) return nullptr;  // 扇区号越界则返回空指针
L28  return reinterpret_cast<const uint32_t*>(SECTOR_ADDRESS[sector]);  // 把首地址转成 32 位字指针
L29  }  // 扇区只读视图函数到此结束
L31  bool eraseSector(uint32_t sector) override {  // 擦除指定编号的整个扇区
L32  if (sector > 1u) return false;  // 扇区号非法就直接返回失败
L33  FLASH_EraseInitTypeDef init = {};  // 准备 HAL 擦除所需参数结构
L34  init.TypeErase    = FLASH_TYPEERASE_SECTORS;  // 擦除类型为整扇区擦除
L35  init.Sector       = HAL_SECTOR_ID[sector];  // 按 HAL 编号指定要擦的扇区
L36  init.NbSectors    = 1;  // 一次调用只擦除一个扇区
L37  init.VoltageRange = FLASH_VOLTAGE_RANGE_3;  // 选 2.7 到 3.6V 电压档，按 32 位字编程
L38  uint32_t badSector = 0xFFFFFFFFu;  // 擦除失败扇区的输出参数
L39  uint32_t primask = __get_PRIMASK();  // 保存当前中断屏蔽状态
L40  __disable_irq();  // 擦除期间关中断，避免长停顿被中断放大
L41  HAL_FLASH_Unlock();  // 先解锁 flash 控制寄存器才能擦
L42  clearErrorFlags();  // 先清错误标志，避免误报失败
L43  HAL_StatusTypeDef st = HAL_FLASHEx_Erase(&init, &badSector);  // 执行扇区擦除，典型耗时 1.1 秒
L44  HAL_FLASH_Lock();  // 擦除完成后立即把 flash 重新上锁
L45  __set_PRIMASK(primask);  // 恢复调用前的中断屏蔽状态
L46  return st == HAL_OK && badSector == 0xFFFFFFFFu;  // HAL 成功且无坏扇区才算擦除成功
L47  }  // 扇区擦除函数到此结束
L49  bool programWord(uint32_t sector, uint32_t wordIndex, uint32_t value) override {  // 按字序号向目标扇区编程一个字
L50  if (sector > 1u) return false;  // 扇区号非法就直接返回失败
L51  volatile uint32_t* addr = reinterpret_cast<volatile uint32_t*>(  // 算出目标字的绝对地址
L52  SECTOR_ADDRESS[sector] + wordIndex * 4u);  // 扇区首地址加字偏移乘 4
L54  if (*addr == value) return true;  // 内容已是期望值就不写，省一次编程停顿
L55  if (*addr != 0xFFFFFFFFu) return false;  // 已被占用则拒绝覆盖，否则会按位与出脏值
L56  uint32_t primask = __get_PRIMASK();  // 保存当前中断屏蔽状态
L57  __disable_irq();  // 单字编程期间也把中断关掉
L58  HAL_FLASH_Unlock();  // 先解锁 flash 控制寄存器才能写
L59  clearErrorFlags();  // 编程前先清掉历史错误标志
L60  HAL_StatusTypeDef st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,  // 以 32 位字为单位编程写入
L61  reinterpret_cast<uint32_t>(addr), value);  // 把要写的地址与数值传给 HAL
L62  HAL_FLASH_Lock();  // 写入完成后即把 flash 重新上锁
L63  __set_PRIMASK(primask);  // 恢复调用前的中断屏蔽状态
L64  return st == HAL_OK && *addr == value;  // HAL 成功且回读一致才算成功
L65  }  // 单字编程函数到此结束
L67  private:  // 以下为内部使用的静态辅助函数
L68  static void clearErrorFlags() {  // 把各类 flash 错误标志位清干净
L69  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |  // 把历史操作常置位的错误标志一起清掉
L70  FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);  // 包括编程对齐错误与并行位错误等
L71  }  // 清错误标志函数到此结束
L72  };  // F4 的 flash 适配端口类定义结束
L73  #endif  // 结束由 STM32F4xx 宏包住的编译块
```
