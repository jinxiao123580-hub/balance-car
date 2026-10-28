#include <Arduino.h>

#include <Wire.h>

#include <SparkFun_BMI270_Arduino_Library.h>

#include <HardwareTimer.h>

#include <math.h>

// MKS TMC2209 V2.0 以独立 STEP/DIR 模式运行；不依赖 PDN_UART 启动。
#define TMC_STANDALONE_STEP_DIR 1
#if !TMC_STANDALONE_STEP_DIR
#include <TMCStepper.h>
#endif

// ============================================================

// STM32F407ZGT6 + BMI270 + DRV8825

// 正式控制版

//

// BMI270：ACC 400Hz + GYRO 400Hz

// 控制循环：固定400Hz

// Kalman：Angle + Gyro Bias

// 新增：基于加速度模长的动态R，降低运动/振动时accPitch权重

// 当前为增强版：小角度速度/位置拉回 + 3.5~6°平滑退外环 + >6°直立PD优先 + 短时0输出保持

// 平衡点：1.0°（balanceAngleOffset，用户实测调1更好）

// 当前：直立PD + 串级速度PI外环 + 位置拉回（转向环关闭）

//

// 已开启串级控制：融合速度作为速度PI反馈，速度PI输出目标倾角，直立PD输出STEP频率。

// 转向PD保留结构，当前参数为0。

// OLED完全不参与运行。

// ============================================================

  
  

// ============================================================

// 固定引脚 —— 不修改

// ============================================================

  

// Motor1

#define MOTOR1_DIR_PIN   PA7

#define MOTOR1_STEP_PIN  PA6      // TIM3_CH1

  

// Motor2

#define MOTOR2_DIR_PIN   PB0

#define MOTOR2_STEP_PIN  PA8      // TIM1_CH1

  

// BMI270 I2C

#define BMI_SCL_PIN      PB8

#define BMI_SDA_PIN      PB9

  

// OLED引脚保留定义，但本版完全不用OLED

#define OLED_SDA_PIN     PF7

#define OLED_SCL_PIN     PF9

#define OLED_VCC_PIN     PC0

#define OLED_GND_PIN     PC2

  

// Encoder1

#define ENCODER1_A_PIN   PA0

#define ENCODER1_B_PIN   PA1

  

// Encoder2

#define ENCODER2_A_PIN   PB6

#define ENCODER2_B_PIN   PB7

  
  

// ============================================================

// 蓝牙串口 —— USART3 (PB10=TX / PB11=RX)

// 模块RXD <- STM32 PB10 (USART3_TX)

// 模块TXD -> STM32 PB11 (USART3_RX)

// 波特率: JDY-23 出厂默认 9600

// ============================================================

  

#define BT_RX_PIN PB11

#define BT_TX_PIN PB10

  

HardwareSerial BTSerial(

    BT_RX_PIN,

    BT_TX_PIN

);

  

const uint32_t BT_BAUD = 9600;

// ============================================================
// TMC2209 双板 UART 配置（独立 STEP/DIR 模式下不编译）
// 后轮TMC (Motor2: PA8 STEP / PB0 DIR) PDN_UART = PA9 (USART1)
// 前轮TMC (Motor1: PA6 STEP / PA7 DIR) PDN_UART = PA2 (USART2)
// 仅配置寄存器(每次上电重写)；STEP/DIR 仍由原逻辑驱动
// ============================================================

#if !TMC_STANDALONE_STEP_DIR
#define R_SENSE 0.11f
#define TMC_GPIO_WRITE_ONLY 1 // 实验模式：无读回，禁止将已发送当作已配置成功
// 前轮当前是否为 TMC2209。false=前轮用 DRV8825(仅 PA6/PA7 脉冲，不配 UART)；
// 验证后轮 TMC spreadCycle 有效后，买新 TMC 换前轮时改回 true 恢复双 TMC 配置。
const bool TMC_FRONT_ACTIVE = false;

// 轻量半双工 Stream：USART 半双工单线 (HDSEL=1)，TMC2209 PDN_UART 用
class HalfDuplexSerial : public Stream {
  protected:
    USART_TypeDef *uart;
    GPIO_TypeDef *gpio;
    uint8_t pinBit;
  public:
    uint32_t rxCount = 0;
    uint32_t txCount = 0;
    uint32_t rxErrors = 0;
    uint8_t lastRx = 0;
    uint8_t txFramePos = 0;
    uint8_t txFrameLen = 0;
    uint32_t bitTicks = 0;
    HalfDuplexSerial(USART_TypeDef *u, GPIO_TypeDef *g, uint8_t b)
        : uart(u), gpio(g), pinBit(b) {}
    void begin(uint32_t baud) {
        rxCount = txCount = rxErrors = 0;
        lastRx = 0;
        txFramePos = txFrameLen = 0;
        // 这里使用真正的 GPIO 位号，不使用 Arduino 板级引脚编号。
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
        (void)RCC->AHB1ENR;
#if TMC_GPIO_WRITE_ONLY
        // USART 半双工空闲时没有把该载板的 17.8 kΩ 下拉拉高。
        // 诊断只写帧改用普通 GPIO，确保空闲和每个停止位均为推挽高电平。
        gpio->BSRR = 1UL << pinBit;
        gpio->MODER = (gpio->MODER & ~(3UL << (2*pinBit))) | (1UL << (2*pinBit));
        gpio->OTYPER &= ~(1UL << pinBit);
        gpio->PUPDR = (gpio->PUPDR & ~(3UL << (2*pinBit))) | (1UL << (2*pinBit));
        gpio->OSPEEDR |= 3UL << (2*pinBit);
        bitTicks = (HAL_RCC_GetHCLKFreq() + baud / 2) / baud;
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        return;
#else
        gpio->MODER   &= ~(3UL << (2*pinBit));
        gpio->MODER   |=  (2UL << (2*pinBit));
        gpio->OTYPER  |=  (1UL << pinBit);       // 单线收发：开漏并在回复时释放总线
        gpio->PUPDR   &= ~(3UL << (2*pinBit));
        gpio->PUPDR   |=  (1UL << (2*pinBit));   // 空闲高电平；长线建议外部上拉
        gpio->OSPEEDR |=  (3UL << (2*pinBit));
        if (pinBit < 8) { gpio->AFR[0] &= ~(0xFUL << (4*pinBit)); gpio->AFR[0] |= (7UL << (4*pinBit)); }
        else         { uint32_t b=pinBit-8; gpio->AFR[1] &= ~(0xFUL << (4*b)); gpio->AFR[1] |= (7UL << (4*b)); }
        if (uart == USART1) RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
        else if (uart == USART2) RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
        uint32_t fck = (uart == USART1) ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
        uart->CR1 = 0;
        uart->CR2 = 0;
        uart->CR3 = USART_CR3_HDSEL;
        uart->BRR = (fck + baud / 2) / baud; // OVER8=0，16 倍过采样
        uart->CR1 = USART_CR1_UE | USART_CR1_RE;
#endif
    }
    int available(void) {
#if TMC_GPIO_WRITE_ONLY
        return 0;
#else
        return (uart->SR & USART_SR_RXNE) ? 1 : 0;
#endif
    }
    int read(void) {
#if TMC_GPIO_WRITE_ONLY
        return -1;
#else
        const uint32_t status = uart->SR;
        if (!(status & USART_SR_RXNE)) return -1;
        rxErrors |= status & (USART_SR_FE | USART_SR_NE | USART_SR_ORE);
        lastRx = (uint8_t)uart->DR;
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
        gpio->BSRR = low;                         // start bit
        edge += bitTicks;
        while ((int32_t)(DWT->CYCCNT - edge) < 0) {}
        for (uint8_t i = 0; i < 8; ++i) {
            gpio->BSRR = (b & (1U << i)) ? high : low;
            edge += bitTicks;
            while ((int32_t)(DWT->CYCCNT - edge) < 0) {}
        }
        gpio->BSRR = high;                        // stop bit and idle
        edge += bitTicks;
        while ((int32_t)(DWT->CYCCNT - edge) < 0) {}
        if (!savedPrimask) __enable_irq();
#else
        // TMCStepper 每字节调用 write()。保持 TE 覆盖整帧，避免字节间总线掉电平。
        // 读帧 4 字节，写帧 8 字节；第 3 字节的 bit7 是写标志。
        if (txFramePos == 0) {
            uart->CR1 &= ~USART_CR1_RE;
            uart->CR1 |= USART_CR1_TE;
        }
        if (txFramePos == 2) txFrameLen = (b & 0x80) ? 8 : 4;
        while (!(uart->SR & USART_SR_TXE)) {}
        uart->DR = b;
        ++txFramePos;
        if (txFrameLen && txFramePos == txFrameLen) {
            while (!(uart->SR & USART_SR_TC)) {}
            uart->CR1 &= ~USART_CR1_TE;          // 回复阶段释放总线
            uart->CR1 |= USART_CR1_RE;
            txFramePos = txFrameLen = 0;
        }
#endif
        txCount++;
        return 1;
    }
    size_t write(const uint8_t *b, size_t n) { for (size_t i=0;i<n;i++) write(b[i]); return n; }
    int peek(void) { return -1; } // 当前仅供 TMCStepper 使用；它不调用 peek()
    void flush(void) {
#if !TMC_GPIO_WRITE_ONLY
        while (!(uart->SR & USART_SR_TC)) {}
#endif
    }
    using Print::write;
};

static HalfDuplexSerial rearTmcUart(USART1, GPIOA, 9);  // 后轮 PA9/USART1
static HalfDuplexSerial frontTmcUart(USART2, GPIOA, 2); // 前轮 PA2/USART2
static TMC2209Stepper rearTmc[4] = {
    {&rearTmcUart, R_SENSE, 0}, {&rearTmcUart, R_SENSE, 1},
    {&rearTmcUart, R_SENSE, 2}, {&rearTmcUart, R_SENSE, 3}
};
static TMC2209Stepper frontTmc[4] = {
    {&frontTmcUart, R_SENSE, 0}, {&frontTmcUart, R_SENSE, 1},
    {&frontTmcUart, R_SENSE, 2}, {&frontTmcUart, R_SENSE, 3}
};

bool configureTmcDriver(TMC2209Stepper &tmc) {
    if (tmc.version() != 0x21) return false;
    const uint8_t ifcntBefore = tmc.IFCNT();
    tmc.begin();               // pdn_disable=1，mstep_reg_select=1
    tmc.toff(3);               // 使能斩波器
    tmc.en_spreadCycle(true);  // 显式关闭 stealthChop，强制 spreadCycle
    tmc.rms_current(1200);    // 以载板 Rsense=0.11 Ω 为前提
    tmc.microsteps(16);       // UART 寄存器优先于 MS1/MS2 微步设置
    const uint8_t ifcntAfter = tmc.IFCNT();
    return (uint8_t)(ifcntAfter - ifcntBefore) >= 5 &&
           tmc.version() == 0x21 && tmc.en_spreadCycle() &&
           tmc.mstep_reg_select() && tmc.toff() == 3 &&
           tmc.microsteps() == 16;
}

bool findAndConfigureTmc(TMC2209Stepper (&candidates)[4], uint8_t (&versions)[4], uint8_t &address) {
    address = 0xFF;
    for (uint8_t i = 0; i < 4; ++i) {
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
    const bool frontOk = TMC_FRONT_ACTIVE
        ? findAndConfigureTmc(frontTmc, frontVersions, frontAddress)
        : true; // 前轮 DRV8825 或前轮不参与 UART 时跳过
    if (!rearOk || !frontOk) {
        BTSerial.print("TMC UART FAIL rear=");
        BTSerial.print(rearOk);
        BTSerial.print(" front=");
        BTSerial.println(frontOk);
        BTSerial.print("TMC DIAG R[0..3]=");
        for (uint8_t i = 0; i < 4; ++i) { BTSerial.print(rearVersions[i], HEX); BTSerial.print(' '); }
        BTSerial.print("F[0..3]=");
        for (uint8_t i = 0; i < 4; ++i) { BTSerial.print(frontVersions[i], HEX); BTSerial.print(' '); }
        BTSerial.println();
        BTSerial.print("TMC RX rear="); BTSerial.print(rearTmcUart.rxCount);
        BTSerial.print(" front="); BTSerial.print(frontTmcUart.rxCount);
        BTSerial.print(" err="); BTSerial.print(rearTmcUart.rxErrors, HEX);
        BTSerial.print('/'); BTSerial.print(frontTmcUart.rxErrors, HEX);
        BTSerial.print(" last="); BTSerial.print(rearTmcUart.lastRx, HEX);
        BTSerial.print('/'); BTSerial.print(frontTmcUart.lastRx, HEX);
        BTSerial.print(" PA9/PA2="); BTSerial.print((GPIOA->IDR >> 9) & 1);
        BTSerial.print('/'); BTSerial.println((GPIOA->IDR >> 2) & 1);
    } else {
        BTSerial.print("TMC UART address rear="); BTSerial.print(rearAddress);
        BTSerial.print(" front="); BTSerial.println(frontAddress);
    }
    return rearOk && frontOk;
}

#if TMC_GPIO_WRITE_ONLY
void configureTmcWriteOnly(TMC2209Stepper (&candidates)[4]) {
    // 四个可能地址逐个写入，实际只有地址匹配的驱动会接收。
    for (auto &tmc : candidates) {
        tmc.toff(0);               // 配置阶段不使能斩波器
        tmc.begin();                // pdn_disable=1, mstep_reg_select=1
        tmc.I_scale_analog(false);  // 运行电流按 UART 寄存器设置
        tmc.en_spreadCycle(true);
        tmc.rms_current(1200);
        tmc.microsteps(16);
        tmc.toff(3);               // 最后才使能斩波器
    }
}

bool initTmcDriversWriteOnly(bool &rearLineHigh, bool &frontLineHigh) {
    rearTmcUart.begin(115200);
    if (TMC_FRONT_ACTIVE) {
        frontTmcUart.begin(115200);
        delayMicroseconds(20);
    } else {
        frontLineHigh = true; // 前轮为 DRV8825，无 PDN_UART，跳过前轮 UART
    }
    rearLineHigh = (GPIOA->IDR & (1UL << 9)) != 0;
    if (TMC_FRONT_ACTIVE) frontLineHigh = (GPIOA->IDR & (1UL << 2)) != 0;
    if (!rearLineHigh || (TMC_FRONT_ACTIVE && !frontLineHigh)) {
        GPIOA->MODER &= ~((3UL << (2*9)) | (3UL << (2*2)));
        return false;
    }
    configureTmcWriteOnly(rearTmc);
    if (TMC_FRONT_ACTIVE) configureTmcWriteOnly(frontTmc);
    return true;
}
#endif
#endif // !TMC_STANDALONE_STEP_DIR

  
  

// ============================================================

// 方向参数

// ============================================================

  

// BMI270相对历史竖装方向绕X轴旋转180°；Y/Z在readBMI270()中同时翻转。

const float GY_DIRECTION = 1.0f;

  

// IMU前向线加速度方向。

// 当前按竖装BMI270历史方向给出初始值。

// 如果手推车辆向前时 imuForwardAccelG 长期显示为负，改成 +1.0f。

const float IMU_FORWARD_ACCEL_DIRECTION = -1.0f;

  

// 转向角速度方向

const float TURN_GYRO_DIRECTION = 1.0f;

  

// 整体平衡方向

int balanceDirection = -1;

  

// 电机安装方向

bool motor1Invert = false;

bool motor2Invert = true;    // MKS TMC2209 V2.0 后轮与前轮方向相反：只翻转后轮 DIR 极性

  

// 编码器方向

int encoder1Direction = 1;

int encoder2Direction = -1;   // V10 实机确认值：编码器2方向反（8/21 实测 enc1/enc2 计数符号相反）

  
  

// ============================================================

// 机械平衡角

// ============================================================

  

float balanceAngleOffset = 4.0f; // 用户面板参数 O：上电默认平衡目标角

  

// ============================================================

// PID参数

// ============================================================

  

// ------------------------------------------------------------

// 直立环 PD

// ------------------------------------------------------------

  

float balanceKp = 1000.0f; // 直立环P：角度误差1°→STEP加速度原始输出

float balanceKd = 100.0f;  // 直立环D：角速度阻尼

// 直立环最终输出限幅 Hz（finalBalanceOutput 钳位）。防大角度时输出冲到±25kHz超高频失步抱死。
// 注意：限幅不能太小（试过±2000/±4000站不住=恢复力被砍），12000 是折中值。
float BALANCE_OUTPUT_LIMIT_HZ = 1750.0f;

// 8/21 输出斜坡限速：每控制周期(2.5ms) finalBalanceOutput 最多变化这么多Hz。
// 修复"运行中突然卡死"(失步)：2000/4000限幅站不住(砍恢复力)，30000等效无限幅能站但
// 大角度瞬间0→25kHz失步卡死。斜坡+12000限幅：保留恢复力且电机加速跟得上。
// 成熟步进平衡结构：直立PD不再直接指定STEP速度，而是每10ms给速度增加一个增量。
// S为100Hz每次允许的最大频率增量；40Hz/update = 4000Hz/s。
float BALANCE_OUTPUT_SLEW_HZ = 100.0f;
float BALANCE_ACCEL_SCALE = 0.10f; // 蓝牙R：直立PD输出到每次STEP频率增量的比例

// 高速驱动保护：只限制"继续加速"，不限制制动/反向所需的增量。
// B=保护启动频率，H=高速继续加速的每10ms最大频率增量。
float HIGH_SPEED_ACCEL_START_HZ = 1000.0f;
float HIGH_SPEED_ACCEL_LIMIT_HZ = 40.0f;
float lastFinalBalanceOutput = 0.0f;

// 换向软过渡：命令方向翻转后 REVERSAL_SOFT_TICKS 个端点周期(10ms)内，
// 把增量限幅压到 REVERSAL_SOFT_LIMIT_SCALE 倍。治 DRV8825 换向失步：
// 换向瞬间转子须越过扭矩零点、1/16 微步增量扭矩仅保持扭矩 9.8%，此时
// 满轨爬升最易丢步（日志证实 9/9 次锁定紧跟换向）。软窗口内爬升变缓，
// 给转子时间脱困。注：201421 验证 S/2+150ms 仍会在软窗口内冲满轨，故调狠。
const uint16_t REVERSAL_SOFT_TICKS = 25;       // 25×10ms = 250ms 软窗口
const float REVERSAL_SOFT_LIMIT_SCALE = 0.25f;  // 窗口内限幅×0.25（S=200→50,H=75→18.75）

// 角度死区：|pitch - targetBalanceAngle| < 该值时，直立环输出0（电机不顶）
// 死区越大越不易震荡，但偏了拉不回来；越小纠偏越积极。当前0.5°：8/21从3°→1.5°→0.5°逐步调小
float BALANCE_ANGLE_DEADBAND_DEG = 0.0f;

  
  

// ------------------------------------------------------------

// 串级速度PI外环

//

// 控制结构：

//

// targetForwardSpeed

//        ↓

// 速度误差 = target - Kalman融合速度

//        ↓

// 速度直投（P×WHEEL_TO_HZ_SCALE + 积分Gain）→ 直接叠加STEP Hz

//        ↓

// 直立PD（400Hz）

//        ↓

// STEP频率

//

// 速度外环每50ms执行一次 = 20Hz。

// （旧"速度PI→目标倾角→改targetBalanceAngle"路径已移除：见 .bak_deadcode_20260902）

// ------------------------------------------------------------

  

  

// 很小的速度误差不积分，避免零附近反复积累

float SPEED_ERROR_DEADBAND = 2.0f;   // 速度误差死区(速度环积分门控) 2026-09-02 可调=J

  

// 目标速度变化限幅。

// 当前targetForwardSpeed通常为0，这个参数主要为后续遥控前进保留。

const float MAX_TARGET_FORWARD_SPEED = 8.0f;

  
  

// ------------------------------------------------------------

// 速度P直接STEP快速阻尼

//

// 用于解决“只改变目标角度，轮速响应滞后明显”的问题。

// 这一路直接叠加到直立环STEP频率输出，但权限受姿态限制。

// ------------------------------------------------------------

  

int speedLoopDirection = -1;   // 2026-09-02 1→-1：架空实测定反。原=轮只往单方向转；翻号后=轮来回正反转（方向盘反）
float speedDirectKp = 0.0f;   // 第一版加速度结构关闭直接STEP阻尼，避免与速度PI互相打架
                               // 比速度PI反应快，抑制"轮子被拽着走"的滞后。过大→高频抖动/啸叫，过小→轮速响应肉
float speedPredictHorizon = 0.0f;        // STEP速度预测时间（蓝牙A），单位s；0=关闭预测
const float SPEED_ACCEL_FILTER_ALPHA = 0.80f; // 20Hz速度差分低通：越大越平滑

// 经典速度PI外环（改目标角）：speedAngleCorrection 叠加到目标平衡角，让车倾斜以移动。
float speedKp = -0.02f;          // 速度PI到目标角的比例增益；实车已确认该号为正确方向
float speedKi = 0.0f;            // 先用纯P验证速度方向；启用后采用条件积分/抗饱和
float speedKd = 0.0f;
float MAX_SPEED_ANGLE = 2.0f;
// 20Hz外环每次更新时，目标倾角修正最多变化的角度。
// 用来防止目标角在 +/-N 之间瞬间跳变。
const float SPEED_ANGLE_SLEW_DEG_PER_UPDATE = 0.10f; // 100Hz外环，等效最大10deg/s
float SPEED_INTEGRAL_LIMIT = 50.0f; // 速度PI积分累积上限（误差×秒，非deg）

// 轮速→STEP Hz 量纲标定放大：rawVehicleSpeed 单位是count/50ms，与直立环STEP Hz差数十倍。
// 直投= Kp*Q * speedError * THIS_SCALE，让 Q(speedDirectKp) 从此真正有发力意义。
const float WHEEL_TO_HZ_SCALE = 4.0f;  // 实测定标：usable speed≈30/50ms@均值、finalHz≈2167。scale=4 → 有效Q≈5*4=20直投均值可达final的~15-25%。过大(20)会爆MAX。

  

// 姿态接近目标时允许速度直接项参与

const float SPEED_DIRECT_FULL_ANGLE = 2.0f;    // <=2°：100%

const float SPEED_DIRECT_DISABLE_ANGLE = 4.0f; // >=4°：0%

  

// 角速度太大时，优先救姿态，关闭速度直接项

const float SPEED_DIRECT_MAX_GYRO_DPS = 80.0f;

  
  

// ------------------------------------------------------------

// 位置快速拉回

//

// 作用：车只要离开开机位置一点点，就立即给出反向STEP修正。

// 这一路直接读取编码器累计位置，400Hz更新，不等待50ms速度窗口。

// ------------------------------------------------------------

  

// 标准串级位置外环：位置误差先生成一个受限目标速度，不直接抢占STEP输出。
float positionKp = 5.0f;                  // 蓝牙X：用户面板默认值
float MAX_POSITION_RETURN_SPEED = 1.0f;   // 蓝牙Y：目标回位速度限幅，count/50ms
float POSITION_DEADBAND_COUNT = 0.0f;     // 蓝牙M：位置死区，平均编码器count

  
  

// ------------------------------------------------------------

// 转向环 PD

// 当前关闭

// ------------------------------------------------------------

  

float turnKp = 0.0f;

float turnKd = 0.0f;

  
  

// ============================================================

// 目标值

// ============================================================

  

float targetForwardSpeed = 0.0f;

float targetTurnRate = 0.0f;

  

// ============================================================

// 目标角阶跃测试

//

// TEST_TARGET_ANGLE_OFFSET:

// 当前关闭阶跃测试：TEST_TARGET_ANGLE_OFFSET = 0.0f

// 速度PI同样关闭。

// 本版只用于观察：

//   1. Kalman pitch

//   2. gyroPitchRate

//

// 用于判断落地运动时：

// 是真实车体仍在持续旋转，还是Kalman角度被动态加速度拉偏。

// ============================================================

  

// 目标角阶跃诊断已关闭，正式外环版本不再使用固定阶跃。

  
  

// ============================================================

// 速度PI控制变量

// ============================================================

  

float speedError = 0.0f;
float speedPContribution = 0.0f;
float speedIContribution = 0.0f;
float speedAngleUnclamped = 0.0f;
float speedAngleLimited = 0.0f;


// 速度直投积分（治"往一边溜"）：对准 speedError 累积、直接叠到 STEP Hz，不进平衡角。
// 只加直投积分不碰速度PI目标角路，前进/转弯(targetForwardSpeed/turnKp)语义保留。
float speedDirectIntegral = 0.0f;
float speedDirectIntegralGain = 0.0f;   // 直投积分增益（蓝牙在线调参：V）。0=关
const float SPEED_DIRECT_INTEGRAL_LIMIT = 40.0f; // 积分累积上限（误差累积值，非STEP Hz）

float speedIntegral = 0.0f;          // 经典速度PI积分累积（纯串级，会进目标角）
float speedAngleCorrection = 0.0f;   // 经典速度PI输出的目标倾角修正（叠加到 targetBalanceAngle）
float speedErrorPrev = 0.0f;         // 速度误差微分阻尼：上一拍误差
float speedErrorDotFilt = 0.0f;      // 速度误差微分阻尼：低通后的误差变化率

  


  
  

// ============================================================

// 电机

// ============================================================

  

const float MOTOR_STOP_THRESHOLD_HZ = 1.0f;

const float PWM_UPDATE_MIN_DELTA_HZ = 5.0f;

  

// ============================================================

// 电机输出保持 / 瞬时故障去抖

// ============================================================

  

const bool HOLD_LAST_OUTPUT_ON_ZERO = false;   // 直立输出为0时：true=短暂保持上次频率（防过零抖动），false=立即停

  

// 仅允许短时间保持0输出。

// 400Hz下3帧约7.5ms，避免过零换向时旧方向保持太久。

const uint8_t ZERO_HOLD_MAX_CYCLES = 3;

uint8_t motor1ZeroHoldCount = 0;

uint8_t motor2ZeroHoldCount = 0;

  

// 400Hz下8帧约20ms

const uint8_t IMU_FAIL_CONFIRM_COUNT = 8;   // IMU读取连续失败8次(约20ms)才确认故障，防偶发去抖

uint8_t imuFailCount = 0;

  

const uint8_t FALL_CONFIRM_COUNT = 8;   // 倒地判定连续8帧(约20ms)确认，防瞬间晃倒误触发

uint8_t fallConfirmCount = 0;

  

// 严重倾倒时立即停机

const float EMERGENCY_FALL_ANGLE_DEG = 75.0f;

  
  

// ============================================================

// 大角度直立优先

//

// 小角度：速度环 + 位置环全部参与

// 中等角度：外环平滑减弱

// 大角度：外环完全退出，只保留直立PD救车

// ============================================================

  

float OUTER_FULL_ANGLE_DEG = 5.0f;    // |pitch-平衡点| <= 此角：外环全权限
float OUTER_DISABLE_ANGLE_DEG = 10.0f; // >=此角：外环退出，只留直立PD

// 直立环输出衰减系数（verticalLoopGain）。range 0~1。
// 现在混合输出 = verticalLoopGain*balanceOutput + outerLoopAuthority*(speedDirect+positionDirect)。
// 用户方案：削弱直立环(如0.7)给速度环让权重，速度环直接叠加到轮速而非改目标角。
float verticalLoopGain = 0.70f;

  

// 大角度时速度积分缓慢释放，避免恢复后旧积分继续推车
const float SPEED_INTEGRAL_DECAY_BIG_ANGLE = 0.95f;


  

// 外环当前权限：0~1

float outerLoopAuthority = 1.0f;

  
  

// ============================================================

// 倒地保护

// ============================================================

  

const float FALL_ANGLE_DEG = 75.0f;   // 倒地判定：|pitch-平衡点|>=75°连续8帧(20ms)触发停机+状态清零

  
  

// ============================================================

// 固定400Hz控制

// ============================================================

  

const uint32_t CONTROL_PERIOD_US = 2500;

const float CONTROL_DT = 0.0025f;

  
  

// ============================================================
// BMI270（竖装：X=上下、Z=前后、Y=俯仰转轴）
// ============================================================
  

TwoWire BMIWire(
    BMI_SDA_PIN,
    BMI_SCL_PIN
);
  

BMI270 imu;

// SDO接地使用主地址0x68；若模块SDO拉高，应改为BMI2_I2C_SEC_ADDR(0x69)。
const uint8_t BMI270_ADDRESS = BMI2_I2C_PRIM_ADDR;
  

// ============================================================
// BMI270输出数据率/滤波配置寄存器
// ============================================================
  

const uint8_t BMI270_REG_ACC_CONF = 0x40;
const uint8_t BMI270_REG_GYR_CONF = 0x42;
const uint8_t BMI270_REG_CMD = 0x7E;
const uint8_t BMI270_SOFT_RESET_CMD = 0xB6;
const uint8_t BMI270_ACC_CONF_400HZ = 0xAA;
const uint8_t BMI270_GYR_CONF_400HZ = 0xAA;
  
  

// ============================================================

// 电机硬件定时器

// ============================================================

  

HardwareTimer motor1Timer(TIM2); // 32-bit STEP edge timer, PA6 unchanged

HardwareTimer motor2Timer(TIM5); // 32-bit STEP edge timer, PA8 unchanged

  

float motor1LastCommandHz = 0.0f;

float motor2LastCommandHz = 0.0f;

  
  

// ============================================================

// 编码器 + 多源速度Kalman

// ============================================================

  

volatile int32_t encoder1Count = 0;

volatile int32_t encoder2Count = 0;

  

int32_t lastEncoder1Count = 0;

int32_t lastEncoder2Count = 0;

  

float encoderSpeed1 = 0.0f;

float encoderSpeed2 = 0.0f;

  

// 编码器直接测得的车辆速度，单位：count / 50ms

float rawVehicleSpeed = 0.0f;

// 编码器信任门状态（见 updateOuterLoop）

bool encoder1Trusted = true;
bool encoder2Trusted = true;
uint16_t encoder1MotionTicks = 0;
uint16_t encoder2MotionTicks = 0;
uint16_t encoderTrustWindowTicks = 0;
uint32_t fallStopCount = 0;    // 倒地保护累计触发次数（遥测第43项，只增不减）
int rawSpeedEncoderMode = 0;   // 0=双路平均 1=仅enc1 2=仅enc2（遥测第40项）
// ---- 250ms 遥测窗口极值保持（穿透 250ms 采样盲区，抓短促尖峰）----
// 用途：判定陀螺/加速度计是真的有尖峰（IMU 故障），还是车体真在剧烈振荡（控制增益问题）。
// 每控制拍（400Hz）在 loop 末尾累积，sendTelemetry() 读取后清零。
float telemGyroAbsMax = 0.0f;        // 窗口内 max|gyroPitchRate| 度/s
float telemAccPitchDevMax = 0.0f;    // 窗口内 max|accPitch - gyroIntegAngle| 度
float telemBalHzAbsMax = 0.0f;       // 窗口内 max|balanceOutput| Hz（量化饱和深度）
float telemKalmanBiasMin = 1e9f;     // 窗口内 kalmanBias 最小/最大（首次发送后由 sendTelemetry 重置为当拍值）
float telemKalmanBiasMax = -1e9f;

// 换向软过渡状态（见 updateBalanceLoop）
int32_t prevIntegratedSpeedCmdSign = 0;   // 上一端点周期 integratedMotorSpeedCommand 符号
uint16_t reversalSoftTicksRemaining = 0;  // 换向后剩余软过渡端点周期数

  

// 多源Kalman最终估计速度，仍统一使用 count / 50ms

float vehicleSpeed = 0.0f;

  

// 车辆相对开机/重新站立位置的累计位移，单位为平均编码器count

float vehiclePosition = 0.0f;

  

// 位置零点对应的两个编码器累计计数

int32_t positionZeroCount1 = 0;

int32_t positionZeroCount2 = 0;

  

// 目标位置固定为0，即尽量回到启动位置

float targetVehiclePosition = 0.0f;

// 位置串级外环诊断量
float positionError = 0.0f;
float positionSpeedTarget = 0.0f;
bool positionHoldWasEnabled = false;
void captureCurrentPositionTarget();

  
  

// ------------------------------------------------------------

// IMU前向加速度

//

// 竖装BMI270：X近似竖直，Z近似前后。

// 利用Pitch去掉Z轴中的重力分量，再作为速度Kalman预测输入。

// ------------------------------------------------------------

  

float imuForwardAccelG = 0.0f;

float imuForwardAccelFilteredG = 0.0f;

  

// 400Hz低通，抑制步进电机高频振动

const float IMU_ACCEL_LPF_ALPHA = 0.05f;

// 编码器速度一阶低通系数（20Hz外环，每50ms一次）。
// encoderSpeed = (1-a)·旧 + a·新。a越小越平滑但响应越慢。
// 默认0.4，兼顾平滑与跟踪。震荡明显时调小(0.2)，响应优先时调大(0.8)。
const float ENCODER_SPEED_LPF_ALPHA = 0.4f;

// --- 编码器信任门（见 updateOuterLoop）---
// updateOuterLoop 由 400Hz 循环每 4 次调用一次 = 100Hz (10ms)。
// 实机 2026-09-17 20:41 日志证实二号编码器会整段无脉冲：66s 里约 50s
// 行程为 0，而同期一号编码器正常，且两台电机收到完全相同的命令。
// 这是该路接线/连接器的间歇故障，不是电机失步。
// 坏通道仍会偶发单个噪声脉冲，所以用滑窗累计“有脉冲”的拍数，
// 不能用连续计数，否则计数永远攒不满。
const uint16_t ENCODER_TRUST_WINDOW_TICKS = 50;  // 500ms 判定窗
const uint16_t ENCODER_TRUST_ACTIVE_TICKS = 5;   // 窗内 >=5 拍有脉冲即视为活跃
const int32_t  ENCODER_TRUST_MIN_COUNTS = 1;     // 单拍 >=1 count 才算“有脉冲”

  

// 防止偶发振动尖峰直接把速度预测带飞

const float IMU_ACCEL_CLAMP_G = 0.60f;

  

// 用于速度预测时的小加速度死区。

// 静止附近的IMU零偏/振动不再持续积分成假速度。

const float IMU_SPEED_ACCEL_DEADBAND_G = 0.03f;

  

// 每个400Hz周期，IMU最多只允许把速度估计改变这么多。

// 防止偶发冲击直接把融合速度带飞。

const float MAX_IMU_SPEED_DELTA_PER_CYCLE = 0.02f;

  
  

// ------------------------------------------------------------

// 速度Kalman参数

//

// 状态：vehicleSpeed

// 预测：IMU线加速度

// 测量：50ms编码器速度

//

// 由于当前没有轮径、编码器每米计数等绝对标定参数，

// IMU_ACCEL_TO_SPEED_GAIN采用保守经验比例。

// 后续只需调这个比例，不影响编码器长期基准。

// ------------------------------------------------------------

  

// 把 g 转换为 “count/50ms 每秒的变化率” 的经验系数

float IMU_ACCEL_TO_SPEED_GAIN = 3.0f;

  

// 基础过程噪声

float SPEED_KALMAN_Q_BASE = 0.20f;

  

// 电机命令越大，说明真实速度越可能快速变化，适当增大Q

float SPEED_KALMAN_Q_MOTOR_GAIN = 0.0008f;

  

// 编码器测量噪声：低速时量化严重，所以R较大

float SPEED_KALMAN_R_LOW_SPEED = 0.20f;

  

// 中高速时编码器连续性更好，R减小

float SPEED_KALMAN_R_NORMAL = 0.08f;

  

// Kalman协方差

float speedKalmanP = 1.0f;

  

// 当前使用的Q/R，仅调试观察

float speedKalmanQ = SPEED_KALMAN_Q_BASE;

float speedKalmanR = SPEED_KALMAN_R_LOW_SPEED;

  

// IMU预测可信度。

// 当 |a|-1g 偏差大时，说明振动/线性冲击严重，降低IMU预测作用。

float imuSpeedPredictionTrust = 1.0f;

  
  
  
  

// ============================================================

// IMU数据

// ============================================================

  

float ax = 0.0f;

float ay = 0.0f;

float az = 0.0f;

  

float gx = 0.0f;

float gy = 0.0f;

float gz = 0.0f;

  
  

// ============================================================

// IMU零位

// ============================================================

  

// 竖装BMI270：X轴沿车体上下、Z轴沿前后，车体竖直时原始Pitch应接近0°。
// 只补偿固定安装误差，不把开机时车辆姿态当作零点。
const float PITCH_MOUNT_OFFSET_DEG = 0.0f;
float pitchZero = PITCH_MOUNT_OFFSET_DEG;

float gyroYZero = 0.0f;

float gyroXZero = 0.0f;

  
  

// ============================================================

// 姿态

// ============================================================

  

float accPitch = 0.0f;

float pitch = 0.0f;

  

float gyroPitchRate = 0.0f;

float gyroTurnRate = 0.0f;

float gyroIntegAngle = 0.0f;   // 真实KF输入角（去掉kalmanBias后的gyro积分），400Hz累计

  
  

// ============================================================

// Kalman参数

// Angle + Gyro Bias

// ============================================================

  

float kalmanAngle = 0.0f;

float compAngle = 0.0f;    // 互补滤波角度状态(取代 kalmanAngle)

// ===== Pitch 毛刺抑制 =====
// 400Hz dt=0.0025：车真实单步角位移(倒摆+电机最剧烈)远小于5°/步，
// 单步突变>5° 判为IMU/卡尔曼毛刺，用上一有效值替代，防止大跳变直接冲进直立PD造成震荡。
const float PITCH_GLITCH_DEG = 5.0f;
float pitchPrev = 0.0f;    // 上一拍有效 pitch

float kalmanBias = 0.0f;

  

float P00 = 1.0f;

float P01 = 0.0f;

float P10 = 0.0f;

float P11 = 1.0f;

  

// 快速响应参数

float Q_angle = 0.01f;

float Q_bias = 0.03f;

// ===== 互补滤波(取代 Kalman 姿态融合) =====
// token: alpha=0.95 陀螺权重; 越接近1越信陀螺(平滑/信静止), 越小越信加速度(响应快)
const float COMP_FILTER_INIT = 0.95f;   // 出厂默认陀螺权重, 蓝牙可调(Y字母)
float COMP_ALPHA = 0.99f;               // 互补滤波权重: angle = a*(angle+gyro*dt)+(1-a)*accPitch

  

// 基础测量噪声：静止/低动态时使用
// 注意：当前姿态Kalman实际使用的是下面的 dynamicR（随加速度模长变化），本变量已不使用，仅保留定义。
float R_measure = 0.03f;

  

// ============================================================

// 动态加速度可信度

//

// 原理：

// 加速度模长接近1g -> 加速度计主要测到重力，可信度高

// 加速度模长偏离1g -> 存在线性加速度/振动，降低加速度角权重

//

// dynamicR 越大，Kalman越不相信accPitch，越依赖陀螺仪预测。

// ============================================================

  

float accelMagnitude = 1.0f;

float accelMagnitudeError = 0.0f;

float dynamicR = 0.03f;

  

// 先用比较保守的一组阈值

const float ACC_ERR_LOW  = 0.04f;

const float ACC_ERR_HIGH = 0.10f;

// 方向分歧（innovation）阈值：|accPitch - gyroIntegAngle| 超过该值判为加速度计毛刺，
// 立即把 dynamicR 拉满、只信陀螺积分。真实运动时 accPitch 与 gyroInt 同步 → 差值小 → 不误伤。
const float ACC_INNOV_HIGH = 20.0f;

  

const float R_LOW_DYNAMIC  = 0.02f;

const float R_MID_DYNAMIC  = 0.20f;   // 已不使用（8/21改连续插值后仅用LOW/HIGH两档），保留定义无害

const float R_HIGH_DYNAMIC = 1.20f;

  
  

// ============================================================

// 控制变量

// ============================================================

  

float targetBalanceAngle = 4.0f;  // 初始值：与 balanceAngleOffset 一致

  

float balanceOutput = 0.0f;
float balancePOutput = 0.0f;
float balanceDOutput = 0.0f;

float turnOutput = 0.0f;

  

// 速度P直接STEP修正

float speedDirectOutput = 0.0f;
float speedDirectUnclampedOutput = 0.0f;
float speedAccelerationFiltered = 0.0f;
float speedPredictionDelta = 0.0f;
float speedPredicted = 0.0f;
float speedPredictPrevSpeed = 0.0f;

  

// 速度直接项权限：= outerLoopAuthority（连续0~1，大角度时降为0退出）

float speedDirectAuthority = 0.0f;
float speedDirectAppliedOutput = 0.0f;

  

// 直立PD + 速度直接STEP阻尼后的最终前后控制量

float finalBalanceOutput = 0.0f;
float mixedBalanceOutput = 0.0f;
float limitedBalanceOutput = 0.0f;
float balanceAccelRawOutput = 0.0f;      // 100Hz本次未限幅STEP频率增量 Hz/update
float balanceAccelLimitedOutput = 0.0f;  // 100Hz加速度限幅后的增量 Hz/update
float balanceAccelEffectiveLimit = 0.0f; // 本拍实际使用的增量限幅 Hz/update
bool highSpeedAccelProtectionActive = false;
float balanceAccelSubstepOutput = 0.0f;  // 把一个100Hz端点增量均分到4个400Hz小步
float integratedMotorSpeedCommand = 0.0f; // 对增量积分得到的STEP目标频率 Hz
uint8_t balanceAccelDivider = 0;
const uint8_t BALANCE_ACCEL_UPDATE_DIVIDER = 4; // 400Hz / 4 = 100Hz

  

float lastTurnError = 0.0f;

void clearControlOutputDiagnostics()
{
    balancePOutput = 0.0f;
    balanceDOutput = 0.0f;
    balanceOutput = 0.0f;
    turnOutput = 0.0f;
    speedDirectOutput = 0.0f;
    speedDirectUnclampedOutput = 0.0f;
    speedAccelerationFiltered = 0.0f;
    speedPredictionDelta = 0.0f;
    speedPredicted = 0.0f;
    speedPredictPrevSpeed = vehicleSpeed;
    speedDirectAppliedOutput = 0.0f;
    speedDirectAuthority = 0.0f;
    positionSpeedTarget = 0.0f;
    positionError = 0.0f;
    mixedBalanceOutput = 0.0f;
    limitedBalanceOutput = 0.0f;
    finalBalanceOutput = 0.0f;
    balanceAccelRawOutput = 0.0f;
    balanceAccelLimitedOutput = 0.0f;
    balanceAccelEffectiveLimit = 0.0f;
    highSpeedAccelProtectionActive = false;
    balanceAccelSubstepOutput = 0.0f;
    integratedMotorSpeedCommand = 0.0f;
    prevIntegratedSpeedCmdSign = 0;
    reversalSoftTicksRemaining = 0;
    balanceAccelDivider = 0;
    lastFinalBalanceOutput = 0.0f;
}

  
  

// ============================================================

// 调度

// ============================================================

  

uint32_t lastControlUs = 0;
uint32_t controlOverrunCount = 0;
uint32_t controlMaxGapUs = 0;
uint32_t controlMaxWorkUs = 0;

uint32_t outerLoopDivider = 0;

  
  

// ============================================================

// 蓝牙遥测

// 控制400Hz，每40个周期发送一次 = 10Hz

// 数据格式包含速度预测链：speedAccel1000、predSpeed1000、predDelta1000、
// directUnclampedHz，便于区分预测、独立H限幅、权限及最终混合输出。
// 后缀100/1000=×100/×1000取整，Hz字段为整数STEP频率。电脑端脚本见 pinghengche_ble_ready/

// ============================================================

  

const uint32_t TELEMETRY_DIVIDER = 100;  // 400Hz/100 = 4Hz；扩展控制链遥测后仍控制在9600波特预算内

uint32_t telemetryDivider = 0;

// ============================================================
// 非阻塞遥测 txBuffer（9600 波特很慢，直接 print 会阻塞 400Hz 控制循环）
// 方案：sendTelemetry 只把整行写入环形缓冲；loop 末尾按 800B/s 预算 flush。
// 预算不足时丢整行（不部分发送）。
// ============================================================

static const uint16_t TX_BUF_SIZE = 4096;

static char txBuffer[TX_BUF_SIZE];

static volatile uint16_t txHead = 0;

static volatile uint16_t txTail = 0;

// 每控制周期(2.5ms)累加的发送预算（字节）
// 800B/s * 0.0025s = 2B/周期
static const float TX_BUDGET_PER_CYCLE = 2.0f;

static float txBudgetBytes = 0.0f;

static const float TX_BUDGET_MAX = 40.0f;   // 预算上限，避免积压暴写

// 写入环形缓冲（整行入队；空间不足则丢弃该行）
static bool txPushLine(const char* line, uint16_t len)
{
    if (len == 0 || len >= TX_BUF_SIZE)
        return false;
    // 检查是否有连续空间放 [len 字节 + 换行]
    uint16_t space;
    if (txHead >= txTail)
        space = (TX_BUF_SIZE - 1) - (txHead - txTail);
    else
        space = (txTail - txHead) - 1;
    if (space < len + 1)
        return false;
    for (uint16_t i = 0; i < len; i++)
    {
        txBuffer[txHead] = line[i];
        txHead = (txHead + 1) % TX_BUF_SIZE;
    }
    txBuffer[txHead] = '\n';
    txHead = (txHead + 1) % TX_BUF_SIZE;
    return true;
}

// loop 末尾按预算 flush（非阻塞：只写 UART 当前可容纳的字节）
void flushTelemetryTx()
{
    static uint32_t lastBudgetUs = micros();
    uint32_t budgetNowUs = micros();
    txBudgetBytes += (uint32_t)(budgetNowUs - lastBudgetUs) * 0.0008f;
    lastBudgetUs = budgetNowUs;
    if (txBudgetBytes > TX_BUDGET_MAX)
        txBudgetBytes = TX_BUDGET_MAX;

    while (txHead != txTail && txBudgetBytes >= 1.0f)
    {
        int avail = BTSerial.availableForWrite();
        if (avail <= 0)
            break;
        int toSend = (int)txBudgetBytes;
        if (toSend > avail)
            toSend = avail;
        if (toSend <= 0)
            break;
        uint16_t n = 0;
        while (n < toSend && txHead != txTail)
        {
            BTSerial.write((uint8_t)txBuffer[txTail]);
            txTail = (txTail + 1) % TX_BUF_SIZE;
            n++;
        }
        txBudgetBytes -= (float)n;
    }
}

// 参数回显前丢弃尚未发送完的旧遥测，并先补换行，防止确认文本插入D行中间。
void prepareCommandResponse() {}
class QueuedBtReply : public Print {
public:
    using Print::write;
    size_t write(uint8_t c) override {
        if (used >= sizeof(buffer) - 1) { overflow = true; return 0; }
        buffer[used++] = (char)c;
        return 1;
    }
    ~QueuedBtReply() { if (!overflow && used) txPushLine(buffer, used); }
private:
    char buffer[768];
    uint16_t used = 0;
    bool overflow = false;
};

  
  

// ============================================================

// 编码器ISR

// ============================================================

  

void encoder1ISR()

{

    if (

        digitalRead(ENCODER1_A_PIN)

        ==

        digitalRead(ENCODER1_B_PIN)

    )

    {

        encoder1Count += encoder1Direction;

    }

    else

    {

        encoder1Count -= encoder1Direction;

    }

}

  
  

void encoder2ISR()

{

    if (

        digitalRead(ENCODER2_A_PIN)

        ==

        digitalRead(ENCODER2_B_PIN)

    )

    {

        encoder2Count += encoder2Direction;

    }

    else

    {

        encoder2Count -= encoder2Direction;

    }

}

  
  

// ============================================================

// BMI270底层寄存器写（仅用于400Hz配置回读）

// ============================================================

  

bool bmiWriteRegister(

    uint8_t reg,

    uint8_t value

)

{

    BMIWire.beginTransmission(

        BMI270_ADDRESS

    );

  

    BMIWire.write(reg);

    BMIWire.write(value);

  

    uint8_t error =

        BMIWire.endTransmission();

  

    delayMicroseconds(500);

  

    return

        error == 0;

}

  
  

// ============================================================

// BMI270底层寄存器读

// ============================================================

  

bool bmiReadRegister(

    uint8_t reg,

    uint8_t &value

)

{

    BMIWire.beginTransmission(

        BMI270_ADDRESS

    );

  

    BMIWire.write(reg);

  

    if (

        BMIWire.endTransmission(false)

        != 0

    )

    {

        return false;

    }

  
  

    uint8_t count =

        BMIWire.requestFrom(

            BMI270_ADDRESS,

            (uint8_t)1

        );

  
  

    if (

        count != 1

        ||

        !BMIWire.available()

    )

    {

        return false;

    }

  
  

    value =

        BMIWire.read();

  
  

    return true;

}

  
  

// ============================================================

// 配置BMI270为400Hz

// ============================================================

  

bool configureBMI270_400Hz()
{
    // 历史实车配置：ODR=400Hz，性能滤波模式。
    if (!bmiWriteRegister(BMI270_REG_ACC_CONF, BMI270_ACC_CONF_400HZ))
    {
        return false;
    }
    delay(5);

    if (!bmiWriteRegister(BMI270_REG_GYR_CONF, BMI270_GYR_CONF_400HZ))
    {
        return false;
    }
    delay(10);

    uint8_t accConf = 0;
    uint8_t gyrConf = 0;
    return bmiReadRegister(BMI270_REG_ACC_CONF, accConf)
        && bmiReadRegister(BMI270_REG_GYR_CONF, gyrConf)
        && accConf == BMI270_ACC_CONF_400HZ
        && gyrConf == BMI270_GYR_CONF_400HZ;
}
  
  

// ============================================================

// BMI270读取；SparkFun库返回g和deg/s，后续融合量纲无需修改

// ============================================================

  

bool readBMI270()
{
    if (imu.getSensorData() != BMI2_OK)
        return false;

    // 当前实装相对历史竖装方向前后反向：绕X(上下)轴旋转180°。
    // 因此X保持，Y/Z同时翻转；这保证加速度角与俯仰角速度符号一致。
    ax = imu.data.accelX;
    ay = -imu.data.accelY;
    az = -imu.data.accelZ;
    gx = imu.data.gyroX;
    gy = -imu.data.gyroY;
    gz = -imu.data.gyroZ;

    return true;
}
  
  

// ============================================================

// BMI270初始化；beginI2C内部完成芯片识别、配置文件装载和传感器使能

// ============================================================

  

bool initBMI270()
{
    // 冷上电时传感器电源可能晚于MCU稳定。初始化阶段先使用100kHz，
    // 完成配置文件装载后，setup()再切换到400kHz。
    delay(1500);

    BMIWire.begin();
    BMIWire.setClock(100000);
    delay(300);

    for (int i = 0; i < 12; i++)
    {
        BMIWire.beginTransmission(BMI270_ADDRESS);
        uint8_t probeError = BMIWire.endTransmission();
        if (probeError != 0)
        {
            delay(500);
            continue;
        }

        // 清除上一次未完成的配置装载状态，使每次重试从确定状态开始。
        bmiWriteRegister(BMI270_REG_CMD, BMI270_SOFT_RESET_CMD);
        delay(50);

        int8_t result = imu.beginI2C(BMI270_ADDRESS, BMIWire);
        if (result == BMI2_OK)
        {
            delay(200);
            if (imu.getSensorData() == BMI2_OK)
                return true;
        }

        BTSerial.print("BMI270 init retry ");
        BTSerial.print(i + 1);
        BTSerial.print(" err=");
        BTSerial.println((int)result);
        delay(500);
    }

    return false;
}
  
  

// ============================================================

// 加速度Pitch

// 竖装BMI270：X = 上下

// Z = 前后；Y = 俯仰转轴

// ============================================================

  

float calculateAccelPitch()

{

    return

        atan2f(

            -az,

            -ax

        )

        *

        180.0f

        /

        PI;

}

  
  

// ============================================================

// IMU校准

// 开机保持车体静止

// ============================================================

  

bool calibrateIMU()
{
    // 开机静止采样只用于估计陀螺仪零偏和初始绝对倾角。
    // calculateAccelPitch() 直接表示重力相对 IMU 轴的倾角；
    // 只减去固定安装偏置，不减去开机时车体姿态。
    const float CAL_STATIC_GYRO_DPS = 4.0f;   // 静止判据：角速度小于此值(°/s)
    const float CAL_GRAV_MAG_TOL = 0.25f;      // 静止判据：|模长-1g| 容差(g)
    const int SAMPLE_TARGET = 400;              // 需累计的有效样本数
    const uint32_t CAL_TIMEOUT_MS = 20000;      // 总等待上限(毫秒) 保护

    delay(300);   // 上电稳定
    float pitchSum = 0.0f, gyroYSum = 0.0f, gyroXSum = 0.0f;
    int valid = 0;
    uint32_t tStart = millis();
    while (valid < SAMPLE_TARGET && (millis() - tStart) < CAL_TIMEOUT_MS)
    {
        if (readBMI270())
        {
            float mag = sqrtf(ax*ax + ay*ay + az*az);
            bool still = (fabsf(gy) < CAL_STATIC_GYRO_DPS)
                      && (fabsf(mag - 1.0f) < CAL_GRAV_MAG_TOL);
            if (still)
            {
                pitchSum += calculateAccelPitch();
                gyroYSum += gy;
                gyroXSum += gx;
                valid++;
            }
        }
        delay(3);
    }
    if (valid < SAMPLE_TARGET / 2)
    {
        return false;
    }
    pitchZero = PITCH_MOUNT_OFFSET_DEG;
    gyroYZero = gyroYSum / (float)valid;   // 陀螺零偏
    gyroXZero = gyroXSum / (float)valid;
    const float initialPitch = pitchSum / (float)valid - pitchZero;
    accPitch  = initialPitch;
    pitch     = initialPitch;
    kalmanAngle = initialPitch;
    kalmanBias  = 0.0f;
    compAngle   = initialPitch;
    pitchPrev   = initialPitch;
    gyroIntegAngle = initialPitch;
    P00 = 1.0f; P01 = 0.0f; P10 = 0.0f; P11 = 1.0f;
    return true;
}

  
  

// ============================================================

// Kalman

// ============================================================

  

float updateKalman(

    float measuredAngle,

    float measuredRate,

    float dt

)

{

    // --------------------------------------------------------

    // 1. 状态预测

    // --------------------------------------------------------

  

    float rate =

        measuredRate

        -

        kalmanBias;

  
  

    kalmanAngle +=

        dt

        *

        rate;

  
  

    // --------------------------------------------------------

    // 2. 协方差预测

    // --------------------------------------------------------

  

    P00 +=

        dt

        *

        (

            dt * P11

            -

            P01

            -

            P10

            +

            Q_angle

        );

  
  

    P01 -=

        dt

        *

        P11;

  
  

    P10 -=

        dt

        *

        P11;

  
  

    P11 +=

        Q_bias

        *

        dt;

  
  

    // --------------------------------------------------------

    // 3. 测量创新

    // --------------------------------------------------------

  

    float innovation =

        measuredAngle

        -

        kalmanAngle;

  
  

    // --------------------------------------------------------

    // 动态测量噪声

    //

    // 车辆静止/低动态时 dynamicR ≈ 0.03

    // 车辆加速或振动明显时 dynamicR 增大，

    // 从而暂时减少加速度角对Kalman的修正。

    // --------------------------------------------------------

  

    float S =

        P00

        +

        dynamicR;

  
  

    // --------------------------------------------------------

    // 4. Kalman增益

    // --------------------------------------------------------

  

    float K0 =

        P00

        /

        S;

  
  

    float K1 =

        P10

        /

        S;

  
  

    // --------------------------------------------------------

    // 5. 状态修正

    // --------------------------------------------------------

  

    kalmanAngle +=

        K0

        *

        innovation;

  
  

    kalmanBias +=

        K1

        *

        innovation;

  
  

    // --------------------------------------------------------

    // 6. 协方差修正

    // --------------------------------------------------------

  

    float oldP00 = P00;

    float oldP01 = P01;

  
  

    P00 -=

        K0

        *

        oldP00;

  
  

    P01 -=

        K0

        *

        oldP01;

  
  

    P10 -=

        K1

        *

        oldP00;

  
  

    P11 -=

        K1

        *

        oldP01;

  
  

    return

        kalmanAngle;

}

// ============================================================
// 互补滤波(取代 Kalman)： 
//   angle = COMP_ALPHA*(angle + gyroRate*dt) + (1-COMP_ALPHA)*accPitch
//   响应快、参数少，不会像 Kalman 高Q/高R 那样“太信静止/滤过头”。
//   COMP_ALPHA 越接近1 越信陀螺积分(平滑)，越接近0 越信加速度(跟得快)。
// ============================================================

float updateComplementary(

    float measuredAngle,

    float measuredRate,

    float dt

)

{

    compAngle =

        COMP_ALPHA

        *

        (compAngle + measuredRate * dt)

        +

        (1.0f - COMP_ALPHA)

        *

        measuredAngle;

    return

        compAngle;

}

  
  

// ============================================================

// 姿态更新

// ============================================================

  

void updateAttitude()

{

    // --------------------------------------------------------

    // 1. 原始加速度角

    // --------------------------------------------------------

  

    accPitch =

        calculateAccelPitch()

        -

        pitchZero;

  
  

    // --------------------------------------------------------

    // 2. Pitch轴陀螺仪角速度

    // --------------------------------------------------------

  

    gyroPitchRate =

        GY_DIRECTION

        *

        (

            gy

            -

            gyroYZero

        );

  
  

    // --------------------------------------------------------

    // 3. 计算三轴加速度模长

    //

    // SparkFun BMI270库已将加速度换算为g。

    // 静止时理论上接近1g。

    // --------------------------------------------------------

  

    accelMagnitude =

        sqrtf(

            ax * ax

            +

            ay * ay

            +

            az * az

        );

  
  

    accelMagnitudeError =

        fabsf(

            accelMagnitude

            -

            1.0f

        );

  
  

    // --------------------------------------------------------

    // 4. 根据 |a|-1g 动态调整R

    //

    // 越偏离1g，说明线性加速度/振动越明显，

    // 这时accPitch可信度降低。

    // --------------------------------------------------------

  

    // 连续插值：加速度模长误差 0 -> ACC_ERR_HIGH 平滑映射 R
    // 静止/低速：R小(0.02)，acc权重高，抑制陀螺零偏积分漂移
    // 高速/振动：R大(1.20)，acc权重低，防止线性加速度污染角度
    // 双判据：(1) 加速度模长误差（振动/线性加速度）(2) 方向分歧 innovation（毛刺）
    // 真实运动：accPitch 与 gyroInt 同步变化 → innovRatio 小 → 不影响
    // accPitch 单拍毛刺：gyroInt 不动 → innovRatio 大 → dynamicR 拉满 → 只信陀螺
    float errRatio = accelMagnitudeError / ACC_ERR_HIGH;
    if (errRatio > 1.0f) { errRatio = 1.0f; }
    float innovRatio = fabsf(accPitch - gyroIntegAngle) / ACC_INNOV_HIGH;
    if (innovRatio > 1.0f) { innovRatio = 1.0f; }
    float combinedRatio = (errRatio > innovRatio) ? errRatio : innovRatio;
    dynamicR = R_LOW_DYNAMIC + (R_HIGH_DYNAMIC - R_LOW_DYNAMIC) * combinedRatio;

  
  

    // --------------------------------------------------------

    // 5. Kalman融合

    // --------------------------------------------------------

  

    pitch =

        updateKalman(

            accPitch,

            gyroPitchRate,

            CONTROL_DT

        );

    // ---- Pitch 毛刺抑制：单步突变>±5° 判为IMU/卡尔曼毛刺，用上一有效值替代 ----
    // 用步长差而非绝对限幅：车正常还要走大角度(±10°+)，绝对限幅会砍正常反馈；
    // 而 400Hz 下单步>5° 是倒摆+电机也到不了的物理上限，只会掐真毛刺，不动正常响应。
    {
        float dPitch = pitch - pitchPrev;
        if (dPitch > PITCH_GLITCH_DEG) pitch = pitchPrev;
        else if (dPitch < -PITCH_GLITCH_DEG) pitch = pitchPrev;
        else pitchPrev = pitch;
    }

    // 真实KF输入角 = gyroPitchRate 去掉 kalmanBias 后在 400Hz 积分。
    // 只发这个积分角，电脑端不再做 5Hz 快照积分，根治面板紫线漂移。
    gyroIntegAngle += (gyroPitchRate - kalmanBias) * CONTROL_DT;

  
  

    // --------------------------------------------------------

    // 6. 根据Pitch去除Z轴重力分量，得到前向线加速度趋势

    //

    // 静止时：

    // az ≈ -sin(pitch)

    //

    // 因此：

    // az + sin(pitch) ≈ 0

    //

    // 再乘方向参数统一为“向前加速为正”。

    // --------------------------------------------------------

  

    float pitchRad =

        pitch

        *

        DEG_TO_RAD;

  
  

    imuForwardAccelG =

        IMU_FORWARD_ACCEL_DIRECTION

        *

        (

            az

            +

            sinf(pitchRad)

        );

  
  

    // 尖峰限幅

    if (

        imuForwardAccelG

        >

        IMU_ACCEL_CLAMP_G

    )

    {

        imuForwardAccelG =

            IMU_ACCEL_CLAMP_G;

    }

  
  

    if (

        imuForwardAccelG

        <

        -IMU_ACCEL_CLAMP_G

    )

    {

        imuForwardAccelG =

            -IMU_ACCEL_CLAMP_G;

    }

  
  

    // 轻度低通

    imuForwardAccelFilteredG =

        (1.0f - IMU_ACCEL_LPF_ALPHA)

        *

        imuForwardAccelFilteredG

        +

        IMU_ACCEL_LPF_ALPHA

        *

        imuForwardAccelG;

  
  

    // 根据加速度模长偏离1g的程度降低IMU预测可信度

    if (

        accelMagnitudeError

        <

        ACC_ERR_LOW

    )

    {

        imuSpeedPredictionTrust =

            1.0f;

    }

    else if (

        accelMagnitudeError

        <

        ACC_ERR_HIGH

    )

    {

        imuSpeedPredictionTrust =

            0.45f;

    }

    else

    {

        imuSpeedPredictionTrust =

            0.12f;

    }

  
  

    // --------------------------------------------------------

    // 7. 转向角速度

    // --------------------------------------------------------

  

    gyroTurnRate =

        TURN_GYRO_DIRECTION

        *

        (

            gx

            -

            gyroXZero

        );

}

  
  

// ============================================================

// 编码器初始化

// ============================================================

  

void initEncoders()

{

    pinMode(

        ENCODER1_A_PIN,

        INPUT_PULLUP

    );

  

    pinMode(

        ENCODER1_B_PIN,

        INPUT_PULLUP

    );

  

    pinMode(

        ENCODER2_A_PIN,

        INPUT_PULLUP

    );

  

    pinMode(

        ENCODER2_B_PIN,

        INPUT_PULLUP

    );

  
  

    attachInterrupt(

        digitalPinToInterrupt(

            ENCODER1_A_PIN

        ),

        encoder1ISR,

        CHANGE

    );

  
  

    attachInterrupt(

        digitalPinToInterrupt(

            ENCODER2_A_PIN

        ),

        encoder2ISR,

        CHANGE

    );

}

  
  

// ============================================================

// 电机初始化

// ============================================================

  

// 1MHz 32-bit timers schedule GPIO STEP edges; no PWM preload latency.
struct StepOutput {
    HardwareTimer* timer;
    uint32_t mask;
    volatile uint32_t periodTicks = 1000000;
    volatile uint32_t risingEdges = 0;
    volatile bool running = false;
    volatile bool high = false;
    bool dir = false;
    StepOutput(HardwareTimer* t, uint32_t m) : timer(t), mask(m) {}
};
StepOutput step1(&motor1Timer, 1UL << 6);
StepOutput step2(&motor2Timer, 1UL << 8);
const uint32_t STEP_HIGH_US = 4;
const uint32_t DIR_SETUP_US = 4;
const float MAX_STEP_HZ = 20000.0f;
void stepEdge(StepOutput& out) {
    TIM_TypeDef* tim = out.timer->getHandle()->Instance;
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
void motor1EdgeISR() { stepEdge(step1); }
void motor2EdgeISR() { stepEdge(step2); }
void stopStepOutput(StepOutput& out) {
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
void initMotors() {
    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    pinMode(MOTOR1_STEP_PIN, OUTPUT);
    pinMode(MOTOR2_STEP_PIN, OUTPUT);
    digitalWrite(MOTOR1_DIR_PIN, LOW);
    digitalWrite(MOTOR2_DIR_PIN, LOW);
    GPIOA->BSRR = (step1.mask | step2.mask) << 16;
    StepOutput* outputs[] = {&step1, &step2};
    for (StepOutput* out : outputs) {
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
void commandStepOutput(StepOutput& out, uint32_t dirPin, bool invert, float commandHz) {
    if (!isfinite(commandHz) || fabsf(commandHz) < MOTOR_STOP_THRESHOLD_HZ) {
        stopStepOutput(out);
        return;
    }
    bool dir = (commandHz >= 0.0f) != invert;
    uint32_t period = (uint32_t)(1000000.0f / fminf(fabsf(commandHz), MAX_STEP_HZ) + 0.5f);
    if (!out.running || dir != out.dir) {
        stopStepOutput(out);
        delayMicroseconds(DIR_SETUP_US);
        digitalWrite(dirPin, dir ? HIGH : LOW);
        out.dir = dir;
        delayMicroseconds(DIR_SETUP_US);
        uint32_t saved = __get_PRIMASK();
        __disable_irq();
        out.periodTicks = period;
        TIM_TypeDef* tim = out.timer->getHandle()->Instance;
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
    TIM_TypeDef* tim = out.timer->getHandle()->Instance;
    uint32_t oldPeriod = out.periodTicks;
    out.periodTicks = period;
    if (!out.high && period != oldPeriod) {
        // Preserve fractional progress while retiming the LOW interval.
        tim->CR1 &= ~TIM_CR1_CEN;
        uint32_t oldLow = oldPeriod - STEP_HIGH_US;
        uint32_t newLow = period - STEP_HIGH_US;
        uint32_t elapsed = tim->CNT;
        if (elapsed >= oldLow) elapsed = oldLow - 1;
        uint32_t nextCount = (uint32_t)((uint64_t)elapsed * newLow / oldLow);
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

  

void stopMotor1() { stopStepOutput(step1); motor1LastCommandHz = 0.0f; }

  
  

void stopMotor2() { stopStepOutput(step2); motor2LastCommandHz = 0.0f; }

  
  

void stopBothMotors()

{

    stopMotor1();

    stopMotor2();

}

  
  

// ============================================================

// Motor1

// ============================================================

  

void setMotor1(float commandHz) {
    commandStepOutput(step1, MOTOR1_DIR_PIN, motor1Invert, commandHz);
    motor1LastCommandHz = step1.running ? copysignf(fminf(fabsf(commandHz), MAX_STEP_HZ), commandHz) : 0.0f;
}

  
  

// ============================================================

// Motor2

// ============================================================

  

void setMotor2(float commandHz) {
    commandStepOutput(step2, MOTOR2_DIR_PIN, motor2Invert, commandHz);
    motor2LastCommandHz = step2.running ? copysignf(fminf(fabsf(commandHz), MAX_STEP_HZ), commandHz) : 0.0f;
}

  
  

// ============================================================

// 多源速度Kalman

//

// 400Hz：用IMU前向线加速度做预测

// 20Hz ：用编码器速度做测量修正

//

// 电机命令不直接当“真实速度”，只用来动态调整过程噪声Q：

// 控制命令大 -> 系统更可能快速改变速度 -> Kalman更允许状态变化。

// ============================================================

  

void predictVehicleSpeedKalman()

{

    // --------------------------------------------------------

    // 1. IMU只负责“短期趋势预测”

    //

    // 编码器负责长期速度绝对值；

    // IMU不允许长期主导vehicleSpeed。

    // --------------------------------------------------------

  

    float accelForSpeed =

        imuForwardAccelFilteredG;

  
  

    // 小加速度死区：

    // 防止0.01~0.02g级别的零偏不断积分成假速度。

    if (

        fabsf(accelForSpeed)

        <

        IMU_SPEED_ACCEL_DEADBAND_G

    )

    {

        accelForSpeed =

            0.0f;

    }

  
  

    float imuDeltaSpeed =

        accelForSpeed

        *

        imuSpeedPredictionTrust

        *

        IMU_ACCEL_TO_SPEED_GAIN

        *

        CONTROL_DT;

  
  

    // 单周期预测限幅

    if (

        imuDeltaSpeed

        >

        MAX_IMU_SPEED_DELTA_PER_CYCLE

    )

    {

        imuDeltaSpeed =

            MAX_IMU_SPEED_DELTA_PER_CYCLE;

    }

  
  

    if (

        imuDeltaSpeed

        <

        -MAX_IMU_SPEED_DELTA_PER_CYCLE

    )

    {

        imuDeltaSpeed =

            -MAX_IMU_SPEED_DELTA_PER_CYCLE;

    }

  
  

    vehicleSpeed +=

        imuDeltaSpeed;

  
  

    // --------------------------------------------------------

    // 2. 动态过程噪声Q

    //

    // 电机命令越大，系统越可能快速改变速度，

    // 因此允许Kalman状态更快变化。

    //

    // 注意：balanceOutput只调Q，不直接当作真实速度。

    // --------------------------------------------------------

  

    speedKalmanQ =

        SPEED_KALMAN_Q_BASE

        +

        SPEED_KALMAN_Q_MOTOR_GAIN

        *

        fabsf(balanceOutput);

  
  

    speedKalmanP +=

        speedKalmanQ

        *

        CONTROL_DT;

}

  
  

void correctVehicleSpeedKalman(

    float encoderMeasurement

)

{

    // --------------------------------------------------------

    // 编码器作为“速度绝对值主测量”

    //

    // 低速时虽然存在0/1 count量化，

    // 但R不再设得过大，避免Kalman长期跟着IMU预测跑偏。

    // --------------------------------------------------------

  

    if (

        fabsf(encoderMeasurement)

        <

        2.0f

    )

    {

        speedKalmanR =

            SPEED_KALMAN_R_LOW_SPEED;

    }

    else

    {

        speedKalmanR =

            SPEED_KALMAN_R_NORMAL;

    }

  
  

    float innovation =

        encoderMeasurement

        -

        vehicleSpeed;

  
  

    float innovationCovariance =

        speedKalmanP

        +

        speedKalmanR;

  
  

    float K =

        speedKalmanP

        /

        innovationCovariance;

  
  

    vehicleSpeed +=

        K

        *

        innovation;

  
  

    speedKalmanP =

        (1.0f - K)

        *

        speedKalmanP;

  
  

    // 防止P数值过小导致后续测量修正越来越迟钝

    if (

        speedKalmanP

        <

        0.02f

    )

    {

        speedKalmanP =

            0.02f;

    }

}

  
  

// ============================================================

// 漂移拉回外环

// 成熟步进平衡车串级外环：100Hz速度PI -> 小目标倾角。
// 编码器增量换算为既有count/50ms量纲，保持参数和遥测单位不变。

// ============================================================

  

void updateOuterLoop()

{

    int32_t count1;

    int32_t count2;

  

    noInterrupts();

  

    count1 =

        encoder1Count;

  

    count2 =

        encoder2Count;

  

    interrupts();

  
  

    // --------------------------------------------------------

    // 1. 10ms编码器增量，乘5换算为count/50ms

    // --------------------------------------------------------

  

    int32_t delta1 =

        count1

        -

        lastEncoder1Count;

  
  

    int32_t delta2 =

        count2

        -

        lastEncoder2Count;

  
  

    lastEncoder1Count =

        count1;

  

    lastEncoder2Count =

        count2;

  
  

    encoderSpeed1 =

        (1.0f - ENCODER_SPEED_LPF_ALPHA) * encoderSpeed1 + ENCODER_SPEED_LPF_ALPHA * ((float)delta1 * 5.0f);

  
  

    encoderSpeed2 =

        (1.0f - ENCODER_SPEED_LPF_ALPHA) * encoderSpeed2 + ENCODER_SPEED_LPF_ALPHA * ((float)delta2 * 5.0f);

    // --------------------------------------------------------
    // 1b. 编码器信任门
    //
    // 整段剔除不可信的一路，而不是把下面的 -0.5f 硬改成 -1.0f。
    // 后者等价于“永远假设 enc2 = 0”，会在这路恢复正常时把车速
    // 读成真实值的 2 倍，比原来的固定偏低更危险。
    // --------------------------------------------------------

    if (delta1 >= ENCODER_TRUST_MIN_COUNTS || delta1 <= -ENCODER_TRUST_MIN_COUNTS)
        encoder1MotionTicks++;

    if (delta2 >= ENCODER_TRUST_MIN_COUNTS || delta2 <= -ENCODER_TRUST_MIN_COUNTS)
        encoder2MotionTicks++;

    encoderTrustWindowTicks++;

    if (encoderTrustWindowTicks >= ENCODER_TRUST_WINDOW_TICKS)
    {
        const bool encoder1Active = (encoder1MotionTicks >= ENCODER_TRUST_ACTIVE_TICKS);

        const bool encoder2Active = (encoder2MotionTicks >= ENCODER_TRUST_ACTIVE_TICKS);

        // 车本身静止时两路都不活跃，不能据此改判，保持上一次结论。

        if (encoder1Active || encoder2Active)
        {
            encoder1Trusted = encoder1Active;

            encoder2Trusted = encoder2Active;
        }

        encoder1MotionTicks = 0;

        encoder2MotionTicks = 0;

        encoderTrustWindowTicks = 0;
    }

    rawSpeedEncoderMode =
        (encoder1Trusted && encoder2Trusted) ? 0 : (encoder1Trusted ? 1 : 2);

  
  

    // --------------------------------------------------------

    // 2. 编码器车辆速度

    //

    // 当前沿用此前实测约定：

    // 向前 = 正

    // 向后 = 负

    // --------------------------------------------------------

  

    if (encoder1Trusted && encoder2Trusted)

        rawVehicleSpeed = -0.5f * (encoderSpeed1 + encoderSpeed2);

    else if (encoder1Trusted)

        rawVehicleSpeed = -encoderSpeed1;

    else

        rawVehicleSpeed = -encoderSpeed2;

  
  

    // --------------------------------------------------------

    // 3. 用编码器修正速度Kalman

    //

    // 此时vehicleSpeed已经由：

    // 400Hz IMU预测 + 20Hz编码器修正

    // 得到融合速度。

    // --------------------------------------------------------

  

    correctVehicleSpeedKalman(

        rawVehicleSpeed

    );

  
  

    // --------------------------------------------------------
    // 4. 标准串级位置P外环：位置误差 -> 受限目标速度
    // --------------------------------------------------------

    bool positionHoldEnabled = positionKp > 0.000001f;
    if (positionHoldEnabled && !positionHoldWasEnabled)
    {
        captureCurrentPositionTarget();
        positionError = 0.0f;
    }
    positionHoldWasEnabled = positionHoldEnabled;

    if (positionHoldEnabled)
    {
        float effectivePositionError = positionError;
        if (effectivePositionError > POSITION_DEADBAND_COUNT)
            effectivePositionError -= POSITION_DEADBAND_COUNT;
        else if (effectivePositionError < -POSITION_DEADBAND_COUNT)
            effectivePositionError += POSITION_DEADBAND_COUNT;
        else
            effectivePositionError = 0.0f;

        positionSpeedTarget = positionKp * effectivePositionError;
        if (positionSpeedTarget > MAX_POSITION_RETURN_SPEED)
            positionSpeedTarget = MAX_POSITION_RETURN_SPEED;
        else if (positionSpeedTarget < -MAX_POSITION_RETURN_SPEED)
            positionSpeedTarget = -MAX_POSITION_RETURN_SPEED;
        targetForwardSpeed = positionSpeedTarget;
    }
    else
    {
        positionSpeedTarget = 0.0f;
        targetForwardSpeed = 0.0f;
    }

    // --------------------------------------------------------
    // 5. 总目标速度安全限幅
    // --------------------------------------------------------

  

    if (

        targetForwardSpeed

        >

        MAX_TARGET_FORWARD_SPEED

    )

    {

        targetForwardSpeed =

            MAX_TARGET_FORWARD_SPEED;

    }

  
  

    if (

        targetForwardSpeed

        <

        -MAX_TARGET_FORWARD_SPEED

    )

    {

        targetForwardSpeed =

            -MAX_TARGET_FORWARD_SPEED;

    }

  
  

    // --------------------------------------------------------

    // 6. 速度误差

    // --------------------------------------------------------

  

    speedError =

        targetForwardSpeed

        -

        vehicleSpeed;

  
  

    // --------------------------------------------------------

    // 7. 速度PI生成目标角。第一版严格采用成熟串级结构，

    //

    // 不使用速度预测、STEP直投或位置回位。

    // --------------------------------------------------------

  

    // 经典速度PI目标角通道：100Hz更新。
    float speedErrDot = (speedError - speedErrorPrev) * 100.0f;
    speedErrorDotFilt = 0.7f * speedErrorDotFilt + 0.3f * speedErrDot;   // 低通滤噪声
    speedErrorPrev = speedError;
    speedPContribution = speedKp * speedError;
    float speedDContribution = speedKd * speedErrorDotFilt;

    // 条件积分 + 抗饱和：禁止在外环无权限或输出继续撞向N限幅时累积。
    if (fabsf(speedKi) < 0.000001f)
    {
        speedIntegral = 0.0f;
    }
    else if (outerLoopAuthority >= 0.2f && fabsf(speedError) > SPEED_ERROR_DEADBAND)
    {
        float candidateIntegral = speedIntegral + speedError * 0.01f;
        if (candidateIntegral > SPEED_INTEGRAL_LIMIT) candidateIntegral = SPEED_INTEGRAL_LIMIT;
        else if (candidateIntegral < -SPEED_INTEGRAL_LIMIT) candidateIntegral = -SPEED_INTEGRAL_LIMIT;

        float currentUnclamped = speedLoopDirection *
            (speedPContribution + speedKi * speedIntegral + speedDContribution);
        float candidateUnclamped = speedLoopDirection *
            (speedPContribution + speedKi * candidateIntegral + speedDContribution);

        if (fabsf(candidateUnclamped) <= MAX_SPEED_ANGLE ||
            fabsf(candidateUnclamped) < fabsf(currentUnclamped))
        {
            speedIntegral = candidateIntegral;
        }
    }

    speedIContribution = speedKi * speedIntegral;
    speedAngleUnclamped = speedLoopDirection *
        (speedPContribution + speedIContribution + speedDContribution);
    float desiredSpeedAngle = speedAngleUnclamped;

    // 旧预测和STEP直投实验通道完全退出控制，仅保留零值遥测以兼容历史CSV。
    speedDirectUnclampedOutput = 0.0f;
    speedDirectOutput = 0.0f;
    speedAccelerationFiltered = 0.0f;
    speedPredictionDelta = 0.0f;
    speedPredicted = vehicleSpeed;
    speedPredictPrevSpeed = vehicleSpeed;

  
  

    if (desiredSpeedAngle > MAX_SPEED_ANGLE)
            desiredSpeedAngle = MAX_SPEED_ANGLE;

  
  

    if (desiredSpeedAngle < -MAX_SPEED_ANGLE)
            desiredSpeedAngle = -MAX_SPEED_ANGLE;
    speedAngleLimited = desiredSpeedAngle;

    // 速度环全部关闭时立即清零历史修正，保证纯直立环测试不被残留目标角污染。
    if (fabsf(speedKp) < 0.000001f && fabsf(speedKi) < 0.000001f && fabsf(speedKd) < 0.000001f)
    {
        speedAngleCorrection = 0.0f;
        speedIntegral = 0.0f;
        speedPContribution = 0.0f;
        speedIContribution = 0.0f;
        speedAngleUnclamped = 0.0f;
        speedAngleLimited = 0.0f;
        speedDirectIntegral = 0.0f;
        speedErrorDotFilt = 0.0f;
        speedErrorPrev = speedError;
        speedDirectOutput = 0.0f;
        speedDirectUnclampedOutput = 0.0f;
        speedAccelerationFiltered = 0.0f;
        speedPredictionDelta = 0.0f;
        speedPredicted = vehicleSpeed;
        speedPredictPrevSpeed = vehicleSpeed;
        speedDirectAuthority = 0.0f;
        positionSpeedTarget = 0.0f;
        targetForwardSpeed = 0.0f;
    }
    else
    {
        float speedAngleDelta = desiredSpeedAngle - speedAngleCorrection;

        if (speedAngleDelta > SPEED_ANGLE_SLEW_DEG_PER_UPDATE)
            speedAngleDelta = SPEED_ANGLE_SLEW_DEG_PER_UPDATE;
        else if (speedAngleDelta < -SPEED_ANGLE_SLEW_DEG_PER_UPDATE)
            speedAngleDelta = -SPEED_ANGLE_SLEW_DEG_PER_UPDATE;

        speedAngleCorrection += speedAngleDelta;
    }

  
  

    // --------------------------------------------------------

}

  
  

// ============================================================

// 外环权限管理

//

// 使用相对机械平衡角，而不是相对 targetBalanceAngle。

// 这样速度PI改变目标角时，不会影响“大角度判定”。

// ============================================================

  

void updateOuterLoopAuthority()

{

    float uprightError =

        pitch

        -

        balanceAngleOffset;

  

    float absUprightError =

        fabsf(

            uprightError

        );

  
  

    if (

        absUprightError

        <=

        OUTER_FULL_ANGLE_DEG

    )

    {

        outerLoopAuthority =

            1.0f;

    }

    else if (

        absUprightError

        <

        OUTER_DISABLE_ANGLE_DEG

    )

    {

        outerLoopAuthority =

            (

                OUTER_DISABLE_ANGLE_DEG

                -

                absUprightError

            )

            /

            (

                OUTER_DISABLE_ANGLE_DEG

                -

                OUTER_FULL_ANGLE_DEG

            );

    }

    else

    {

        outerLoopAuthority =

            0.0f;

    }

  
  

    // 目标角必须受同一个权限控制。

    // 大角度时直接回到机械平衡角，让直立PD全力救车。

    // 用户方案：速度环不再改目标角（只直接叠加轮速 speedDirect），

    // 因此这里目标角固定 = 机械平衡角；速度PI把 speedAngleCorrection 叠加到目标角，让车倾斜以移动。

    targetBalanceAngle =

        balanceAngleOffset

        +

        outerLoopAuthority

        *

        speedAngleCorrection;

  
  

    // 大角度时释放历史积分，避免恢复后旧积分继续把车推走

    if (

        outerLoopAuthority

        <

        0.2f

    )

    {

        speedIntegral *= SPEED_INTEGRAL_DECAY_BIG_ANGLE;

    }

}

  
  

// ============================================================

// 位置快速拉回

//

// 400Hz直接读取编码器累计位置。

// 不经过50ms速度窗口，所以只要编码器出现少量位移就开始反向拉回。

// ============================================================

  

void updatePositionHold()

{

    int32_t count1;

    int32_t count2;

  

    noInterrupts();

  

    count1 =

        encoder1Count;

  

    count2 =

        encoder2Count;

  

    interrupts();

  
  

    // 与 rawVehicleSpeed 使用同一正负约定：

    // 向前 = 正，向后 = 负

    vehiclePosition =

        -0.5f

        *

        (

            (float)(

                count1

                -

                positionZeroCount1

            )

            +

            (float)(

                count2

                -

                positionZeroCount2

            )

        );

  
  

    positionError =

        targetVehiclePosition

        -

        vehiclePosition;

  
  

}

// 把当前位置设为新的驻车位置。用于启用位置环、跌倒恢复和面板手动重设。
void captureCurrentPositionTarget()
{
    int32_t count1;
    int32_t count2;
    noInterrupts();
    count1 = encoder1Count;
    count2 = encoder2Count;
    positionZeroCount1 = count1;
    positionZeroCount2 = count2;
    interrupts();

    vehiclePosition = 0.0f;
    targetVehiclePosition = 0.0f;
    positionError = 0.0f;
    positionSpeedTarget = 0.0f;
    targetForwardSpeed = 0.0f;
    speedIntegral = 0.0f;
    speedErrorPrev = 0.0f;
    speedErrorDotFilt = 0.0f;
    speedAccelerationFiltered = 0.0f;
    speedPredictionDelta = 0.0f;
    speedPredicted = vehicleSpeed;
    speedPredictPrevSpeed = vehicleSpeed;
}

  
  

// ============================================================

// 直立环 PD

// ============================================================

  

void updateBalanceLoop()

{

    float angleError =

        targetBalanceAngle

        -

        pitch;

  
  

    // --------------------------------------------------------

    // 1. 直立PD

    // --------------------------------------------------------

    // 连续角度死区：死区外扣除死区宽度，避免 P 项在边界从 0 突跳。
    // 死区内只将 P 项误差置零，保留陀螺仪 D 阻尼来抑制车体晃动。
    float effectiveAngleError = 0.0f;

    if (angleError > BALANCE_ANGLE_DEADBAND_DEG)

    {

        effectiveAngleError = angleError - BALANCE_ANGLE_DEADBAND_DEG;

    }

    else if (angleError < -BALANCE_ANGLE_DEADBAND_DEG)

    {

        effectiveAngleError = angleError + BALANCE_ANGLE_DEADBAND_DEG;

    }

    balancePOutput = balanceDirection * balanceKp * effectiveAngleError;
    balanceDOutput = balanceDirection * (-balanceKd * gyroPitchRate);
    balanceOutput = balancePOutput + balanceDOutput;

  
  

    // --------------------------------------------------------

    // 2. 直接速度/位置修正权限

    //

    // 与目标角使用同一个 outerLoopAuthority。

    // 大角度时速度环和位置环全部退出。

    // --------------------------------------------------------

  

    speedDirectAuthority = 0.0f;

  
  

    // --------------------------------------------------------
    // 3. 步进电机加速度式直立控制
    //
    // B-ROBOT类成熟结构：直立PD给出速度增量，累加后才得到STEP频率。
    // 100Hz计算下一个频率端点；端点增量在随后的4个400Hz周期线性执行。
    // 这样R/S/B/H/L和每10ms累计变化量均保持不变，只消除频率阶跃。
    // --------------------------------------------------------

    balanceAccelDivider++;
    if (balanceAccelDivider >= BALANCE_ACCEL_UPDATE_DIVIDER)
    {
        balanceAccelDivider = 0;
        balanceAccelRawOutput = BALANCE_ACCEL_SCALE * balanceOutput;

        // 低速时保留 S 的完整纠偏能力；只有已经处于高速、且本拍仍在
        // 朝同一方向继续加速时，才收紧增量，避免STEP频率再陡升而丢步。
        // 反向增量是制动，始终使用 S，不能被这层保护削弱。
        balanceAccelEffectiveLimit = BALANCE_OUTPUT_SLEW_HZ;
        highSpeedAccelProtectionActive = false;
        const bool acceleratingFurther =
            integratedMotorSpeedCommand * balanceAccelRawOutput > 0.0f;
        if (fabsf(integratedMotorSpeedCommand) >= HIGH_SPEED_ACCEL_START_HZ &&
            acceleratingFurther)
        {
            balanceAccelEffectiveLimit = HIGH_SPEED_ACCEL_LIMIT_HZ;
            if (balanceAccelEffectiveLimit > BALANCE_OUTPUT_SLEW_HZ)
                balanceAccelEffectiveLimit = BALANCE_OUTPUT_SLEW_HZ;
            highSpeedAccelProtectionActive = true;
        }

        // 换向软过渡生效：翻转后的窗口内把增量限幅再压小一档
        if (reversalSoftTicksRemaining > 0)
        {
            reversalSoftTicksRemaining--;
            balanceAccelEffectiveLimit *= REVERSAL_SOFT_LIMIT_SCALE;
        }

        balanceAccelLimitedOutput = balanceAccelRawOutput;

        if (balanceAccelLimitedOutput > balanceAccelEffectiveLimit)
            balanceAccelLimitedOutput = balanceAccelEffectiveLimit;
        else if (balanceAccelLimitedOutput < -balanceAccelEffectiveLimit)
            balanceAccelLimitedOutput = -balanceAccelEffectiveLimit;

        balanceAccelSubstepOutput =
            balanceAccelLimitedOutput / (float)BALANCE_ACCEL_UPDATE_DIVIDER;
    }

    // 最小平滑层：每2.5ms推进原100Hz端点增量的1/4。
    // 不重新计算PD，不改变端点、总斜率或任何现有调参量。
    float stepIncrement = balanceAccelSubstepOutput;
    float candidateSpeed = integratedMotorSpeedCommand + stepIncrement;
    int nextSign = candidateSpeed > 1.0f ? 1 : (candidateSpeed < -1.0f ? -1 : 0);
    if (nextSign != 0 && prevIntegratedSpeedCmdSign != 0 &&
        nextSign != prevIntegratedSpeedCmdSign) {
        if (reversalSoftTicksRemaining == 0) {
            balanceAccelSubstepOutput *= REVERSAL_SOFT_LIMIT_SCALE;
            balanceAccelLimitedOutput *= REVERSAL_SOFT_LIMIT_SCALE;
            balanceAccelEffectiveLimit *= REVERSAL_SOFT_LIMIT_SCALE;
            stepIncrement = balanceAccelSubstepOutput;
        }
        reversalSoftTicksRemaining = REVERSAL_SOFT_TICKS;
    }
    integratedMotorSpeedCommand += stepIncrement;
    int actualSign = integratedMotorSpeedCommand > 1.0f ? 1 :
                     (integratedMotorSpeedCommand < -1.0f ? -1 : 0);
    if (actualSign != 0) prevIntegratedSpeedCmdSign = actualSign;
    if (integratedMotorSpeedCommand > BALANCE_OUTPUT_LIMIT_HZ)
        integratedMotorSpeedCommand = BALANCE_OUTPUT_LIMIT_HZ;
    else if (integratedMotorSpeedCommand < -BALANCE_OUTPUT_LIMIT_HZ)
        integratedMotorSpeedCommand = -BALANCE_OUTPUT_LIMIT_HZ;

    speedDirectAppliedOutput = 0.0f;
    mixedBalanceOutput = integratedMotorSpeedCommand;

    if (mixedBalanceOutput > BALANCE_OUTPUT_LIMIT_HZ)
        limitedBalanceOutput = BALANCE_OUTPUT_LIMIT_HZ;
    else if (mixedBalanceOutput < -BALANCE_OUTPUT_LIMIT_HZ)
        limitedBalanceOutput = -BALANCE_OUTPUT_LIMIT_HZ;
    else
        limitedBalanceOutput = mixedBalanceOutput;

    // 频率已经由100Hz加速度积分平滑变化，不再对结果做第二次slew。
    finalBalanceOutput = limitedBalanceOutput;
    lastFinalBalanceOutput = finalBalanceOutput;

}

  
  

// ============================================================

// 转向环 PD

//

// 当前Kp/Kd = 0

// ============================================================

  

void updateTurnLoop()

{

    float turnError =

        targetTurnRate

        -

        gyroTurnRate;

  
  

    float turnDerivative =

        (

            turnError

            -

            lastTurnError

        )

        /

        CONTROL_DT;

  
  

    lastTurnError =

        turnError;

  
  

    turnOutput =

        turnKp

        *

        turnError

        +

        turnKd

        *

        turnDerivative;

}

  
  

// ============================================================

// 蓝牙发送

// 网页原来的解析格式无需修改：

// time_ms,accPitch,kalmanPitch

// ============================================================

  

void sendTelemetry()
{
    // 完整控制链：直立P/D、目标角通道、直接制动原始/权限后、
    // 混合、总限幅、slew最终值、转向及左右电机命令。
    char line[640];   // 48字段：理论最坏 48*12+13=589 < 640，实测约 375
    int n = snprintf(line, sizeof(line),
        "D,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lu,%d,%d,%d,%d,%d",
        (unsigned long)millis(),
        (int)(pitch * 100.0f),                 // 1  Kalman倾角 *100
        (int)(gyroPitchRate * 100.0f),         // 2  角速度 *100
        (int)(targetBalanceAngle * 100.0f),    // 3  目标角 *100
        (int)balancePOutput,                   // 4  直立P项 Hz
        (int)balanceDOutput,                   // 5  直立D项 Hz
        (int)balanceOutput,                    // 6  直立PD合计 Hz
        (int)(speedAngleCorrection * 100.0f),  // 7  速度目标角修正 *100
        (int)(outerLoopAuthority * 1000.0f),   // 8  外环权限 *1000
        (int)speedDirectOutput,                // 9  直接制动独立限幅后、权限前 Hz
        (int)speedDirectAppliedOutput,         // 10 权限后的直接制动 Hz
        (int)(positionSpeedTarget * 1000.0f),  // 11 位置环生成的目标速度 *1000
        (int)mixedBalanceOutput,               // 12 直立+直接制动，限幅前 Hz
        (int)limitedBalanceOutput,             // 13 总输出限幅后、slew前 Hz
        (int)finalBalanceOutput,               // 14 slew后最终前后输出 Hz
        (int)turnOutput,                       // 15 转向环输出 Hz
        (int)(vehicleSpeed * 1000.0f),         // 16 融合速度 *1000
        (int)(rawVehicleSpeed * 1000.0f),      // 17 编码器原始速度 *1000
        (int)(speedAccelerationFiltered * 1000.0f), // 18 滤波速度加速度 *1000
        (int)(speedPredicted * 1000.0f),       // 19 预测速度 *1000
        (int)(speedPredictionDelta * 1000.0f), // 20 预测增量 *1000
        (int)speedDirectUnclampedOutput,       // 21 STEP独立H限幅前 Hz
        (int)(vehiclePosition * 100.0f),       // 22 位置 *100
        (int)(positionError * 100.0f),         // 23 位置误差 *100
        (long)encoder1Count,                   // 24 编码器1计数
        (long)encoder2Count,                   // 25 编码器2计数
        (int)motor1LastCommandHz,              // 26 电机1实际命令 Hz
        (int)motor2LastCommandHz,              // 27 电机2实际命令 Hz
        (int)(accPitch * 100.0f),              // 28 加速度计角度 *100
        (int)(kalmanBias * 1000.0f),           // 29 Kalman偏置 *1000
        (int)(gyroIntegAngle * 100.0f),        // 30 陀螺积分角 *100
        (int)balanceAccelRawOutput,            // 31 直立PD换算的原始频率增量 Hz/update
        (int)balanceAccelLimitedOutput,        // 32 S限幅后的频率增量 Hz/update
        (int)integratedMotorSpeedCommand,      // 33 增量积分后的STEP频率 Hz
        (int)(speedError * 1000.0f),           // 34 速度误差 *1000
        (int)(speedPContribution * 100.0f),    // 35 速度P项角度贡献 *100
        (int)(speedIContribution * 100.0f),    // 36 速度I项角度贡献 *100
        (int)(speedAngleUnclamped * 100.0f),   // 37 N限幅前速度目标角 *100
        (int)(speedAngleLimited * 100.0f),     // 38 N限幅后、slew前目标角 *100
        (int)balanceAccelEffectiveLimit,       // 39 本拍实际STEP频率增量限幅 Hz/update
        (int)rawSpeedEncoderMode,              // 40 编码器来源 0=双路平均 1=仅enc1 2=仅enc2
        (int)(dynamicR * 1000.0f),             // 41 dynamicR*1000（0.02~1.20，越大越不信accPitch）
        (int)(accelMagnitudeError * 1000.0f),  // 42 |加速度模长-1g|*1000（动态R的驱动量，判据是否选错看这里）
        (unsigned long)fallStopCount,          // 43 倒地保护累计触发次数（抓250ms采样漏掉的短促误触发）
        (int)(telemGyroAbsMax * 100.0f),       // 44 窗口内max|gyro| *100  ← 判别陀螺是否有尖峰
        (int)(telemAccPitchDevMax * 100.0f),   // 45 窗口内max|accPitch-gyroIntegAngle| *100
        (int)telemBalHzAbsMax,                 // 46 窗口内max|balanceOutput| Hz（饱和深度）
        (int)(telemKalmanBiasMin * 1000.0f),   // 47 窗口内 kalmanBias 最小值 *1000
        (int)(telemKalmanBiasMax * 1000.0f));  // 48 窗口内 kalmanBias 最大值 *1000
    if (n > 0)
        txPushLine(line, (uint16_t)n);

    // 窗口极值清零，开始累积下一个 250ms 窗口（必须在上面取值之后）
    telemGyroAbsMax = 0.0f;
    telemAccPitchDevMax = 0.0f;
    telemBalHzAbsMax = 0.0f;
    telemKalmanBiasMin = kalmanBias;
    telemKalmanBiasMax = kalmanBias;
}

  
  

// ============================================================

// setup

// ============================================================

  

void setup()

{

    initMotors();

  

    initEncoders();

  

    // --------------------------------------------------------

    // YahBoom_BL UART5

    // --------------------------------------------------------

  

    BTSerial.begin(

        BT_BAUD

    );

  

    delay(100);

#if TMC_STANDALONE_STEP_DIR
    // STEP/DIR 由原有 TIM1/TIM3 和 DIR 逻辑驱动，不访问 PA9/PA2。
    // MKS 的 EN(低有效)、VREF 电流、MS1/MS2 细分和 SPREAD 模式由硬件决定。
    BTSerial.println("TMC STEP/DIR standalone: UART disabled; mode/current/microsteps hardware-selected");
#elif TMC_GPIO_WRITE_ONLY
    stopBothMotors();
    bool rearLineHigh = false, frontLineHigh = false;
    while (!initTmcDriversWriteOnly(rearLineHigh, frontLineHigh)) {
        stopBothMotors();
        BTSerial.print("TMC WRITE-ONLY WAIT rearLineHigh="); BTSerial.print(rearLineHigh);
        BTSerial.print(" frontLineHigh="); BTSerial.println(frontLineHigh);
        delay(1000);
    }
    BTSerial.print("TMC WRITE-ONLY SENT rearBytes="); BTSerial.print(rearTmcUart.txCount);
    BTSerial.print(" frontBytes="); BTSerial.print(frontTmcUart.txCount);
    BTSerial.println(" spreadCycle=REQUESTED current=1200mA microsteps=16 UNVERIFIED");
#else
    // TMC2209 spreadCycle 配置(后轮)+PA9, 前轮+PA2；每次上电重写
    while (!initTmcDrivers()) {
        stopBothMotors();
        delay(1000);
    }
    BTSerial.println("TMC UART OK: both spreadCycle, 1/16 step");
#endif

  
  

    // --------------------------------------------------------

    // BMI270初始化

    // --------------------------------------------------------

  

    BTSerial.print("PARAMS: compiled defaults");
    BTSerial.println();

    while (!initBMI270())
    {
        stopBothMotors();
        clearControlOutputDiagnostics();
        BTSerial.println("BOOT RETRY: BMI270 init failed");
        delay(1000);
    }

    BMIWire.setClock(400000);
    delay(20);

  
  

    // --------------------------------------------------------

    // 配置400Hz

    // --------------------------------------------------------

  

    while (!configureBMI270_400Hz())
    {
        stopBothMotors();
        clearControlOutputDiagnostics();
        BTSerial.println("BOOT RETRY: BMI270 400Hz config failed");
        delay(500);
    }

  
  

    // --------------------------------------------------------

    // 校准

    // --------------------------------------------------------

  

    while (!calibrateIMU())
    {
        stopBothMotors();
        clearControlOutputDiagnostics();
        BTSerial.println("BOOT RETRY: IMU calibration failed; keep car still");
        delay(1000);
    }

    BTSerial.println("BOOT OK: BMI270 ready");

  
  

    // --------------------------------------------------------

    // 编码器初值

    // --------------------------------------------------------

  

    noInterrupts();

  

    lastEncoder1Count =

        encoder1Count;

  

    lastEncoder2Count =

        encoder2Count;

  

    positionZeroCount1 =

        encoder1Count;

  

    positionZeroCount2 =

        encoder2Count;

  

    interrupts();

  

    vehiclePosition =

        0.0f;

  

    targetVehiclePosition =

        0.0f;

  

    positionError = 0.0f;
    positionSpeedTarget = 0.0f;
    positionHoldWasEnabled = false;

  
  

    // --------------------------------------------------------

    // 速度Kalman + 串级速度PI初始化

    // --------------------------------------------------------

  

    vehicleSpeed =

        0.0f;

  

    rawVehicleSpeed =

        0.0f;

  

    imuForwardAccelG =

        0.0f;

  

    imuForwardAccelFilteredG =

        0.0f;

  

    imuSpeedPredictionTrust =

        1.0f;

  

    speedKalmanP =

        1.0f;

  

    speedKalmanQ =

        SPEED_KALMAN_Q_BASE;

  

    speedKalmanR =

        SPEED_KALMAN_R_LOW_SPEED;

  

    speedError =

        0.0f;

  



  

    speedDirectOutput =

        0.0f;

    speedDirectIntegral =

        0.0f;

  

    speedDirectAuthority =

        0.0f;

  

    finalBalanceOutput =

        0.0f;

  

    targetForwardSpeed =

        0.0f;

  
  

    targetBalanceAngle =

        balanceAngleOffset;

  

    outerLoopDivider =

        0;

  

    telemetryDivider =

        0;

  

    imuFailCount =

        0;

  

    fallConfirmCount =

        0;

  

    motor1ZeroHoldCount =

        0;

  

    motor2ZeroHoldCount =

        0;

  

    outerLoopAuthority =

        1.0f;

  

    // 网页会忽略这一行表头

    BTSerial.println(

        "time_ms,fused_speed,speed_direct_hz"

    );

  
  

    lastControlUs =

        micros();

}

  
  

// ============================================================

// loop

// 固定400Hz

// ============================================================

  

// ============================================================
// 蓝牙在线调参（在 loop 每圈开头调用，不影响400Hz控制时序）
// 命令格式：一行一个，回车发送。 字母 + 空格 + 数值
//   P <kp>    改 balanceKp
//   D <kd>    改 balanceKd
//   O <offset> 改 balanceAngleOffset（平衡点）
//   Z <deg>   改 BALANCE_ANGLE_DEADBAND_DEG（死区）
//   L <hz>    改 BALANCE_OUTPUT_LIMIT_HZ（输出限幅）
//   S <hz>    改 BALANCE_OUTPUT_SLEW_HZ（每周期最大变化）
//   ?         回显当前全部参数
//   !         回显 TMC 只写请求状态（不是寄存器读回）
// ============================================================

static char btCmdBuf[48];
static uint8_t btCmdLen = 0;
static bool btCmdOverflow = false;

void handleBtCommand(char* cmd, uint8_t len);   // 前置声明

void pollBtCommand()
{
    uint8_t readBudget = 32;
    while (readBudget-- && BTSerial.available() > 0) {
        char c = (char)BTSerial.read();
        if (c == '\n' || c == '\r') {
            if (!btCmdOverflow) handleBtCommand(btCmdBuf, btCmdLen);
            btCmdOverflow = false;
            btCmdLen = 0;
        } else if (btCmdLen < sizeof(btCmdBuf) - 1) {
            btCmdBuf[btCmdLen++] = c;
        } else {
            btCmdOverflow = true;
        }
    }
}

void handleBtCommand(char* cmd, uint8_t len)
{
    QueuedBtReply reply;
    // 跳过前导空格
    while (len > 0 && (*cmd == ' ' || *cmd == '\t')) { cmd++; len--; }
    if (len == 0) return;
    char key = *cmd; cmd++; len--;
    while (len > 0 && (*cmd == ' ' || *cmd == '\t')) { cmd++; len--; }
    uint8_t vlen = len;
    while (vlen > 0 && (cmd[vlen-1] == ' ' || cmd[vlen-1] == '\r' || cmd[vlen-1] == '\n')) vlen--;
    if (key == '?') {
        prepareCommandResponse();
        reply.print("PAR: ");
        reply.print("P="); reply.print(balanceKp, 2);
        reply.print(" D="); reply.print(balanceKd, 2);
        reply.print(" O="); reply.print(balanceAngleOffset, 2);
        reply.print(" Z="); reply.print(BALANCE_ANGLE_DEADBAND_DEG, 2);
        reply.print(" L="); reply.print(BALANCE_OUTPUT_LIMIT_HZ, 1);
        reply.print(" S="); reply.print(BALANCE_OUTPUT_SLEW_HZ, 1);
        reply.print(" R="); reply.print(BALANCE_ACCEL_SCALE, 4);
        reply.print(" F="); reply.print(OUTER_FULL_ANGLE_DEG, 2);
        reply.print(" G="); reply.print(OUTER_DISABLE_ANGLE_DEG, 2);
        reply.print(" T="); reply.print(turnKp, 2);
        reply.print(" U="); reply.print(turnKd, 2);
        reply.print(" K="); reply.print(speedKp, 2);
        reply.print(" I="); reply.print(speedKi, 2);
        reply.print(" N="); reply.print(MAX_SPEED_ANGLE, 2);
        reply.print(" W="); reply.print(SPEED_INTEGRAL_LIMIT, 1);
        reply.print(" J="); reply.print(SPEED_ERROR_DEADBAND, 2);
        reply.print(" E="); reply.print(speedKd, 3);
        reply.print(" Q="); reply.print(speedDirectKp, 3);
        reply.print(" A="); reply.print(speedPredictHorizon, 3);
        reply.print(" B="); reply.print(HIGH_SPEED_ACCEL_START_HZ, 1);
        reply.print(" H="); reply.print(HIGH_SPEED_ACCEL_LIMIT_HZ, 1);
        reply.print(" X="); reply.print(positionKp, 4);
        reply.print(" Y="); reply.print(MAX_POSITION_RETURN_SPEED, 2);
        reply.print(" M="); reply.print(POSITION_DEADBAND_COUNT, 2);
        reply.println();
        return;
    }

    if (key == '!') {
        prepareCommandResponse();
#if TMC_STANDALONE_STEP_DIR
        reply.println("TMC STEP/DIR standalone: hardware-selected mode");
        reply.print("STEP edges="); reply.print(step1.risingEdges);
        reply.print('/'); reply.print(step2.risingEdges);
        reply.print(" periodUs="); reply.print(step1.periodTicks);
        reply.print('/'); reply.print(step2.periodTicks);
        reply.print(" overrun="); reply.print(controlOverrunCount);
        reply.print(" maxGapUs="); reply.print(controlMaxGapUs);
        reply.print(" maxWorkUs="); reply.println(controlMaxWorkUs);
        reply.print("encoderMode="); reply.print(rawSpeedEncoderMode);
        reply.print(" softTicks="); reply.println(reversalSoftTicksRemaining);
#elif TMC_GPIO_WRITE_ONLY
        reply.print("TMC WRITE-ONLY SENT rearBytes="); reply.print(rearTmcUart.txCount);
        reply.print(" frontBytes="); reply.print(frontTmcUart.txCount);
        reply.println(" spreadCycle=REQUESTED microsteps=16 UNVERIFIED");
#else
        reply.println("TMC READBACK MODE");
#endif
        return;
    }

    if (vlen == 0) return;
    char vbuf[32];
    if (vlen >= sizeof(vbuf)) { reply.println("ERR:value too long"); return; }
    for (uint8_t i = 0; i < vlen; i++) vbuf[i] = cmd[i];
    vbuf[vlen] = '\0';

    // 值必须为数字，否则忽略（防 "SAVE"/"DEF" 等非数字串被误当 key+val=0 误设）
    {
        char c0 = vbuf[0];
        if (!((c0>='0'&&c0<='9')||c0=='-'||c0=='.')) return;
    }

    char* valueEnd = nullptr;
    float val = strtof(vbuf, &valueEnd);
    if (valueEnd == vbuf || *valueEnd != '\0' || !isfinite(val)) {
        reply.println("ERR:invalid value");
        return;
    }
    switch (key) {
        case 'P': case 'p': balanceKp = val; break;
        case 'D': case 'd': balanceKd = val; break;
        case 'O': case 'o': balanceAngleOffset = val; break;
        case 'Z': case 'z': BALANCE_ANGLE_DEADBAND_DEG = val; break;
        case 'L': case 'l': BALANCE_OUTPUT_LIMIT_HZ = fabsf(val); break;
        case 'S': case 's': BALANCE_OUTPUT_SLEW_HZ = fabsf(val); break;
        case 'R': case 'r': BALANCE_ACCEL_SCALE = fabsf(val); break;
        case 'F': case 'f': OUTER_FULL_ANGLE_DEG = val; break;
        case 'G': case 'g': OUTER_DISABLE_ANGLE_DEG = val; break;
        case 'T': case 't': turnKp = val; break;
        case 'U': case 'u': turnKd = val; break;
        case 'K': case 'k': speedKp = val; break;
        case 'I': case 'i': speedKi = val; break;
        case 'N': case 'n': MAX_SPEED_ANGLE = val; break;
        case 'W': case 'w': SPEED_INTEGRAL_LIMIT = val; break;
        case 'J': case 'j': SPEED_ERROR_DEADBAND = val; break;
        case 'E': case 'e': speedKd = val; break;
        case 'Q': case 'q': speedDirectKp = fabsf(val); break;
        case 'H': case 'h': HIGH_SPEED_ACCEL_LIMIT_HZ = fabsf(val); break;
        case 'A': case 'a': speedPredictHorizon = fabsf(val); break;
        case 'B': case 'b': HIGH_SPEED_ACCEL_START_HZ = fabsf(val); break;
        case 'X': case 'x': {
            bool wasEnabled = positionKp > 0.000001f;
            positionKp = fabsf(val);
            bool nowEnabled = positionKp > 0.000001f;
            if (!wasEnabled && nowEnabled) captureCurrentPositionTarget();
            if (!nowEnabled) {
                positionSpeedTarget = 0.0f;
                targetForwardSpeed = 0.0f;
            }
            positionHoldWasEnabled = nowEnabled;
            break;
        }
        case 'Y': case 'y': MAX_POSITION_RETURN_SPEED = fabsf(val); break;
        case 'M': case 'm': POSITION_DEADBAND_COUNT = fabsf(val); break;
        case 'C': case 'c': captureCurrentPositionTarget(); break;
        default:
            prepareCommandResponse();
            reply.println("ERR:unknown cmd");
            return;
    }
    prepareCommandResponse();
    reply.print(key);
    reply.print("=");
    reply.print(val, 3);
    reply.println(" OK");
}

void loop()

{

    uint32_t nowUs =

        micros();

  

    // 蓝牙在线调参：每圈都轮询，独立于400Hz时序

    pollBtCommand();
    flushTelemetryTx();
    nowUs = micros();

  
  

    // --------------------------------------------------------

    // 没到2.5ms就直接返回

    // --------------------------------------------------------

  

    if (

        (uint32_t)(

            nowUs

            -

            lastControlUs

        )

        <

        CONTROL_PERIOD_US

    )

    {

        return;

    }

  
  

    // --------------------------------------------------------

    // 保持固定时间基准

    // --------------------------------------------------------

  

    uint32_t controlGapUs = nowUs - lastControlUs;
    if (controlGapUs > controlMaxGapUs) controlMaxGapUs = controlGapUs;
    if (controlGapUs >= CONTROL_PERIOD_US * 2UL) {
        controlOverrunCount++;
        lastControlUs = nowUs;
        stopBothMotors();
        clearControlOutputDiagnostics();
        outerLoopDivider = 0;
    } else {
        lastControlUs += CONTROL_PERIOD_US;
    }
    uint32_t controlWorkStartUs = micros();

  
  

    // --------------------------------------------------------

    // 读取BMI270

    // --------------------------------------------------------

  

    if (

        !readBMI270()

    )

    {

        imuFailCount++;

  

        if (

            imuFailCount

            <

            IMU_FAIL_CONFIRM_COUNT

        )

        {

            // 保持上一有效STEP输出

            return;

        }

  

        stopBothMotors();
        clearControlOutputDiagnostics();

  

        balanceOutput =

            0.0f;

  

        finalBalanceOutput =

            0.0f;

  

        speedDirectOutput =

            0.0f;

        speedDirectIntegral =

            0.0f;

  

        speedDirectAuthority =

            0.0f;

        speedIntegral = 0.0f;
        speedAngleCorrection = 0.0f;
        speedErrorPrev = 0.0f;
        speedErrorDotFilt = 0.0f;
        positionSpeedTarget = 0.0f;
        targetForwardSpeed = 0.0f;

  

        return;

    }

  

    imuFailCount =

        0;

  
  

    // --------------------------------------------------------

    // Kalman姿态

    // --------------------------------------------------------

  

    updateAttitude();

  
  

    // --------------------------------------------------------

    // 多源速度Kalman预测 400Hz

    //

    // IMU负责快速感知“开始加速”的趋势；

    // 编码器会在外环20Hz时进行绝对速度修正。

    // --------------------------------------------------------

  

    predictVehicleSpeedKalman();

  
  

    // --------------------------------------------------------

    // 蓝牙遥测20Hz

    // 放在倒地保护之前，这样即使触发倒地保护，

    // 手机上仍能继续看到IMU/Kalman数据。

    // --------------------------------------------------------

  

    telemetryDivider++;

  

    if (

        telemetryDivider

        >=

        TELEMETRY_DIVIDER

    )

    {

        telemetryDivider =

            0;

  

        sendTelemetry();

    }

  
  

    // --------------------------------------------------------

    // 倒地保护（连续确认，避免单帧尖峰突然停机）

    // --------------------------------------------------------

  

    float fallAngleError =

        fabsf(

            pitch

            -

            balanceAngleOffset

        );

  

    if (

        fallAngleError

        >=

        FALL_ANGLE_DEG

    )

    {

        fallConfirmCount++;

    }

    else

    {

        fallConfirmCount =

            0;

    }

  

    bool confirmedFall =

        fallConfirmCount

        >=

        FALL_CONFIRM_COUNT;

  

    bool emergencyFall =

        fallAngleError

        >=

        EMERGENCY_FALL_ANGLE_DEG;

  

    if (

        confirmedFall

        ||

        emergencyFall

    )

    {

        stopBothMotors();
        clearControlOutputDiagnostics();
        fallStopCount++;   // 倒地保护触发计数：250ms遥测窗无法分辨20ms确认窗内的短促误触发

  

        balanceOutput =

            0.0f;

  

        turnOutput =

            0.0f;

  

        speedError =

            0.0f;

  



        speedDirectOutput =

            0.0f;

        speedDirectIntegral =

            0.0f;

  

        speedDirectAuthority =

            0.0f;

        speedIntegral = 0.0f;
        speedAngleCorrection = 0.0f;
        speedErrorPrev = 0.0f;
        speedErrorDotFilt = 0.0f;

  

        finalBalanceOutput =

            0.0f;

  

        outerLoopAuthority =

            0.0f;

  

        motor1ZeroHoldCount =

            0;

  

        motor2ZeroHoldCount =

            0;

  

        targetForwardSpeed =

            0.0f;

  

        noInterrupts();

  

        int32_t resetCount1 =

            encoder1Count;

  

        int32_t resetCount2 =

            encoder2Count;

  

        interrupts();

  

        lastEncoder1Count =

            resetCount1;

  

        lastEncoder2Count =

            resetCount2;

  

        positionZeroCount1 =

            resetCount1;

  

        positionZeroCount2 =

            resetCount2;

  

        vehiclePosition =

            0.0f;

  

        positionError = 0.0f;
        positionSpeedTarget = 0.0f;
        positionHoldWasEnabled = false;

  

        vehicleSpeed =

            0.0f;

  

        rawVehicleSpeed =

            0.0f;

  

        imuForwardAccelG =

            0.0f;

  

        imuForwardAccelFilteredG =

            0.0f;

  

        speedKalmanP =

            1.0f;

  

        targetBalanceAngle =

            balanceAngleOffset;

  

        return;

    }

  
  

    // --------------------------------------------------------

    // 先在400Hz读取累计位置，20Hz位置外环使用最新位置。
    updatePositionHold();

    // 速度PI外环100Hz（成熟步进平衡车串级结构）

    //

    // 400Hz控制循环，每4次执行一次 = 10ms。

    // --------------------------------------------------------

  

    outerLoopDivider++;

  
  

    if (

        outerLoopDivider

        >=

        4

    )

    {

        outerLoopDivider =

            0;

  

        updateOuterLoop();

    }

  
  

    // --------------------------------------------------------

    // 外环权限管理400Hz

    // 大角度时速度/位置环平滑退出，直立PD优先

    // --------------------------------------------------------

  

    updateOuterLoopAuthority();

  
  

    // --------------------------------------------------------

    // 位置已在外环计算前更新

    // --------------------------------------------------------

  

    // updatePositionHold() 已在本周期前面执行

  
  

    // --------------------------------------------------------

    // 直立环400Hz

    // --------------------------------------------------------

  

    updateBalanceLoop();

  
  

    // --------------------------------------------------------

    // 转向环400Hz

    // --------------------------------------------------------

  

    updateTurnLoop();

  
  

    // --------------------------------------------------------

    // 电机合成

    // --------------------------------------------------------

  

    float motor1Command =

        finalBalanceOutput

        +

        turnOutput;

  
  

    float motor2Command =

        finalBalanceOutput

        -

        turnOutput;

  
  

    setMotor1(

        motor1Command

    );

  
  

    setMotor2(

        motor2Command

    );

    // ---- 250ms 遥测窗口极值累积（必须在所有控制量算完后）----
    {
        float aGyro = fabsf(gyroPitchRate);
        if (aGyro > telemGyroAbsMax) telemGyroAbsMax = aGyro;
        float aDev = fabsf(accPitch - gyroIntegAngle);
        if (aDev > telemAccPitchDevMax) telemAccPitchDevMax = aDev;
        float aBal = fabsf(balanceOutput);
        if (aBal > telemBalHzAbsMax) telemBalHzAbsMax = aBal;
        if (kalmanBias < telemKalmanBiasMin) telemKalmanBiasMin = kalmanBias;
        if (kalmanBias > telemKalmanBiasMax) telemKalmanBiasMax = kalmanBias;
    }

    // 非阻塞遥测 flush（每控制周期按预算发送缓冲内容）

    flushTelemetryTx();
    uint32_t workUs = micros() - controlWorkStartUs;
    if (workUs > controlMaxWorkUs) controlMaxWorkUs = workUs;
}
