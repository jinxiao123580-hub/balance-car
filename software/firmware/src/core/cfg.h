#pragma once
// ============================================================
// core/cfg.h — 硬件引脚、控制周期、阈值、限幅等常量集中定义
// 只放 #define 与 const；不含任何可变状态（可变状态见 state.h）。
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

// 电机驱动模式开关（编译期）：1=纯 STEP/DIR 模式，TMC2209 的细分/电流/斩波由载板硬件
//   （MS1/MS2、VREF、SPREAD 跳线）决定，固件不发 UART；0=编译 TMC2209 双板 UART 配置代码（诊断实验）。
// 当前 =1：本次烧录验证的就是 STEP/DIR 模式；改 0 会改变 setup() 分支、Flash 占用和 TMC 诊断命令输出。
// 谁读：control/motor.h 的 #if、main.cpp setup()、panel/cmd.cpp 的 ! 诊断分支。
#define TMC_STANDALONE_STEP_DIR 1

// ============================================================
// 固定引脚 —— 不修改
// ============================================================
// Motor1
// Motor1（前轮）方向脚 PA7：高/低电平对应正/反向，最终极性由 motor1Invert(false) 决定，
//   见 control/motor.cpp commandStepOutput()；换向时前后各 DIR_SETUP_US 建立时间。
#define MOTOR1_DIR_PIN   PA7
// Motor1 STEP 脉冲脚 PA6：“TIM3_CH1”只是该引脚的定时器复用标注；本工程不用输出比较，
//   实际由 TIM2 中断每拍直接翻转 GPIO（见 control/motor.cpp step1/stepEdge）。
#define MOTOR1_STEP_PIN  PA6      // TIM3_CH1

// Motor2
// Motor2（后轮）方向脚 PB0：MKS TMC2209 V2.0 后轮与前轮安装方向相反，motor2Invert=true 已翻转极性。
#define MOTOR2_DIR_PIN   PB0
// Motor2 STEP 脉冲脚 PA8：“TIM1_CH1”只是复用标注；实际由 TIM5 中断翻转 GPIO（step2）。
#define MOTOR2_STEP_PIN  PA8      // TIM1_CH1

// BMI270 I2C
// BMI270 I2C：硬件 I2C1。PB8=SCL、PB9=SDA；初始化阶段 100kHz，
//   main.cpp setup() 在芯片配置完成后切到 400kHz（BMIWire.setClock）。
#define BMI_SCL_PIN      PB8
// BMI270 I2C 数据脚 PB9（模块 SDO 接地时地址 0x68，见 BMI270_ADDRESS）。
#define BMI_SDA_PIN      PB9

// OLED引脚保留定义，但本版完全不用OLED
// OLED 四根脚仅历史保留：本版完全不用 OLED，main.cpp 不初始化、不驱动，不影响控制时序。
#define OLED_SDA_PIN     PF7
// OLED SCL（PF9）/ SDA（PF7）：历史预留，本版不初始化。
#define OLED_SCL_PIN     PF9
// OLED 电源正（PC0）：历史预留，本版不受控（OLED 未用）。
#define OLED_VCC_PIN     PC0
// OLED 地（PC2）：历史预留，本版不受控。
#define OLED_GND_PIN     PC2

// Encoder1
// Encoder1（前轮）A/B 相 PA0/PA1：内部上拉，只对 A 相双边沿中断计数（B 相电平判方向），
//   见 control/motor.cpp encoder1ISR；方向系数 encoder1Direction=+1。
#define ENCODER1_A_PIN   PA0
#define ENCODER1_B_PIN   PA1

// Encoder2
// Encoder2（后轮）A/B 相 PB6/PB7：同上，方向系数 encoder2Direction=-1 修正安装/接线反向。
#define ENCODER2_A_PIN   PB6
#define ENCODER2_B_PIN   PB7

// ============================================================
// 蓝牙串口 —— USART3 (PB10=TX / PB11=RX)
// 模块RXD <- STM32 PB10 (USART3_TX)
// 模块TXD -> STM32 PB11 (USART3_RX)
// 波特率: JDY-23 出厂默认 9600
// ============================================================
// 蓝牙串口 USART3：PB11=RX（接模块 TXD）、PB10=TX（接模块 RXD）；9600 8N1，
//   接收在 panel::poll() 每圈轮询（每圈最多 32B），发送走非阻塞环形缓冲。
#define BT_RX_PIN PB11
#define BT_TX_PIN PB10

// TMC2209 双板 UART 配置（独立 STEP/DIR 模式 TMC_STANDALONE_STEP_DIR=1 时不编译）
// ↓↓↓ 以下 TMC2209 UART 代码在 TMC_STANDALONE_STEP_DIR=1（当前值）时整段不编译，因此不影响 Flash/RAM；
#if !TMC_STANDALONE_STEP_DIR
// TMC2209 电流采样电阻 0.11Ω（MKS TMC2209 V2.0 载板实装值）；rms_current(1200) 由它换算。
#define R_SENSE 0.11f
// 实验诊断模式：用 GPIO 位翻转模拟 UART 只写、不读回；既然读不回，
//   就不能把“已发送”当作“已配置成功”（日志里只输出 SENT/UNVERIFIED）。
#define TMC_GPIO_WRITE_ONLY 1 // 实验模式：无读回，禁止将已发送当作已配置成功

// 前轮当前是否为 TMC2209。false=前轮用 DRV8825(仅 PA6/PA7 脉冲，不配 UART)；
// 验证后轮 TMC spreadCycle 有效后，买新 TMC 换前轮时改回 true 恢复双 TMC 配置。
// 前轮是否也装 TMC2209：false=前轮为 DRV8825（只吃 PA6/PA7 脉冲，不配 UART）；
//   等换新 TMC 并接好 PDN_UART 后改 true，才恢复双 TMC UART 配置。
const bool TMC_FRONT_ACTIVE = false;
#include <TMCStepper.h>
#endif

// ============================================================
// 常量：方向 / 上升速率 / 阈值 / 限幅（原样搬入，未改一个数值）
// ============================================================
// 蓝牙波特率 9600 8N1（JDY-23 出厂默认）。带宽上限约 960B/s：
//   遥测按 ~800B/s 预算发送，命令回显/参数回显会抢占同一发送缓冲（panel/telemetry.cpp TX_BUDGET_*）。
const uint32_t BT_BAUD = 9600;

// ============================================================
// 方向参数
// ============================================================
// GY_DIRECTION：俯仰角速度的符号修正系数 ｜ 无量纲，只取 ±1 ｜ 当前 +1
//   （readBMI270() 已把 gy 翻成 -gy，此处再定最终正方向）｜ 读：updateAttitude() 算 gyroPitchRate，400Hz。
// BMI270相对历史竖装方向绕X轴旋转180°；Y/Z在readBMI270()中同时翻转。
const float GY_DIRECTION = 1.0f;

// IMU前向线加速度方向。
// 当前按竖装BMI270历史方向给出初始值。
// 如果手推车辆向前时 imuForwardAccelG 长期显示为负，改成 +1.0f。
// 符号系数 ｜ 无量纲 ±1 ｜ 乘在 (az+sin(pitch)) 上，把前向加速度统一成“向前为正”；
//   当前 -1（竖装 BMI270 前后轴向反）｜ 读：updateAttitude() 算 imuForwardAccelG，400Hz。
const float IMU_FORWARD_ACCEL_DIRECTION = - 1.0f;

// 转向角速度方向
// 转向角速度符号系数 ｜ 无量纲 ±1 ｜ 当前 +1 ｜ 读：updateAttitude() 算 gyroTurnRate=dir*(gx-gyroXZero)，
//   400Hz；当前 turnKp/turnKd=0，该量主要用于遥测与转向状态机的角度保护。
const float TURN_GYRO_DIRECTION = 1.0f;

// 换向软过渡：命令方向翻转后 REVERSAL_SOFT_TICKS 个端点周期(10ms)内，
// 把增量限幅压到 REVERSAL_SOFT_LIMIT_SCALE 倍。治 DRV8825 换向失步：
// 换向瞬间转子须越过扭矩零点、1/16 微步增量扭矩仅保持扭矩 9.8%，此时
// 满轨爬升最易丢步（日志证实 9/9 次锁定紧跟换向）。软窗口内爬升变缓，
// 给转子时间脱困。注：201421 验证 S/2+150ms 仍会在软窗口内冲满轨，故调狠。
// 换向软窗口长度 ｜ 单位=100Hz 端点拍，1 拍=10ms ｜ 25 拍=250ms，可写 0~65535。
//   写：balance.cpp 检测到 integratedMotorSpeedCommand 变号时置 25；读：每 100Hz 端点递减 → 窗口内限幅再乘 0.25。
const uint16_t REVERSAL_SOFT_TICKS = 25;  // 25×10ms = 250ms 软窗口

// 软窗口内增量限幅倍率 ｜ 无量纲 0~1 ｜ 0.25 的依据：换向瞬间 1/16 微步增量扭矩仅剩保持扭矩 9.8%，
//   满轨爬升最易丢步（日志 9/9 次锁定紧跟换向）；窗口内 S=200→50、H=75→18.75，让转子越过扭矩零点。
const float REVERSAL_SOFT_LIMIT_SCALE = 0.25f;  // 窗口内限幅×0.25（S=200→50,H=75→18.75）

// 速度环测量低通：车体存在 1~2Hz 前后摆动（轮子不打滑，编码器就是真实速度），
// 摆动 ±13mm/s 乘 speedKp(0.02) ≈ ±0.26° 倾角抖动，正好架在 0.3° 静摩擦门槛两侧。
// 速度外环用低通后的速度，摆动被压掉一个数量级，9mm/s 级的直流指令仍完整保留。
// 速度测量一阶低通时间常数 ｜ 单位 s ｜ 历史值 0.25（约 1~2Hz 摆动被压掉一个数量级）
//   （注：旧描述与当前实现不符，以代码为准：cascade.cpp 里 vehicleSpeedFiltered = vehicleSpeed 直接透传，
//    本低通当前未启用；速度外环现为 100Hz/10ms，不是注释里说的 20Hz。保留此常量只为将来小步试滤波。）
const float SPEED_MEAS_TAU_S = 0.25f;

// 2026-11-19 实测：这是稳定边界，不是"越大越有力"。
// N=2.0 时速度误差一大，外环就反向往死里拽 2°，直接翻车；
// 收到 0.8° 后：起步 0.96° 被截到 0.8°（仍破粘），反向最大也只 0.8°，拽不翻。
// 证据：docs/validation/analysis/20261119-MOTION-CONTROL.md（15:32/15:36）
// 20Hz外环每次更新时，目标倾角修正最多变化的角度。
// 用来防止目标角在 +/-N 之间瞬间跳变。
// 目标倾角 slew 限幅 ｜ 单位 deg/次更新 ｜ 0.10；作用对象= speedAngleCorrection 朝 speedAngleLimited 的每拍最大步进。
//   （注：旧描述与当前实现不符，以代码为准：外环是 100Hz，故 0.10°/拍 = 10°/s；不是 20Hz。）
const float SPEED_ANGLE_SLEW_DEG_PER_UPDATE = 0.10f;  // 100Hz外环，等效最大10deg/s

// ============================================================
// 电机
// ============================================================
// 电机“视为停止”门限 ｜ 单位 Hz ｜ 1.0；commandStepOutput 里 |commandHz|<1 或 NaN 直接停 STEP，
//   避免极低频占空/堵转发热。读：setMotor1/2 @400Hz 每拍。
const float MOTOR_STOP_THRESHOLD_HZ = 1.0f;

// 400Hz下8帧约20ms
// IMU 连续失败确认次数 ｜ 单位=400Hz 帧 ｜ 8 帧≈20ms ｜ 写：main.cpp loop 读失败 ++、成功清零；
//   读：main.cpp 达阈值才停电机+清控制状态（防单次 I2C 抖动误停）。
const uint8_t IMU_FAIL_CONFIRM_COUNT = 8;  // IMU读取连续失败8次(约20ms)才确认故障，防偶发去抖

// 倒地确认次数 ｜ 单位=400Hz 帧 ｜ 8 帧≈20ms ｜ 写：main.cpp 每拍按 |pitch-O|>=FALL_ANGLE_DEG 累加/清零；
//   读：达阈值确认倒地 → 停机+fallStopCount+++全部状态清零（防瞬间晃倒误触发）。
const uint8_t FALL_CONFIRM_COUNT = 8;  // 倒地判定连续8帧(约20ms)确认，防瞬间晃倒误触发

// 严重倾倒时立即停机
// 紧急倾倒角 ｜ 单位 deg ｜ 75；单帧 |pitch-O|>=75° 立即停机，不再等 8 帧确认。读：main.cpp @400Hz。
const float EMERGENCY_FALL_ANGLE_DEG = 75.0f;

// 大角度时速度积分缓慢释放，避免恢复后旧积分继续推车
// 大角度积分释放系数 ｜ 无量纲 0~1 ｜ 0.95，每 400Hz 拍乘一次，1s 衰减到 0.95^400≈1e-9；
//   读：cascade::updateAuthority（outerLoopAuthority<0.2 时），防大角度期间积分越积越多、恢复后推飞。
const float SPEED_INTEGRAL_DECAY_BIG_ANGLE = 0.95f;

// ============================================================
// 倒地保护
// ============================================================
// 倒地判定角 ｜ 单位 deg（相对机械平衡角 O）｜ 75；>=75° 连续 8 帧（20ms）触发停机+状态清零。
//   与 EMERGENCY_FALL_ANGLE_DEG 同值：后者走单帧立即停，前者是确认窗兜底。读：main.cpp @400Hz。
const float FALL_ANGLE_DEG = 75.0f;  // 倒地判定：|pitch-平衡点|>=75°连续8帧(20ms)触发停机+状态清零

// ============================================================
// 固定400Hz控制
// ============================================================
// 固定控制周期 ｜ 单位 µs ｜ 2500µs=400Hz ｜ main.cpp loop 用 lastControlUs 累加对齐；
//   晚点 >=2 个周期（>=5000µs）判定掉拍：controlOverrunCount++、停机清状态并重置分频。
const uint32_t CONTROL_PERIOD_US = 2500;

// 控制周期 ｜ 单位 s ｜ 0.0025 ｜ 所有 400Hz 积分/微分用它：Kalman 角度预测、陀螺积分、
//   IMU→速度预测、转向环微分。与 CONTROL_PERIOD_US 必须同步改。
const float CONTROL_DT = 0.0025f;

// SDO接地使用主地址0x68；若模块SDO拉高，应改为BMI2_I2C_SEC_ADDR(0x69)。
// BMI270 I2C 地址 ｜ 7bit 地址 ｜ BMI2_I2C_PRIM_ADDR=0x68（SDO 脚接地）。
const uint8_t BMI270_ADDRESS = BMI2_I2C_PRIM_ADDR;

// ============================================================
// BMI270输出数据率/滤波配置寄存器
// ============================================================
// BMI270 加速度计配置寄存器（acc_conf）地址。
const uint8_t BMI270_REG_ACC_CONF = 0x40;

// BMI270 陀螺仪配置寄存器（gyr_conf）地址。
const uint8_t BMI270_REG_GYR_CONF = 0x42;

// BMI270 命令寄存器（cmd）地址，用于软复位等一次性命令。
const uint8_t BMI270_REG_CMD = 0x7E;

// 软复位命令字 0xB6；写入后按数据手册需等待，本工程 initBMI270() 里 delay(50)。
const uint8_t BMI270_SOFT_RESET_CMD = 0xB6;

// 加速度计配置值 0xAA ｜ 历史实车验证值（原注释称 400Hz 性能滤波模式）；
//   configureBMI270_400Hz() 写入后读回比对，禁止未实测就改。
const uint8_t BMI270_ACC_CONF_400HZ = 0xAA;

// 陀螺仪配置值 0xAA ｜ 同上；写作后读回比对，不一致则开机重试。
const uint8_t BMI270_GYR_CONF_400HZ = 0xAA;

// 400Hz低通，抑制步进电机高频振动
// 前向加速度一阶 IIR 系数 ｜ 无量纲 0~1 ｜ 0.05 @400Hz → τ≈-dt/ln(1-α)≈51ms；
//   用来压掉步进电机高频振动进入速度预测。读：updateAttitude() 每次 400Hz 执行。
const float IMU_ACCEL_LPF_ALPHA = 0.05f;

// 编码器速度一阶低通系数（20Hz外环，每50ms一次）。
// encoderSpeed = (1-a)·旧 + a·新。a越小越平滑但响应越慢。
// 默认0.4，兼顾平滑与跟踪。震荡明显时调小(0.2)，响应优先时调大(0.8)。
// 编码器速度一阶低通系数 ｜ 无量纲 0~1（原注释给 0.2~0.8 可调）｜ 读：control::tick100，100Hz/10ms。
//   公式 encoderSpeed=(1-α)·旧+α·新；α=0.4 时 τ≈-0.01/ln(0.6)≈19.6ms。
//   （注：旧描述与当前实现不符，以代码为准：外环是 100Hz/10ms，不是注释里的 20Hz/50ms。）
const float ENCODER_SPEED_LPF_ALPHA = 0.4f;

// 编码器标度（外部相机对表标定，见 docs/code-walkthrough/01-INTRO.md §5.2）：
//   10.8 count = 1mm ⇒ 1 count/50ms = 20 count/s = 1.85 mm/s。
//   固件内部只用 count / count/50ms，从不做 mm 换算（mm 换算留给上位机脚本）。
// --- 编码器信任门（见 updateOuterLoop）---
// updateOuterLoop 由 400Hz 循环每 4 次调用一次 = 100Hz (10ms)。
// 实机 2026-09-17 20:41 日志证实二号编码器会整段无脉冲：66s 里约 50s
// 行程为 0，而同期一号编码器正常，且两台电机收到完全相同的命令。
// 这是该路接线/连接器的间歇故障，不是电机失步。
// 坏通道仍会偶发单个噪声脉冲，所以用滑窗累计“有脉冲”的拍数，
// 不能用连续计数，否则计数永远攒不满。
// 信任判定滑窗 ｜ 单位=100Hz 拍（10ms）｜ 50 拍=500ms；窗满后按窗内“有脉冲拍数”重新判定两路信任。
const uint16_t ENCODER_TRUST_WINDOW_TICKS = 50;  // 500ms 判定窗

// 活跃门限 ｜ 单位=拍 ｜ 5 拍（>=50ms 有动作）才算该路活跃；窗内两路都不活跃时保持上次结论（静止不误判）。
const uint16_t ENCODER_TRUST_ACTIVE_TICKS = 5;  // 窗内 >=5 拍有脉冲即视为活跃

// “有脉冲”的计数门限 ｜ 单位 count ｜ 1；单拍 |Δ|>=1 即记一拍活跃（坏通道偶发单脉冲不会长期刷满）。
const int32_t ENCODER_TRUST_MIN_COUNTS = 1;  // 单拍 >=1 count 才算“有脉冲”

// 防止偶发振动尖峰直接把速度预测带飞
// 前向加速度尖峰限幅 ｜ 单位 g ｜ 0.60g≈5.9m/s²；读：updateAttitude() 400Hz，防止偶发冲击把预测速度带飞。
const float IMU_ACCEL_CLAMP_G = 0.60f;

// 用于速度预测时的小加速度死区。
// 静止附近的IMU零偏/振动不再持续积分成假速度。
// 预测死区 ｜ 单位 g ｜ 0.03g≈0.29m/s²；|a| 小于它时预测增量清零，防零偏/振动积分成假速度。
//   读：predictVehicleSpeedKalman() @400Hz。
const float IMU_SPEED_ACCEL_DEADBAND_G = 0.03f;

// 每个400Hz周期，IMU最多只允许把速度估计改变这么多。
// 防止偶发冲击直接把融合速度带飞。
// 单拍速度预测限幅 ｜ 单位 count/50ms per 2.5ms 拍 ｜ 0.02 ⇒ 一秒最多 8 count/50ms（≈14.8mm/s）；
//   读：predictVehicleSpeedKalman() @400Hz。
const float MAX_IMU_SPEED_DELTA_PER_CYCLE = 0.02f;

// ============================================================
// IMU零位
// ============================================================
// 竖装BMI270：X轴沿车体上下、Z轴沿前后，车体竖直时原始Pitch应接近0°。
// 只补偿固定安装误差，不把开机时车辆姿态当作零点。
// IMU 安装角固定偏置 ｜ 单位 deg ｜ 当前 0；只补装配误差，不把开机姿态当零点。
//   读：updateAttitude() 算 accPitch=加速度角-pitchZero，400Hz；calibrateIMU 也会重置它。
const float PITCH_MOUNT_OFFSET_DEG = 0.0f;

// ===== Pitch 毛刺抑制 =====
// 400Hz dt=0.0025：车真实单步角位移(倒摆+电机最剧烈)远小于5°/步，
// 单步突变>5° 判为IMU/卡尔曼毛刺，用上一有效值替代，防止大跳变直接冲进直立PD造成震荡。
// Pitch 毛刺门限 ｜ 单位 deg（单步 2.5ms 内的变化量）｜ 5.0；超限用 pitchPrev 顶替本拍，
//   依据：倒摆+电机最剧烈时单步也远小于 5°。读：updateAttitude() @400Hz。
const float PITCH_GLITCH_DEG = 5.0f;

// 先用比较保守的一组阈值
// 加速度模长误差低门限 ｜ 单位 g ｜ 0.04；| |a|-1g |<0.04 → IMU 预测信任=1.0。
//   读：updateAttitude()（imuSpeedPredictionTrust）与 dynamicR 插值。
const float ACC_ERR_LOW = 0.04f;

// 加速度模长误差高门限 ｜ 单位 g ｜ 0.10；达到后信任降到 0.12、dynamicR 插值权重拉满（只信陀螺）。
const float ACC_ERR_HIGH = 0.10f;

// 方向分歧（innovation）阈值：|accPitch - gyroIntegAngle| 超过该值判为加速度计毛刺，
// 立即把 dynamicR 拉满、只信陀螺积分。真实运动时 accPitch 与 gyroInt 同步 → 差值小 → 不误伤。
// 方向分歧门限 ｜ 单位 deg ｜ 20；|accPitch-gyroIntegAngle|>20 判加速计毛刺，dynamicR 立即拉满。
//   真实运动时两者同步变化 → 差值小，不会误伤。读：updateAttitude() @400Hz。
const float ACC_INNOV_HIGH = 20.0f;

// 动态测量噪声下限 ｜ 单位 deg² ｜ 0.02（加速度计角度方差）；静止/低速时 accPitch 权重高。
//   读：updateAttitude() 插值出 dynamicR，updateKalman 用 S=P00+dynamicR。
const float R_LOW_DYNAMIC = 0.02f;

// 动态测量噪声上限 ｜ 单位 deg² ｜ 1.20；振动/线性加速度时 accPitch 权重最低，防污染角度。
const float R_HIGH_DYNAMIC = 1.20f;

// 直立增量分频比 ｜ 单位=400Hz 拍数 ｜ 4 → 每 10ms 算一次 100Hz 端点增量，再把该增量均分到 4 个 400Hz 小步执行。
//   读：updateBalance()。
const uint8_t BALANCE_ACCEL_UPDATE_DIVIDER = 4;  // 400Hz / 4 = 100Hz

// ============================================================
// 蓝牙遥测
// 控制400Hz，每40个周期发送一次 = 10Hz
// 数据格式包含速度预测链：speedAccel1000、predSpeed1000、predDelta1000、
// directUnclampedHz，便于区分预测、独立H限幅、权限及最终混合输出。
// 后缀100/1000=×100/×1000取整，Hz字段为整数STEP频率。电脑端脚本见 pinghengche_ble_ready/
// ============================================================
// 慢帧分频 ｜ 单位=400Hz 拍 ｜ 400 → 1Hz（D 帧 75 列）；写/读：main.cpp loop @400Hz。
//   （注：旧描述与当前实现不符，以代码为准：不是“每 40 周期=10Hz”；1Hz 是 2026-11-12 为命令通道让带宽后的现值。）
//   （注：旧注释中的 speedAccel1000/predSpeed1000/directUnclampedHz 等预测/直投通道已按 SPEC §5 删除，对应 D 帧列保留填 0。）
const uint32_t TELEMETRY_DIVIDER = 400;  // 400Hz/400 = 1Hz；剩余带宽给下面的 10Hz 调试帧（总输出受 800 B/s 预算限制）

// 10Hz 快速帧：只发量化速度跟踪/姿态需要的 7 个量，避免完整帧占满 9600 波特
// 快帧分频 ｜ 单位=400Hz 拍 ｜ 100 → 4Hz（G 帧 8 列）；写/读：main.cpp loop @400Hz。
//   （注：上一行“10Hz 快速帧”是旧值；2026-11-12 起 10Hz 会挤占命令通道（O 命令延迟 0.7~0.9s），已降到 4Hz。）
const uint32_t FAST_TELEMETRY_DIVIDER = 100;  // 400Hz/100 = 4Hz（2026-11-12 起回到 4Hz：10Hz 会挤占命令通道，实测 O 命令延迟 0.7~0.9s）

// STEP 高电平宽度 ｜ 单位 µs ｜ 4（1MHz 定时器下 4 tick）；读：stepEdge() 高低电平 ARR 切换。
const uint32_t STEP_HIGH_US = 4;

// DIR 建立/保持时间 ｜ 单位 µs ｜ 4；换向时 stop→改 DIR→再启动，前后各等一次。
//   TMC2209 只要求 >=20ns，4µs 很保守，代价是换向时多 8µs。
const uint32_t DIR_SETUP_US = 4;

// STEP 频率硬上限 ｜ 单位 Hz ｜ 20000；setMotor1/2 用 fminf 夹紧，防超频失步/计算溢出。
//   换算：1 STEP=1/16 微步=0.0625mm ⇒ 20000Hz≈1250mm/s（实际输出限幅 L=1750Hz≈109mm/s）。
const float MAX_STEP_HZ = 20000.0f;
