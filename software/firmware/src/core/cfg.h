#pragma once
// ============================================================
// core/cfg.h — 硬件引脚、控制周期、阈值、限幅等常量集中定义
// 只放 #define 与 const；不含任何可变状态（可变状态见 state.h）。
// ============================================================
#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

#define TMC_STANDALONE_STEP_DIR 1

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

// TMC2209 双板 UART 配置（独立 STEP/DIR 模式 TMC_STANDALONE_STEP_DIR=1 时不编译）
#if !TMC_STANDALONE_STEP_DIR
#define R_SENSE 0.11f
#define TMC_GPIO_WRITE_ONLY 1 // 实验模式：无读回，禁止将已发送当作已配置成功

// 前轮当前是否为 TMC2209。false=前轮用 DRV8825(仅 PA6/PA7 脉冲，不配 UART)；
// 验证后轮 TMC spreadCycle 有效后，买新 TMC 换前轮时改回 true 恢复双 TMC 配置。
const bool TMC_FRONT_ACTIVE = false;
#include <TMCStepper.h>
#endif

// ============================================================
// 常量：方向 / 上升速率 / 阈值 / 限幅（原样搬入，未改一个数值）
// ============================================================
const uint32_t BT_BAUD = 9600;

// ============================================================
// 方向参数
// ============================================================
// BMI270相对历史竖装方向绕X轴旋转180°；Y/Z在readBMI270()中同时翻转。
const float GY_DIRECTION = 1.0f;

// IMU前向线加速度方向。
// 当前按竖装BMI270历史方向给出初始值。
// 如果手推车辆向前时 imuForwardAccelG 长期显示为负，改成 +1.0f。
const float IMU_FORWARD_ACCEL_DIRECTION = - 1.0f;

// 转向角速度方向
const float TURN_GYRO_DIRECTION = 1.0f;

// 换向软过渡：命令方向翻转后 REVERSAL_SOFT_TICKS 个端点周期(10ms)内，
// 把增量限幅压到 REVERSAL_SOFT_LIMIT_SCALE 倍。治 DRV8825 换向失步：
// 换向瞬间转子须越过扭矩零点、1/16 微步增量扭矩仅保持扭矩 9.8%，此时
// 满轨爬升最易丢步（日志证实 9/9 次锁定紧跟换向）。软窗口内爬升变缓，
// 给转子时间脱困。注：201421 验证 S/2+150ms 仍会在软窗口内冲满轨，故调狠。
const uint16_t REVERSAL_SOFT_TICKS = 25;  // 25×10ms = 250ms 软窗口

const float REVERSAL_SOFT_LIMIT_SCALE = 0.25f;  // 窗口内限幅×0.25（S=200→50,H=75→18.75）

// 速度环测量低通：车体存在 1~2Hz 前后摆动（轮子不打滑，编码器就是真实速度），
// 摆动 ±13mm/s 乘 speedKp(0.02) ≈ ±0.26° 倾角抖动，正好架在 0.3° 静摩擦门槛两侧。
// 速度外环用低通后的速度，摆动被压掉一个数量级，9mm/s 级的直流指令仍完整保留。
const float SPEED_MEAS_TAU_S = 0.25f;

// 2026-11-19 实测：这是稳定边界，不是"越大越有力"。
// N=2.0 时速度误差一大，外环就反向往死里拽 2°，直接翻车；
// 收到 0.8° 后：起步 0.96° 被截到 0.8°（仍破粘），反向最大也只 0.8°，拽不翻。
// 证据：docs/validation/analysis/20261119-MOTION-CONTROL.md（15:32/15:36）
// 20Hz外环每次更新时，目标倾角修正最多变化的角度。
// 用来防止目标角在 +/-N 之间瞬间跳变。
const float SPEED_ANGLE_SLEW_DEG_PER_UPDATE = 0.10f;  // 100Hz外环，等效最大10deg/s

// ============================================================
// 电机
// ============================================================
const float MOTOR_STOP_THRESHOLD_HZ = 1.0f;

// 400Hz下8帧约20ms
const uint8_t IMU_FAIL_CONFIRM_COUNT = 8;  // IMU读取连续失败8次(约20ms)才确认故障，防偶发去抖

const uint8_t FALL_CONFIRM_COUNT = 8;  // 倒地判定连续8帧(约20ms)确认，防瞬间晃倒误触发

// 严重倾倒时立即停机
const float EMERGENCY_FALL_ANGLE_DEG = 75.0f;

// 大角度时速度积分缓慢释放，避免恢复后旧积分继续推车
const float SPEED_INTEGRAL_DECAY_BIG_ANGLE = 0.95f;

// ============================================================
// 倒地保护
// ============================================================
const float FALL_ANGLE_DEG = 75.0f;  // 倒地判定：|pitch-平衡点|>=75°连续8帧(20ms)触发停机+状态清零

// ============================================================
// 固定400Hz控制
// ============================================================
const uint32_t CONTROL_PERIOD_US = 2500;

const float CONTROL_DT = 0.0025f;

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

const uint16_t ENCODER_TRUST_ACTIVE_TICKS = 5;  // 窗内 >=5 拍有脉冲即视为活跃

const int32_t ENCODER_TRUST_MIN_COUNTS = 1;  // 单拍 >=1 count 才算“有脉冲”

// 防止偶发振动尖峰直接把速度预测带飞
const float IMU_ACCEL_CLAMP_G = 0.60f;

// 用于速度预测时的小加速度死区。
// 静止附近的IMU零偏/振动不再持续积分成假速度。
const float IMU_SPEED_ACCEL_DEADBAND_G = 0.03f;

// 每个400Hz周期，IMU最多只允许把速度估计改变这么多。
// 防止偶发冲击直接把融合速度带飞。
const float MAX_IMU_SPEED_DELTA_PER_CYCLE = 0.02f;

// ============================================================
// IMU零位
// ============================================================
// 竖装BMI270：X轴沿车体上下、Z轴沿前后，车体竖直时原始Pitch应接近0°。
// 只补偿固定安装误差，不把开机时车辆姿态当作零点。
const float PITCH_MOUNT_OFFSET_DEG = 0.0f;

// ===== Pitch 毛刺抑制 =====
// 400Hz dt=0.0025：车真实单步角位移(倒摆+电机最剧烈)远小于5°/步，
// 单步突变>5° 判为IMU/卡尔曼毛刺，用上一有效值替代，防止大跳变直接冲进直立PD造成震荡。
const float PITCH_GLITCH_DEG = 5.0f;

// 先用比较保守的一组阈值
const float ACC_ERR_LOW = 0.04f;

const float ACC_ERR_HIGH = 0.10f;

// 方向分歧（innovation）阈值：|accPitch - gyroIntegAngle| 超过该值判为加速度计毛刺，
// 立即把 dynamicR 拉满、只信陀螺积分。真实运动时 accPitch 与 gyroInt 同步 → 差值小 → 不误伤。
const float ACC_INNOV_HIGH = 20.0f;

const float R_LOW_DYNAMIC = 0.02f;

const float R_HIGH_DYNAMIC = 1.20f;

const uint8_t BALANCE_ACCEL_UPDATE_DIVIDER = 4;  // 400Hz / 4 = 100Hz

// ============================================================
// 蓝牙遥测
// 控制400Hz，每40个周期发送一次 = 10Hz
// 数据格式包含速度预测链：speedAccel1000、predSpeed1000、predDelta1000、
// directUnclampedHz，便于区分预测、独立H限幅、权限及最终混合输出。
// 后缀100/1000=×100/×1000取整，Hz字段为整数STEP频率。电脑端脚本见 pinghengche_ble_ready/
// ============================================================
const uint32_t TELEMETRY_DIVIDER = 400;  // 400Hz/400 = 1Hz；剩余带宽给下面的 10Hz 调试帧（总输出受 800 B/s 预算限制）

// 10Hz 快速帧：只发量化速度跟踪/姿态需要的 7 个量，避免完整帧占满 9600 波特
const uint32_t FAST_TELEMETRY_DIVIDER = 100;  // 400Hz/100 = 4Hz（2026-11-12 起回到 4Hz：10Hz 会挤占命令通道，实测 O 命令延迟 0.7~0.9s）

const uint32_t STEP_HIGH_US = 4;

const uint32_t DIR_SETUP_US = 4;

const float MAX_STEP_HZ = 20000.0f;
