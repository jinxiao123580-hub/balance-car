#pragma once
// ============================================================
// core/state.h — 全局可变状态的 extern 声明（唯一的跨模块状态通道）
// 定义在各自的 owner .cpp 里；任何模块都可以通过本头文件读写这些量，
// 但模块间不得 include 对方的实现头（依赖方向见 SPEC §2）。
// ============================================================
#include <Arduino.h>
#include <Wire.h>

// ------------------------------------------------------------
// 单位约定（全工程统一；本节所有 extern 的解释都按它读）：
//   速度 = count/50ms（10.8 count=1mm ⇒ 1 count/50ms = 20 count/s = 1.85 mm/s；
//     100Hz 下 10ms 增量 ×5 即换算成 count/50ms）；
//   位置 = 平均编码器 count（10.8 count=1mm）；角度 = deg；角速度 = deg/s；
//   STEP 频率 = Hz（1 STEP=1/16 微步=0.0625mm ⇒ 1Hz=0.0625mm/s；L=1750Hz≈109mm/s）。
// 每条 extern 注释格式：含义｜单位与范围｜谁写｜谁读｜更新频率。
// ------------------------------------------------------------
// 蓝牙串口实例（定义在 panel/telemetry.cpp）
// BTSerial：蓝牙串口实例（USART3，9600 8N1）｜ 单位/范围：无（HardwareSerial 对象）｜
//   写：各模块 print（遥测/命令回显/诊断）｜ 读：panel::poll() 每圈最多 32B 取命令行 ｜
//   频率：接收每圈轮询；发送 1~4Hz 遥测 + 命令回显。
extern HardwareSerial BTSerial;
// BMI270 I2C 总线（定义在 control/imu.cpp）
// BMIWire：BMI270 的 I2C 总线实例（PB9=SDA/PB8=SCL）｜ 单位/范围：无（TwoWire 对象）｜
//   写/读：bmiWriteRegister/bmiReadRegister、imu.beginI2C/getSensorData（control/imu.cpp）｜
//   频率：400Hz 读一次；时钟初始化 100kHz，main setup 配置完成后切 400kHz。
extern TwoWire BMIWire;

// manualMotionWasBusy：^ 前后运动闭环是否接管中 ｜ 无量纲 bool ｜
//   写：remote::motionDriveTick（进入=true、退出=false）｜
//   读：remote::motionWasBusy → main → control::outerLoop（用于清 speedCmd）｜ 100Hz。
extern bool manualMotionWasBusy;
// balanceDirection：直立 PD 输出方向系数 ｜ 无量纲 ±1，当前 -1（实测定向）｜
//   写：无（编译期常量，仅 balance.cpp 定义）｜ 读：updateBalance 算 P/D 项 ｜ 400Hz。
extern int balanceDirection;
// motor1Invert：Motor1 DIR 极性补偿 ｜ bool，当前 false（前轮正装）｜
//   写：无（常量）｜ 读：commandStepOutput 每次电机命令 ｜ 400Hz。
extern bool motor1Invert;
// motor2Invert：Motor2 DIR 极性补偿 ｜ bool，当前 true（MKS 载板后轮与前轮方向相反）｜
//   写：无（常量）｜ 读：commandStepOutput ｜ 400Hz。
extern bool motor2Invert;
// encoder1Direction：编码器1 计数方向 ｜ ±1，当前 +1 ｜ 写：无（常量）｜
//   读：encoder1ISR 每个 A 相边沿（kHz 级）。
extern int encoder1Direction;
// encoder2Direction：编码器2 计数方向 ｜ ±1，当前 -1（8/21 实测两路计数符号相反）｜
//   写：无（常量）｜ 读：encoder2ISR 每个 A 相边沿。
extern int encoder2Direction;
// balanceAngleOffset：机械平衡角（面板参数 O）｜ 单位 deg ｜ 默认 2.99，随装配/上电漂移（实测约 2~5；
//   4.24 会漂 +28.5mm/s）｜ 写：panel O/LOAD/DEF、flash 读回 ｜ 读：updateAuthority（相对角基准）、
//   main 倒地判据、remote 位置/点动角度保护、遥测 ｜ 400Hz 级。
extern float balanceAngleOffset;
// balanceKp：直立环 P 增益（面板 P）｜ 单位 Hz/deg（再经 R 缩放成 Hz/100Hz 增量）｜ 默认 1000 ｜
//   写：panel P/LOAD/DEF ｜ 读：updateBalance ｜ 400Hz。
extern float balanceKp;
// balanceKd：直立环 D 增益（面板 D）｜ 单位 Hz/(deg/s) ｜ 默认 100 ｜ 写：panel D/LOAD/DEF ｜
//   读：updateBalance ｜ 400Hz。
extern float balanceKd;
// BALANCE_OUTPUT_LIMIT_HZ：直立输出绝对限幅（面板 L）｜ 单位 Hz ｜ 默认 1750（≈109mm/s 极速；
//   ±2000/4000 站不住、30000 大角度瞬间失步）｜ 写：panel L/LOAD/DEF ｜ 读：updateBalance 积分后钳位、
//   remote 转向上限计算 ｜ 400Hz。
extern float BALANCE_OUTPUT_LIMIT_HZ;
// BALANCE_OUTPUT_SLEW_HZ：每个 100Hz 端点允许的 STEP 频率增量上限（面板 S）｜ 单位 Hz/update ｜
//   默认 100（=10000Hz/s）｜ 写：panel S/LOAD/DEF ｜ 读：updateBalance 100Hz 端点。
extern float BALANCE_OUTPUT_SLEW_HZ;
// BALANCE_ACCEL_SCALE：直立 PD 输出→频率增量的比例 R（面板 R）｜ 无量纲 ｜ 默认 0.10 ｜
//   写：panel R/LOAD/DEF ｜ 读：updateBalance（balanceAccelRawOutput=R*balanceOutput）｜ 100Hz。
extern float BALANCE_ACCEL_SCALE;
// HIGH_SPEED_ACCEL_START_HZ：高速加速保护启动频率（面板 B）｜ 单位 Hz ｜ 默认 1000 ｜
//   写：panel B/LOAD/DEF ｜ 读：updateBalance ｜ 100Hz。
extern float HIGH_SPEED_ACCEL_START_HZ;
// HIGH_SPEED_ACCEL_LIMIT_HZ：高速段继续加速的增量上限（面板 H）｜ 单位 Hz/update ｜ 默认 40（=4000Hz/s）｜
//   写：panel H/LOAD/DEF ｜ 读：updateBalance；只收紧同向加速，不限制制动/反向 ｜ 100Hz。
extern float HIGH_SPEED_ACCEL_LIMIT_HZ;
// lastFinalBalanceOutput：finalBalanceOutput 的上一拍副本 ｜ 单位 Hz ｜ 写：updateBalance/clearOutputDiagnostics ｜
//   读：当前无读者（历史诊断保留，未进遥测）｜ 400Hz。
extern float lastFinalBalanceOutput;
// BALANCE_ANGLE_DEADBAND_DEG：直立角度死区（面板 Z）｜ 单位 deg ｜ 默认 0.0（8/21 由 3°→1.5°→0.5°→0 调小）｜
//   写：panel Z/LOAD/DEF ｜ 读：updateBalance ｜ 400Hz。
extern float BALANCE_ANGLE_DEADBAND_DEG;
// SPEED_ERROR_DEADBAND：速度误差积分门控死区（面板 J）｜ 单位 count/50ms ｜ 默认 2 ｜
//   写：panel J/LOAD/DEF ｜ 读：outerLoop 决定是否累积 speedIntegral ｜ 100Hz。
extern float SPEED_ERROR_DEADBAND;
// MAX_TARGET_FORWARD_SPEED：目标速度上限（MS 命令，RAM-only）｜ 单位 count/50ms ｜ 默认 30（≈55.5mm/s；
//   8 太小会锁在静摩擦主导区）｜ 写：panel MS、SP 钳位 ｜ 读：outerLoop 安全限幅 ｜ 100Hz。
extern float MAX_TARGET_FORWARD_SPEED;
// speedLoopDirection：速度误差→角度修正的符号 ｜ 无量纲 ±1 ｜ 当前 -1（2026-09-02 架空实测定反后翻号）｜
//   写：无（常量）｜ 读：outerLoop、remote 点动方向 ｜ 100Hz。
extern int speedLoopDirection;
// speedKp：速度环 P 增益（面板 K / SV 命令）｜ 单位 deg/(count/50ms) ｜ 默认 -0.10（-0.02 太小：SP=8 只给
//   0.16° 低于破粘 0.5~0.7°；>=0.20 会猛冲撞翻）｜ 写：panel K/SV/LOAD/DEF ｜ 读：outerLoop ｜ 100Hz。
extern float speedKp;
// speedKi：速度环 I 增益（面板 I）｜ 单位 deg/((count/50ms)·s) ｜ 默认 0（当前纯 P 验证）｜
//   写：panel I/LOAD/DEF ｜ 读：outerLoop（条件积分+抗饱和）｜ 100Hz。
extern float speedKi;
// speedKd：速度环 D 增益（面板 E）｜ 单位 deg·s/(count/50ms) ｜ 默认 0 ｜ 写：panel E/LOAD/DEF ｜
//   读：outerLoop ｜ 100Hz。
extern float speedKd;
// MAX_SPEED_ANGLE：速度修正目标角硬限幅（面板 N）｜ 单位 deg ｜ 默认 0.8（N=2.0 会反向拽 2° 翻车，
//   0.8 是稳定边界）｜ 写：panel N/LOAD/DEF ｜ 读：outerLoop 钳位 ｜ 100Hz。
extern float MAX_SPEED_ANGLE;
// SPEED_INTEGRAL_LIMIT：速度积分累积上限（面板 W）｜ 单位 (count/50ms)·s（误差×秒）｜ 默认 50 ｜
//   写：panel W/LOAD/DEF ｜ 读：outerLoop 抗饱和候选积分钳位 ｜ 100Hz。
extern float SPEED_INTEGRAL_LIMIT;
// positionKp：位置 P 外环增益（面板 X）｜ 单位 (count/50ms)/count ｜ 默认 0.02（实测安全；原 5.0 会把静止
//   漂移放大到 230counts/s）｜ 写：panel X/LOAD/DEF ｜ 读：remote 位置保持/运动计划 ｜ 100Hz。
extern float positionKp;
// MAX_POSITION_RETURN_SPEED：位置环目标回位速度限幅（面板 Y）｜ 单位 count/50ms ｜ 默认 0.30（≈0.56mm/s）｜
//   写：panel Y/LOAD/DEF ｜ 读：remote 位置保持/运动计划 ｜ 100Hz。
extern float MAX_POSITION_RETURN_SPEED;
// POSITION_DEADBAND_COUNT：位置死区（面板 M）｜ 单位 平均编码器 count ｜ 默认 12（≈1.11mm）｜
//   写：panel M/LOAD/DEF ｜ 读：remote 位置保持/运动计划 ｜ 100Hz。
extern float POSITION_DEADBAND_COUNT;
// turnKp：转向环 P 增益（面板 T）｜ 单位 Hz/deg ｜ 默认 0（转向环当前关闭）｜ 写：panel T/LOAD/DEF ｜
//   读：remote::updateTurn ｜ 400Hz。
extern float turnKp;
// turnKd：转向环 D 增益（面板 U）｜ 单位 Hz/(deg/s) ｜ 默认 0 ｜ 写：panel U/LOAD/DEF ｜
//   读：remote::updateTurn ｜ 400Hz。
extern float turnKd;
// targetForwardSpeed：速度环目标速度 ｜ 单位 count/50ms ｜ 范围 ±MAX_TARGET_FORWARD_SPEED ｜
//   写：outerLoop(SP/点动)、remote 位置环、motionDrive、故障清零 ｜ 读：outerLoop(speedError)、remote、遥测 ｜ 100Hz 级。
extern float targetForwardSpeed;
// speedCmd：SP 命令设定的目标速度（唯一速度设定入口）｜ 单位 count/50ms ｜ 范围 ±MAX_TARGET_FORWARD_SPEED ｜
//   写：panel SP/MS 命令 ｜ 读：outerLoop（^ 运动时清零）｜ 随命令更新、100Hz 检查。
extern float speedCmd;
// targetTurnRate：转向目标角速度 ｜ 单位 deg/s ｜ 当前恒 0（无写入者，转向环只保留结构）｜
//   写：无 ｜ 读：updateTurn ｜ 400Hz。
extern float targetTurnRate;
// speedError：速度环误差 = targetForwardSpeed - vehicleSpeed ｜ 单位 count/50ms ｜
//   写：outerLoop、故障清零 ｜ 读：outerLoop(P/D)、遥测第34列 ｜ 100Hz。
extern float speedError;
// speedPContribution：速度 P 项 = speedKp*speedError ｜ 单位 deg ｜ 写：outerLoop ｜
//   读：outerLoop 合成、遥测第35列 ｜ 100Hz。
extern float speedPContribution;
// speedIContribution：速度 I 项 = speedKi*speedIntegral ｜ 单位 deg ｜ 写：outerLoop ｜
//   读：outerLoop 合成、遥测第36列 ｜ 100Hz。
extern float speedIContribution;
// speedAngleUnclamped：N 限幅前的速度修正角 = dir*(P+I+D)+jogLeanOffset ｜ 单位 deg ｜
//   写：outerLoop ｜ 读：outerLoop 限幅、遥测第37列 ｜ 100Hz。
extern float speedAngleUnclamped;
// speedAngleLimited：N 限幅后、slew 前的目标角 ｜ 单位 deg，范围 ±MAX_SPEED_ANGLE ｜
//   写：outerLoop ｜ 读：outerLoop slew、遥测第38列 ｜ 100Hz。
extern float speedAngleLimited;
// speedIntegral：速度 I 积分累积 ｜ 单位 (count/50ms)·s ｜ 范围 ±SPEED_INTEGRAL_LIMIT（另有抗饱和判定）｜
//   写：outerLoop、故障/运动清零、remote 捕获位置 ｜ 读：outerLoop ｜ 100Hz。
extern float speedIntegral;
// speedAngleCorrection：速度环输出、实际叠加到目标倾角的修正量（经 0.10°/拍 slew）｜ 单位 deg ｜
//   写：outerLoop、clearSpeedLoopForMotionBypass ｜ 读：updateAuthority、remote 运动 seed、遥测第7列 ｜ 100Hz。
extern float speedAngleCorrection;
// speedErrorPrev：上一拍速度误差（微分用）｜ 单位 count/50ms ｜ 写：outerLoop、故障清零 ｜
//   读：outerLoop ｜ 100Hz。
extern float speedErrorPrev;
// speedErrorDotFilt：低通后的速度误差变化率 ｜ 单位 (count/50ms)/s ｜ 写：outerLoop（0.7/0.3 低通）｜
//   读：outerLoop 的 D 项 ｜ 100Hz。
extern float speedErrorDotFilt;
// imuFailCount：IMU 连续读取失败计数 ｜ 单位 次（0~255）｜ 写：main.cpp loop（失败++、成功清零）｜
//   读：main 停机判据、panel 参数写/点动/命令守卫 ｜ 400Hz。
extern uint8_t imuFailCount;
// fallConfirmCount：倒地确认连续帧计数 ｜ 单位 帧（0~255）｜ 写：main.cpp loop ｜
//   读：main.cpp 确认倒地（>=FALL_CONFIRM_COUNT）｜ 400Hz。
extern uint8_t fallConfirmCount;
// OUTER_FULL_ANGLE_DEG：外环全权限角（面板 F）｜ 单位 deg（相对 O）｜ 默认 5.0 ｜
//   写：panel F/LOAD/DEF ｜ 读：updateAuthority ｜ 400Hz。
extern float OUTER_FULL_ANGLE_DEG;
// OUTER_DISABLE_ANGLE_DEG：外环退出角（面板 G）｜ 单位 deg（相对 O）｜ 默认 10.0 ｜
//   写：panel G/LOAD/DEF ｜ 读：updateAuthority ｜ 400Hz。
extern float OUTER_DISABLE_ANGLE_DEG;
// outerLoopAuthority：外环权限 0~1 ｜ 无量纲（5°~10° 间线性插值）｜ 写：updateAuthority 400Hz、故障清零 ｜
//   读：outerLoop 积分门控、遥测第8列 ｜ 400Hz 更新 / 100Hz 使用。
extern float outerLoopAuthority;
// motor1LastCommandHz：Motor1 实际下发命令（限幅后；停=0）｜ 单位 Hz ｜ 写：setMotor1/stopMotor1 ｜
//   读：遥测第26列 ｜ 400Hz 写、1Hz 读。
extern float motor1LastCommandHz;
// motor2LastCommandHz：Motor2 实际下发命令（限幅后；停=0）｜ 单位 Hz ｜ 写：setMotor2/stopMotor2 ｜
//   读：遥测第27列 ｜ 400Hz 写、1Hz 读。
extern float motor2LastCommandHz;
// encoder1Count：编码器1 累计计数（volatile）｜ 单位 count（int32，按无符号回绕处理）｜
//   写：encoder1ISR 每个 A 相边沿（kHz 级）｜ 读：tick100 关中断快照、positionHold/capture、main setup/倒地重同步、
//   遥测第24列。
extern volatile int32_t encoder1Count;
// encoder2Count：编码器2 累计计数（volatile）｜ 单位 count ｜ 写：encoder2ISR ｜
//   读：同 encoder1Count（tick100/位置/main/遥测第25列）｜ kHz 级写。
extern volatile int32_t encoder2Count;
// lastEncoder1Count：上一拍 tick100 的编码器1 快照（算 10ms 增量）｜ 单位 count ｜
//   写：tick100、main setup/倒地重同步 ｜ 读：tick100 ｜ 100Hz。
extern int32_t lastEncoder1Count;
// lastEncoder2Count：上一拍 tick100 的编码器2 快照（算 10ms 增量）｜ 单位 count ｜ 同上 ｜ 100Hz。
extern int32_t lastEncoder2Count;
// encoderSpeed1：编码器1 低通速度 ｜ 单位 count/50ms（10ms 增量×5 后经 α=0.4 低通）｜
//   写/读：tick100 内部 → rawVehicleSpeed 合成 ｜ 100Hz。
extern float encoderSpeed1;
// encoderSpeed2：编码器2 低通速度 ｜ 单位 count/50ms ｜ 写/读：tick100 内部 ｜ 100Hz。
extern float encoderSpeed2;
// rawVehicleSpeed：编码器直接测得车速 ｜ 单位 count/50ms ｜ 符号：向前=正（合成时取负号，见 tick100）｜
//   写：tick100 ｜ 读：correctVehicleSpeedKalman、遥测第17列、故障清零 ｜ 100Hz。
extern float rawVehicleSpeed;
// encoder1Trusted：编码器1 信任标志 ｜ bool ｜ 写：tick100 滑窗判定（500ms）｜
//   读：tick100/位置保持/点动/遥测/命令守卫 ｜ 500ms 更新。
extern bool encoder1Trusted;
// encoder2Trusted：编码器2 信任标志 ｜ bool ｜ 写：tick100 滑窗判定 ｜ 读：同上 ｜ 500ms 更新。
extern bool encoder2Trusted;
// encoder1MotionTicks：窗内编码器1 有脉冲的拍数 ｜ 单位 拍（0~50）｜ 写/读：tick100 滑窗 ｜
//   100Hz 累加、窗满清零。
extern uint16_t encoder1MotionTicks;
// encoder2MotionTicks：窗内编码器2 有脉冲的拍数 ｜ 单位 拍（0~50）｜ 写/读：tick100 滑窗 ｜ 100Hz。
extern uint16_t encoder2MotionTicks;
// encoderTrustWindowTicks：信任判定窗已累计拍数 ｜ 单位 拍（0~50）｜ 写/读：tick100 ｜ 100Hz。
extern uint16_t encoderTrustWindowTicks;
// fallStopCount：倒地保护累计触发次数 ｜ 单位 次（uint32，只增不减）｜ 写：main.cpp loop ｜
//   读：遥测第43列 ｜ 事件计数。
extern uint32_t fallStopCount;
// rawSpeedEncoderMode：当前速度来源模式 ｜ 0=双路平均 1=仅enc1 2=仅enc2（3=都不可信，仅内部）｜
//   写：tick100 ｜ 读：位置保持/点动/遥测第40列/命令守卫 ｜ 100Hz。
extern int rawSpeedEncoderMode;
// telemGyroAbsMax：250ms 窗口内 max|gyroPitchRate| ｜ 单位 deg/s ｜ 写：diagAccumulate 400Hz、
//   sendSlow 后清零 ｜ 读：遥测第44列 ｜ 窗口 250ms。
extern float telemGyroAbsMax;
// telemAccPitchDevMax：窗口内 max|accPitch-gyroIntegAngle| ｜ 单位 deg ｜ 写：diagAccumulate ｜
//   读：遥测第45列 ｜ 窗口 250ms。
extern float telemAccPitchDevMax;
// telemBalHzAbsMax：窗口内 max|balanceOutput| ｜ 单位 Hz ｜ 写：diagAccumulate ｜ 读：遥测第46列（看饱和深度）｜ 250ms。
extern float telemBalHzAbsMax;
// telemKalmanBiasMin：窗口内 kalmanBias 最小值 ｜ 单位 deg/s ｜ 初值 1e9；sendSlow 后重置为当拍值 ｜
//   读：遥测第47列 ｜ 250ms。
extern float telemKalmanBiasMin;
// telemKalmanBiasMax：窗口内 kalmanBias 最大值 ｜ 单位 deg/s ｜ 初值 -1e9 ｜ 读：遥测第48列 ｜ 250ms。
extern float telemKalmanBiasMax;
// prevIntegratedSpeedCmdSign：上一拍 integratedMotorSpeedCommand 的符号（-1/0/1）｜ 无量纲 ｜
//   写/读：updateBalance 换向检测 ｜ 400Hz 更新、100Hz 判定。
extern int32_t prevIntegratedSpeedCmdSign;
// reversalSoftTicksRemaining：换向软窗口剩余端点拍数 ｜ 单位 100Hz 拍（0~25）｜ 写：updateBalance 检测到换向后
//   置 REVERSAL_SOFT_TICKS ｜ 读：updateBalance 每端点递减、诊断命令 ! ｜ 100Hz。
extern uint16_t reversalSoftTicksRemaining;
// vehicleSpeed：Kalman 融合车速（控制主量）｜ 单位 count/50ms，向前为正 ｜ 写：predict 400Hz + correct 100Hz ｜
//   读：outerLoop 速度误差、remote 点动/运动、遥测第16列 ｜ 400Hz 预测 / 100Hz 编码器修正。
extern float vehicleSpeed;
// telemetrySpeedSnapshot100Hz：外环时刻的 vehicleSpeed 快照（遥测第19列）｜ 单位 count/50ms ｜
//   写：outerLoop/故障清零 ｜ 读：遥测第19列 ｜ 100Hz。
extern float telemetrySpeedSnapshot100Hz;
// 速度环整体关闭时请求远控清零 jog 倾角（由 main 立即转发 remote::resetLean()）
// leanResetRequested：请求清零 jog 倾角的握手标志 ｜ bool ｜ 写：outerLoop（speedKp/Ki/Kd 全 0 时置 true）、
//   main 处理完置 false ｜ 读：main.cpp 100Hz 分支 ｜ 事件型。
extern bool leanResetRequested;
// vehicleSpeedFiltered：速度环“测量低通后”速度（历史预留）｜ 单位 count/50ms ｜ 写：outerLoop = vehicleSpeed
//   （当前直接透传，低通未启用）｜ 读：当前无控制读者 ｜ 100Hz。
extern float vehicleSpeedFiltered;
// vehiclePosition：相对开机/重新站立点的累计位移 ｜ 单位 平均编码器 count（10.8 count=1mm）｜
//   写：remote positionHold/capture（400Hz/100Hz）｜ 读：位置点动/运动计划/遥测第22列与 G 帧 ｜ 400Hz。
extern float vehiclePosition;
// positionZeroCount1：位置零点对应的编码器1 累计计数（历史记录）｜ 单位 count ｜ 写：main setup/倒地重置 ｜
//   读：当前无读者（位置环改用相对累计 vehiclePosition）｜ 开机/倒地时写。
extern int32_t positionZeroCount1;
// positionZeroCount2：位置零点对应的编码器2 累计计数（历史记录）｜ 单位 count ｜ 写：main setup/倒地重置 ｜
//   读：当前无读者 ｜ 开机/倒地时写。
extern int32_t positionZeroCount2;
// targetVehiclePosition：驻车/点动目标位置 ｜ 单位 平均编码器 count ｜ 写：remote 点动/位置保持/面板 C 捕获 ｜
//   读：remote 位置误差、遥测 ｜ 100Hz 级。
extern float targetVehiclePosition;
// positionError：位置误差 = targetVehiclePosition - vehiclePosition ｜ 单位 count ｜
//   写：remote 点动/位置保持、main 故障清零、clearOutputDiagnostics ｜ 读：位置 P 外环、遥测第23列 ｜
//   400Hz 更新 / 100Hz 使用。
extern float positionError;
// positionSpeedTarget：位置环生成的目标速度 ｜ 单位 count/50ms ｜ 写：remote 位置保持/运动计划、
//   outerLoop(点动)、故障清零 ｜ 读：遥测第11列、remote/main ｜ 100Hz。
extern float positionSpeedTarget;
// positionHoldWasEnabled：位置保持上一拍是否启用（上升沿触发捕获目标）｜ bool ｜ 写：remote 位置保持、
//   面板 X/点动 ｜ 读：remote/遥测 manualControlMode ｜ 400Hz/100Hz。
extern bool positionHoldWasEnabled;
// positionHoldSuspended：位置保持暂停标志（手动/运动接管时禁止累积旧目标误差）｜ bool ｜
//   写：remote positionHold、resetFault ｜ 读：remote 位置保持/面板 X/遥测 ｜ 400Hz。
extern bool positionHoldSuspended;
// imuForwardAccelG：IMU 前向线加速度（已去重力趋势）｜ 单位 g，向前为正，尖峰限幅 ±0.60 ｜
//   写：updateAttitude 400Hz ｜ 读：predictVehicleSpeedKalman、故障清零 ｜ 400Hz。
extern float imuForwardAccelG;
// imuForwardAccelFilteredG：前向加速度低通值（α=0.05，τ≈51ms）｜ 单位 g ｜ 写：updateAttitude 400Hz ｜
//   读：predictVehicleSpeedKalman ｜ 400Hz。
extern float imuForwardAccelFilteredG;
// IMU_ACCEL_TO_SPEED_GAIN：加速度→速度预测增益 ｜ 单位 (count/50ms)/(g·s) ｜ 3.0：1g 持续 1s 只贡献
//   3 count/50ms（≈5.6mm/s），故意远小于物理真值——此路只补短期趋势，绝对速度由编码器修正｜ 写：无 ｜ 读：predict。
extern float IMU_ACCEL_TO_SPEED_GAIN;
// SPEED_KALMAN_Q_BASE：速度 Kalman 过程噪声基值 ｜ 单位 (count/50ms)² ｜ 0.20 ｜ 写：无 ｜
//   读：predict（每拍 Q=基值+电机增益×|balanceOutput|）｜ 400Hz。
extern float SPEED_KALMAN_Q_BASE;
// SPEED_KALMAN_Q_MOTOR_GAIN：过程噪声随电机命令增长系数 ｜ 单位 (count/50ms)²/Hz ｜ 0.0008 ｜
//   读：predict；命令越大越允许状态快速变化（不是把命令当速度）｜ 400Hz。
extern float SPEED_KALMAN_Q_MOTOR_GAIN;
// SPEED_KALMAN_R_LOW_SPEED：低速测量噪声 ｜ 单位 (count/50ms)² ｜ 0.20（|编码器速度|<2 时用，容忍 0/1 count 量化）｜
//   读：correctVehicleSpeedKalman ｜ 100Hz。
extern float SPEED_KALMAN_R_LOW_SPEED;
// SPEED_KALMAN_R_NORMAL：常规测量噪声 ｜ 单位 (count/50ms)² ｜ 0.08（|编码器速度|>=2 时用）｜ 读：correct ｜ 100Hz。
extern float SPEED_KALMAN_R_NORMAL;
// speedKalmanP：速度估计协方差 P ｜ 单位 (count/50ms)² ｜ 初值 1.0，下限 0.02 ｜ 写/读：predict（P+=Q·dt）、
//   correct（K=P/(P+R)）｜ 400Hz/100Hz。
extern float speedKalmanP;
// speedKalmanQ：当前过程噪声 ｜ 单位 (count/50ms)² ｜ 每拍按 |balanceOutput| 重算 ｜ 写/读：predict ｜ 400Hz。
extern float speedKalmanQ;
// speedKalmanR：当前测量噪声 ｜ 单位 (count/50ms)² ｜ 0.08/0.20 二选一 ｜ 写/读：correct ｜ 100Hz。
extern float speedKalmanR;
// imuSpeedPredictionTrust：IMU 预测可信度 ｜ 无量纲 ｜ 1.0/0.45/0.12 三档（按加速度模长误差分档）｜
//   写：updateAttitude 400Hz ｜ 读：predict、main 初始化 ｜ 400Hz。
extern float imuSpeedPredictionTrust;
// ax：加速度计 X 原始读数 ｜ 单位 g（±2g 量程）｜ 写：readBMI270 400Hz ｜ 读：updateAttitude/calibrateIMU。
extern float ax;
// ay：加速度计 Y 原始读数（readBMI270 里取反 -data.accelY）｜ 单位 g ｜ 同上 ｜ 400Hz。
extern float ay;
// az：加速度计 Z 原始读数（readBMI270 里取反 -data.accelZ）｜ 单位 g ｜ 同上 ｜ 400Hz。
extern float az;
// gx：陀螺 X 原始角速度（=转向轴）｜ 单位 deg/s ｜ 写：readBMI270 400Hz ｜ 读：updateAttitude(gyroTurnRate)/calibrateIMU。
extern float gx;
// gy：陀螺 Y 原始角速度（=俯仰轴，readBMI270 里取反）｜ 单位 deg/s ｜ 写：readBMI270 ｜ 读：updateAttitude。
extern float gy;
// gz：陀螺 Z 原始角速度（偏航轴，readBMI270 里取反）｜ 单位 deg/s ｜ 写：readBMI270 ｜ 读：当前控制不用（仅保留原始值）。
extern float gz;
// pitchZero：俯仰安装零点偏置 ｜ 单位 deg ｜ 初值 PITCH_MOUNT_OFFSET_DEG ｜ 写：calibrateIMU 重置 ｜
//   读：updateAttitude 算 accPitch=加速度角-pitchZero ｜ 开机一次。
extern float pitchZero;
// gyroYZero：俯仰轴陀螺零偏 ｜ 单位 deg/s ｜ 写：calibrateIMU（静止 400 样本均值）｜
//   读：updateAttitude 算 gyroPitchRate ｜ 开机一次。
extern float gyroYZero;
// gyroXZero：转向轴陀螺零偏 ｜ 单位 deg/s ｜ 写：calibrateIMU ｜ 读：updateAttitude 算 gyroTurnRate ｜ 开机一次。
extern float gyroXZero;
// accPitch：加速度计解算倾角（已减 pitchZero）｜ 单位 deg ｜ 写：updateAttitude 400Hz ｜ 读：updateKalman 测量、
//   遥测第28列、diag ｜ 400Hz。
extern float accPitch;
// pitch：Kalman 融合倾角（控制主量，含毛刺抑制）｜ 单位 deg ｜ 写：updateAttitude 400Hz ｜
//   读：直立环/外环/倒地判据/手动/遥测 ｜ 400Hz。
extern float pitch;
// gyroPitchRate：俯仰角速度（已减零偏、已定向）｜ 单位 deg/s ｜ 写：updateAttitude 400Hz ｜
//   读：直立 D 项、updateKalman、遥测/diag ｜ 400Hz。
extern float gyroPitchRate;
// gyroTurnRate：转向角速度（gx 减零偏后定向）｜ 单位 deg/s ｜ 写：updateAttitude 400Hz ｜
//   读：updateTurn 的 legacy PD、遥测 ｜ 400Hz。
extern float gyroTurnRate;
// gyroIntegAngle：陀螺积分角（去 kalmanBias 后在 400Hz 积分）｜ 单位 deg ｜ 写：updateAttitude 每拍累加 ｜
//   读：updateAttitude 算加速计分歧、遥测第30列 ｜ 400Hz。
extern float gyroIntegAngle;
// kalmanAngle：Kalman 内部角度状态（原始输出）｜ 单位 deg ｜ 写：updateKalman、calibrateIMU 初始化 ｜
//   读：updateKalman 返回值；控制用 pitch（毛刺抑制后）｜ 400Hz。
extern float kalmanAngle;
// compAngle：互补滤波角度残留状态 ｜ 单位 deg ｜ 写：calibrateIMU 初始化 ｜ 读：无（互补滤波已删除，仅历史保留）。
extern float compAngle;
// pitchPrev：上一有效 pitch（毛刺抑制基准）｜ 单位 deg ｜ 写/读：updateAttitude 毛刺判定 ｜ 400Hz。
extern float pitchPrev;
// kalmanBias：Kalman 估计的陀螺零偏 ｜ 单位 deg/s ｜ 写：updateKalman 每拍修正 ｜ 读：updateAttitude 积分/分歧、
//   遥测第29列、diag ｜ 400Hz。
extern float kalmanBias;
// P00：角度估计方差 ｜ 单位 deg² ｜ 初值 1.0 ｜ 写/读：updateKalman 预测+修正、calibrateIMU 初始化 ｜ 400Hz。
extern float P00;
// P01：角度-零偏协方差 ｜ 单位 deg·(deg/s) ｜ 初值 0 ｜ 写/读：updateKalman ｜ 400Hz。
extern float P01;
// P10：零偏-角度协方差 ｜ 单位 (deg/s)·deg ｜ 初值 0 ｜ 写/读：updateKalman（结构上 P10=P01）｜ 400Hz。
extern float P10;
// P11：零偏估计方差 ｜ 单位 (deg/s)² ｜ 初值 1.0 ｜ 写/读：updateKalman ｜ 400Hz。
extern float P11;
// Q_angle：角度过程噪声 ｜ 单位 deg²/s ｜ 0.01 ｜ 写：无（常量）｜ 读：updateKalman 协方差预测 ｜ 400Hz。
extern float Q_angle;
// Q_bias：零偏过程噪声 ｜ 单位 (deg/s)²/s ｜ 0.03 ｜ 写：无（常量）｜ 读：updateKalman ｜ 400Hz。
extern float Q_bias;
// accelMagnitude：三轴加速度模长 ｜ 单位 g ｜ 静止≈1.0 ｜ 写：updateAttitude ｜ 读：算 accelMagnitudeError。
extern float accelMagnitude;
// accelMagnitudeError：| |a|-1g | ｜ 单位 g ｜ 写：updateAttitude ｜ 读：dynamicR/信任分档、遥测第42列 ｜ 400Hz。
extern float accelMagnitudeError;
// dynamicR：加速度角动态测量噪声 ｜ 单位 deg² ｜ 0.02~1.20 连续插值 ｜ 写：updateAttitude 400Hz ｜
//   读：updateKalman（S=P00+R）、遥测第41列 ｜ 400Hz。
extern float dynamicR;
// targetBalanceAngle：直立环目标倾角 = O + authority*(normalAuthority*speedAngleCorrection+leanOffset) ｜
//   单位 deg ｜ 写：updateAuthority 400Hz、setup/故障清零 ｜ 读：updateBalance、手动 seed、遥测第3列 ｜ 400Hz。
extern float targetBalanceAngle;
// balanceOutput：直立 PD 原始输出（P+D）｜ 单位 Hz（等效）｜ 写：updateBalance 400Hz ｜
//   读：速度预测 Q、遥测第6列、diag ｜ 400Hz。
extern float balanceOutput;
// balancePOutput：直立 P 项 ｜ 单位 Hz ｜ 写：updateBalance 400Hz ｜ 读：遥测第4列 ｜ 400Hz。
extern float balancePOutput;
// balanceDOutput：直立 D 项 ｜ 单位 Hz ｜ 写：updateBalance 400Hz ｜ 读：遥测第5列 ｜ 400Hz。
extern float balanceDOutput;
// turnOutput：转向环输出（左右轮差动 Hz）｜ 单位 Hz ｜ 写：updateTurn 400Hz、故障清零 ｜
//   读：mixMotors、遥测第15列 ｜ 400Hz。
extern float turnOutput;
// finalBalanceOutput：直立最终前后输出（已限幅）｜ 单位 Hz ｜ 写：updateBalance 400Hz、故障清零 ｜
//   读：mixMotors、转向环限幅计算、遥测第14列 ｜ 400Hz。
extern float finalBalanceOutput;
// mixedBalanceOutput：限幅前输出（增量积分后）｜ 单位 Hz ｜ 写：updateBalance ｜ 读：限幅、遥测第12列 ｜ 400Hz。
extern float mixedBalanceOutput;
// limitedBalanceOutput：总限幅后输出 ｜ 单位 Hz ｜ 写：updateBalance ｜ 读：finalBalanceOutput 赋值、遥测第13列 ｜ 400Hz。
extern float limitedBalanceOutput;
// balanceAccelRawOutput：100Hz 端点未限幅频率增量 ｜ 单位 Hz/update ｜ 写：updateBalance ｜
//   读：限幅计算、遥测第31列 ｜ 100Hz。
extern float balanceAccelRawOutput;
// balanceAccelLimitedOutput：S/H/软窗口限幅后的频率增量 ｜ 单位 Hz/update ｜ 写：updateBalance ｜
//   读：均分小步、遥测第32列 ｜ 100Hz。
extern float balanceAccelLimitedOutput;
// balanceAccelEffectiveLimit：本拍实际增量限幅 ｜ 单位 Hz/update ｜ 写：updateBalance ｜ 读：限幅、遥测第39列 ｜ 100Hz。
extern float balanceAccelEffectiveLimit;
// highSpeedAccelProtectionActive：高速加速保护本次是否生效 ｜ bool ｜ 写：updateBalance ｜
//   读：当前无读者（诊断保留）｜ 100Hz。
extern bool highSpeedAccelProtectionActive;
// balanceAccelSubstepOutput：端点增量均分到 4 个 400Hz 小步的值 ｜ 单位 Hz/2.5ms ｜ 写/读：updateBalance ｜ 400Hz。
extern float balanceAccelSubstepOutput;
// integratedMotorSpeedCommand：增量积分得到的 STEP 目标频率 ｜ 单位 Hz ｜ 范围 ±BALANCE_OUTPUT_LIMIT_HZ ｜
//   写：updateBalance 400Hz ｜ 读：加速保护判据、遥测第33列 ｜ 400Hz。
extern float integratedMotorSpeedCommand;
// balanceAccelDivider：400Hz→100Hz 分频计数 ｜ 0~3 ｜ 写/读：updateBalance ｜ 400Hz。
extern uint8_t balanceAccelDivider;
// lastTurnError：转向环上一拍误差（微分用）｜ 单位 deg/s ｜ 写/读：updateTurn ｜ 400Hz。
extern float lastTurnError;
// lastControlUs：上一控制拍时间戳 ｜ 单位 µs（micros() 约 71min 回绕，用无符号差）｜
//   写/读：main.cpp loop 定拍 ｜ 400Hz。
extern uint32_t lastControlUs;
// controlOverrunCount：掉拍累计次数（控制间隔 >=2 个周期）｜ 单位 次 ｜ 写：main.cpp ｜
//   读：panel diag/命令 ! ｜ 事件计数。
extern uint32_t controlOverrunCount;
// controlMaxGapUs：观测到的最大控制间隔 ｜ 单位 µs ｜ 写：main.cpp ｜ 读：panel diag/! ｜ 事件计数。
extern uint32_t controlMaxGapUs;
// controlMaxWorkUs：单拍最大控制耗时 ｜ 单位 µs ｜ 写：main.cpp loop 末尾 ｜ 读：panel diag/! ｜ 事件计数。
extern uint32_t controlMaxWorkUs;
// outerLoopDivider：400Hz→100Hz 外环分频计数 ｜ 0~3 ｜ 写/读：main.cpp loop ｜ 400Hz。
extern uint32_t outerLoopDivider;
// telemetryDivider：慢帧分频计数 ｜ 0~399 ｜ 写/读：main.cpp loop ｜ 400Hz。
extern uint32_t telemetryDivider;
// fastTelemetryDivider：快帧分频计数 ｜ 0~99 ｜ 写/读：main.cpp loop ｜ 400Hz。
extern uint32_t fastTelemetryDivider;
// btRxBytes：蓝牙累计接收字节数 ｜ 单位 字节（uint32 回绕）｜ 写：panel::poll ｜ 读：遥测 G 帧第8列 ｜ 每圈轮询。
extern uint32_t btRxBytes;
