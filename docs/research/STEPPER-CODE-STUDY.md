# 别人的步进平衡车代码研究（可运行工程，带源码摘录）

> 目的：不看综述，直接读**能跑的代码**，找出"别人做了、我们没做"的具体实现。
> 所有摘录均来自公开仓库（URL 见每节标题），仅摘关键控制段。

---

## 1. Wangshucai/BalanceCar（STM32F103 + 步进 + MPU6050 DMP）

仓库：https://github.com/Wangshucai/BalanceCar
文件：`BalanceCar/STM32BalanceCar/HARDWORD/CONTROL/control.c`

### 参数
```c
float Balance_Kp=350, Balance_Kd=0, Velocity_Kp=40, Velocity_Ki=0.7;   // PID参数
```

### 中断里的串级结构（直立 → 速度 → 转向 → 电机）
```c
Angle_PWM    = balance(Angle_Balance, Gyro_Balance);   // 直立环（P 用角度，D 用陀螺）
velocity_PWM = velocity(Balance_PWM, Balance_PWM);     // 速度环，输入是"上一个直立输出"
Turn_PWM     = turn(...);
Balance_PWM  = Angle_PWM + velocity_PWM;               // ★ 速度环输出直接叠加到电机 PWM
Moto1 = Balance_PWM - Turn_PWM;
Moto2 = Balance_PWM + Turn_PWM;
```

### 直立环
```c
int balance(float Angle, float Gyro)
{
  float Bias = Angle - ZHONGZHI;          // 目标角与实测角偏差
  return Balance_Kp*Bias + Balance_Kd*Gyro;
}
```

### 速度环（关键）
```c
int velocity(int velocity_left, int velocity_right)
{
  static float Encoder_Last, Encoder, Velocity, Movement;
  static float Encoder_Integral;

  Encoder = Mean_Filter(velocity_left, velocity_right);   // 左右均值 + 一阶滤波
  Encoder *= 0.7;
  Encoder += Encoder_Last*0.3;
  Encoder_Last = Encoder;

  Encoder_Integral += Encoder;        // ★ 积分出位移
  Encoder_Integral += Movement;       // ★ 遥控前进/后退直接加进积分（Movement=±10000）

  if(Encoder_Integral> 210000) Encoder_Integral= 210000;   // 积分限幅
  if(Encoder_Integral<-210000) Encoder_Integral=-210000;

  Velocity = Encoder*Velocity_Kp/100 + Encoder_Integral*Velocity_Ki/100;  // 纯 PI
  return Velocity;
}
```

**要点**
- 速度环是 **PI**，且积分项承担"顶住摩擦、维持巡航"的全部职责；
- 速度环输出**直接加在电机 PWM 上**，不是去改目标角；
- 前进指令进的是**积分器**，所以遥控前进 = 给积分器一个偏置，车会持续加速到目标速度。

---

## 2. kamilpinas12/Self_Balancing_Robot（STM32 + **TMC2209** + NEMA17，LQR）

硬件与我们最接近（TMC2209 + 42 步进）。仓库：https://github.com/kamilpinas12/Self_Balancing_Robot
文件：`robot_stm/Core/Src/robot.c`

### 5 个 LQR 增益
```c
robot.K1 =  2.013;    // 角度
robot.K2 =  0.2526;   // 角速度
robot.K3 = -1.0544;   // 位置误差
robot.K4 = -0.7421;   // 速度
robot.K5 = -0.1536;   // 位置积分
robot.set_angle = -0.06;
```

### 控制步（一次算完全部状态）
```c
robot.d_angle = (mpu.x_angle - robot.prev_angle) / TIME_DELTA;
robot.d_angle = filter(&robot.d_angle_filter, -mpu.gy*DEG2RAD);

robot.pos = stepper1.step_counter * STEP2METERS;          // ★ 位置直接用"已发步数"
robot.d_pos = filter(&robot.d_pos_filter, robot.speed*WHEEL_RADIUS);
robot.pos_error = robot.set_pos - robot.pos;

if(robot.controller_connected){
  robot.set_pos = robot.pos + robot.set_speed*0.05;       // ★★ 速度指令 → 位置斜坡
  robot.pos_int = 0;
}
robot.pos_int -= robot.pos_error * TIME_DELTA;

saturation(-0.35, 0.35, &robot.pos_error);                // 位置误差限幅 ±0.35 m

robot.speed_delta = (robot.set_angle - mpu.x_angle) * robot.K1
                  - robot.d_angle * robot.K2
                  + robot.pos_error * robot.K3
                  + (robot.set_speed*0 - robot.d_pos) * robot.K4
                  - robot.pos_int * robot.K5;

saturation(-0.3, 0.3, &robot.speed_delta);
robot.speed += robot.speed_delta * METERS2RAD;
stepper_set_speed(&stepper1, robot.speed - robot.set_rotate_speed);
stepper_set_speed(&stepper2, robot.speed + robot.set_rotate_speed);
```

**要点**
- **速度指令并不是直接进速度环，而是变成"移动的位置目标"**：`set_pos = pos + v*0.05`；
- 然后由位置误差/位置积分去产生持续倾角 → 天然能巡航，且不依赖"速度误差×K"；
- **位置反馈来自 stepper 的 step_counter**（不需要编码器）；
- 输出限幅在**速度增量**上（±0.3 m/s），角度本身不设 0.8° 死限。

---

## 3. kairobyte/bipedal_self_balance（步进 + 舵机腿，角度环 PID）

仓库：https://github.com/kairobyte/bipedal_self_balance
文件：`3. Code/2. Firmware/self_balancing_firmware/src/main.cpp`

```c
error = angle - SET_POINT;
error = ke*error + (1-ke)*lastError;      // 一阶低通
if (fabs(error) < DEADBAND) error = 0;    // 死区

integral += error*dt;  integral = constrain(integral, -50, 50);
pTerm = kp*error + kp_boost*error*error*sign;   // P + 非线性平方项

command = pTerm + ki*integral - kd*derivative;

if (error != 0 && fabs(command) > 0 && fabs(command) < MIN_SPEED) {
    command = (command > 0) ? MIN_SPEED : -MIN_SPEED;   // ★★ 最小速度（静摩擦补偿）
} else if (error == 0) {
    command = 0;
}
command = command*kc + (1-kc)*lastCommand;              // 输出低通
command = constrain(command, -MAX_COMMAND, MAX_COMMAND);
setStepperSpeed((int)command);
```

**要点**
- **最小速度钳位**：算出来只要不为零，就至少给 `MIN_SPEED` —— 这是步进+静摩擦场景的"死区补偿"；
- 死区 `< DEADBAND` 时**直接清零**（避免在 0 附近来回抖）；
- 输出低通，防步进丢步/共振。

---

## 4. lgitrt/BalancingRobot（STM32G474 + A4988 + AS5600，LQR 与 PID 双实现）

仓库：https://github.com/lgitrt/BalancingRobot
文件：`STM32Code/BalancingRobot/Core/Src/controller.c`

```c
/* 速度指令 → 位置参考积分 ★ */
reference_position_m += in->requested_speed_m_s * dt;
float position_offset = in->position_m - reference_position_m;

/* LQR：位置误差进状态反馈 */
float acceleration = -(K_POSITION*position_error + K_VELOCITY*velocity_error
                     + K_PITCH*pitch_error + K_PITCH_RATE*pitch_rate);
/* 位置积分：持续的位置误差说明平衡点偏了 */
if (position_hold && fabsf(acceleration) < MAX_ACCEL)
    position_trim_rad = clamp(position_trim_rad - K_INTEGRAL*position_error*dt, ...);

/* PID 版：位置环输出叠加到速度目标，并且"正反馈"减速 */
float position_correction = clamp(-KP*position_offset, MAX_SPEED);
float velocity_error = velocity - (requested_speed + position_correction);
acceleration = KP_PITCH*pitch_error + KI_PITCH*integral + KD_PITCH*pitch_rate
             + KP_VELOCITY*velocity_error;
if (fabsf(acceleration) < MAX_ACCEL) pitch_integral = next_integral;  // 抗饱和
```

**要点**
- 速度指令统一变成**位置参考**（`reference_position += v*dt`）；
- 积分只在"未饱和"时累积（抗饱和）；
- 注释明确指出：**减速要靠轮子先移到重心下方**（正反馈语义），这正是我们 SP 极性反复踩的坑。

---

## 5. 四个工程的共同点（这就是我们缺的东西）

| 共同点 | Wangshucai | kamilpinas | kairobyte | lgitrt | 我们 |
|---|---|---|---|---|---|
| 速度/位置指令 → **积分或位置斜坡** | ✅ 加进积分器 | ✅ `set_pos=pos+v*0.05` | —（角度环 PI） | ✅ `ref_pos+=v*dt` | ❌ 直接 `K*err` |
| 速度环**带积分** | ✅ Ki=0.7 | ✅ K5 位置积分 | ✅ ki | ✅ | ❌ I=0 |
| **最小速度/死区补偿** | — | — | ✅ `MIN_SPEED` | — | ❌ 无 |
| 限幅在**电机输出/速度增量** | ✅ ±5000 PWM | ✅ ±0.3 m/s | ✅ ±MAX_COMMAND | ✅ ±MAX_ACCEL | ⚠️ 在"目标角修正"上 `N=0.8°` |
| 位置反馈来源 | 编码器 | **步数**（无编码器） | — | AS5600 | 编码器 |

**一句话**：四个能跑的工程里，**没有一个**是"速度误差 × P 直接变成倾角偏置、然后被 0.8° 卡死"的结构。
它们要么把速度指令积成位置参考（kamilpinas/lgitrt），要么让积分器扛住摩擦（Wangshucai），要么给输出加最小速度（kairobyte）。

**我们的结构缺了"能把速度指令变成持续倾角"的那一层**，所以只可能"点动"。

---

## 6. 照抄到我们代码里的三个补丁（按改动量排序）

### 补丁 A：输出最小速度（改 1 个函数，5 行）
文件：`software/firmware/src/control/motor.cpp` 的 `setMotor1/setMotor2` 或 `mixMotors` 之后
```c
// 步进 + 静摩擦：算出来的输出只要不为 0，就至少给 MIN_HZ（仿 kairobyte MIN_SPEED）
static const float MIN_HZ = 60.0f;      // 待实测：能带动但不抖
if (cmd != 0.0f && fabsf(cmd) < MIN_HZ) cmd = (cmd > 0 ? MIN_HZ : -MIN_HZ);
```
- 预期：小 SP 不再"只抖一下"，点动变连续爬行；
- 验证：`SP 8`，`--max-travel 25`，看位移与稳态速度；
- 风险：`MIN_HZ` 过大 → 低速抖动/爬行停不下来，从 30Hz 起试。

### 补丁 B：速度环加积分（改 1 个文件）
文件：`software/firmware/src/control/cascade.cpp` 的 `control::outerLoop()`
```c
// 结构已有（speedKi/speedIntegral/条件积分/抗饱和都在），只需放开 I 并给初值
// 仿 Wangshucai：速度环 = P*err + I*∫err，输出仍走 N 限幅
K = -0.02~-0.03;   // 先按 N/|K| ≥ 满速误差 选 K（30 count/50ms → |K|≤0.027）
I = 0.2~0.5;       // 从 P 的 10~20% 起（ROCKO 说最多可到 80%）
```
- 预期：稳态不再归零，能维持一个小巡航速度；
- 风险：I 会重新引入极限环（我们 11-19 踩过），**必须先配合补丁 A 和更小的 K**，并在非饱和区验证。

### 补丁 C：速度指令改位置斜坡（结构级，仿 kamilpinas）
文件：`cascade.cpp` + `speed_est.cpp`
```c
// 每 10ms：
targetPosition += speedCmd / 50.0f;                 // speedCmd 单位 count/50ms → count
float velCmd = positionKp * (targetPosition - vehiclePosition);   // 位置→速度
velCmd = clamp(velCmd, -MAX_VEL_CMD, MAX_VEL_CMD);
speedError = velCmd - vehicleSpeed;
```
- 预期：连续巡航、可停在指定位置；这正是我们 `X/Y/M` 位置环已经有雏形但没启用的东西；
- 风险：位置环超调、目标位置积分漂移；需要限幅与死区。

---

## 7. 出处

- https://github.com/Wangshucai/BalanceCar （STM32F103 步进平衡车，control.c）
- https://github.com/kamilpinas12/Self_Balancing_Robot （STM32 + TMC2209 + NEMA17，robot.c）
- https://github.com/kairobyte/bipedal_self_balance （步进，main.cpp 含 MIN_SPEED）
- https://github.com/lgitrt/BalancingRobot （STM32G474 + A4988 + AS5600，controller.c）
- https://mc.dfrobot.com.cn/article-375358.html （Arduino Nano + A4988 步进自平衡，DFRobot 转载）
- https://github.com/Bulduper/BTW-Robot （ESP8266 + 步进 + MPU6050，串级）
