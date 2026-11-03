# IMU：Bosch BMI270

## 型号与来源

- Bosch Sensortec **BMI270**，6 轴（3 轴加速度 + 3 轴陀螺）。
- 数据手册：`docs/references/vendor/BMI270/icoss3867257160f4fe7265cfd94e2b.pdf`，文档号 **BST-BMI270-DS000-05**，Rev 1.3，2020-11。
- 商品页截图：`docs/references/vendor/BMI270/商品页详情.png`。

## 关键参数

| 项目 | 值 | 来源 |
| --- | --- | --- |
| 加速度量程 | ±2 / ±4 / ±8 / ±16 g | 数据手册 p2 |
| 陀螺量程 | ±125 / ±250 / ±500 / ±1000 / ±2000 dps | 数据手册 p2 |
| 输出位宽 | 16 bit | 数据手册 p2 |
| 接口 | I2C / SPI（本机用 I2C） | 数据手册；固件 `Wire` |
| 本项目采样 | 400 Hz（ACC + GYRO 各 400 Hz），Kalman 解算 pitch + 陀螺零偏 | 固件头部注释 |

## 接线与引脚

- SCL = PB8，SDA = PB9（`main.cpp` 宏 `BMI_SCL_PIN` / `BMI_SDA_PIN`）。
- 安装方向：竖装；相对历史 MPU6050 横装绕 X 轴旋转 180°，Y/Z 在 `readBMI270()` 中同时翻转（固件注释，2026-09 已验证，勿随意改）。

## 实测记录

- 2026-09：改用 BMI270 后，姿态解算与直立环正常工作；未见 I2C 掉线记录。
- 固件对加速度模长做动态 R 调整（`accelMagErr`、`dynamicRx` 遥测列），抑制运动/振动时的姿态抖动。

## 已知问题

- 手册 Rev 1.3 的寄存器描述与 SparkFun 库封装存在版本差，改配置时以库的 `setAccelRange` 等接口为准，别直接照抄手册地址。
- 陀螺零偏随温度漂移，固件用 Kalman 在线估计（`kalmanBias` 遥测列）。

## 待验证

- 实际量程配置（±g / ±dps）与 ODR 寄存器值未在文档中记录，建议从固件初始化代码抄一份到这里。
- I2C 地址（0x68 / 0x69）未记录。
