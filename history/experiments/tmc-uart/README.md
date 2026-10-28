# TMC2209寄存器读回试验说明

| 属性 | 内容 |
| --- | --- |
| 文档编号 | PH-TMC-UART-TEST |
| 更新日期 | 2026-10-28，Asia/Shanghai；时分未记录 |

## 工程组成

完整src和platformio.ini，目标black_f407zg，独立于主平衡固件。源码设计为读取version、IFCNT与GCONF等信息，不输出STEP。

## 验证状态

文件已归集并校验。过去尚未留下成功读回数据，本轮未烧录该工程；源码中的目标条件不代表已通过。
