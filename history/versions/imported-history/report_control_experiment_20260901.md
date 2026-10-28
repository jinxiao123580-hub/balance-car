# 直立输出缩放与速度直接叠加试验报告

| 属性 | 内容 |
| --- | --- |
| 报告编号 | PH-EXP-20260901 |
| 记录时间 | 2026-09-01 15:38，来源为变更文件时间 |
| 规范化日期 | 2026-10-28，Asia/Shanghai |
| 验证等级 | 编译通过，烧录失败，实机效果未验证 |

## 1. 试验目标

评估将直立环输出缩放至0.7，并将速度/位置直接项叠加到轮速的控制方案。

## 2. 实施内容

新增verticalLoopGain=0.70，目标倾角保持balanceAngleOffset，混合输出为：

```text
finalBalanceOutput =
    verticalLoopGain * balanceOutput
    + outerLoopAuthority * (speedDirectOutput + positionDirectOutput)
```

蓝牙R命令在该阶段表示直立缩放系数，面板加入相应参数；后期R改为频率增量比例，两个版本不能混用。

备份文件名main.cpp.bak_vertical_loop_0.7_20260901，现归档于固件快照目录。

## 3. 验证结果

固件编译成功：RAM4012B、Flash58288B；面板语法检查通过。两次烧录出现OpenOCD open failed，当时未完成写入，不能据编译结果判定实机效果。接口枚举或连接异常为当时排查方向，未形成独立硬件测量结论。

## 4. 后续演进

后期主链回到速度环修正目标角及频率增量积分结构。该方案何时首次实机运行、是否执行过单独对照，资料不足，保持未验证状态。
