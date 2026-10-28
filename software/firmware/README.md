# 当前主固件

更新日期：2026-10-28，Asia/Shanghai。

完整PlatformIO主工程位于本目录，环境black_f407zg。执行pio run -e black_f407zg构建；ST-Link烧录命令为pio run -e black_f407zg -t upload。

当前main.cpp保持迁移前字节和控制参数；新路径独立构建及主机运动回归通过。本轮目录调整未烧录。

- [系统与引脚](../../docs/system-overview.md)
- [默认参数](../../docs/parameters.md)
- [验证状态](../../docs/validation/README.md)

历史固件快照位于history/versions/firmware-snapshots；独立UART等旧试验位于history/experiments，不是主控制程序的附加启动步骤。
