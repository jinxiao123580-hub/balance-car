# 当前版本构建与运行

更新日期：2026-10-28，Asia/Shanghai。当前固件、面板及硬件资料位于主目录；历史资料统一位于history。

## 1. 固件构建

在VS Code / PlatformIO中打开software/firmware，或从仓库根执行：

```powershell
cd software/firmware
pio run -e black_f407zg
```

Windows也可双击build-firmware.bat。烧录使用ST-Link：

```powershell
pio run -e black_f407zg -t upload
```

本轮结构整理未修改主程序、控制参数或重新烧录。[默认参数](docs/parameters.md)

## 2. 蓝牙面板

本机双击open-panel.bat。新电脑安装：

```powershell
cd software/ble-panel
py -3 -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\ble_tune_panel.bat
```

device.local.bat保存可选的本机地址配置，由Git忽略。无地址配置时，面板按服务和设备名扫描。面板输入、设备回显和实际下发分别核对。

新录制默认写入docs/validation/recordings。截图docs/images/ble-panel-offline.png为离线界面，不代表已连接实车。

## 3. 数据分析

当前维护工具位于software/analysis。历史数据样例：

```powershell
python software/analysis/analyze_ble_log.py history/data/raw/ble_tune_log_20261020_104252_893_p44156.csv
```

专题旧脚本在history/software/analysis。历史文件保留其条件和协议版本，分析前核对时间回退、参数变化和人工操作。

## 4. 硬件与机械

当前归集源文件在hardware和mechanical。引脚和实现见[系统说明](docs/system-overview.md)。BOM、Gerber、加工图与完整装配依赖尚待核验，软件可构建不代表硬件资料已经生产审查。

## 5. 历史与原件

过往报告、CSV、独立试验、旧程序和早期演示在[history](history/README.md)。独立TMC读回工程不是当前主控制路径，也不表示曾成功读回。

旧位置原件留在仓库外中转区，未永久删除；新工程无需引用中转区的源码或运行环境。当前版本验证状态见[验证说明](docs/validation/README.md)。
