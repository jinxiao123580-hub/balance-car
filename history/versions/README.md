# 历史版本与试验资料归档说明

| 属性 | 内容 |
| --- | --- |
| 文档编号 | PH-ARCHIVE-INDEX |
| 更新日期 | 2026-10-28，Asia/Shanghai；时分未记录 |

## 归档内容

| 目录 | 内容与状态 |
| --- | --- |
| firmware-snapshots | 16个历史固件备份，以.txt结尾防止误编译 |
| panel-snapshots | 两份历史面板版本 |
| imported-history | 规范化历史试验和阶段状态报告 |
| diagnostic-output | 当时诊断输出及辅助脚本，未保证适配当前协议 |
| maintenance-scripts | 可能直接改写源码的历史脚本，以.py.txt留存 |

## 版本边界

当前主程序位于software/firmware，面板位于software/ble-panel。历史地址、参数和字段定义不直接用于当前版本。技术差异见[勘误报告](../reports/conflicts.md)。改写前文档及旧原件保存在仓库外中转区。
