# 当前数据分析工具

更新日期：2026-10-28，Asia/Shanghai。

保留维护的工具：analyze_ble_log.py、cpr_estimate.py、binned_ratio.py、verify_trust_gate.py。其他专题旧脚本已归入history/software/analysis。

主分析工具支持显式传入CSV；需要默认历史路径的工具读取history/data/raw。新录制位于docs/validation/recordings，不覆盖旧记录。

```powershell
python software/analysis/analyze_ble_log.py history/data/raw/ble_tune_log_20261020_104252_893_p44156.csv
```

不同遥测版本、人工操作区间和软件命令/物理测量分别判断。迁移读取检查不等于所有统计均已统一复核。
