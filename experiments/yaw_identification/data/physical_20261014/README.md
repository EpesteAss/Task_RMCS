# 2026-10-14 Yaw 直接力矩辨识数据

这批原始采集由 `physical_data/` 整理到 `data/physical_20261014/`，方便在 `data/` 下查找与提交。共有 62 个采集目录；其中包含预检、被保护中止的试采、完整正式试验以及最终盲测，**不能把全部目录都算作有效训练轮次**。

每个目录尽量完整保留原始 `feedback.csv`、实验 `config.yaml`、会话 JSON 和现场记录。`MANIFEST.csv` 按相对路径列出每个原始文件的大小和 SHA-256。需要重新归档时，从仓库根目录运行：

```bash
python3 experiments/yaw_identification/archive_physical_data.py
```

最终统一模型的 **16 轮训练数据**及 **1 轮独立盲测**由 `physical_analysis/20261014_global_narx_holdout/model.json` 明确列出。盲测目录是 `20261014_144902_607776/`。模型复算命令：

```bash
python3 experiments/yaw_identification/global_narx_identification.py
```

这份归档和 `physical_data/` 中的原始文件逐文件校验一致。报告中的控制器对比没有使用这批直接力矩数据。
