# 最新控制参数的最终对照

## 左右各 15°、三档速度（已完成）

`control_baseline_wide.yaml` 和 `control_tuned_wide.yaml` 都使用同一份 pitch 托举配置、左右各 15° 的 yaw 目标，以及 0.10、0.15、0.25 Hz 三轮不同周期。每轮 20 s，周期依次约为 10、6.67、4 s。两组共享相同的 yaw 力矩、变化率和软件行程限制；基准组保留原 yaw PID 增益且无前馈，优化组使用调优后的增益与前馈。因此这组比较的是**共同安全约束下**增益及前馈的效果。

首次宽幅基准测试只完成前两轮，第三轮 yaw 超速触发 fault 4；那份 CSV 不是有效的三轮对照。把两组角度环速度上限统一调至 0.5 rad/s，并让 yaw 减力比加力更快后，最终基准组 `20261006_070745_870317` 与优化组 `20261006_072257_499687` 均完成三轮、无故障。三轮总体角度 RMSE 从 1.283° 降至 0.821°（改善 36.0%），P95 绝对误差改善 39.8%，力矩 RMS 增加 10.0%。另一份优化组日志 `20261006_071639_653522` 中断，未纳入对照。统计和逐轮曲线已保存于 `control_analysis_wide_latest/`。

**复测换组时必须先确认 OFF、停止旧进程、重新启动终端 A**，新配置和代码才会生效。发生抖动或 fault 时保持支撑并停止，不要连续强行重试。

上机前确认从启动中心到左右两端至少各有 16° 的机械空间，现场支撑好发射架。若旧实验的终端 A 还开着，先确认采集终端已显示 `Confirmed OFF, both torque commands zero`，再在 A 按 Ctrl+C；确认 `pgrep -a -x rmcs_executor` 没有旧实验进程。不要让两份控制程序同时连接电机。

小电脑 SSH 终端 A：

```bash
cd ~/yaw_identification
service rmcs stop
bash start-control.bash baseline-wide
```

另一个 SSH 终端 B：

```bash
cd ~/yaw_identification
bash collect-control.bash baseline-wide 3
```

等 B 报告三轮完整且 OFF 两轴力矩为零，再结束 A。随后把两条命令中的 `baseline-wide` 换成 `tuned-wide` 重复。两组尽量维持相同负载、初始中心和温度。完成后运行：

```bash
cd ~/yaw_identification
bash analyze-final.bash wide
```

脚本会核对三轮频率顺序与目标轨迹，输出 `control_analysis_wide_latest/control_comparison.json`、总体曲线和逐轮曲线 `control_comparison_trial_1.svg` 至 `_3.svg`。如果某轮提前故障，报告会拒绝把它算作完整的三档比较。**不要把先前 `baseline-final` 的 5° 数据直接和 `tuned-wide` 的 15° 数据比较。**

## 左右各 5°、同速三次重复（已有基准数据）

前期报告里的 26.9%—28.9% 改善来自旧版 pitch 托举和 yaw 平顺性参数。`control_baseline_final.yaml` 与 `control_tuned_final.yaml` 固定了相同的最新 pitch 托举、5° 世界 pitch 目标和 yaw 速度滤波；差别只在 yaw PID、前馈和输出约束。两组目标都是左右各 5°、0.25 Hz、20 秒，每组采 3 次。最新优化参数的改善幅度必须用这组新数据重算。

运行前现场支撑好发射架，确认从当前中心到左右两侧有至少 16° 空间，并确认电机温度低于 40°C。实验期间保持人在场、SSH 不断线。采集程序会自动抬升、扫动，结束时缓慢卸掉 pitch 力矩并确认两个轴命令为零。

在小电脑 SSH **终端 A**：

```bash
cd ~/yaw_identification
service rmcs stop
bash start-control.bash baseline-final
```

看到 `Starting control-baseline-final OFF` 后，在另一个 SSH **终端 B**：

```bash
cd ~/yaw_identification
bash collect-control.bash baseline-final 3
```

等 B 显示 `Session complete: 3 full sweeps` 和 `Confirmed OFF, both torque commands zero`。若故障或提前停止，先检查原因，不能把该次数据作为有效对照。然后在 A 按 Ctrl+C，确认旧进程结束：

```bash
pgrep -a -x rmcs_executor
```

正常应无输出。如果仍有进程，确认已经 OFF 且 pitch 有可靠支撑，再对显示的 PID 执行 `kill -TERM PID`；退出仍卡住时才执行 `kill -KILL PID`。不要在托举有力时杀进程。

用同样两终端采优化组；A 运行 `bash start-control.bash tuned-final`，B 运行 `bash collect-control.bash tuned-final 3`。两组尽量维持相同负载、起始姿态和温度。最后再次确认 OFF 并结束 A。

在小电脑执行：

```bash
cd ~/yaw_identification
bash analyze-final.bash
```

脚本会显示选中的两份最新 CSV，核对完整试验数、目标轨迹、时长和 pitch 配置；通过后生成 `control_analysis_final_latest/control_comparison.json` 和带时间、角度、误差、力矩坐标轴的 `control_comparison.svg`。若最新运行中断，脚本会报错；先检查它打印的 CSV 路径，不要拿旧数据冒充本轮结果。

把最终数据备份到笔记本开发容器（在 `/workspaces/RMCS` 下执行，IP 按实际连接地址修改）：

```bash
scp -r -P 2022 root@192.168.3.168:/root/yaw_identification/control_data/control-baseline-final experiments/yaw_identification/control_data/
scp -r -P 2022 root@192.168.3.168:/root/yaw_identification/control_data/control-tuned-final experiments/yaw_identification/control_data/
scp -r -P 2022 root@192.168.3.168:/root/yaw_identification/control_analysis_final_latest experiments/yaw_identification/
```

早先的 `tuned-wide` 记录只有优化组，不能用它们计算相对改善。新增的 `baseline-wide` 与下一次 `tuned-wide` 三轮完成后，才能做左右各 15° 的同轨迹比较。完成新对照后，把实测数值、温度及曲线补进 `FINAL_REPORT.md`，再重新生成 PDF。
