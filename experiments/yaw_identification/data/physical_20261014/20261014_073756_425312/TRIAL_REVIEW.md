# 固定 Pitch 的 2 Hz、0.60 N·m 完整试采复核

- Pitch 机械固定，电机命令全程 0 N·m；Yaw 外壳最近实测 30°C，现场人员监视温度。
- profile 2、2 Hz、0.60 N·m，Sweep 完整 4 s，共 400 个 10 ms 样本。Yaw 相对起点在 Sweep 期间为 −0.665° 至 +0.324°，最终 OFF 时约 −0.187°；最大绝对 IMU Yaw 角速度约 0.149 rad/s。`fault=0`、`physical_guard_stop=0`、`completed=1`，会话确认 OFF 与两轴命令 0；结束进程后无 `rmcs_executor`。
- 把三轮完整试采按 0.5 Hz、1 Hz、2 Hz 顺序合并，用前两轮训练、2 Hz 留出验证。线性和光滑摩擦模型都没有通过预测检查：反馈力矩输入的速度拟合度约 −2.31%、角度 RMSE 约 1.151°；命令力矩输入约 −2.33%、1.152°。见[联合报告](../../physical_analysis/20261014_fixed_pitch_fit_2_validate_1/REPORT.md)。因此现阶段**不接受**报告中的惯量、阻尼等数值为实车物理参数。
- 后续补一轮反号 1 Hz、0.60 N·m，判断方向相关滑移及力矩符号；原有速度/行程保护不放宽。
