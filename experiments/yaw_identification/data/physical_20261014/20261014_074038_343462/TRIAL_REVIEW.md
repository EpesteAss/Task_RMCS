# 固定 Pitch 的反号 1 Hz、0.60 N·m 完整试采复核

- profile 4、反号 1 Hz、0.60 N·m，Sweep 完整 4 s、400 个 10 ms 采样；Pitch 命令与 Yaw PID 均为 0。
- 相对本轮起点，Sweep 角度范围 −0.813° 至 +0.879°，最终 OFF 时约 −0.082°；最大绝对 IMU Yaw 角速度约 0.190 rad/s。`fault=0`、`physical_guard_stop=0`、`completed=1`；会话确认 OFF 和两轴零命令，实验进程已结束。
- 与前三轮完整数据合并，用前三轮训练、本轮验证。反馈力矩输入无法辨识正惯量；命令力矩输入的留出速度拟合度约 −1.52%、角度 RMSE 约 0.546°。见[联合报告](../../physical_analysis/20261014_fixed_pitch_fit_3_validate_reverse/REPORT.md)。不接受这些拟合参数为实车物理参数。
