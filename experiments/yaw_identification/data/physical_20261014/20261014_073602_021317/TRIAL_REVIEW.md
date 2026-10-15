# 固定 Pitch 的 1 Hz、0.60 N·m 完整试采复核

- Pitch 机械固定，电机命令全程 0 N·m；Yaw 外壳最近实测 30°C，现场人员监视温度。Yaw 温度遥测仍固定 125°C。
- profile 1、1 Hz、0.60 N·m，Sweep 完整 4 s，共 400 个 10 ms 采样。Yaw 相对起点在 Sweep 期间为 −2.77° 至 +0.02°，最终 OFF 时为 −1.58°；最大绝对 IMU Yaw 角速度约 0.247 rad/s，最大绝对 Yaw 命令力矩 0.600 N·m，反馈力矩 0.620 N·m。
- 全程 Yaw PID、前馈和 Pitch 命令均为 0。`fault=0`、`physical_guard_stop=0`、`completed=1`；会话确认 OFF 与两轴命令 0。结束实验进程后 `pgrep` 无 `rmcs_executor`。
- 连同早前的 0.5 Hz 完整试采，目前只有两轮完整数据，尚不足以评估参数对新频率/反向运动的预测。准备另一频率单轮后再做联合拟合与独立验证。
