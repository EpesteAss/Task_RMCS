# Yaw 机械本体辨识：候选参数检查

原始数据：`/workspaces/RMCS/experiments/yaw_identification/physical_analysis/20261014_fixed_pitch_three_trials.csv`

方程：`theta_dot=omega; J*omega_dot=tau-B*omega-Fc*tanh(omega/0.02)+bias`。

模型不读入 Yaw PID 增益或目标角度；只有力矩输入和角度、角速度状态。

采集是否有无 Yaw PID 证据：**有**。

下表是条件性候选值：尚未确认日志中 Nm 与实际 Yaw 轴力矩的换算。

| 输入 | 模型 | J (kg m²) | B (Nm s/rad) | Fc (Nm) | 偏置 (Nm) | 留出速度拟合度 | 角度 RMSE |
|---|---|---:|---:|---:|---:|---:|---:|
| feedback | linear | 64.03 | 0 | 0 | -0.0004265 | -2.31% | 1.151 deg |
| feedback | smooth_friction | 64.03 | 0 | 0 | -0.0004265 | -2.31% | 1.151 deg |
| command | linear | 17.36 | 0 | 0 | 4.236e-06 | -2.33% | 1.152 deg |
| command | smooth_friction | 17.36 | 0 | 0 | 4.236e-06 | -2.33% | 1.152 deg |

反馈输入来自电流换算的力矩估计；command 是限幅/变化率处理后的总命令。

训练与验证按完整轮次先后分开，未根据验证分数选择模型或调整系数。模型只用每轮开头一个状态初始化，之后整段自由预测。

`results.json` 包含逐轮结果、25/100 ms 窗口敏感性和整轮重采样范围。重采样范围不包括力矩标定误差、反馈偏差或模型失配。

参数估计仍是初步积分回归。旧闭环数据可能存在估计偏差；无 PID 采集不自动保证物理参数准确。

下一步：核对力矩常数与传动折算；采集相同负载/姿态下的正反向、多频率数据；固定模型后用新会话验证，必要时加入有证据的执行器动态或位置相关负载。
