# 控制方向作业提交入口

## 龙门架

- 实现：`rmcs_ws/src/rmcs_core/src/hardware/gantry.cpp`、`rmcs_ws/src/rmcs_core/src/controller/motor/gantry_controller.cpp`。
- 配置与实机操作说明：`rmcs_ws/src/rmcs_bringup/config/gantry.yaml`、`gantry.README.md`。
- 三台 M2006 的分工为上方 ID 1 控制 yaw、下方左 ID 2 与右 ID 3 控制 pitch。当前配置是带自动上限找平的单环手动测试；几何尺寸和安全行程尚未实测，完整角度位置控制保持锁定。实机前请先阅读配置目录中的说明。

## 步兵 C 车 yaw 系统辨识与控制优化

- 操作入口：`experiments/yaw_identification/QUICKSTART.md`。
- 实验插件及脚本：`experiments/yaw_identification/addon/`、`closed_loop_cascade_study.py`、`compare_control.py`。
- 文档：`experiments/yaw_identification/FINAL_REPORT.md` 和同目录的 PDF。
- 最终模型复算数据：`data/20261007_022850_274011/feedback.csv` 的前 18 轮用于训练与选型，`data/20261007_043913_088815/feedback.csv` 的最后 9 轮用于冻结后验证。
- 原版与优化版成对数据：`control_data/control-baseline-wide/20261006_070745_870317/feedback.csv` 和 `control_data/control-tuned-wide/20261006_072257_499687/feedback.csv`。

冻结后 9 轮完整自由预测的原始 IMU 速度拟合度为 91.23%；相同 50 ms 因果低通施加于实测和预测后，总体为 95.75%，最低速 0.05 Hz 为 90.47%。原始最低速为 79.98%。左右各 15 度的三档成对控制实验中，角度 RMSE 从 1.283 度降到 0.821 度。模型拟合与控制器对比使用不同实验，请按报告中的输入、滤波和验证划分分别解读。

未纳入本次提交的中间尝试日志仍保存在实验机器的本地工作区；上述最终数据和配置快照足以复算主要结论。

本地提交前验证：`rmcs_core` 与实验插件编译通过；4 个龙门架逻辑测试、2 个插件测试和 11 个 Python 工具测试通过。使用上面列出的原始 CSV 重新运行模型与控制对比脚本，所得 JSON 与保存的结果完全一致。PDF 公式改用可移植的 ASCII 写法，并检查过文字提取及图表排版。
