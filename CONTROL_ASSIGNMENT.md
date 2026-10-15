# 控制方向作业提交入口

## 龙门架

- 实现：`rmcs_ws/src/rmcs_core/src/hardware/gantry.cpp`、`rmcs_ws/src/rmcs_core/src/controller/motor/gantry_controller.cpp`。
- 配置与实机操作说明：`rmcs_ws/src/rmcs_bringup/config/gantry.yaml`、`gantry.README.md`。
- 三台 M2006 的分工为上方 ID 1 控制 yaw、下方左 ID 2 与右 ID 3 控制 pitch。当前配置是带自动上限找平的单环手动测试；几何尺寸和安全行程尚未实测，完整角度位置控制保持锁定。实机前请先阅读配置目录中的说明。
- 实操验收待补：纯升降时检查发射架实际水平度；仅操作 yaw 时用倾角仪记录中位和左右两端的实际 pitch。当前手动模式不主动补偿 yaw 引起的 pitch 几何变化，不能把两项写成已实测通过。

## 步兵 C 车 yaw 系统辨识与控制优化

- 验收补充要求：采集期间 Yaw 不使用 PID，辨识机械本体的惯量、阻尼及摩擦。该项已完成实车采集、留出验证和模型前馈 A/B；输出轴实际力矩尚未用独立传感器标定，因此物理参数以日志电机力矩刻度为条件。

- 新物理辨识操作入口：`experiments/yaw_identification/PHYSICAL_IDENTIFICATION.md`；历史闭环实验入口：`experiments/yaw_identification/QUICKSTART.md`。
- 实验插件及脚本：`experiments/yaw_identification/addon/`、`closed_loop_cascade_study.py`、`compare_control.py`。
- 文档：`experiments/yaw_identification/FINAL_REPORT.md` 和同目录的 PDF。
- 最终物理模型数据：`experiments/yaw_identification/data/physical_20261014/`；频响拟合使用正相 0.5、1、2 Hz，反相 0.5 Hz 整轮留出验证。
- 零前馈与模型前馈成对数据：`control_data/control-model-baseline-fixed/20261015_015001_319800/` 和 `control_data/control-model-ff-fixed/20261015_015157_513409/`。

最终采用含黏性阻尼和库仑摩擦的二阶转动模型。较低角度工作区参数为 `J=0.1238 kg·m²`、`B=0.2753 N·m·s/rad`、`F_c=0.2197 N·m`；反相 0.5 Hz 留出轮次的已运动区间速度拟合度为 90.2%，角度 RMSE 为 2.50°。相同 PID 和轨迹下加入模型前馈后，三档合并角度 RMSE 从 1.288° 降至 1.191°，改善 7.55%。

未纳入本次提交的中间尝试日志仍保存在实验机器的本地工作区；上述最终数据和配置快照足以复算主要结论。

本地提交前验证：实验插件的 4 项 CTest 和辨识工具的 24 项 Python 测试通过；PDF 已检查文字提取、公式和图表排版。
