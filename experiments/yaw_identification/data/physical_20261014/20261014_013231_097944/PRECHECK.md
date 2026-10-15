# 2026-10-14 OFF 反馈复查

北京时间约 09:32，经现场确认原 `sweep.yaml` 控制器已 OFF、两轴命令归零、Pitch 有支撑且 Yaw 两侧有净空后，使用车上 `service rmcs stop` 结束原程序。确认无 `rmcs_executor` 后，以原 `identification_physical.yaml` 启动独立实验程序；该配置仍为 `allow_arm: false`。只保持 OFF 采集约 12.7 秒，随后结束程序并再次确认无 `rmcs_executor`。

原始记录为同目录 `feedback.csv` 和 `config.yaml`，共 1273 行。除第一行初始化零值外，Yaw 温度反馈全为 125°C；Pitch 温度为 37–38°C。所有行的 Yaw、Pitch 命令力矩均为 0，实验状态均为 OFF，完成轮数和故障标志均为 0，`physical_identification` 均为 1。Pitch 角度在初始化后稳定在约 0.1973 rad。

运行时再次只读查询 `/rmcs/service/robot_status`，上板出现 `Top::Can0: 0x141`，Pitch 原始编码器读数为 5035。此前缺失的 Pitch 反馈在此次复查中已恢复，但恢复原因和长期稳定性尚未查清。Yaw 的 125°C 反馈异常仍在；现场尚未提供本次外壳实测温度。没有 Arm 或执行力矩激励，继续保持 `allow_arm: false`。
