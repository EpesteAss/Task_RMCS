# 低位 Pitch 模式 OFF 预检

2026-10-14 北京时间约 10:09，车载隔离目录运行 `identification-physical-low-pitch-temp130`，源配置 `allow_arm: false`，未发送 Arm 或 Sweep。车载插件重建并通过 4 项测试；静态预检通过。

CSV 共 900 行、约 9 s。`state=0`、`fault=0` 全程不变；Pitch/Yaw 命令力矩全为 0；`physical_identification=1`、`passive_pitch=1` 全程不变。Pitch 世界角从约 −8.25° 缓慢到 −7.95°，在设定的 −10°～−6° 范围内；Pitch IMU 角速度约 −0.038～+0.044 rad/s（排除启动前 0.1 s）。Pitch 温度有效读数最终 35°C；Yaw 温度仍为固定 125°C。Yaw 角度在启动第 1 个采样间有一次约 0.91 rad 的初始化跳变，之后固定；Yaw 命令全为 0，无由此推断的机械运动。

用 `timeout -s INT 10s` 结束预检时，底层 `librmcs_core` 的上板陀螺仪 USB 回调打印段错误堆栈。随后检查发现 `rmcs_executor` 仍在运行，已单独发送 TERM；再次检查无该进程。段错误发生在退出阶段，尚未定位根因，完整输出保存在 `runtime.log`。此预检不能证明运行中的故障处理会完成干净退出。

低位模式的源配置现开放 Arm，供现场执行单轮试采；高位三倍配置继续锁定。试采前仍需现场确认 Yaw 外壳实际温度、低位净空和急停状态。不要把这份 OFF 数据用于辨识。
