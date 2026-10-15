# 高位受控回落模式 OFF 预检

2026-10-14 北京时间约 10:21，在车载隔离目录运行 `identification-physical-raised-park-temp130-cycle`，未发送 Arm 或 Sweep。源配置 `allow_arm: true`、`allow_sweep: false`；本次只有 OFF 读数。

CSV 2200 行、约 22 s：状态 0、故障 0 全程不变；Pitch/Yaw 控制力矩始终为零；`controlled_pitch_parking=1` 全程。Pitch 世界角约 −7.74° 至 −7.53°，位于采集端 −8°±2° 的起始窗口。Pitch 有效温度最终 35°C；Yaw 温度读数仍固定 125°C。车载构建 4 项测试通过，cycle 和 trial 配置静态预检通过。

预检后 `stop-safe.bash` 确认两轴零力矩。底层 `rmcs_executor` 收到 TERM 后仍未退出，最后在已确认 OFF 的状态下发送 KILL；再次检查无残留进程。此为已有的底层退出异常，仍未定位。没有做 Pitch 抬升；下一步是有人现场监看的单次 Pitch 升降空跑，Yaw Sweep 在配置中禁用。
