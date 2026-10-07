# 第二版丰富激励辨识

目标是提高整段自由预测质量。新版每组包含 9 轮正弦目标，频率依次为
0.05、0.075、0.10、0.125、0.16、0.20、0.25、0.32、0.40 Hz；目标角度为
左右各15°，单轮24 s并平滑启停。软件越界保护设为启动中心左右各16°，给目标
峰值处留1°制动余量；总 yaw 命令仍受3.6 N·m和1 rad/s超速保护约束。会话在
64°C主动OFF，控制器保留65°C硬故障保护，避免两个阈值重合导致pitch突然卸力。
上机前必须确认两侧至少各有16°机械空间。

## 1. 从笔记本同步到小电脑

在笔记本开发容器 `/workspaces/RMCS` 执行。若小电脑 IP 改变，替换下面的地址：

```bash
scp -r -P 2022 experiments/yaw_identification/addon \
  root@192.168.3.168:/root/yaw_identification/
scp -P 2022 \
  experiments/yaw_identification/identification_rich.yaml \
  experiments/yaw_identification/run.py \
  experiments/yaw_identification/session.py \
  experiments/yaw_identification/identify.py \
  experiments/yaw_identification/start-rich.bash \
  experiments/yaw_identification/collect-rich.bash \
  root@192.168.3.168:/root/yaw_identification/
```

## 2. 在小电脑构建和静态检查

```bash
cd ~/yaw_identification
bash build-addon.bash
source addon_install/setup.zsh
python3 run.py identification-rich --check-only
```

## 3. 先采 9 轮安全检查

终端 A：

```bash
cd ~/yaw_identification
service rmcs stop
bash start-rich.bash
```

终端 B：

```bash
cd ~/yaw_identification
bash collect-rich.bash --trials 9
```

9轮约需5分钟。现场观察每轮接近目标边界时应平滑减速，实际角度不得越过左右16°或出现明显冲击；
任何异常按 `Ctrl+C`，脚本会请求 OFF 并缓慢卸掉 pitch 力矩。硬件断电仍是最终急停。

不要直接用 `pkill` 作为停止动作。需要从第三个终端停止时先执行
`bash stop-safe.bash`，看到两轴命令均为零后，才能结束 `rmcs_executor`。

完成后执行：

```bash
cd ~/yaw_identification
python3 identify.py --latest
```

把终端输出或新生成的 `analysis_output/.../REPORT.md` 发回来检查。确认9种 profile
各出现1次且没有 fault 后，保持终端A运行，正式数据按三组各9轮采集。每组结束
会回到OFF。脚本允许低于45°C启动，并在达到64°C时主动停止；控制器达到65°C会
触发硬故障。为提高一次完成9轮
的概率，仍建议尽量冷却到35°C左右再开始下一组：

```bash
# 第1组
bash collect-rich.bash --trials 9

# 等温度低于40°C后，第2组
bash collect-rich.bash --trials 9

# 再次冷却后，第3组
bash collect-rich.bash --trials 9

# 三组全部完成、停止终端A并保存CSV后再分析
python3 identify.py --latest
```

必须在每组之间人工检查温度并等待冷却。27轮分成三组：前9轮拟合候选模型，
中间9轮选择模型，最后9轮最终验证。
每组完整覆盖9种频率，避免验证集只包含某一类运动。
