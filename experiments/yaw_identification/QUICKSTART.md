# C 车系统辨识：独立操作说明

当前设置：**世界 pitch 目标 +5°**（允许稳定误差 1.5°），yaw 相对 `arm` 时的位置
允许 **左 15°、右 15°**。这是行程限制，不保证每轮摆满 15°。
扫频力矩幅值 ±0.96 N·m，总输出上限 ±3.6 N·m；频率 0.2→3 Hz，每轮 20 s。
采集以 100 Hz 写入永久目录 `~/yaw_identification/data/日期时间/feedback.csv`。
本轮根据实测 3.7° 时约 4.07 N·m 的保持力矩，将 pitch 保持阶段的重力前馈
调至 3.5。大角度验证的低角度抬升使用 3.45，并在接近 3°净空时平滑过渡，
避免抬升超速；最终力矩上限仍为 4.5 N·m。先用一分钟验证实际角度和温度。

原车在较低 pitch 时可能发生干涉。第一次使用新 5° 设置先做一分钟检查；
观察实际净空，不要手动助转。持续接近 4 N·m 托住 pitch 会发热，30 分钟只是
最长计划时长，不保证能持续到时。采集程序在任一电机到 55°C 时提前停止并卸力。
必须人在旁边，准备好 pitch 卸力时的机械支撑。不要让电脑睡眠或关闭 SSH。

## 1. 连接小电脑

在笔记本 RMCS 开发容器终端执行：

```bash
set-remote alliance-deformable-5.local
# 选择 IPv4 对应序号，例如 0
ssh-remote
```

以下命令都在 SSH 登录后的小电脑上执行。另开终端时再执行 `ssh-remote`。

## 2. 终端 A：启动采集服务（不会立即运动）

可靠支撑发射架后停止原车服务：

```bash
service rmcs stop
cd ~/yaw_identification
bash start.bash
```

看到 `Experiment OFF` 和 `Permanent recording directory` 后保持此终端开着。
参数快照自动保存为同目录的 `config.yaml`，日志不再放在重启会清空的 `/tmp`。
如果报 `Another rmcs_executor is running`，先回到原先启动它的终端正常停止；
不要同时启动两份。可以用 `pgrep -a -x rmcs_executor` 查看。

## 3. 终端 B：先试一分钟，再正式采集

```bash
cd ~/yaw_identification
bash collect.bash --minutes 1
```

**这条命令会自动抬升，然后扫频。** 确认没有机械干涉后，正式运行：

```bash
bash collect.bash --minutes 30
```

程序自动发送心跳、arm，等待 READY，再重复“20 秒扫频＋5 秒保持”。
若心跳未到控制器，脚本会明确报错并保持 OFF；检查终端 A 的日志和
`heartbeat_age_s`，不要把“arm 已排队”当作已开始抬升。
30 分钟通常得到约 65～70 个完整试验，具体数量受等待和状态发布延迟影响。
保持阶段 pitch 仍然有力，不是冷却阶段。到时会自动 `off` 并确认两轴输出为零。
完成后不自动重新启动。脚本执行摘要保存在 `sessions/*.json`。

不要同时运行 `console.py` 和 `session.py`。如果提示已有操作端，先退出另一个。
同一终端 A 可以承接多次采集；要把一分钟试验和正式试验分为两个 CSV，等 B
确认 OFF 后，在 A 按 Ctrl+C，再运行 `bash start.bash` 创建新的记录目录。

## 4. 停止与故障

- 在终端 B 按 **Ctrl+C**：脚本会发送 `off`；yaw 立即归零，pitch 按
  1.0 N·m/s 缓慢卸力，确认零输出后退出。状态 5 表示正在卸力。
- 实验结束先确认 `Confirmed OFF, both torque commands zero`，再在终端 A 按 Ctrl+C。
- SSH 断开或心跳超过 1.5 s 未收到时，控制器卸力。不能依赖网络替代现场断电手段。
- `state=0/1/2/3/4` 分别是 OFF、抬升、保持、扫频、故障。
- 故障码：1 心跳丢失；2 无效数据/周期；3 pitch 编码器越界；4 超速；
  5 yaw 超过 ±15°；6 电机达到 65°C；7 抬升超时；8 高力矩无位移。
- 自动采集的 55°C 停止早于控制器 65°C 硬保护。停止后支撑、冷却并排查，不能反复强行重启。
- 本车 RMCS 底层已知在退出时可能崩溃或挂住。只有已确认 OFF 和零输出后，才可
  对 `pgrep -a -x rmcs_executor` 列出的旧实验 PID 执行 `kill -KILL PID`。
  不要在仍托住 pitch 时直接杀进程。

需要单独操作时，保持 A 运行，在 B 执行：

```bash
cd ~/yaw_identification
source addon_install/setup.zsh
python3 console.py
```

输入 `arm` 抬升，`sweep` 扫一轮，`hold` 取消扫频但继续托住，`off` 卸力，`quit` 卸力并退出。

## 5. 分析（不需要电机在线）

直接分析，无需安装额外依赖（小电脑已有 numpy）：

```bash
cd ~/yaw_identification
python3 identify.py --latest
```

如果希望生成 PNG 而非 SVG，可选安装绘图依赖：

```bash
bash setup-analysis.bash
.venv/bin/python identify.py --latest
```

也可以明确选择文件：

```bash
python3 identify.py data/日期时间/feedback.csv
```

如果安装绘图库失败，直接用 `python3 identify.py --latest`，仍会生成模型、文字报告
和 SVG 曲线。SVG 可以直接用浏览器打开。

结果放在 `analysis_output/对应记录/`：

- `results.json`：连续二阶、带摩擦二阶、延迟角度 ARX 和滤波速度 ARX 模型的系数，训练/验证试验编号与误差。
- `REPORT.md`：结果说明。
- `linear_validation.svg`、`friction_validation.svg`（有 matplotlib 时为 PNG）：留出试验的整段预测与实测曲线。

模型为 `θ̇=ω, ω̇=aω+bu+c`，摩擦模型另外加 `f*tanh(ω/0.02)`。
分析还会拟合离散二阶角度模型
`θ[k]=p1θ[k-1]+p2θ[k-2]+bu[k-delay]+c`。输入延迟只在训练试验内部的
嵌套划分上选择，最终留出试验不参与选型；该模型用于提高自由预测能力，连续
模型仍用于参数物理解释和控制器初始设计。
分析还会拟合 12 ms 低通滤波后的二阶速度 ARX 模型
`ωf[k+1]=q1ωf[k]+q2ωf[k-1]+bu[k]+c`。报告中的约 93.7% 是独立留出
试验上的 10 ms 单步预测拟合度；整段自由仿真拟合度另行报告，不能把两个指标
混写成同一种“准确率”。
完整试验按时间分为前 70% 拟合、后 30% 验证；中断片段跳过。只有一轮时使用
该轮前 70% 和后 30%，会明确提示缺少独立试验。
`velocity_fit_percent` 越高越好，负值表示比预测平均速度还差；
`physical_signs_plausible=false` 或 `simulation_diverged=true` 时不能据此调控制器。
闭环测量噪声和摩擦会使参数有偏，模型不等于已验证的真实惯量。

## 6. 备份到笔记本

在笔记本开发容器 `/workspaces/RMCS` 终端执行，IP 换为本次连接找到的地址：

```bash
scp -r -P 2022 root@192.168.3.168:/root/yaw_identification/data experiments/yaw_identification/
scp -r -P 2022 root@192.168.3.168:/root/yaw_identification/analysis_output experiments/yaw_identification/
```

## 7. 以后改配置/重新编译

`identification.yaml` 的 `world_pitch_target_deg` 是实际抬头目标。
`yaw_span_rad=0.2617993877991494` 对应左右各 15°。
`duration=20` 是单轮扫频时长；总时间用 `collect.bash --minutes 30`，**不要把 duration 改成 1800**。
修改配置要在 OFF 后重启终端 A 才生效。

代码已经编译安装，无需每次重新 prepare 或编译。如果确实要重新生成配置，使用：

```bash
python3 prepare.py --source /rmcs_install/share/rmcs_bringup/config/deformable-infantry-omni-c.yaml --enable-arm --pitch-gravity-ff-gain 3.5
bash build-addon.bash
```

无 DR16 的辨识、控制调优和旧版基准/优化实机对比已经完成。10 月 6 日更新了
pitch 托举与 yaw 平顺性参数；最新版本的成对复测命令和待补数据见
`FINAL_COMPARISON.md`。`baseline.yaml`/`tuned.yaml` 是原车控制参考配置，
仍依赖遥控输入；不要拿它们替代 `control_*_final.yaml` 的自动实验。

## 8. 丰富激励辨识

需要继续提高整段自由预测时，使用 [RICH_IDENTIFICATION.md](RICH_IDENTIFICATION.md)
中的第二版流程。先采9轮确认左右各15°目标运动和九种频率完整，再分三组各采9轮
正式数据；软件保护为左右各16°。采集脚本允许低于45°C启动、64°C主动停机，
控制器保留65°C硬故障保护，但组间
仍建议冷却到35°C左右，以免9轮尚未完成就触发温度保护。不要把
旧的 `collect.bash` 和新的 `collect-rich.bash` 混在同一个运行进程中。

## 9. 优化控制器的大角度验证

`control_tuned_wide.yaml` 保留最终 Kp10 参数，将目标扩大为左右各 15°。三轮频率
依次为 0.10、0.15、0.25 Hz（周期 10、6.67、4 s），启停包络为 3 s。抬升阶段
重力前馈为 3.45；arm 后先保持初始角度 0.4 s，以 8 N·m/s 建立托举力矩，
再用 0.3 s 渐增到 10.5°/s 的抬升目标速度，并以 6 N·m/s 调整力矩。力矩硬上限仍为
4.5 N·m。软件行程保护为左右各 16°，只用于容纳
目标峰值处的编码器噪声；上机前必须确认从启动中心到两侧至少各有 16°机械空间。
为减小低速走格子，pitch 轨迹最多领先实际位置 0.8°；接近上限时按经过滤波的
实测速度连续调整轨迹，并在低速、远离目标时提供最多 0.1 N·m 的渐变摩擦补偿。
pitch 使用 12 ms 速度滤波；
yaw 使用 6 ms 速度滤波、Kp10 和 12 N·m/s 力矩变化率，以保证边缘
换向时有足够阻尼和制动力。

终端 A：

```bash
service rmcs stop
cd ~/yaw_identification
bash start-control.bash tuned-wide
```

终端 B 采三轮不同周期：

```bash
cd ~/yaw_identification
bash collect-control.bash tuned-wide 3
```

三轮会自动按慢、中、快顺序运行，不需要中途修改配置。
若要在左右各 15° 下与原 yaw PID 增益做成对对照，使用相同流程的
`baseline-wide`，再重做一组 `tuned-wide`；详细步骤见 `FINAL_COMPARISON.md`。
