# C 车系统辨识：独立操作说明

当前设置：**世界 pitch 目标 +5°**（允许稳定误差 1.5°），yaw 相对 `arm` 时的位置
允许 **左 15°、右 15°**。这是行程限制，不保证每轮摆满 15°。
扫频力矩幅值 ±0.96 N·m，总输出上限 ±3.6 N·m；频率 0.2→3 Hz，每轮 20 s。
采集以 100 Hz 写入永久目录 `~/yaw_identification/data/日期时间/feedback.csv`。
本轮根据实测 3.7° 时约 4.07 N·m 的保持力矩，将 pitch 保持阶段的重力前馈
调至 3.5。低角度抬升仍使用已验证的 2.575，并在接近 3°净空时平滑过渡，
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

- 在终端 B 按 **Ctrl+C**：脚本会发送 `off`，确认零输出后退出。
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

- `results.json`：线性二阶模型、带摩擦二阶模型的系数，训练/验证试验编号、误差。
- `REPORT.md`：结果说明。
- `linear_validation.svg`、`friction_validation.svg`（有 matplotlib 时为 PNG）：留出试验的整段预测与实测曲线。

模型为 `θ̇=ω, ω̇=aω+bu+c`，摩擦模型另外加 `f*tanh(ω/0.02)`。
分析还会拟合离散二阶角度模型
`θ[k]=p1θ[k-1]+p2θ[k-2]+bu[k-delay]+c`。输入延迟只在训练试验内部的
嵌套划分上选择，最终留出试验不参与选型；该模型用于提高自由预测能力，连续
模型仍用于参数物理解释和控制器初始设计。
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

当前交付完成的是无 DR16 的采集、重复实验和模型分析流程。
原作业中的控制器调优、原始与优化后实机对比仍需后续完成；
现有 baseline/tuned 仍依赖原车遥控输入，不要直接用来替代上述流程。
