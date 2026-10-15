#!/usr/bin/env python3
"""Identify a mechanical yaw model from torque/angle/velocity, without yaw PID parameters.

The model is J*omega_dot = torque - B*omega - Fc*tanh(omega/0.02) + bias.
Absolute parameters are conditional on the logged torque calibration. Old
feedback-controlled captures are explicitly restricted to diagnostic use.
"""
import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np

from identify import read_log, trials
from svg_plot import validation_plot


TORQUES = {"feedback": "/gimbal/yaw/torque", "command": "/gimbal/yaw/control_torque"}


def capture_is_pid_free(data):
    keys = ("/yaw_experiment/physical_identification", "/yaw_experiment/yaw_pid_torque",
            "/yaw_experiment/yaw_feedforward")
    if any(key not in data for key in keys):
        return False
    # Check the complete capture, including pauses, not only moving samples.
    return bool(np.all(data[keys[0]] == 1)
                and np.all(data[keys[1]] == 0) and np.all(data[keys[2]] == 0))


def load_trials(path, data, channel):
    selected = dict(data)
    selected[TORQUES["command"]] = data[TORQUES[channel]]
    accepted, rejected = trials(selected)
    return accepted, rejected


def integral_blocks(group, friction, window_s=.05):
    blocks = []
    for trial in group:
        t, v, torque = trial["t"], trial["omega"], trial["u"]
        step = max(2, round(window_s / np.median(np.diff(t))))
        rows, targets = [], []
        for start in range(0, len(t)-step, step):
            stop = start+step
            dt = np.diff(t[start:stop+1])
            speed = v[start:stop+1]
            row = [-float(np.sum(.5*(speed[:-1]+speed[1:])*dt)),
                   float(np.sum(torque[start:stop]*dt)), float(t[stop]-t[start])]
            if friction:
                direction = np.tanh(speed/.02)
                row.append(-float(np.sum(.5*(direction[:-1]+direction[1:])*dt)))
            rows.append(row)
            targets.append(v[stop]-v[start])
        if rows:
            blocks.append((np.asarray(rows), np.asarray(targets)))
    return blocks


def estimate(blocks, friction):
    if not blocks:
        raise ValueError("No usable integral windows")
    x, y = np.vstack([b[0] for b in blocks]), np.concatenate([b[1] for b in blocks])
    scale = np.linalg.norm(x, axis=0)
    if len(x) < 20 or np.any(scale < 1e-10):
        raise ValueError("Not enough excitation to identify the mechanical parameters")
    normalized = x/scale
    if np.linalg.matrix_rank(normalized) < x.shape[1]:
        raise ValueError("Mechanical parameters cannot be separated by this data")
    # Damping rate, torque gain and friction rate must be nonnegative. Bias is
    # free. Enumerate the few active boundaries; do not change a fitted sign.
    constrained = (0, 1, 3) if friction else (0, 1)
    best = None
    for mask in itertools.product((False, True), repeat=len(constrained)):
        fixed = {index for index, active in zip(constrained, mask) if active}
        free = [index for index in range(x.shape[1]) if index not in fixed]
        coefficients = np.zeros(x.shape[1])
        coefficients[free] = np.linalg.lstsq(normalized[:, free], y, rcond=None)[0]
        if any(coefficients[index] < -1e-10 for index in constrained):
            continue
        for index in constrained:
            coefficients[index] = max(0., coefficients[index])
        loss = float(np.mean((normalized@coefficients-y)**2))
        if best is None or loss < best[0]:
            best = (loss, coefficients/scale)
    if best is None or best[1][1] <= 1e-9:
        raise ValueError("Positive finite inertia is not identifiable under physical constraints")
    damping, gain, bias = best[1][:3]
    parameters = {"J_kg_m2": float(1/gain), "B_Nm_s_rad": float(damping/gain),
                  "bias_Nm": float(bias/gain),
                  "Fc_Nm": float(best[1][3]/gain) if friction else 0.}
    return {"parameters_conditional_on_torque_calibration": parameters,
            "normalized_design_condition": float(np.linalg.cond(normalized)),
            "integral_windows": len(x), "velocity_increment_rmse_rad_s": math.sqrt(best[0])}


def simulate(trial, parameters):
    j, b, fc, bias = (parameters[key] for key in ("J_kg_m2", "B_Nm_s_rad", "Fc_Nm", "bias_Nm"))
    if not (j > 0 and b >= 0 and fc >= 0 and all(math.isfinite(v) for v in (j, b, fc, bias))):
        raise ValueError("Invalid mechanical parameters")
    angle, velocity = float(trial["theta"][0]), float(trial["omega"][0])
    predicted = np.empty((len(trial["t"]), 2))
    predicted[0] = angle, velocity
    for k, dt in enumerate(np.diff(trial["t"]), 1):
        torque = float(trial["u"][k-1])
        if fc == 0:
            acceleration = (torque+bias)/j
            rate = b/j
            if rate <= 1e-10:
                angle += velocity*dt + .5*acceleration*dt*dt
                velocity += acceleration*dt
            else:
                decay_integral = -math.expm1(-rate*dt)/rate
                steady = acceleration/rate
                angle += (velocity-steady)*decay_integral + steady*dt
                velocity += (steady-velocity)*(-math.expm1(-rate*dt))
        else:
            count = max(1, math.ceil(dt/.002))
            h = float(dt/count)
            def acceleration(v):
                return (torque - b*v - fc*math.tanh(v/.02) + bias)/j
            for _ in range(count):
                k1 = acceleration(velocity)
                k2 = acceleration(velocity+.5*h*k1)
                k3 = acceleration(velocity+.5*h*k2)
                k4 = acceleration(velocity+h*k3)
                angle += h*(velocity + 2*(velocity+.5*h*k1)
                            + 2*(velocity+.5*h*k2) + velocity+h*k3)/6
                velocity += h*(k1+2*k2+2*k3+k4)/6
        if not math.isfinite(angle+velocity) or abs(velocity) > 1e6:
            raise ValueError("Free-run simulation diverged")
        predicted[k] = angle, velocity
    return predicted


def evaluate(group, parameters):
    speeds, speed_errors, angle_errors, each, predictions = [], [], [], [], []
    for trial in group:
        prediction = simulate(trial, parameters)
        ev = prediction[1:, 1]-trial["omega"][1:]
        ea = prediction[1:, 0]-trial["theta"][1:]
        measured = trial["omega"][1:]
        rmse = float(np.sqrt(np.mean(ev**2)))
        each.append({"trial": trial["number"], "profile": trial["profile"],
                     "duration_s": float(trial["t"][-1] - trial["t"][0]),
                     "velocity_fit_percent": float(100*(1-rmse/max(np.std(measured), 1e-12))),
                     "velocity_rmse_rad_s": rmse,
                     "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(ea**2))))})
        speeds.extend(measured); speed_errors.extend(ev); angle_errors.extend(ea)
        predictions.append(prediction)
    rmse = float(np.sqrt(np.mean(np.asarray(speed_errors)**2)))
    return {"velocity_fit_percent": float(100*(1-rmse/max(np.std(speeds), 1e-12))),
            "velocity_rmse_rad_s": rmse,
            "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(np.asarray(angle_errors)**2)))),
            "per_trial": each}, predictions


def bootstrap(blocks, friction, count=100):
    rng = np.random.default_rng(20261009)
    samples = []
    for _ in range(count):
        try:
            result = estimate([blocks[k] for k in rng.integers(0, len(blocks), len(blocks))], friction)
            samples.append(result["parameters_conditional_on_torque_calibration"])
        except ValueError:
            continue
    return {"attempted": count, "accepted": len(samples),
            "interpretation": "Whole-trial resampling variability only; excludes calibration error and systematic bias.",
            "percentile_95_ranges": {key: np.percentile([s[key] for s in samples], [2.5, 97.5]).tolist()
                                     for key in samples[0]} if samples else {}}


def analyze(path, train_count, out, allow_legacy=False):
    source_hash = hashlib.sha256(path.read_bytes()).hexdigest()
    data = read_log(path)
    pid_free = capture_is_pid_free(data)
    if not pid_free and not allow_legacy:
        raise ValueError("This capture does not prove PID-free yaw acquisition. Collect physical mode data, "
                         "or use --allow-legacy only for a labelled diagnostic report.")
    out.mkdir(parents=True, exist_ok=False)
    result = {"source": str(path.resolve()), "source_sha256": source_hash,
              "model": "theta_dot=omega; J*omega_dot=tau-B*omega-Fc*tanh(omega/0.02)+bias",
              "contains_pid_parameters": False, "pid_free_capture_evidence": pid_free,
              "status": "physical_candidate_requires_calibration_and_validation" if pid_free
                        else "legacy_diagnostic_not_eligible_for_pid_free_acquisition_requirement",
              "torque_calibration": "Unverified. J/B/Fc/bias values assume logged Nm equals yaw-axis Nm.",
              "evaluation": "Chronological complete-trial holdout; existing inspected logs are not a new blind test.",
              "channels": {}}
    for channel in TORQUES:
        if TORQUES[channel] not in data:
            continue
        group, rejected = load_trials(path, data, channel)
        if not 2 <= train_count < len(group):
            raise ValueError(f"Need >=2 training trials plus validation; found {len(group)} complete trials")
        training, validation = group[:train_count], group[train_count:]
        channel_result = {"training_trials": [t["number"] for t in training],
                          "validation_trials": [t["number"] for t in validation],
                          "rejected": rejected, "models": {}}
        for friction in (False, True):
            name = "linear" if not friction else "smooth_friction"
            try:
                blocks = integral_blocks(training, friction)
                model = estimate(blocks, friction)
                parameters = model["parameters_conditional_on_torque_calibration"]
                model["trial_bootstrap"] = bootstrap(blocks, friction)
                model["window_sensitivity"] = {}
                for window in (.025, .10):
                    try:
                        model["window_sensitivity"][str(window)] = estimate(
                            integral_blocks(training, friction, window), friction)["parameters_conditional_on_torque_calibration"]
                    except ValueError as error:
                        model["window_sensitivity"][str(window)] = {"error": str(error)}
                model["validation"], predictions = evaluate(validation, parameters)
                worst = int(np.argmax([r["angle_rmse_deg"] for r in model["validation"]["per_trial"]]))
                for index in sorted({0, worst}):
                    label = "Current-based torque estimate" if channel == "feedback" else "Total torque command"
                    validation_plot(out/f"{channel}_{name}_trial_{validation[index]['number']}.svg",
                                    validation[index], predictions[index], label)
                channel_result["models"][name] = model
            except ValueError as error:
                channel_result["models"][name] = {"error": str(error)}
        result["channels"][channel] = channel_result
    if not result["channels"]:
        raise ValueError("No torque channels in CSV")
    if hashlib.sha256(path.read_bytes()).hexdigest() != source_hash:
        raise ValueError("Source changed while analysing; use a completed, stopped capture")
    (out/"results.json").write_text(json.dumps(result, indent=2, allow_nan=False)+"\n")
    lines = ["# Yaw 机械本体辨识：候选参数检查", "",
             f"原始数据：`{result['source']}`", "",
             "方程：`theta_dot=omega; J*omega_dot=tau-B*omega-Fc*tanh(omega/0.02)+bias`。", "",
             "模型不读入 Yaw PID 增益或目标角度；只有力矩输入和角度、角速度状态。", "",
             f"采集是否有无 Yaw PID 证据：**{'有' if pid_free else '没有；旧数据仅作诊断，不能满足本次采集要求'}**。", "",
             "下表是条件性候选值：尚未确认日志中 Nm 与实际 Yaw 轴力矩的换算。", "",
             "| 输入 | 模型 | J (kg m²) | B (Nm s/rad) | Fc (Nm) | 偏置 (Nm) | 留出速度拟合度 | 角度 RMSE |",
             "|---|---|---:|---:|---:|---:|---:|---:|"]
    for channel, record in result["channels"].items():
        for name, model in record["models"].items():
            if "error" in model:
                lines.append(f"| {channel} | {name} | 无法辨识：{model['error']} | | | | | |")
                continue
            p, v = model["parameters_conditional_on_torque_calibration"], model["validation"]
            lines.append(f"| {channel} | {name} | {p['J_kg_m2']:.4g} | {p['B_Nm_s_rad']:.4g} | "
                         f"{p['Fc_Nm']:.4g} | {p['bias_Nm']:.4g} | {v['velocity_fit_percent']:.2f}% | {v['angle_rmse_deg']:.3f} deg |")
    lines += ["", "反馈输入来自电流换算的力矩估计；command 是限幅/变化率处理后的总命令。", "",
              "训练与验证按完整轮次先后分开，未根据验证分数选择模型或调整系数。模型只用每轮开头一个状态初始化，之后整段自由预测。", "",
              "`results.json` 包含逐轮结果、25/100 ms 窗口敏感性和整轮重采样范围。重采样范围不包括力矩标定误差、反馈偏差或模型失配。", "",
              "参数估计仍是初步积分回归。旧闭环数据可能存在估计偏差；无 PID 采集不自动保证物理参数准确。", "",
              "下一步：核对力矩常数与传动折算；采集相同负载/姿态下的正反向、多频率数据；固定模型后用新会话验证，必要时加入有证据的执行器动态或位置相关负载。", ""]
    (out/"REPORT.md").write_text("\n".join(lines))
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, required=True)
    parser.add_argument("--train-trials", type=int, required=True)
    parser.add_argument("--out", type=Path, required=True, help="New output directory; never overwrites an existing run")
    parser.add_argument("--allow-legacy", action="store_true", help="Diagnostic only; does not satisfy PID-free acquisition")
    args = parser.parse_args()
    try:
        result = analyze(args.log, args.train_trials, args.out, args.allow_legacy)
    except (ValueError, FileExistsError) as error:
        parser.error(str(error))
    print(json.dumps({"status": result["status"], "pid_free_capture_evidence": result["pid_free_capture_evidence"],
                      "report": str(args.out/"REPORT.md")}, indent=2))


if __name__ == "__main__":
    main()
