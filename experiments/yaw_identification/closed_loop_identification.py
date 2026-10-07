#!/usr/bin/env python3
"""Identify a second-order closed-loop yaw tracking model from full sweeps.

Input is the known yaw angle reference, not motor torque. The model predicts
the entire 24-second response from the first two angle/velocity samples.
"""

import argparse
import json
from html import escape
from pathlib import Path

import numpy as np
import yaml

from identify import read_log, trials


HERE = Path(__file__).resolve().parent


def read_config(path):
    values = yaml.safe_load(path.read_text())["yaw_experiment"]["ros__parameters"]
    if not values["tracking_test"]:
        raise ValueError("The configuration must enable tracking_test")
    frequencies = values["tracking_frequencies_hz"]
    if len(frequencies) != 9:
        raise ValueError("Expected nine tracking frequencies")
    return {"frequencies_hz": frequencies,
            "amplitude_deg": values["tracking_amplitude_deg"],
            "ramp_s": values["tracking_ramp_s"],
            "duration_s": values["duration"],
            "speed_limit_rad_s": min(abs(values["yaw_angle_output_min"]),
                                     abs(values["yaw_angle_output_max"]))}


def reference(trial, config):
    time = trial["t"]
    ramp = config["ramp_s"]
    duration = config["duration_s"]
    envelope = np.ones_like(time)
    entering = time < ramp
    exiting = duration - time < ramp
    envelope[entering] = .5 * (1 - np.cos(np.pi * time[entering] / ramp))
    envelope[exiting] = .5 * (1 - np.cos(np.pi * (duration-time[exiting]) / ramp))
    frequency = config["frequencies_hz"][trial["profile"]]
    return np.deg2rad(config["amplitude_deg"]) * envelope * np.sin(2*np.pi*frequency*time)


def grouped_arrays(trials_group, config):
    length = min(len(item["t"]) for item in trials_group)
    return {"time": np.array([item["t"][:length] for item in trials_group]),
            "angle": np.array([item["theta"][:length] for item in trials_group]),
            "velocity": np.array([item["omega"][:length] for item in trials_group]),
            "reference": np.array([reference(item, config)[:length]
                                   for item in trials_group])}


def simulate(parameters, group, speed_limit):
    """theta_dot=omega; omega_dot=-damping*omega+stiffness*(r-theta)+bias."""
    damping, stiffness = np.exp(parameters[:2])
    bias = parameters[2]
    time = group["time"]
    predicted_angle = np.empty_like(group["angle"])
    predicted_velocity = np.empty_like(group["velocity"])
    predicted_angle[:, :2] = group["angle"][:, :2]
    predicted_velocity[:, :2] = group["velocity"][:, :2]
    angle = predicted_angle[:, 1].copy()
    velocity = predicted_velocity[:, 1].copy()
    for k in range(2, time.shape[1]):
        step = time[:, k] - time[:, k-1]
        acceleration = (-damping*velocity
                        + stiffness*(group["reference"][:, k-1]-angle) + bias)
        velocity = np.clip(velocity + step*acceleration, -speed_limit, speed_limit)
        angle = angle + step*velocity
        predicted_velocity[:, k] = velocity
        predicted_angle[:, k] = angle
    return predicted_angle, predicted_velocity


def residual(parameters, group, speed_limit):
    _, velocity = simulate(parameters, group, speed_limit)
    return (velocity[:, 2:] - group["velocity"][:, 2:]).ravel()


def fit(group, speed_limit, initial=None):
    # Positive damping and stiffness are represented in log coordinates.
    parameters = (np.array([np.log(2.0), np.log(25.0), 0.0])
                  if initial is None else initial.copy())
    regularization = .01
    current = residual(parameters, group, speed_limit)
    loss = float(np.mean(current**2))
    for _ in range(40):
        columns = []
        for j in range(len(parameters)):
            changed = parameters.copy()
            changed[j] += .001
            columns.append((residual(changed, group, speed_limit)-current)/.001)
        jacobian = np.column_stack(columns)
        hessian = jacobian.T@jacobian/len(current)
        gradient = jacobian.T@current/len(current)
        accepted = False
        for _ in range(12):
            step = np.linalg.solve(hessian + regularization*np.eye(3), -gradient)
            proposed = parameters + step
            if np.max(np.abs(proposed[:2])) > 5 or abs(proposed[2]) > 20:
                regularization *= 5
                continue
            candidate = residual(proposed, group, speed_limit)
            candidate_loss = float(np.mean(candidate**2))
            if candidate_loss < loss:
                parameters, current, loss = proposed, candidate, candidate_loss
                regularization = max(regularization*.5, 1e-8)
                accepted = True
                break
            regularization *= 4
        if not accepted or np.linalg.norm(step) < 1e-6:
            break
    return parameters


def evaluate(group, parameters, speed_limit):
    angle, velocity = simulate(parameters, group, speed_limit)
    measured_velocity = group["velocity"]
    error = velocity-measured_velocity
    measured_std = float(np.std(measured_velocity))
    by_profile = []
    for k, trial_error in enumerate(error):
        profile_std = float(np.std(measured_velocity[k]))
        by_profile.append({"profile": k, "velocity_fit_percent": float(
            100*(1-np.sqrt(np.mean(trial_error**2))/max(profile_std, 1e-12)))})
    return {"velocity_fit_percent": float(100*(1-np.sqrt(np.mean(error**2))
                                                /max(measured_std, 1e-12))),
            "velocity_rmse_rad_s": float(np.sqrt(np.mean(error**2))),
            "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(
                (angle-group["angle"])**2)))),
            "per_profile": by_profile}, angle, velocity


def write_plot(path, trial, time, target, angle, velocity):
    panels = [("Yaw angle (deg)", [("Target", np.rad2deg(target), "#777777"),
                                    ("Measured", np.rad2deg(trial["theta"][:len(time)]), "#1565c0"),
                                    ("Model", np.rad2deg(angle), "#d35400")]),
              ("Yaw velocity (rad/s)", [("Measured", trial["omega"][:len(time)], "#1565c0"),
                                          ("Model", velocity, "#d35400")])]
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="640" viewBox="0 0 1000 640">',
           '<rect width="1000" height="640" fill="white"/>',
           '<g font-family="sans-serif" font-size="13" fill="#222">',
           f'<text x="80" y="28" font-size="18">24 s closed-loop yaw prediction, profile {trial["profile"]}</text>']
    for index, (label, curves) in enumerate(panels):
        left, top, width, height = 90, 80+index*280, 850, 195
        low = min(float(np.min(values)) for _, values, _ in curves)
        high = max(float(np.max(values)) for _, values, _ in curves)
        margin = max((high-low)*.08, .001)
        low, high = low-margin, high+margin
        svg.append(f'<text x="{left}" y="{top-22}">{escape(label)}</text>')
        for tick in range(6):
            x = left+tick*width/5
            y = top+height-tick*height/5
            svg.append(f'<path d="M {left} {y} H {left+width} M {x} {top} V {top+height}" stroke="#ddd" fill="none"/>')
            svg.append(f'<text x="{left-8}" y="{y+4}" text-anchor="end">{low+(high-low)*tick/5:.3g}</text>')
            svg.append(f'<text x="{x}" y="{top+height+20}" text-anchor="middle">{time[-1]*tick/5:.3g}</text>')
        sampled = np.unique(np.linspace(0, len(time)-1, min(2000, len(time))).astype(int))
        for number, (name, values, color) in enumerate(curves):
            points = " ".join(f"{left+time[k]/time[-1]*width:.2f},{top+(high-values[k])/(high-low)*height:.2f}" for k in sampled)
            svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.4"/>')
            svg.append(f'<text x="{left+360+number*150}" y="{top-22}" fill="{color}">{escape(name)}</text>')
        svg.append(f'<text x="{left+width/2}" y="{top+height+43}" text-anchor="middle">Time (s)</text>')
    svg.append('</g></svg>')
    path.write_text("\n".join(svg)+"\n")


def analyze(first_log, validation_log, config_path, output):
    config = read_config(config_path)
    for snapshot in (first_log.parent/"config.yaml", validation_log.parent/"config.yaml"):
        if snapshot.exists() and read_config(snapshot) != config:
            raise ValueError(f"Experiment configuration differs from recorded snapshot: {snapshot}")
    earlier, _ = trials(read_log(first_log))
    later, rejected = trials(read_log(validation_log))
    if len(earlier) < 18 or len(later) < 9 or rejected:
        raise ValueError("Need two earlier groups and nine complete validation sweeps")
    first, second, third = earlier[:9], earlier[9:18], later[-9:]
    expected = list(range(9))
    if any([item["profile"] for item in group] != expected
           for group in (first, second, third)):
        raise ValueError("Each group must contain profiles 0 through 8 in order")
    groups = [grouped_arrays(items, config) for items in (first, second, third)]
    speed_limit = config["speed_limit_rad_s"]
    first_parameters = fit(groups[0], speed_limit)
    selection, _, _ = evaluate(groups[1], first_parameters, speed_limit)
    combined = {key: np.concatenate([groups[0][key], groups[1][key]], axis=0)
                for key in groups[0]}
    final_parameters = fit(combined, speed_limit, first_parameters)
    validation, angle, velocity = evaluate(groups[2], final_parameters, speed_limit)
    result = {"model": "theta_dot=omega; omega_dot=-a*omega+k*(reference-theta)+c; |omega|<=speed_limit",
              "input": "known commanded yaw reference, not motor torque",
              "sources": {"train_and_selection": str(first_log),
                          "validation": str(validation_log), "config": str(config_path)},
              "config": config,
              "parameters": {"damping_per_s": float(np.exp(final_parameters[0])),
                             "tracking_stiffness_per_s2": float(np.exp(final_parameters[1])),
                             "bias_rad_s2": float(final_parameters[2])},
              "split": {"train": [1, 9], "selection": [10, 18],
                        "refit": [1, 18], "validation": [19, 27]},
              "selection_velocity_fit_percent": selection["velocity_fit_percent"],
              "validation": validation,
              "limitation": "This identifies the existing closed-loop controller plus plant under the recorded target profiles. It does not replace the torque-input mechanical model or prove new controller gains. Independent status depends on freezing this model and its configuration before collecting the validation session."}
    output.mkdir(parents=True, exist_ok=True)
    (output/"results.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
    profile = third[-1]
    n = groups[2]["time"].shape[1]
    write_plot(output/"validation_profile_8.svg", profile, groups[2]["time"][-1],
               groups[2]["reference"][-1], angle[-1], velocity[-1])
    write_plot(output/"validation_profile_0.svg", third[0], groups[2]["time"][0],
               groups[2]["reference"][0], angle[0], velocity[0])
    report = ["# Yaw closed-loop second-order identification", "",
              "Input is the known yaw angle target. The model simulates the complete 24 s response after two initial samples. It includes the configured 0.5 rad/s speed limit.",
              "", "Equation: theta_dot = omega; omega_dot = -a*omega + k*(reference-theta) + c; |omega| <= 0.5 rad/s.",
              "", "| Data | Full-run velocity fit | Speed RMSE (rad/s) | Angle RMSE (deg) |",
              "|---|---:|---:|---:|",
              f"| Nine-profile evaluation | {validation['velocity_fit_percent']:.2f}% | {validation['velocity_rmse_rad_s']:.5f} | {validation['angle_rmse_deg']:.3f} |",
              "", "| Profile | Velocity fit |", "|---:|---:|",
              *[f"| {item['profile']} | {item['velocity_fit_percent']:.2f}% |"
                for item in validation["per_profile"]],
              "", "This is a target-to-response model of the controlled yaw system. The torque-to-response model uses a different input, so its percentage must be reported separately. Treat a session as independent only when the model and configuration were frozen before collection.",
              "", "[Low-speed profile 0 plot](validation_profile_0.svg) | [Profile 8 plot](validation_profile_8.svg) | [Machine-readable results](results.json)", ""]
    (output/"REPORT.md").write_text("\n".join(report))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first-log", required=True, type=Path)
    parser.add_argument("--validation-log", required=True, type=Path)
    parser.add_argument("--config", type=Path, default=HERE/"identification_rich.yaml")
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(analyze(args.first_log, args.validation_log,
                             args.config, args.out), ensure_ascii=False, indent=2))
