#!/usr/bin/env python3
"""Analyze a completed alternating torque trial using only saved local CSVs."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml

from identify import read_log, trials
from physical_identification import capture_is_pid_free, estimate, evaluate, integral_blocks
from svg_plot import validation_plot


HERE = Path(__file__).resolve().parent
DEFAULT_ALTERNATING = HERE / "physical_data/20261014_085047_131865"
DEFAULT_POSITIVE = HERE / "physical_data/20261014_084310_283272"
DEFAULT_POSITIVE_LOW = HERE / "physical_data/20261014_083952_694248"
DEFAULT_NEGATIVE = HERE / "physical_data/20261014_083442_122982"


def one_trial(folder):
    path = folder / "feedback.csv"
    data = read_log(path)
    accepted, rejected = trials(data)
    if len(accepted) != 1 or rejected:
        raise ValueError(f"Expected exactly one completed sweep in {path}: {rejected}")
    if not capture_is_pid_free(data):
        raise ValueError(f"Yaw PID-free acquisition cannot be verified in {path}")
    return data, accepted[0], hashlib.sha256(path.read_bytes()).hexdigest()


def nearest(trial, seconds):
    return int(np.argmin(np.abs(trial["t"] - seconds)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--alternating", type=Path, default=DEFAULT_ALTERNATING)
    parser.add_argument("--positive-training", type=Path, default=DEFAULT_POSITIVE)
    parser.add_argument("--positive-low-training", type=Path, default=DEFAULT_POSITIVE_LOW)
    parser.add_argument("--negative-training", type=Path, default=DEFAULT_NEGATIVE)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)

    data, trial, source_hash = one_trial(args.alternating)
    config = yaml.safe_load((args.alternating / "config.yaml").read_text())
    p = config["yaw_experiment"]["ros__parameters"]
    if not (p["physical_alternating_pulses"] and p["mechanically_fixed_pitch"]
            and p["passive_pitch"] and not p["physical_step_pulse"]):
        raise ValueError("The saved configuration is not the fixed-Pitch alternating trial")
    session = json.loads((args.alternating / "session.json").read_text())
    if session["outcome"] != "complete" or session["completed_trials"] != 1:
        raise ValueError("The saved session did not complete exactly one trial")

    t, theta, omega, command = (trial[k] for k in ("t", "theta", "omega", "u"))
    active = np.flatnonzero(data["/yaw_experiment/state"] == 3)
    feedback = data["/gimbal/yaw/torque"][active]
    pulse_start = float(p["physical_alternating_start_s"])
    on = float(p["physical_alternating_on_s"])
    gap = float(p["physical_alternating_gap_s"])
    cycles = int(p["physical_alternating_cycles"])
    half_period = on + gap
    pulses = []
    for half in range(2 * cycles):
        start = pulse_start + half * half_period
        end = start + on
        first, last = nearest(trial, start), nearest(trial, end)
        sign = int(p["physical_alternating_first_direction"]) * (1 if half % 2 == 0 else -1)
        pulses.append({
            "number": half + 1, "direction": sign,
            "start_s": start, "end_s": end,
            "angle_start_deg": float(np.rad2deg(theta[first])),
            "angle_end_deg": float(np.rad2deg(theta[last])),
            "angle_change_deg": float(np.rad2deg(theta[last] - theta[first])),
            "velocity_start_rad_s": float(omega[first]),
            "velocity_end_rad_s": float(omega[last]),
            "command_impulse_Nm_s": float(np.trapz(command[first:last + 1], t[first:last + 1])),
            "feedback_impulse_Nm_s": float(np.trapz(feedback[first:last + 1], t[first:last + 1])),
        })

    cycle_boundaries = [0.0] + [pulse_start + 2 * i * half_period for i in range(1, cycles + 1)]
    cycle_angles = [float(np.rad2deg(theta[nearest(trial, time)])) for time in cycle_boundaries]
    dt = np.diff(t)
    summary = {
        "source": str((args.alternating / "feedback.csv").resolve()),
        "source_sha256": source_hash,
        "samples": len(t), "median_interval_s": float(np.median(dt)),
        "maximum_interval_s": float(np.max(dt)),
        "yaw_start_deg": float(np.rad2deg(np.unwrap(data["/gimbal/yaw/angle"][active])[0])),
        "yaw_range_relative_deg": [float(np.rad2deg(np.min(theta))), float(np.rad2deg(np.max(theta)))],
        "yaw_end_relative_deg": float(np.rad2deg(theta[-1])),
        "imu_integrated_deg": float(np.rad2deg(np.trapz(omega, t))),
        "peak_absolute_imu_speed_rad_s": float(np.max(np.abs(omega))),
        "command_range_Nm": [float(np.min(command)), float(np.max(command))],
        "command_integral_Nm_s": float(np.trapz(command, t)),
        "feedback_integral_Nm_s": float(np.trapz(feedback, t)),
        "max_absolute_pitch_command_Nm": float(np.max(np.abs(data["/gimbal/pitch/control_torque"]))),
        "max_absolute_yaw_pid_Nm": float(np.max(np.abs(data["/yaw_experiment/yaw_pid_torque"]))),
        "max_absolute_yaw_feedforward_Nm": float(np.max(np.abs(data["/yaw_experiment/yaw_feedforward"]))),
        "fault_values": sorted(set(data["/yaw_experiment/fault"].tolist())),
        "guard_values": sorted(set(data["/yaw_experiment/physical_guard_stop"].tolist())),
        "cycle_boundary_times_s": cycle_boundaries,
        "cycle_boundary_angles_deg": cycle_angles,
        "cycle_changes_deg": np.diff(cycle_angles).tolist(),
        "pulses": pulses,
        "model_challenge": {},
    }

    pooled_training = []
    for label, folder in (("positive_high_92deg", args.positive_training),
                          ("positive_low_91deg", args.positive_low_training),
                          ("negative_92deg", args.negative_training)):
        training_data, training, training_hash = one_trial(folder)
        pooled_training.append(training)
        training_active = np.flatnonzero(training_data["/yaw_experiment/state"] == 3)
        fitted = estimate(integral_blocks([training], friction=True), friction=True)
        metrics, predictions = evaluate([trial], fitted["parameters_conditional_on_torque_calibration"])
        summary["model_challenge"][label] = {
            "training_source_sha256": training_hash,
            "training_start_yaw_deg": float(np.rad2deg(
                training_data["/gimbal/yaw/angle"][training_active][0])),
            "parameters_conditional_on_torque_calibration":
                fitted["parameters_conditional_on_torque_calibration"],
            "velocity_fit_percent": metrics["velocity_fit_percent"],
            "velocity_rmse_rad_s": metrics["velocity_rmse_rad_s"],
            "angle_rmse_deg": metrics["angle_rmse_deg"],
            "predicted_final_angle_deg": float(np.rad2deg(predictions[0][-1, 0])),
        }
        validation_plot(args.out / f"{label}_heldout.svg", trial, predictions[0])

    pooled = estimate(integral_blocks(pooled_training, friction=True), friction=True)
    pooled_metrics, pooled_predictions = evaluate(
        [trial], pooled["parameters_conditional_on_torque_calibration"])
    summary["model_challenge"]["pooled_three_steps"] = {
        "parameters_conditional_on_torque_calibration":
            pooled["parameters_conditional_on_torque_calibration"],
        "velocity_fit_percent": pooled_metrics["velocity_fit_percent"],
        "velocity_rmse_rad_s": pooled_metrics["velocity_rmse_rad_s"],
        "angle_rmse_deg": pooled_metrics["angle_rmse_deg"],
        "predicted_final_angle_deg": float(np.rad2deg(pooled_predictions[0][-1, 0])),
    }
    validation_plot(args.out / "pooled_three_steps_heldout.svg", trial, pooled_predictions[0])

    if hashlib.sha256((args.alternating / "feedback.csv").read_bytes()).hexdigest() != source_hash:
        raise ValueError("Alternating source changed during analysis")
    (args.out / "results.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
    print(args.out / "results.json")


if __name__ == "__main__":
    main()
