#!/usr/bin/env python3
"""Fit a two-state yaw model and compare closed-loop trial logs."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def read_csv(path: Path) -> dict[str, np.ndarray]:
    with path.open(newline="", encoding="utf-8") as stream:
        records = list(csv.DictReader(stream))
    # The collector may still be writing when a snapshot is copied. Ignore
    # only the unfinished final row; malformed rows inside the trial are errors.
    if records and any(value is None or value == "" for value in records[-1].values()):
        records.pop()
    if not records:
        raise ValueError(f"Empty CSV: {path}")
    if any(value is None or value == "" for row in records for value in row.values()):
        raise ValueError(f"Incomplete CSV row before end of file: {path}")
    return {name: np.array([float(row[name]) for row in records], dtype=float)
            for name in records[0]}


def series(data: dict[str, np.ndarray], suffix: str) -> np.ndarray:
    yaw_key = "/gimbal/yaw" + suffix
    if yaw_key in data:
        return data[yaw_key]
    matches = [value for name, value in data.items() if name.endswith(suffix)]
    if len(matches) != 1:
        raise ValueError(f"Expected one column ending in {suffix!r}, found {len(matches)}")
    return matches[0]


def plot_lines(path: Path, x: np.ndarray, lines: list[tuple[str, np.ndarray]], ylabel: str) -> None:
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(9, 4))
    for label, values in lines:
        axis.plot(x, values, label=label, linewidth=1)
    axis.set(xlabel="Time (s)", ylabel=ylabel)
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(path, dpi=160)
    plt.close(figure)


def fit_model(path: Path, out: Path) -> dict:
    data = read_csv(path)
    if "/yaw_experiment/state" in data:
        active = np.flatnonzero(data["/yaw_experiment/state"] == 3)
        if not len(active):
            raise ValueError("No sweep samples: pitch holding alone is not an identification experiment")
        groups = np.split(active, np.flatnonzero(np.diff(active) > 1) + 1)
        selected = groups[-1]
        if selected[-1] + 1 >= len(data["/yaw_experiment/state"]):
            raise ValueError("Sweep is still running or log ended before completion")
        if data["/yaw_experiment/state"][selected[-1] + 1] != 2:
            raise ValueError("Last sweep was interrupted by pitch recovery/fault; record a complete trial")
        if "/yaw_experiment/completed" not in data or data["/yaw_experiment/completed"][selected[-1] + 1] != 1:
            raise ValueError("Last sweep did not complete its full duration")
        data = {key: value[selected] for key, value in data.items()}
        data["elapsed_s"] = data["/yaw_experiment/time_s"] - data["/yaw_experiment/time_s"][0]
    t = data.get("elapsed_s")
    if t is None:
        raise ValueError("Use the SweptFrequencyController CSV with an elapsed_s column")
    theta = np.unwrap(series(data, "/angle"))
    # The C-car controller closes its yaw velocity loop with the gimbal IMU.
    # Motor velocity can remain quantized at zero during small movements.
    omega = series(data, "/velocity_imu")
    u = series(data, "/control_torque")
    if not all(np.isfinite(values).all() for values in (t, theta, omega, u)):
        raise ValueError("Non-finite values in sweep feedback; collect a clean trial")
    angle_span = float(np.ptp(theta))
    if angle_span < 0.001:
        raise ValueError(
            f"Yaw did not move enough for identification: angle span {angle_span:.6g} rad. "
            "Increase sweep amplitude and record another complete trial")
    # omega_dot = a*omega + b*u + c; theta_dot = omega.
    # This keeps an explicit second-order angle model while allowing bias/friction.
    dt = np.diff(t)
    valid = (np.isfinite(dt) & (dt > 0) & (dt < 0.05)
             & np.isfinite(omega[:-1]) & np.isfinite(omega[1:])
             & np.isfinite(u[:-1]))
    if valid.sum() < 100:
        raise ValueError("Too few valid, time-ordered samples")
    indices = np.flatnonzero(valid)
    boundary = int(len(indices) * 0.7)
    train, validation = indices[:boundary], indices[boundary:]
    if len(validation) < 20:
        raise ValueError("Need at least 20 validation samples")
    design = np.column_stack((omega[train], u[train], np.ones(len(train))))
    if np.linalg.matrix_rank(design) < 3 or np.std(u[train]) < 1e-6:
        raise ValueError("Insufficient excitation to identify model parameters")
    derivative = (omega[train + 1] - omega[train]) / dt[train]
    a, b, c = np.linalg.lstsq(design, derivative, rcond=None)[0]
    if a >= 0:
        print("WARNING: fitted damping is non-positive; check excitation, units and data")
    # One-step held-out validation avoids using training rows for reported error.
    predicted_omega = omega[validation] + dt[validation] * (
        a * omega[validation] + b * u[validation] + c)
    predicted_theta = theta[validation] + dt[validation] * omega[validation]
    velocity_rmse = float(np.sqrt(np.mean((predicted_omega - omega[validation + 1]) ** 2)))
    angle_rmse = float(np.sqrt(np.mean((predicted_theta - theta[validation + 1]) ** 2)))
    metrics = {
        "model": "theta_dot=omega; omega_dot=a*omega+b*u+c",
        "a_per_s": float(a), "b_rad_per_s2_per_torque_unit": float(b),
        "c_rad_per_s2": float(c),
        "time_constant_s": float(-1 / a) if a < 0 else None,
        "train_pairs": len(train), "validation_pairs": len(validation),
        "validation_velocity_one_step_rmse_rad_s": velocity_rmse,
        "validation_angle_one_step_rmse_rad": angle_rmse,
        "measured_angle_span_rad": angle_span,
        "note": "b combines motor torque calibration with inertia; J and K cannot be separated without calibration",
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / "model.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    plot_lines(out / "validation_velocity.png", t[validation + 1] - t[validation[0]],
               [("measured", omega[validation + 1]), ("one-step prediction", predicted_omega)],
               "Yaw velocity (rad/s)")
    return metrics


def compare(path: Path, out: Path, name: str, sample_hz: float) -> dict:
    data = read_csv(path)
    error = series(data, "/control_angle_error")
    torque = series(data, "/control_torque")
    valid = np.isfinite(error) & np.isfinite(torque)
    if valid.sum() < 10:
        raise ValueError(f"No valid enabled-control samples in {path}")
    indices = np.flatnonzero(valid)
    error, torque = error[valid], torque[valid]
    results = {
        "samples": len(error),
        "error_rmse_rad": float(np.sqrt(np.mean(error ** 2))),
        "error_p95_abs_rad": float(np.percentile(np.abs(error), 95)),
        "peak_abs_torque_command": float(np.max(np.abs(torque))),
        "torque_rms_command": float(np.sqrt(np.mean(torque ** 2))),
        "note": "Metrics are comparable only for matched target trajectories and similar starting states",
    }
    plot_lines(out / f"{name}_error.png", indices / sample_hz,
               [(name, error)], "Yaw angle error (rad)")
    return results


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sweep", type=Path, help="CSV emitted by SweptFrequencyController")
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--tuned", type=Path)
    parser.add_argument("--sample-hz", type=float, default=500.0,
                        help="collector sample rate (1000/write_interval by default)")
    parser.add_argument("--out", type=Path, default=Path("analysis_output"))
    args = parser.parse_args()
    if not any((args.sweep, args.baseline, args.tuned)):
        parser.error("Provide --sweep and/or --baseline/--tuned")
    if args.sample_hz <= 0:
        parser.error("--sample-hz must be positive")
    args.out.mkdir(parents=True, exist_ok=True)
    result = {}
    if args.sweep:
        result["identification"] = fit_model(args.sweep, args.out)
    for name in ("baseline", "tuned"):
        path = getattr(args, name)
        if path:
            result[name] = compare(path, args.out, name, args.sample_hz)
    (args.out / "results.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
