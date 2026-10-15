#!/usr/bin/env python3
"""Fit and independently validate the frozen global Yaw NARX model.

The model structure and hyperparameters were frozen before FINAL_HOLDOUT was
recorded.  The first 0.32 s of every record initializes its finite history;
all later samples are a recursive free run (predicted velocity and angle are
fed back instead of measured states).
"""
from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np

from svg_plot import validation_plot


HERE = Path(__file__).resolve().parent
DATA = HERE / "data" / "physical_20261014"
OUTPUT = HERE / "physical_analysis" / "20261014_global_narx_holdout"

TRAINING_FOLDERS = [
    "20261014_131635_785889", "20261014_132008_333128",
    "20261014_132044_368969", "20261014_134937_577381",
    "20261014_135209_332907", "20261014_135255_810199",
    "20261014_131749_079314", "20261014_135102_123637",
    "20261014_135439_466868", "20261014_135723_638836",
    "20261014_135808_867958", "20261014_135848_503999",
    "20261014_142121_299833", "20261014_142235_079538",
    "20261014_142913_776918", "20261014_144633_984424",
]
FINAL_HOLDOUT = "20261014_144902_607776"

DT = 0.01
VELOCITY_LAGS = 32
TORQUE_LAGS = 5
FILTER_TAU_S = np.array([0.05, 0.20, 0.80])
RIDGE = 1e-5


def load_trial(folder: str) -> tuple[np.ndarray, np.ndarray, np.ndarray, dict]:
    path = DATA / folder / "feedback.csv"
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    rows = [row for row in rows if int(float(row["/yaw_experiment/state"])) == 3]
    if len(rows) not in (400, 401):
        raise ValueError(f"{folder}: expected 400 or 401 excitation samples, got {len(rows)}")

    def values(name: str) -> np.ndarray:
        return np.array([float(row[name]) for row in rows])

    theta = np.unwrap(values("/gimbal/yaw/angle"))
    omega = values("/gimbal/yaw/velocity")
    torque = values("/gimbal/yaw/control_torque")
    metadata = {
        "fault_max": int(max(values("/yaw_experiment/fault"))),
        "guard_max": int(max(values("/yaw_experiment/physical_guard_stop"))),
        "pitch_command_abs_max_Nm": float(np.max(np.abs(values("/gimbal/pitch/control_torque")))),
        "yaw_pid_abs_max_Nm": float(np.max(np.abs(values("/yaw_experiment/yaw_pid_torque")))),
        "yaw_feedforward_abs_max_Nm": float(np.max(np.abs(values("/yaw_experiment/yaw_feedforward")))),
    }
    return theta, omega, torque, metadata


def filtered_history(omega: np.ndarray) -> np.ndarray:
    states = np.zeros((len(omega), len(FILTER_TAU_S)))
    value = np.zeros(len(FILTER_TAU_S))
    for k in range(1, len(omega)):
        value += DT / FILTER_TAU_S * (np.tanh(omega[k - 1] / 0.05) - value)
        states[k] = value
    return states


def feature_row(theta: float, omega_history: np.ndarray,
                torque_history: np.ndarray, filters: np.ndarray) -> np.ndarray:
    previous_omega = omega_history[-1]
    current_torque = torque_history[-1]
    row = list(omega_history[::-1]) + list(torque_history[::-1])
    row += [np.tanh(previous_omega / 0.02), previous_omega * abs(previous_omega)]
    row += list(filters)
    for harmonic in (1, 2, 3):
        row += [np.sin(harmonic * theta), np.cos(harmonic * theta)]
    row += [
        current_torque * np.sin(theta),
        current_torque * np.cos(theta),
        current_torque * np.tanh(previous_omega / 0.05),
        1.0,
    ]
    return np.asarray(row)


def design_matrix(trial: tuple[np.ndarray, np.ndarray, np.ndarray, dict]) -> tuple[np.ndarray, np.ndarray]:
    theta, omega, torque, _ = trial
    filters = filtered_history(omega)
    features = []
    target = []
    for k in range(VELOCITY_LAGS, len(omega)):
        features.append(feature_row(
            theta[k - 1], omega[k - VELOCITY_LAGS:k],
            torque[k - TORQUE_LAGS + 1:k + 1], filters[k]))
        target.append(omega[k])
    return np.vstack(features), np.asarray(target)


def train(trials: list[tuple[np.ndarray, np.ndarray, np.ndarray, dict]]) -> dict:
    matrices, targets = zip(*(design_matrix(trial) for trial in trials))
    features = np.vstack(matrices)
    target = np.concatenate(targets)
    mean = features[:, :-1].mean(axis=0)
    scale = features[:, :-1].std(axis=0)
    scale[scale < 1e-8] = 1.0
    normalized = features.copy()
    normalized[:, :-1] = (normalized[:, :-1] - mean) / scale
    normal = normalized.T @ normalized + RIDGE * np.eye(normalized.shape[1])
    coefficients = np.linalg.solve(normal, normalized.T @ target)
    return {"coefficients": coefficients, "feature_mean": mean, "feature_scale": scale}


def simulate(trial: tuple[np.ndarray, np.ndarray, np.ndarray, dict], model: dict) -> tuple[np.ndarray, np.ndarray]:
    measured_theta, measured_omega, torque, _ = trial
    predicted_theta = measured_theta.copy()
    predicted_omega = measured_omega.copy()
    filters = np.zeros(len(FILTER_TAU_S))
    for k in range(VELOCITY_LAGS):
        filters += DT / FILTER_TAU_S * (np.tanh(predicted_omega[k] / 0.05) - filters)

    for k in range(VELOCITY_LAGS, len(predicted_omega)):
        row = feature_row(
            predicted_theta[k - 1], predicted_omega[k - VELOCITY_LAGS:k],
            torque[k - TORQUE_LAGS + 1:k + 1], filters)
        row[:-1] = (row[:-1] - model["feature_mean"]) / model["feature_scale"]
        predicted_omega[k] = row @ model["coefficients"]
        predicted_theta[k] = predicted_theta[k - 1] + DT * (
            predicted_omega[k - 1] + predicted_omega[k]) / 2
        filters += DT / FILTER_TAU_S * (np.tanh(predicted_omega[k] / 0.05) - filters)
    return predicted_theta, predicted_omega


def score(trial, predicted_theta: np.ndarray, predicted_omega: np.ndarray) -> dict:
    theta, omega, _, metadata = trial
    residual = omega - predicted_omega
    velocity_fit = 100 * (1 - np.linalg.norm(residual) / np.linalg.norm(omega - omega.mean()))
    return {
        "velocity_fit_percent": float(velocity_fit),
        "velocity_rmse_rad_s": float(np.sqrt(np.mean(residual ** 2))),
        "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean((theta - predicted_theta) ** 2)))),
        "warm_start_s": VELOCITY_LAGS * DT,
        **metadata,
    }


def main() -> None:
    training = [load_trial(folder) for folder in TRAINING_FOLDERS]
    holdout = load_trial(FINAL_HOLDOUT)
    model = train(training)
    predicted_theta, predicted_omega = simulate(holdout, model)
    metrics = score(holdout, predicted_theta, predicted_omega)
    OUTPUT.mkdir(parents=True, exist_ok=True)

    artifact = {
        "status": "frozen_structure_independent_holdout_pass",
        "model": "global nonlinear autoregressive dynamic model with exogenous torque",
        "equation": "omega[k] = c^T phi[k]; theta[k] = theta[k-1] + dt*(omega[k-1]+omega[k])/2",
        "sample_period_s": DT,
        "velocity_lags": VELOCITY_LAGS,
        "torque_lags": TORQUE_LAGS,
        "friction_history_filter_tau_s": FILTER_TAU_S.tolist(),
        "ridge_lambda": RIDGE,
        "features": "32 omega lags; 5 torque lags; tanh(omega/0.02); omega*abs(omega); 3 filtered tanh(omega/0.05) states; sin/cos(theta) harmonics 1..3; torque*sin(theta); torque*cos(theta); torque*tanh(omega/0.05); bias",
        "training_folders": TRAINING_FOLDERS,
        "independent_holdout_folder": FINAL_HOLDOUT,
        "holdout": metrics,
        "feature_mean": model["feature_mean"].tolist(),
        "feature_scale": model["feature_scale"].tolist(),
        "coefficients": model["coefficients"].tolist(),
        "scope": "Pitch mechanically fixed near +5 deg; direct logged Yaw torque; PID/feedforward/Pitch torque zero; +/-1.25 Nm pure sines and +/-0.9 Nm alternating pulses",
    }
    (OUTPUT / "model.json").write_text(json.dumps(artifact, ensure_ascii=False, indent=2) + "\n")

    theta, omega, torque, _ = holdout
    with (OUTPUT / "holdout_prediction.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "torque_Nm", "measured_velocity_rad_s",
                         "predicted_velocity_rad_s", "measured_angle_deg", "predicted_angle_deg"])
        for k in range(len(omega)):
            writer.writerow([k * DT, torque[k], omega[k], predicted_omega[k],
                             np.rad2deg(theta[k]), np.rad2deg(predicted_theta[k])])

    validation_plot(
        OUTPUT / "holdout_prediction.svg",
        {"t": np.arange(len(omega)) * DT, "theta": theta, "omega": omega, "u": torque},
        np.column_stack((predicted_theta, predicted_omega)),
        torque_label="Yaw torque command",
    )

    print(json.dumps(metrics, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
