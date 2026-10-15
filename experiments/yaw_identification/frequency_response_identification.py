#!/usr/bin/env python3
"""Identify moving Yaw dynamics from steady high-amplitude sine trials."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from identify import read_log, trials


HERE = Path(__file__).resolve().parent
SOURCES = [
    ("positive_0.5_hz", "20261014_131635_785889", 0.5, 2.0, True),
    ("reverse_0.5_hz_holdout", "20261014_131749_079314", 0.5, 2.0, False),
    ("positive_1_hz", "20261014_132008_333128", 1.0, 1.0, True),
    ("positive_2_hz", "20261014_132044_368969", 2.0, 1.0, True),
]
TORQUE_AMPLITUDE = 1.25


def response(folder, frequency, start):
    source = folder / "feedback.csv"
    trial = trials(read_log(source))[0][0]
    selected = trial["t"] >= start
    t, u, velocity = (trial[key][selected] for key in ("t", "u", "omega"))
    basis = np.exp(-2j * np.pi * frequency * t)
    transfer = np.dot(velocity - velocity.mean(), basis) / np.dot(u - u.mean(), basis)
    impedance = 1 / transfer
    return {
        "feedback_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "frequency_hz": frequency,
        "steady_window_start_s": start,
        "velocity_gain_rad_s_per_Nm": float(abs(transfer)),
        "phase_deg": float(np.angle(transfer, deg=True)),
        "effective_inertia_kg_m2": float(impedance.imag / (2 * np.pi * frequency)),
        "effective_real_impedance_Nm_s_rad": float(impedance.real),
        "motion_range_deg": float(np.rad2deg(np.ptp(trial["theta"]))),
        "peak_speed_rad_s": float(np.max(np.abs(trial["omega"]))),
    }


def predict(frequency, inertia, damping, coulomb):
    omega = 2 * np.pi * frequency
    friction_fundamental = 4 * coulomb / np.pi
    quadratic = damping * damping + (omega * inertia) ** 2
    linear = 2 * damping * friction_fundamental
    constant = friction_fundamental ** 2 - TORQUE_AMPLITUDE ** 2
    speed_amplitude = (-linear + np.sqrt(linear * linear - 4 * quadratic * constant)) / (2 * quadratic)
    return {
        "velocity_gain_rad_s_per_Nm": float(speed_amplitude / TORQUE_AMPLITUDE),
        "phase_deg": float(-np.rad2deg(np.arctan2(
            omega * inertia * speed_amplitude,
            damping * speed_amplitude + friction_fundamental))),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=HERE / "data/physical_20261014")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    records = {}
    for label, folder, frequency, start, training in SOURCES:
        record = response(args.data_root / folder, frequency, start)
        record["training"] = training
        records[label] = record
    training = [record for record in records.values() if record["training"]]
    speed_amplitude = TORQUE_AMPLITUDE * np.array(
        [record["velocity_gain_rad_s_per_Nm"] for record in training])
    real_impedance = np.array(
        [record["effective_real_impedance_Nm_s_rad"] for record in training])
    design = np.column_stack((np.ones(len(training)), 4 / (np.pi * speed_amplitude)))
    damping, coulomb = np.linalg.lstsq(design, real_impedance, rcond=None)[0]
    inertia_values = np.array([record["effective_inertia_kg_m2"] for record in training])
    inertia = float(inertia_values.mean())
    holdout = records["reverse_0.5_hz_holdout"]
    predicted = predict(holdout["frequency_hz"], inertia, damping, coulomb)
    result = {
        "status": "validated_moving_high_amplitude_describing_function",
        "model": "T1 = (B + j*Omega*J + 4*Fc/(pi*A_velocity))*V1; A_velocity is peak velocity amplitude",
        "scope": "Pitch mechanically fixed near +5 deg; 1.25 Nm sine; moving periodic response",
        "torque_calibration": "Conditional on logged motor torque units; output-shaft calibration unavailable.",
        "parameters": {"J_kg_m2": inertia, "B_Nm_s_rad": float(damping),
                       "Fc_Nm": float(coulomb)},
        "training_inertia_range_kg_m2": [float(inertia_values.min()), float(inertia_values.max())],
        "trials": records,
        "reverse_0.5_hz_holdout": {
            "predicted": predicted,
            "measured": {key: holdout[key] for key in
                         ("velocity_gain_rad_s_per_Nm", "phase_deg")},
            "gain_error_percent": float(100 * (predicted["velocity_gain_rad_s_per_Nm"] /
                                                holdout["velocity_gain_rad_s_per_Nm"] - 1)),
            "phase_error_deg": float(predicted["phase_deg"] - holdout["phase_deg"]),
        },
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    print(args.out)


if __name__ == "__main__":
    main()
