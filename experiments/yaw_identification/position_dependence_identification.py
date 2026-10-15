#!/usr/bin/env python3
"""Compare frozen low-angle and newly fitted high-angle Yaw moving models."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from frequency_response_identification import predict, response
from identify import read_log, sliced, trials
from physical_identification import capture_is_pid_free, evaluate


HERE = Path(__file__).resolve().parent
DATA = HERE / "data/physical_20261014"
OLD_MODEL = HERE / "physical_analysis/20261014_high_amplitude_frequency_response/frequency_response.json"
SOURCES = [
    ("high_reverse_0p5_train", "20261014_134937_577381", .5, 2.0, True),
    ("high_reverse_0p5_repeat_holdout", "20261014_135102_123637", .5, 2.0, False),
    ("high_1hz_train", "20261014_135209_332907", 1.0, 1.0, True),
    ("mid_2hz_train", "20261014_135255_810199", 2.0, 1.0, True),
    ("mid_positive_0p5_holdout", "20261014_135439_466868", .5, 2.0, False),
    ("lower_positive_0p5_check", "20261014_135723_638836", .5, 2.0, False),
    ("low_positive_0p5_check", "20261014_135808_867958", .5, 2.0, False),
    ("low_reverse_0p5_check", "20261014_135848_503999", .5, 2.0, False),
]


def score(trial, params):
    index = int(np.searchsorted(trial["t"], 2.0))
    result = {}
    for name, window in (("full_0_to_4_s", trial),
                         ("moving_2_to_4_s", sliced(trial, index, len(trial["t"])))):
        metrics, _ = evaluate([window], params)
        result[name] = {key: metrics[key] for key in
                        ("velocity_fit_percent", "velocity_rmse_rad_s", "angle_rmse_deg")}
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    frozen = json.loads(OLD_MODEL.read_text())
    records = {}
    trial_by_label = {}
    for label, folder, hz, steady_start, training in SOURCES:
        source = DATA / folder / "feedback.csv"
        raw = read_log(source)
        group, rejected = trials(raw)
        if len(group) != 1 or rejected or not capture_is_pid_free(raw):
            raise ValueError(f"Not one complete PID-free trial: {source}")
        if (np.max(raw["/yaw_experiment/fault"]) != 0
                or np.max(raw["/yaw_experiment/physical_guard_stop"]) != 0
                or np.max(np.abs(raw["/gimbal/pitch/control_torque"])) != 0):
            raise ValueError(f"Fault, guard, or Pitch torque in {source}")
        record = response(DATA / folder, hz, steady_start)
        active = np.flatnonzero(raw["/yaw_experiment/state"] == 3)
        record.update({
            "source": str(source.relative_to(HERE)),
            "session": str((DATA / folder / "session.json").relative_to(HERE)),
            "config_sha256": hashlib.sha256((DATA / folder / "config.yaml").read_bytes()).hexdigest(),
            "start_angle_deg": float(np.rad2deg(raw["/gimbal/yaw/angle"][active[0]])),
            "angle_min_deg": float(np.rad2deg(np.min(raw["/gimbal/yaw/angle"][active]))),
            "angle_max_deg": float(np.rad2deg(np.max(raw["/gimbal/yaw/angle"][active]))),
            "training_high_angle_model": training,
        })
        records[label] = record
        trial_by_label[label] = group[0]
    training = [record for record in records.values() if record["training_high_angle_model"]]
    speed_amplitude = 1.25 * np.array([r["velocity_gain_rad_s_per_Nm"] for r in training])
    real_impedance = np.array([r["effective_real_impedance_Nm_s_rad"] for r in training])
    design = np.column_stack((np.ones(len(training)), 4 / (np.pi * speed_amplitude)))
    damping, coulomb = np.linalg.lstsq(design, real_impedance, rcond=None)[0]
    inertia = float(np.mean([r["effective_inertia_kg_m2"] for r in training]))
    high = {"J_kg_m2": inertia, "B_Nm_s_rad": float(damping), "Fc_Nm": float(coulomb)}
    old = frozen["parameters"]
    results = {}
    for label, record in records.items():
        pred = predict(record["frequency_hz"], inertia, damping, coulomb)
        record["high_angle_prediction"] = {
            **pred,
            "gain_error_percent": float(100 * (pred["velocity_gain_rad_s_per_Nm"] /
                                               record["velocity_gain_rad_s_per_Nm"] - 1)),
            "phase_error_deg": float(pred["phase_deg"] - record["phase_deg"]),
        }
        results[label] = {
            "old_low_angle_model": score(trial_by_label[label], {**old, "bias_Nm": 0.0}),
            "new_high_angle_model": score(trial_by_label[label], {**high, "bias_Nm": 0.0}),
        }
    output = {
        "status": "two_conditional_moving_models_not_global_physical_model",
        "old_model_source": str(OLD_MODEL.relative_to(HERE)),
        "old_model_parameters": old,
        "new_high_angle_model_parameters": high,
        "new_high_angle_model_training": [label for label, record in records.items()
                                          if record["training_high_angle_model"]],
        "caveat": "Angles are motor-encoder coordinates. All models use logged torque units; static startup, load history, and shaft torque calibration remain unresolved.",
        "records": records,
        "time_domain_free_simulation": results,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(output, indent=2, allow_nan=False) + "\n")
    print(args.out)


if __name__ == "__main__":
    main()
