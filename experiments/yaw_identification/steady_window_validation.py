#!/usr/bin/env python3
"""Freeze a later-cycle Yaw model, then test future sweeps without refitting."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from analyze_alternating_trial import one_trial
from identify import read_log, sliced, trials
from physical_identification import capture_is_pid_free, estimate, evaluate, integral_blocks
from svg_plot import validation_plot


HERE = Path(__file__).resolve().parent
DEFAULT_TRAINING = [
    HERE / "physical_data/20261014_085047_131865",
    HERE / "physical_data/20261014_122053_543065",
    HERE / "physical_data/20261014_122309_735803",
]
WINDOW = (1.3, 3.3)
MODEL = "J*omega_dot=command_torque-B*omega-Fc*tanh(omega/0.02)+bias"


def later_cycles(trial, start=WINDOW[0], stop=WINDOW[1]):
    first, after = np.searchsorted(trial["t"], [start, stop])
    if after - first < 100:
        raise ValueError("Later-cycle window has too few samples")
    return sliced(trial, first, after)


def metrics_for(group, parameters):
    score, _ = evaluate(group, parameters)
    return {key: score[key] for key in
            ("velocity_fit_percent", "velocity_rmse_rad_s", "angle_rmse_deg")}


def freeze(args):
    group, hashes = [], []
    for folder in args.training:
        _, trial, digest = one_trial(folder)
        group.append(later_cycles(trial))
        hashes.append({"folder": str(folder.resolve()), "feedback_sha256": digest})
    if len(group) < 3:
        raise ValueError("At least three complete training sweeps are required")
    cross_validation = []
    for held in range(len(group)):
        training = [trial for index, trial in enumerate(group) if index != held]
        fitted = estimate(integral_blocks(training, friction=True), friction=True)
        cross_validation.append({"held_out_feedback_sha256": hashes[held]["feedback_sha256"],
                                 "parameters": fitted["parameters_conditional_on_torque_calibration"],
                                 **metrics_for([group[held]], fitted["parameters_conditional_on_torque_calibration"])})
    fitted = estimate(integral_blocks(group, friction=True), friction=True)
    result = {
        "status": "frozen_candidate_requires_new_session_validation_and_torque_calibration",
        "model": MODEL,
        "input_channel": "/gimbal/yaw/control_torque",
        "time_window_s": list(WINDOW),
        "window_reason": "The first pulse pair showed large history dependence; use only pairs 2 and 3.",
        "training_sources": hashes,
        "fit": fitted,
        "leave_one_trial_out": cross_validation,
        "parameter_units": {"J_kg_m2": "kg m^2", "B_Nm_s_rad": "N m s/rad",
                            "Fc_Nm": "N m", "bias_Nm": "N m"},
        "interpretation": "Conditional effective parameters only; logged Nm to real yaw-axis Nm is unverified.",
    }
    args.out.mkdir(parents=True, exist_ok=False)
    target = args.out / "frozen_model.json"
    target.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(target)


def validate(args):
    frozen = json.loads(args.frozen_model.read_text())
    if frozen["model"] != MODEL or frozen["time_window_s"] != list(WINDOW):
        raise ValueError("Frozen model or evaluation window differs from this script")
    source = args.holdout / "feedback.csv"
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    data = read_log(source)
    group, rejected = trials(data)
    if rejected or not group or not capture_is_pid_free(data):
        raise ValueError("Holdout requires complete PID-free sweeps only")
    if any(source["feedback_sha256"] == digest for source in frozen["training_sources"]):
        raise ValueError("Holdout source is already in training data")
    if not np.all(data["/yaw_experiment/physical_guard_stop"] == 0):
        raise ValueError("Holdout stopped at a physical guard")
    if not np.all(data["/gimbal/pitch/control_torque"] == 0):
        raise ValueError("Holdout Pitch command was not zero")
    selected = [later_cycles(trial) for trial in group]
    parameters = frozen["fit"]["parameters_conditional_on_torque_calibration"]
    metrics, predictions = evaluate(selected, parameters)
    args.out.mkdir(parents=True, exist_ok=False)
    for trial, prediction in zip(selected, predictions):
        validation_plot(args.out / f"heldout_trial_{trial['number']}.svg", trial, prediction)
    result = {"frozen_model": str(args.frozen_model.resolve()),
              "holdout_source": str(source.resolve()),
              "holdout_feedback_sha256": digest,
              "trial_count": len(group),
              "profiles": [trial["profile"] for trial in group],
              "time_window_s": list(WINDOW),
              "parameters_unchanged": parameters,
              "metrics": metrics}
    if hashlib.sha256(source.read_bytes()).hexdigest() != digest:
        raise ValueError("Holdout source changed during validation")
    target = args.out / "validation.json"
    target.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n")
    print(target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training", nargs="+", type=Path, default=DEFAULT_TRAINING)
    parser.add_argument("--frozen-model", type=Path)
    parser.add_argument("--holdout", type=Path)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    if bool(args.frozen_model) != bool(args.holdout):
        parser.error("Pass both --frozen-model and --holdout to validate a future sweep")
    validate(args) if args.holdout else freeze(args)


if __name__ == "__main__":
    main()
