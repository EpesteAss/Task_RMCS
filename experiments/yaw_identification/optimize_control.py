#!/usr/bin/env python3
"""Derive a conservative yaw controller and matched baseline/tuned test configs."""
from __future__ import annotations

import argparse
from copy import deepcopy
import json
import math
from pathlib import Path

import yaml

from identify import fit, read_log, trials


HERE = Path(__file__).resolve().parent


def make_config(base: dict, name: str, control: dict) -> dict:
    config = deepcopy(base)
    params = config["yaw_experiment"]["ros__parameters"]
    params.update({
        "tracking_test": True,
        "tracking_amplitude_deg": 5.0,
        "tracking_frequency_hz": 0.25,
        "tracking_ramp_s": 1.0,
        "duration": 20.0,
        "sweep_amplitude": 0.0,
    })
    params.update(control)
    collector = config["yaw_collector"]["ros__parameters"]
    collector["csv_path"] = f"/tmp/c_yaw_control_{name}_<timestamp>.csv"
    for signal in ("/yaw_experiment/yaw_target_offset", "/yaw_experiment/yaw_error",
                   "/yaw_experiment/yaw_feedforward", "/gimbal/yaw/temperature"):
        if signal not in collector["signals"]:
            collector["signals"].append(signal)
    return config


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("data", type=Path, help="long identification feedback.csv")
    parser.add_argument("--burn-in-trials", type=int, default=30)
    parser.add_argument("--natural-frequency", type=float, default=12.0, help="desired rad/s")
    parser.add_argument("--damping", type=float, default=0.9)
    args = parser.parse_args()
    accepted, rejected = trials(read_log(args.data))
    if not (0 <= args.burn_in_trials < len(accepted) - 2):
        parser.error("burn-in must leave at least three complete trials")
    stable = accepted[args.burn_in_trials:]
    coeff, condition, windows = fit(stable, False)
    a, b, c = map(float, coeff)
    if not (a < 0 and b > 0):
        raise SystemExit("Stable-segment model has nonphysical signs; do not synthesize gains")

    # For omega_dot=a*omega+b*u+c and u=Kv*(Ka*e-omega), the
    # characteristic polynomial is s^2+(-a+b*Kv)s+b*Kv*Ka.
    wn, zeta = args.natural_frequency, args.damping
    velocity_kp = (2 * zeta * wn + a) / b
    if velocity_kp <= 0:
        raise SystemExit("Requested poles require a nonpositive velocity gain")
    angle_kp = wn * wn / (b * velocity_kp)
    ff_velocity, ff_acceleration, ff_bias = -a / b, 1 / b, -c / b

    source = yaml.safe_load((HERE / "vehicle-c.source.yaml").read_text())
    original = source["gimbal_controller"]["ros__parameters"]
    baseline = {
        "yaw_angle_kp": original["yaw_angle_kp"],
        "yaw_angle_ki": original["yaw_angle_ki"],
        "yaw_angle_kd": original["yaw_angle_kd"],
        "yaw_angle_integral_min": -0.5,
        "yaw_angle_integral_max": 0.5,
        "yaw_angle_output_min": -100.0,
        "yaw_angle_output_max": 100.0,
        "yaw_velocity_kp": original["yaw_velocity_kp"],
        "yaw_velocity_ki": original["yaw_velocity_ki"],
        "yaw_velocity_kd": original["yaw_velocity_kd"],
        "yaw_velocity_integral_min": original["yaw_velocity_integral_min"],
        "yaw_velocity_integral_max": original["yaw_velocity_integral_max"],
        "yaw_velocity_output_min": -100.0,
        "yaw_velocity_output_max": 100.0,
        "yaw_velocity_ff_torque_gain": 0.0,
        "yaw_acceleration_ff_torque_gain": 0.0,
        "yaw_bias_ff_torque": 0.0,
        "yaw_torque_limit": 4.5,
        "yaw_torque_slew_Nm_s": 1e6,
    }
    tuned = {
        "yaw_angle_kp": angle_kp,
        "yaw_angle_ki": 0.0,
        "yaw_angle_kd": 0.0,
        "yaw_angle_integral_min": -0.5,
        "yaw_angle_integral_max": 0.5,
        "yaw_angle_output_min": -1.2,
        "yaw_angle_output_max": 1.2,
        "yaw_velocity_kp": velocity_kp,
        "yaw_velocity_ki": original["yaw_velocity_ki"],
        "yaw_velocity_kd": 0.0,
        "yaw_velocity_integral_min": -3.0,
        "yaw_velocity_integral_max": 3.0,
        "yaw_velocity_output_min": -3.6,
        "yaw_velocity_output_max": 3.6,
        "yaw_velocity_ff_torque_gain": ff_velocity,
        "yaw_acceleration_ff_torque_gain": ff_acceleration,
        "yaw_bias_ff_torque": ff_bias,
        "yaw_torque_limit": 3.6,
        "yaw_torque_slew_Nm_s": 12.0,
    }
    base = yaml.safe_load((HERE / "identification.yaml").read_text())
    for name, values in (("baseline", baseline), ("tuned", tuned)):
        path = HERE / f"control_{name}.yaml"
        path.write_text(yaml.safe_dump(make_config(base, name, values), sort_keys=False))

    # Keep a directly deployable RMCS parameter-only candidate as well.  This
    # uses the real DeformableInfantryGimbalController; the automated harness
    # above adds the feedforward/constraints needed for matched no-DR16 tests.
    rmcs_tuned = yaml.safe_load((HERE / "baseline.yaml").read_text())
    rmcs_params = rmcs_tuned["gimbal_controller"]["ros__parameters"]
    rmcs_params.update({
        "yaw_angle_kp": angle_kp,
        "yaw_angle_ki": 0.0,
        "yaw_angle_kd": 0.0,
        "yaw_velocity_kp": velocity_kp,
        "yaw_velocity_ki": original["yaw_velocity_ki"],
        "yaw_velocity_kd": 0.0,
        "yaw_velocity_integral_min": -3.0,
        "yaw_velocity_integral_max": 3.0,
    })
    (HERE / "tuned.yaml").write_text(yaml.safe_dump(rmcs_tuned, sort_keys=False))

    record = {
        "stage": "controller_candidate_generated_not_yet_tested",
        "identification_data": str(args.data),
        "complete_trials": len(accepted),
        "rejected_trials": rejected,
        "stable_trials_used": [trial["number"] for trial in stable],
        "model": {
            "equation": "theta_dot=omega; omega_dot=a*omega+b*u+c",
            "a": a, "b": b, "c": c,
            "normalized_design_condition": condition,
            "regression_windows": windows,
        },
        "design": {
            "desired_natural_frequency_rad_s": wn,
            "desired_damping_ratio": zeta,
            "formula": {
                "velocity_kp": "(2*zeta*wn+a)/b",
                "angle_kp": "wn^2/(b*velocity_kp)",
                "velocity_feedforward": "-a/b",
                "acceleration_feedforward": "1/b",
                "bias_feedforward": "-c/b",
            },
        },
        "baseline": baseline,
        "tuned_candidate": tuned,
        "matched_test": {
            "target": "5 deg sine with 1 s raised-cosine start/end envelope",
            "frequency_hz": 0.25,
            "duration_s": 20.0,
            "planned_repetitions_each": 3,
            "metrics": ["angle RMSE", "absolute-error p95", "torque RMS", "peak torque",
                        "saturation fraction"],
        },
        "implementation_note": (
            "Baseline uses the original RMCS cascaded PID gains and the same copied PidCalculator; "
            "the deterministic target and pitch holder replace unavailable DR16 input."
        ),
        "pitch_holder_safety": {
            "observed_fault": "fault 4 while raising from about -8.3 deg; pitch speed reached the 0.8 rad/s hard trip",
            "cause": "the symmetric 5 Nm/s torque slew kept about -4.1 Nm applied after the PID demand had fallen to about -2.3 Nm",
            "correction": "retain the hard speed trip; apply torque at 5 Nm/s and release torque at 30 Nm/s",
        },
        "deployable_parameter_candidate": {
            "file": "tuned.yaml",
            "controller": "rmcs_core::controller::gimbal::DeformableInfantryGimbalController",
            "scope": "PID gains only; model feedforward and output constraints are implemented in the experiment addon",
        },
    }
    (HERE / "optimization_record.json").write_text(
        json.dumps(record, indent=2, allow_nan=False) + "\n")
    print(json.dumps({"model": record["model"], "baseline": baseline,
                      "tuned_candidate": tuned}, indent=2))
    print("Wrote control_baseline.yaml, control_tuned.yaml, optimization_record.json")


if __name__ == "__main__":
    main()
