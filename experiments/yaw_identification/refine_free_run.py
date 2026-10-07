#!/usr/bin/env python3
"""Compare second-order yaw models on three profile-matched groups of sweeps.

Group 1 fits candidates, group 2 selects the model, groups 1+2 refit it,
and group 3 is used once for final 24-second free-run validation.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from identify import read_log, trials
from svg_plot import validation_plot


def features(torque, velocity, dead_zone, variant, speed_scale):
    positive = np.maximum(torque - dead_zone, 0.0)
    negative = np.minimum(torque + dead_zone, 0.0)
    if variant == "symmetric":
        return [positive + negative]
    if variant == "directional":
        return [positive, negative]
    if variant == "friction":
        return [positive + negative, np.tanh(velocity / speed_scale)]
    if variant == "directional_friction":
        return [positive, negative, np.tanh(velocity / speed_scale)]
    raise ValueError(variant)


def fit(group, dead_zone, delay, variant, speed_scale):
    rows, targets = [], []
    first = max(2, delay)
    for trial in group:
        theta, torque, time = trial["theta"], trial["u"], trial["t"]
        speed = np.diff(theta) / np.diff(time)
        velocity = np.r_[speed[0], speed]
        terms = features(torque[first - delay:len(theta) - delay],
                         velocity[first - 1:len(theta) - 1],
                         dead_zone, variant, speed_scale)
        x = np.column_stack((theta[first - 1:-1], theta[first - 2:-2],
                             *terms, np.ones(len(theta) - first)))
        rows.append(x)
        targets.append(theta[first:])
    design, target = np.vstack(rows), np.concatenate(targets)
    scale = np.linalg.norm(design, axis=0)
    if np.any(scale < 1e-10):
        raise ValueError("Unexcited feature")
    coefficient = np.linalg.lstsq(design / scale, target, rcond=None)[0] / scale
    poles = np.roots([1.0, -coefficient[0], -coefficient[1]])
    input_count = 2 if variant.startswith("directional") else 1
    sensible = (np.max(np.abs(poles)) < 1.0
                and np.all(coefficient[2:2 + input_count] > 0))
    return coefficient, bool(sensible)


def predict(trial, coefficient, dead_zone, delay, variant, speed_scale):
    measured, torque, time = trial["theta"], trial["u"], trial["t"]
    predicted = np.empty_like(measured)
    first = max(2, delay)
    predicted[:first] = measured[:first]
    for k in range(first, len(measured)):
        velocity = (predicted[k - 1] - predicted[k - 2]) / (time[k - 1] - time[k - 2])
        terms = features(torque[k - delay], velocity, dead_zone, variant, speed_scale)
        predicted[k] = (coefficient[0] * predicted[k - 1]
                        + coefficient[1] * predicted[k - 2]
                        + np.dot(coefficient[2:-1], terms) + coefficient[-1])
        if not np.isfinite(predicted[k]) or abs(predicted[k]) > 1e3:
            return None
    return predicted


def evaluate(group, coefficient, dead_zone, delay, variant, speed_scale):
    errors, measured, angle_errors, by_profile = [], [], [], []
    for trial in group:
        theta = predict(trial, coefficient, dead_zone, delay, variant, speed_scale)
        if theta is None:
            return None
        speed = np.gradient(theta, trial["t"])
        error = speed - trial["omega"]
        errors.append(error)
        measured.append(trial["omega"])
        angle_errors.append(theta - trial["theta"])
        rmse = float(np.sqrt(np.mean(error**2)))
        std = float(np.std(trial["omega"]))
        by_profile.append({"profile": trial["profile"],
                           "velocity_fit_percent": 100 * (1 - rmse / max(std, 1e-12))})
    error, actual = np.concatenate(errors), np.concatenate(measured)
    return {"velocity_fit_percent": 100 * (1 - float(np.sqrt(np.mean(error**2)))
                                                / max(float(np.std(actual)), 1e-12)),
            "velocity_rmse_rad_s": float(np.sqrt(np.mean(error**2))),
            "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(np.concatenate(angle_errors)**2)))),
            "per_profile": by_profile}


def analyze(first_log, third_log):
    earlier, rejected_earlier = trials(read_log(first_log))
    later, rejected_later = trials(read_log(third_log))
    if len(earlier) < 18 or len(later) < 9:
        raise ValueError("Expected at least 18 earlier sweeps and at least 9 complete later sweeps")
    # The first log contains later hot-motor attempts. Only the first two
    # profile-complete groups belong to the training/selection protocol.
    # A running collector can append several sessions to the same CSV. Taking
    # the last complete group makes a newly collected nine-profile session the
    # validation set without requiring an executor restart.
    complete = earlier[:18] + later[-9:]
    groups = [complete[i:i + 9] for i in (0, 9, 18)]
    profiles = [[item["profile"] for item in group] for group in groups]
    if profiles[0] != profiles[1] or profiles[0] != profiles[2]:
        raise ValueError("The three groups must have matching excitation profiles")

    candidates = []
    for variant in ("symmetric", "directional", "friction", "directional_friction"):
        for dead_zone in (0.0, 0.15, 0.25, 0.35, 0.45):
            for delay in (1, 2, 3):
                for speed_scale in ((0.02, 0.08) if "friction" in variant else (0.02,)):
                    try:
                        coefficient, sensible = fit(groups[0], dead_zone, delay, variant, speed_scale)
                        if not sensible:
                            continue
                        score = evaluate(groups[1], coefficient, dead_zone, delay, variant, speed_scale)
                    except (ValueError, np.linalg.LinAlgError):
                        continue
                    if score is not None:
                        candidates.append((score["velocity_fit_percent"], variant, dead_zone,
                                           delay, speed_scale))
    if not candidates:
        raise ValueError("No stable candidate with positive torque response")
    selected = max(candidates)
    selection_fit, variant, dead_zone, delay, speed_scale = selected
    coefficient, sensible = fit(complete[:18], dead_zone, delay, variant, speed_scale)
    if not sensible:
        raise ValueError("Selected model became unstable after refitting")
    validation = evaluate(groups[2], coefficient, dead_zone, delay, variant, speed_scale)
    if validation is None:
        raise ValueError("Selected model diverged on validation")
    return {"sources": {"first_two_groups": str(first_log), "held_out_group": str(third_log)},
            "excluded_later_trials_in_first_log": len(earlier) - 18,
            "rejected_later_trials_in_first_log": rejected_earlier,
            "excluded_earlier_trials_in_held_out_log": len(later) - 9,
            "rejected_trials_in_held_out_log": rejected_later,
            "split": {"fit": [1, 9], "select": [10, 18],
                                            "refit": [1, 18], "validate": [19, 27]},
            "candidate_count": len(candidates),
            "selected_on_group_2": {"variant": variant, "dead_zone_Nm": dead_zone,
                                    "delay_samples": delay, "speed_scale_rad_s": speed_scale,
                                    "velocity_fit_percent": selection_fit},
            "coefficients": coefficient.tolist(),
            "validation": validation,
            "note": "Velocity fit is full 24 s free-run. The third group does not select coefficients or hyperparameters. It is independent evidence when it was collected after the model structure and selection grid were frozen."}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first-log", required=True, type=Path)
    parser.add_argument("--third-log", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()
    result = analyze(args.first_log, args.third_log)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    third, _ = trials(read_log(args.third_log))
    chosen = result["selected_on_group_2"]
    representative = third[-1]
    angle = predict(representative, np.asarray(result["coefficients"]),
                    chosen["dead_zone_Nm"], chosen["delay_samples"],
                    chosen["variant"], chosen["speed_scale_rad_s"])
    velocity = np.gradient(angle, representative["t"])
    validation_plot(args.out.with_name("validation_profile_8.svg"), representative,
                    np.column_stack((angle, velocity)))
    rows = result["validation"]["per_profile"]
    report = ["# Yaw 24 s free-run model refinement", "",
              "Three groups each contain nine frequency profiles. Group 1 fits candidates; group 2 selects the model; groups 1 and 2 refit coefficients; group 3 tests full-trial free-run prediction.", "",
              "| Model | Velocity fit | Velocity RMSE (rad/s) | Angle RMSE (deg) |",
              "|---|---:|---:|---:|",
              "| Earlier second-order + torque dead zone | 48.95% | 0.11748 | 5.95 |",
              f"| Second-order + torque dead zone + velocity-direction friction | {result['validation']['velocity_fit_percent']:.2f}% | {result['validation']['velocity_rmse_rad_s']:.5f} | {result['validation']['angle_rmse_deg']:.2f} |",
              "", "Model: theta[k] = p1*theta[k-1] + p2*theta[k-2] + b*phi(u[k-2]) + f*tanh(v[k-1]/0.02) + c; phi(u) = sign(u)*max(abs(u)-0.15,0). v[k-1] comes from the model's two previous angles. Sampling interval is about 0.01 s.",
              "", "Parameters (p1, p2, b, f, c): " + ", ".join(f"{x:.9g}" for x in result["coefficients"]),
              "", "| Profile | Velocity fit |", "|---:|---:|",
              *[f"| {row['profile']} | {row['velocity_fit_percent']:.2f}% |" for row in rows],
              "", "[Profile 8 validation plot](validation_profile_8.svg) | [Machine-readable results](results.json)",
              "", "Low-frequency profiles remain weak. Treat the third group as independent evidence only when it was collected after the model structure and selection grid were frozen. This is an offline identification model; it does not change the live controller.", ""]
    args.out.with_name("REPORT.md").write_text("\n".join(report))
    print(json.dumps(result, indent=2, ensure_ascii=False))
