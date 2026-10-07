#!/usr/bin/env python3
"""Explore a torque dead zone for 24 s free-run yaw prediction.

Chronological split: three equal profile-complete groups. The first group fits
candidate parameters, the second chooses the dead zone and input delay, the
first two refit, and the third reports final validation. Both the historical
18-trial/six-profile log and the new 27-trial/nine-profile log are supported.
The final six trials were already inspected for the earlier model, so this
comparison is exploratory; fresh trials are needed for confirmation.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from identify import evaluate_delayed_arx2, fit_delayed_arx2, read_log, trials
from svg_plot import validation_plot


HERE = Path(__file__).resolve().parent
DEFAULT_LOG = HERE / "data/20261006_133045_810815/feedback.csv"


def with_dead_zone(trial, threshold):
    torque = trial["u"]
    effective = np.sign(torque) * np.maximum(np.abs(torque) - threshold, 0)
    return {**trial, "u": effective}


def transformed(items, threshold):
    return [with_dead_zone(trial, threshold) for trial in items]


def stable(coefficients):
    return bool(np.max(np.abs(np.roots([1, -coefficients[0], -coefficients[1]]))) < 1)


def analyze(path, out):
    complete, rejected = trials(read_log(path))
    if len(complete) not in (18, 27) or rejected:
        raise ValueError("Expected exactly 18 or 27 complete sweeps with no rejected trials")
    group = len(complete) // 3
    first, middle, final = complete[:group], complete[group:2*group], complete[2*group:]
    if [trial["profile"] for trial in first] != [trial["profile"] for trial in middle] \
            or [trial["profile"] for trial in first] != [trial["profile"] for trial in final]:
        raise ValueError("The three chronological groups must have matching profiles")

    candidates = []
    for threshold in (0, .05, .10, .15, .20, .25, .30, .35):
        fit_trials = transformed(first, threshold)
        select_trials = transformed(middle, threshold)
        for delay in range(7):
            coeff, _, _ = fit_delayed_arx2(fit_trials, delay)
            if not stable(coeff):
                continue
            metrics, _ = evaluate_delayed_arx2(select_trials, coeff, delay)
            if not metrics["simulation_diverged"]:
                candidates.append((metrics["velocity_fit_percent"], threshold, delay))
    if not candidates:
        raise ValueError("No stable candidate")
    selection_fit, threshold, delay = max(candidates, key=lambda item: item[0])

    coeff, _, samples = fit_delayed_arx2(transformed(complete[:2*group], threshold), delay)
    if not stable(coeff):
        raise ValueError("Refitted model is unstable")
    metrics, plots = evaluate_delayed_arx2(transformed(final, threshold), coeff, delay)

    original_coeff, _, _ = fit_delayed_arx2(complete[:2*group], 1)
    original_metrics, _ = evaluate_delayed_arx2(final, original_coeff, 1)
    by_profile = []
    for trial in final:
        old, _ = evaluate_delayed_arx2([trial], original_coeff, 1)
        new, _ = evaluate_delayed_arx2([with_dead_zone(trial, threshold)], coeff, delay)
        by_profile.append({"profile": trial["profile"],
                           "measured_velocity_std_rad_s": float(np.std(trial["omega"])),
                           "original_velocity_fit_percent": old["velocity_fit_percent"],
                           "dead_zone_velocity_fit_percent": new["velocity_fit_percent"]})

    out.mkdir(parents=True, exist_ok=True)
    result = {
        "data": str(path), "split": {"fit": [1, group], "select": [group+1, 2*group],
                                      "refit": [1, 2*group],
                                      "validation": [2*group+1, 3*group]},
        "selected_on_middle_six": {"dead_zone_Nm": threshold,
                                   "delay_samples": delay,
                                   "velocity_fit_percent": selection_fit},
        "model": {"equation": "theta[k]=p1*theta[k-1]+p2*theta[k-2]+b*phi(u[k-delay])+c; phi(u)=sign(u)*max(abs(u)-dead_zone,0)",
                  "coefficients": coeff.tolist(), "fit_samples": samples,
                  "poles": [[float(z.real), float(z.imag)] for z in np.roots([1, -coeff[0], -coeff[1]])]},
        "original_held_out": original_metrics,
        "dead_zone_held_out": metrics,
        "per_profile": by_profile,
        "caveat": "Exploratory comparison on a previously inspected holdout; collect fresh trials for independent confirmation. Dead zone is an effective model parameter, not a measured friction torque.",
    }
    (out / "results.json").write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    validation_plot(out / "validation_profile_1.svg", final[1], plots[1][1])
    report = ["# 24 s free-run yaw prediction: exploratory dead-zone model", "",
              f"Three chronological groups of {group} trials: group 1 fits candidates, group 2 selects the dead zone and delay, groups 1–2 refit, and group 3 compares. All groups contain the same profiles.", "",
              "| Model | 24 s velocity fit | Velocity RMSE (rad/s) | Angle RMSE (deg) |",
              "|---|---:|---:|---:|",
              f"| Original delayed second-order ARX | {original_metrics['velocity_fit_percent']:.2f}% | {original_metrics['velocity_rmse_rad_s']:.5f} | {original_metrics['angle_rmse_deg']:.3f} |",
              f"| Second-order ARX + torque dead zone | {metrics['velocity_fit_percent']:.2f}% | {metrics['velocity_rmse_rad_s']:.5f} | {metrics['angle_rmse_deg']:.3f} |",
              "", f"Selected dead zone: {threshold:.2f} N·m; delay: {delay} samples. Middle-six selection fit: {selection_fit:.2f}%.",
              "", "| Profile | Measured velocity std (rad/s) | Original velocity fit | Dead-zone velocity fit |",
              "|---:|---:|---:|---:|",
              *[f"| {row['profile']} | {row['measured_velocity_std_rad_s']:.4f} | {row['original_velocity_fit_percent']:.2f}% | {row['dead_zone_velocity_fit_percent']:.2f}% |" for row in by_profile],
              "", "The dead zone represents an effective low-torque response and is not an independently calibrated friction measurement. The final six trials had already been inspected for the original model, so the comparison is exploratory. A new session is required to confirm generalization.",
              "", "Three low-dynamic profiles still have negative free-run velocity fit. The modest pooled improvement does not solve long-horizon prediction across all regimes.",
              "", "[Representative validation plot](validation_profile_1.svg) and [machine-readable results](results.json).", ""]
    (out / "REPORT.md").write_text("\n".join(report))
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--log", type=Path, default=DEFAULT_LOG)
    parser.add_argument("--out", type=Path, default=HERE / "analysis_output/20261006_133045_810815_dead_zone")
    args = parser.parse_args()
    result = analyze(args.log, args.out)
    print(f"Original: {result['original_held_out']['velocity_fit_percent']:.2f}%; "
          f"dead zone: {result['dead_zone_held_out']['velocity_fit_percent']:.2f}%")
    print(args.out)
