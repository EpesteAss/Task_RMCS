#!/usr/bin/env python3
"""Offline study of torque-to-yaw-velocity models with free-run validation."""

import argparse
import json
from pathlib import Path

import numpy as np

from identify import read_log, trials


def smooth(values, times, tau):
    if tau == 0:
        return values
    result = np.empty_like(values)
    result[0] = values[0]
    for k in range(1, len(values)):
        alpha = (times[k] - times[k - 1]) / (tau + times[k] - times[k - 1])
        result[k] = result[k - 1] + alpha * (values[k] - result[k - 1])
    return result


def input_terms(u, v, dead_zone, variant, scale):
    drive = np.sign(u) * np.maximum(np.abs(u) - dead_zone, 0.0)
    if variant == "friction":
        return drive, np.tanh(v / scale)
    return (drive,)


def fit(group, order, delay, dead_zone, variant, scale, tau):
    rows, targets = [], []
    start = max(order, delay)
    for trial in group:
        velocity = smooth(trial["omega"], trial["t"], tau)
        torque = trial["u"]
        terms = input_terms(torque[start-delay:len(velocity)-delay],
                            velocity[start-1:-1], dead_zone, variant, scale)
        design = np.column_stack((*[velocity[start-j:len(velocity)-j]
                                    for j in range(1, order+1)],
                                  *terms, np.ones(len(velocity)-start)))
        rows.append(design)
        targets.append(velocity[start:])
    x, y = np.vstack(rows), np.concatenate(targets)
    norm = np.linalg.norm(x, axis=0)
    if np.any(norm < 1e-10):
        raise ValueError("Unexcited feature")
    coeff = np.linalg.lstsq(x/norm, y, rcond=None)[0]/norm
    poles = np.roots([1, *(-coeff[:order])])
    sensible = np.max(np.abs(poles)) < 1 and coeff[order] > 0
    return coeff, bool(sensible)


def simulate(trial, coeff, order, delay, dead_zone, variant, scale, tau):
    measured = smooth(trial["omega"], trial["t"], tau)
    predicted = np.empty_like(measured)
    start = max(order, delay)
    predicted[:start] = measured[:start]
    for k in range(start, len(predicted)):
        terms = input_terms(trial["u"][k-delay], predicted[k-1], dead_zone, variant, scale)
        predicted[k] = (sum(coeff[j]*predicted[k-1-j] for j in range(order))
                        + np.dot(coeff[order:-1], terms) + coeff[-1])
        if not np.isfinite(predicted[k]) or abs(predicted[k]) > 10:
            return None
    return predicted


def evaluate(group, coeff, order, delay, dead_zone, variant, scale, tau):
    errors, measured, profiles = [], [], []
    for trial in group:
        predicted = simulate(trial, coeff, order, delay, dead_zone, variant, scale, tau)
        if predicted is None:
            return None
        actual = trial["omega"]
        err = predicted - actual
        errors.append(err)
        measured.append(actual)
        profiles.append({"profile": trial["profile"], "velocity_fit_percent": float(
            100*(1-np.sqrt(np.mean(err**2))/max(np.std(actual), 1e-12)))})
    error, actual = np.concatenate(errors), np.concatenate(measured)
    return {"velocity_fit_percent": float(100*(1-np.sqrt(np.mean(error**2))
                                                   /max(np.std(actual), 1e-12))),
            "velocity_rmse_rad_s": float(np.sqrt(np.mean(error**2))),
            "per_profile": profiles}


def analyze(first_path, validation_path):
    earlier, _ = trials(read_log(first_path))
    later, rejected = trials(read_log(validation_path))
    if len(earlier) < 18 or len(later) < 9 or rejected:
        raise ValueError("Need 18 earlier and 9 complete validation trials")
    first, selection, validation = earlier[:9], earlier[9:18], later[-9:]
    if [t["profile"] for t in first] != [t["profile"] for t in selection] \
            or [t["profile"] for t in first] != [t["profile"] for t in validation]:
        raise ValueError("Profile sequence differs between groups")
    candidates = []
    for order in (1, 2):
        for delay in (1, 2, 3):
            for dead_zone in (0, .15, .35):
                for variant in ("simple", "friction"):
                    for tau in (0, .02):
                        for scale in ((.02, .08) if variant == "friction" else (.02,)):
                            try:
                                coeff, sensible = fit(first, order, delay, dead_zone,
                                                      variant, scale, tau)
                                if not sensible:
                                    continue
                                metrics = evaluate(selection, coeff, order, delay,
                                                   dead_zone, variant, scale, tau)
                            except (ValueError, np.linalg.LinAlgError):
                                continue
                            if metrics is not None:
                                candidates.append((metrics["velocity_fit_percent"],
                                                   order, delay, dead_zone, variant, scale, tau))
    if not candidates:
        raise ValueError("No stable model")
    selected = max(candidates)
    selection_fit, order, delay, dead_zone, variant, scale, tau = selected
    coeff, sensible = fit(first+selection, order, delay, dead_zone, variant, scale, tau)
    if not sensible:
        raise ValueError("Refitted model unstable")
    return {"candidate_count": len(candidates), "selected": {
        "order_of_velocity": order, "delay_samples": delay, "dead_zone_Nm": dead_zone,
        "variant": variant, "speed_scale_rad_s": scale, "filter_tau_s": tau,
        "selection_fit_percent": selection_fit}, "coefficients": coeff.tolist(),
        "validation": evaluate(validation, coeff, order, delay, dead_zone,
                               variant, scale, tau)}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--first-log", required=True, type=Path)
    parser.add_argument("--validation-log", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    options = parser.parse_args()
    result = analyze(options.first_log, options.validation_log)
    options.out.parent.mkdir(parents=True, exist_ok=True)
    options.out.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps(result, indent=2, ensure_ascii=False))
