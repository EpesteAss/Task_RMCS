#!/usr/bin/env python3
"""Compare matched baseline and tuned yaw tracking trials."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from xml.sax.saxutils import escape

import numpy as np
import yaml

from identify import read_log


def complete_trials(path: Path) -> tuple[list[dict], list[dict]]:
    data = read_log(path)
    state = data["/yaw_experiment/state"]
    active = np.flatnonzero(state == 3)
    accepted, rejected = [], []
    for number, idx in enumerate(np.split(active, np.flatnonzero(np.diff(active) > 1) + 1), 1):
        if not len(idx):
            continue
        after = idx[-1] + 1
        if (after >= len(state) or state[after] != 2
                or data["/yaw_experiment/completed"][after] != 1):
            rejected.append({"trial": number, "reason": "interrupted or incomplete"})
            continue
        accepted.append({
            "number": number,
            "time": data["/yaw_experiment/time_s"][idx] - data["/yaw_experiment/time_s"][idx[0]],
            "target": data["/yaw_experiment/yaw_target_offset"][idx],
            "error": data["/yaw_experiment/yaw_error"][idx],
            "torque": data["/gimbal/yaw/control_torque"][idx],
            "pitch": np.rad2deg(data["/yaw_experiment/pitch_world"][idx]),
            "pitch_temp": data["/gimbal/pitch/temperature"][idx],
            "yaw_temp": data["/gimbal/yaw/temperature"][idx],
            "frequency_hz": (float(np.median(data["/yaw_experiment/tracking_frequency_hz"][idx]))
                             if "/yaw_experiment/tracking_frequency_hz" in data else None),
        })
    if not accepted:
        raise ValueError(f"No complete tracking trials in {path}")
    return accepted, rejected


def metrics(trials: list[dict], torque_limit: float) -> dict:
    error = np.concatenate([x["error"] for x in trials])
    torque = np.concatenate([x["torque"] for x in trials])
    target = np.concatenate([x["target"] for x in trials])
    return {
        "complete_trials": len(trials),
        "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(error**2)))),
        "angle_mae_deg": float(np.rad2deg(np.mean(np.abs(error)))),
        "angle_p95_abs_deg": float(np.rad2deg(np.percentile(np.abs(error), 95))),
        "peak_abs_error_deg": float(np.rad2deg(np.max(np.abs(error)))),
        "target_peak_deg": float(np.rad2deg(np.max(np.abs(target)))),
        "torque_rms_Nm": float(np.sqrt(np.mean(torque**2))),
        "peak_abs_torque_Nm": float(np.max(np.abs(torque))),
        "torque_limit_fraction": float(np.mean(np.abs(torque) >= torque_limit * 0.995)),
        "pitch_world_range_deg": [float(min(x["pitch"].min() for x in trials)),
                                  float(max(x["pitch"].max() for x in trials))],
        "max_pitch_temp_C": float(max(x["pitch_temp"].max() for x in trials)),
        "max_yaw_temp_C": float(max(x["yaw_temp"].max() for x in trials)),
    }


def svg_plot(path: Path, baseline: dict, tuned: dict) -> None:
    width, height, left, right = 1100, 820, 98, 28
    panels = [
        ("Yaw angle", "Angle (deg)", 75, 275,
         [("Target", "target", "#222222"), ("Original gains", "actual", "#d1495b"),
          ("Tuned", "actual", "#00798c")]),
        ("Tracking error", "Error (deg)", 325, 515,
         [("Original gains", "error", "#d1495b"), ("Tuned", "error", "#00798c")]),
        ("Yaw command", "Torque (N m)", 565, 755,
         [("Original gains", "torque", "#d1495b"), ("Tuned", "torque", "#00798c")]),
    ]
    frequency = baseline.get("frequency_hz")
    title = f"Matched yaw tracking: trial {baseline['number']}"
    if frequency is not None:
        title += f" ({frequency:.2f} Hz)"
    chunks = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
              '<rect width="100%" height="100%" fill="white"/>',
              f'<text x="98" y="27" font-family="sans-serif" font-size="20">{escape(title)}</text>']
    x_max = max(float(baseline["time"][-1]), float(tuned["time"][-1]))
    x_max = max(x_max, 1.0)
    for title, ylabel, top, bottom, series in panels:
        def values(trial: dict, key: str) -> np.ndarray:
            if key == "actual":
                return np.rad2deg(trial["target"] - trial["error"])
            if key in ("target", "error"):
                return np.rad2deg(trial[key])
            return trial[key]

        magnitude = max(float(np.max(np.abs(values(trial, key))))
                        for _, key, _ in series for trial in (baseline, tuned))
        bound = max(magnitude * 1.1, 0.5)
        y_center = (top + bottom) / 2
        y_scale = (bottom - top) / (2 * bound)
        x_left, x_right = left, width - right
        chunks.append(f'<text x="{left}" y="{top - 22}" font-family="sans-serif" font-size="17">{escape(title)}</text>')
        chunks.append(f'<text x="24" y="{y_center:.1f}" transform="rotate(-90 24 {y_center:.1f})" text-anchor="middle" font-family="sans-serif" font-size="13">{escape(ylabel)}</text>')
        for tick in np.linspace(-bound, bound, 5):
            y = y_center - tick * y_scale
            chunks.append(f'<line x1="{x_left}" y1="{y:.1f}" x2="{x_right}" y2="{y:.1f}" stroke="#e0e0e0"/>')
            chunks.append(f'<text x="{x_left - 9}" y="{y + 4:.1f}" text-anchor="end" font-family="sans-serif" font-size="11">{tick:.1f}</text>')
        for tick in np.linspace(0, x_max, 6):
            x = x_left + (x_right - x_left) * tick / x_max
            chunks.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{bottom}" stroke="#eeeeee"/>')
            chunks.append(f'<text x="{x:.1f}" y="{bottom + 16}" text-anchor="middle" font-family="sans-serif" font-size="11">{tick:.1f}</text>')
        chunks.append(f'<path d="M {x_left} {top} V {bottom} H {x_right}" fill="none" stroke="#333333" stroke-width="1.3"/>')
        chunks.append(f'<text x="{(x_left + x_right) / 2:.1f}" y="{bottom + 34}" text-anchor="middle" font-family="sans-serif" font-size="12">Time (s)</text>')
        for name, key, color in series:
            trial = tuned if name == "Tuned" else baseline
            t, v = trial["time"], values(trial, key)
            take = np.linspace(0, len(t) - 1, min(len(t), 1600)).astype(int)
            points = " ".join(f"{x_left + (x_right - x_left) * t[j] / x_max:.1f},{y_center - v[j] * y_scale:.1f}"
                              for j in take)
            dash = ' stroke-dasharray="5 4"' if name == "Target" else ""
            chunks.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.6"{dash}/>')
        legend_x = x_right - 325
        for j, (name, _, color) in enumerate(series):
            x = legend_x + j * 111
            chunks.append(f'<line x1="{x}" y1="{top + 15}" x2="{x + 23}" y2="{top + 15}" stroke="{color}" stroke-width="2.5"/>')
            chunks.append(f'<text x="{x + 27}" y="{top + 19}" font-family="sans-serif" font-size="11">{escape(name)}</text>')
    chunks.append('</svg>')
    path.write_text("\n".join(chunks) + "\n")


def check_matched(baseline_path: Path, tuned_path: Path,
                  baseline: list[dict], tuned: list[dict]) -> dict:
    """Reject comparisons with different target trajectories or pitch setups."""
    if len(baseline) != len(tuned):
        raise ValueError("Different numbers of complete trials")
    baseline_config = baseline_path.parent / "config.yaml"
    tuned_config = tuned_path.parent / "config.yaml"
    if not baseline_config.exists() or not tuned_config.exists():
        raise ValueError("Each run needs its saved config.yaml snapshot")
    p = [yaml.safe_load(path.read_text())["yaw_experiment"]["ros__parameters"]
         for path in (baseline_config, tuned_config)]
    profile_keys = ("tracking_test", "tracking_amplitude_deg", "tracking_frequency_hz",
                    "tracking_frequencies_hz", "tracking_ramp_s", "duration")
    pitch_keys = ("world_pitch_target_deg", "pitch_ramp_rad_s",
                  "pitch_target_lead_limit_deg", "pitch_target_soft_tau_s",
                  "pitch_stiction_compensation_Nm", "pitch_precharge_s",
                  "pitch_precharge_torque_slew_Nm_s", "pitch_start_ramp_s",
                  "pitch_gravity_ff_gain", "pitch_gravity_ff_raise_gain",
                  "pitch_torque_limit", "pitch_torque_slew_Nm_s",
                  "pitch_velocity_filter_tau_s")
    for key in profile_keys + pitch_keys:
        if p[0].get(key) != p[1].get(key):
            raise ValueError(f"Unmatched configs: {key} differs")
    if p[0].get("tracking_amplitude_deg", 0) >= 15:
        wide_safety_keys = (
            "yaw_span_rad", "yaw_torque_limit", "yaw_torque_slew_Nm_s",
            "yaw_torque_release_slew_Nm_s", "yaw_angle_output_min",
            "yaw_angle_output_max", "yaw_velocity_output_min",
            "yaw_velocity_output_max", "yaw_velocity_integral_min",
            "yaw_velocity_integral_max",
        )
        for key in wide_safety_keys:
            if p[0].get(key) != p[1].get(key):
                raise ValueError(f"Unmatched wide safety limit: {key} differs")
    peak_target_difference_deg = 0.0
    matched_frequencies_hz = []
    for number, (base, candidate) in enumerate(zip(baseline, tuned), 1):
        expected = (p[0].get("tracking_frequencies_hz") or
                    [p[0].get("tracking_frequency_hz")])
        expected_frequency = expected[(number - 1) % len(expected)]
        for trial in (base, candidate):
            observed = trial.get("frequency_hz")
            if observed is not None and abs(observed - expected_frequency) > 0.001:
                raise ValueError(f"Trial {number} logged frequency {observed} Hz, expected {expected_frequency} Hz")
        matched_frequencies_hz.append(expected_frequency)
        end = min(float(base["time"][-1]), float(candidate["time"][-1]))
        if abs(float(base["time"][-1] - candidate["time"][-1])) > 0.1:
            raise ValueError(f"Trial {number} durations differ")
        grid = np.linspace(0, end, 200)
        target_difference = np.max(np.abs(
            np.interp(grid, base["time"], base["target"])
            - np.interp(grid, candidate["time"], candidate["target"])))
        difference_deg = float(np.rad2deg(target_difference))
        peak_target_difference_deg = max(peak_target_difference_deg, difference_deg)
        # Logs may start one or two 100 Hz samples apart. Allow that phase
        # jitter, scaled to the configured amplitude/frequency, but reject a
        # different target trajectory even when the YAML profiles match.
        tolerance_deg = max(0.1, p[0]["tracking_amplitude_deg"]
                            * 2 * np.pi * expected_frequency * 0.02 + 0.02)
        if difference_deg > tolerance_deg:
            raise ValueError(
                f"Trial {number} target trajectories differ by {difference_deg:.3f} deg"
            )
    return {"profile": {key: p[0].get(key) for key in profile_keys},
            "pitch_configuration_matched": True,
            "target_trajectories_matched": True,
            "trial_frequencies_hz": matched_frequencies_hz,
            "max_sampled_target_difference_deg": peak_target_difference_deg,
            "yaw_torque_limits_Nm": [p[0]["yaw_torque_limit"], p[1]["yaw_torque_limit"]]}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--tuned", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("control_analysis"))
    args = parser.parse_args()
    base_trials, base_rejected = complete_trials(args.baseline)
    tuned_trials, tuned_rejected = complete_trials(args.tuned)
    matched = check_matched(args.baseline, args.tuned, base_trials, tuned_trials)
    result = {
        "baseline_source": str(args.baseline), "tuned_source": str(args.tuned),
        "baseline": metrics(base_trials, matched["yaw_torque_limits_Nm"][0]),
        "tuned": metrics(tuned_trials, matched["yaw_torque_limits_Nm"][1]),
        "rejected": {"baseline": base_rejected, "tuned": tuned_rejected},
        "comparison_valid": True,
        "matched_conditions": matched,
        "note": "Compare only matched target profiles and similar starting temperature/load.",
    }
    result["per_trial"] = [
        {"trial": number, "frequency_hz": matched["trial_frequencies_hz"][number - 1],
         "baseline": metrics([base], matched["yaw_torque_limits_Nm"][0]),
         "tuned": metrics([candidate], matched["yaw_torque_limits_Nm"][1])}
        for number, (base, candidate) in enumerate(zip(base_trials, tuned_trials), 1)
    ]
    result["change_percent"] = {
        key: 100 * (result["tuned"][key] / result["baseline"][key] - 1)
        for key in ("angle_rmse_deg", "angle_p95_abs_deg", "torque_rms_Nm",
                    "peak_abs_torque_Nm") if result["baseline"][key] > 0
    }
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "control_comparison.json").write_text(json.dumps(result, indent=2) + "\n")
    svg_plot(args.out / "control_comparison.svg", base_trials[0], tuned_trials[0])
    for number, (base, candidate) in enumerate(zip(base_trials, tuned_trials), 1):
        svg_plot(args.out / f"control_comparison_trial_{number}.svg", base, candidate)
    print(json.dumps(result, indent=2))
    print(f"Results: {args.out}")


if __name__ == "__main__":
    main()
