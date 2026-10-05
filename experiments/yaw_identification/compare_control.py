#!/usr/bin/env python3
"""Compare matched baseline and tuned yaw tracking trials."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

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
    width, height, margin = 1000, 620, 65
    panels = [("Yaw error (deg)", "error", 70, 260), ("Torque (Nm)", "torque", 360, 550)]
    colors = {"baseline": "#d1495b", "tuned": "#00798c"}
    chunks = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
              '<rect width="100%" height="100%" fill="white"/>']
    for title, key, top, bottom in panels:
        all_values = []
        for trial in (baseline, tuned):
            values = np.rad2deg(trial[key]) if key == "error" else trial[key]
            all_values.extend(values.tolist())
        bound = max(abs(min(all_values)), abs(max(all_values)), 1e-6) * 1.1
        chunks += [f'<text x="{margin}" y="{top-18}" font-size="18">{title}</text>',
                   f'<line x1="{margin}" y1="{(top+bottom)/2}" x2="{width-margin}" y2="{(top+bottom)/2}" stroke="#aaa"/>']
        for name, trial in (("baseline", baseline), ("tuned", tuned)):
            t, values = trial["time"], trial[key]
            if key == "error": values = np.rad2deg(values)
            take = np.linspace(0, len(t)-1, min(len(t), 1500)).astype(int)
            x = margin + (width-2*margin) * t[take] / max(t[-1], 1e-9)
            y = (top+bottom)/2 - (bottom-top)/2 * values[take] / bound
            points = " ".join(f"{a:.1f},{b:.1f}" for a, b in zip(x, y))
            chunks.append(f'<polyline points="{points}" fill="none" stroke="{colors[name]}" stroke-width="1.5"/>')
    chunks += ['<line x1="700" y1="25" x2="740" y2="25" stroke="#d1495b" stroke-width="3"/>',
               '<text x="748" y="30" font-size="15">baseline</text>',
               '<line x1="830" y1="25" x2="870" y2="25" stroke="#00798c" stroke-width="3"/>',
               '<text x="878" y="30" font-size="15">tuned</text>', '</svg>']
    path.write_text("\n".join(chunks) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--tuned", type=Path, required=True)
    parser.add_argument("--out", type=Path, default=Path("control_analysis"))
    args = parser.parse_args()
    base_trials, base_rejected = complete_trials(args.baseline)
    tuned_trials, tuned_rejected = complete_trials(args.tuned)
    result = {
        "baseline_source": str(args.baseline), "tuned_source": str(args.tuned),
        "baseline": metrics(base_trials, 4.5), "tuned": metrics(tuned_trials, 3.6),
        "rejected": {"baseline": base_rejected, "tuned": tuned_rejected},
        "comparison_valid": len(base_trials) == len(tuned_trials),
        "note": "Compare only matched target profiles and similar starting temperature/load.",
    }
    result["change_percent"] = {
        key: 100 * (result["tuned"][key] / result["baseline"][key] - 1)
        for key in ("angle_rmse_deg", "angle_p95_abs_deg", "torque_rms_Nm",
                    "peak_abs_torque_Nm") if result["baseline"][key] > 0
    }
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "control_comparison.json").write_text(json.dumps(result, indent=2) + "\n")
    svg_plot(args.out / "control_comparison.svg", base_trials[0], tuned_trials[0])
    print(json.dumps(result, indent=2))
    print(f"Results: {args.out}")


if __name__ == "__main__":
    main()
