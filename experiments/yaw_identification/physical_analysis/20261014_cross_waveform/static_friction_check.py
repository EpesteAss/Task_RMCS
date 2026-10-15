#!/usr/bin/env python3
"""Exploratory static-friction check using the already frozen moving model.

Only breakaway threshold changes. Select it on the first positive pure sine;
print full-sweep held-out scores on reverse and later positive pure sines.
This is diagnostic, not a calibrated shaft model.
"""

import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT))
from identify import read_log, trials  # noqa: E402


DATA = ROOT / "physical_data"
IDS = {
    "train_positive": "20261014_130051_829626",
    "holdout_reverse": "20261014_130158_204634",
    "holdout_after_square": "20261014_130505_738958",
    "holdout_square": "20261014_130432_374019",
}


def predict(trial, params, threshold):
    j, b, fc, bias = (params[key] for key in
                      ("J_kg_m2", "B_Nm_s_rad", "Fc_Nm", "bias_Nm"))
    angle = float(trial["theta"][0])
    velocity = float(trial["omega"][0])
    result = np.empty((len(trial["t"]), 2))
    result[0] = angle, velocity
    for k, dt in enumerate(np.diff(trial["t"]), 1):
        torque = float(trial["u"][k - 1] + bias)
        count = max(1, math.ceil(dt / .002))
        h = dt / count
        for _ in range(count):
            if abs(velocity) < .01 and abs(torque) <= threshold:
                velocity = 0.
                continue
            acceleration = (torque - b * velocity - fc * math.tanh(velocity / .02)) / j
            next_velocity = velocity + h * acceleration
            if velocity * next_velocity < 0 and abs(torque) <= threshold:
                next_velocity = 0.
            angle += h * (velocity + next_velocity) / 2
            velocity = next_velocity
        result[k] = angle, velocity
    return result


def score(trial, prediction):
    velocity_error = prediction[1:, 1] - trial["omega"][1:]
    angle_error = prediction[1:, 0] - trial["theta"][1:]
    return {"velocity_fit_percent": float(100 * (1 - np.sqrt(np.mean(velocity_error ** 2)) /
                                                 np.std(trial["omega"][1:]))),
            "angle_rmse_deg": float(np.rad2deg(np.sqrt(np.mean(angle_error ** 2))))}


def main():
    params = json.loads((HERE.parent / "20261014_steady_window_candidate/frozen_model.json")
                        .read_text())["fit"]["parameters_conditional_on_torque_calibration"]
    group = {label: trials(read_log(DATA / folder / "feedback.csv"))[0][0]
             for label, folder in IDS.items()}
    options = np.linspace(.23, .9, 68)
    fitted = min(options, key=lambda threshold: score(group["train_positive"],
                         predict(group["train_positive"], params, threshold))["angle_rmse_deg"])
    result = {"threshold_selected_on": "train_positive", "static_threshold_Nm_conditional": float(fitted),
              "fixed_moving_parameters": params,
              "scores": {label: score(trial, predict(trial, params, fitted))
                         for label, trial in group.items()}}
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
