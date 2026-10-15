#!/usr/bin/env python3
"""Check a fixed frequency-response plant by free time-domain simulation."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from identify import read_log, sliced, trials
from physical_identification import evaluate


HERE = Path(__file__).resolve().parent
MODEL = HERE / "physical_analysis/20261014_high_amplitude_frequency_response/frequency_response.json"
DATA = HERE / "physical_data"
FOLDERS = {
    "positive_0.5_hz_training": "20261014_131635_785889",
    "reverse_0.5_hz_holdout": "20261014_131749_079314",
    "positive_1_hz_training": "20261014_132008_333128",
    "positive_2_hz_training": "20261014_132044_368969",
}


def metrics(trial, parameters):
    score, _ = evaluate([trial], parameters)
    return {key: score[key] for key in
            ("velocity_fit_percent", "velocity_rmse_rad_s", "angle_rmse_deg")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    model = json.loads(MODEL.read_text())
    parameters = {**model["parameters"], "bias_Nm": 0.0}
    results = {}
    for label, folder in FOLDERS.items():
        source = DATA / folder / "feedback.csv"
        digest = hashlib.sha256(source.read_bytes()).hexdigest()
        key = "reverse_0.5_hz_holdout" if "reverse" in label else label.replace("_training", "")
        if model["trials"][key]["feedback_sha256"] != digest:
            raise ValueError(f"Source hash differs from frozen frequency-response model: {source}")
        group, rejected = trials(read_log(source))
        if len(group) != 1 or rejected:
            raise ValueError(f"Expected one complete sweep in {source}")
        trial = group[0]
        index = int(np.searchsorted(trial["t"], 2.0))
        results[label] = {
            "source_feedback_sha256": digest,
            "full_0_to_4_s": metrics(trial, parameters),
            "moving_2_to_4_s": metrics(sliced(trial, index, len(trial["t"])), parameters),
        }
    result = {
        "model_source": str(MODEL.relative_to(HERE)),
        "parameters_unchanged": parameters,
        "evaluation": "Free simulation initialized once at the start of each stated window; no measured-state updates within that window.",
        "window_caveat": "The 2-4 s window excludes the known startup mismatch and is a scoped moving-state check, not a full-sweep validation.",
        "trials": results,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(result, indent=2) + "\n")
    print(args.out)


if __name__ == "__main__":
    main()
