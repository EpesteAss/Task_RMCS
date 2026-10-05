#!/usr/bin/env python3
"""Validate frozen identified models on independent experiment logs without refitting."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from identify import evaluate, evaluate_delayed_arx2, read_log, trials


def coefficients(model: dict, names: tuple[str, ...]) -> list[float]:
    return [float(model["coefficients"][name]) for name in names]


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True,
                        help="results.json produced by identify.py")
    parser.add_argument("--csv", type=Path, action="append", required=True,
                        help="independent feedback.csv; repeat for multiple logs")
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()

    frozen = json.loads(args.model.read_text())
    linear = frozen["models"]["linear"]
    arx = frozen["models"]["delayed_arx2"]
    linear_coeff = coefficients(linear, ("a", "b", "c"))
    arx_coeff = coefficients(arx, ("p1", "p2", "b", "c"))
    delay = int(arx["delay_samples"])

    report = {
        "frozen_model_source": str(args.model.resolve()),
        "method": "All coefficients and delay are frozen; each listed CSV is independent and no refitting occurs.",
        "datasets": [],
        "interpretation": (
            "These tracking logs use a narrower excitation spectrum than the identification chirp. "
            "A lower external fit measures cross-experiment transfer and does not replace the original held-out result."
        ),
    }
    for path in args.csv:
        accepted, rejected = trials(read_log(path))
        linear_metrics, _ = evaluate(accepted, linear_coeff)
        arx_metrics, _ = evaluate_delayed_arx2(accepted, arx_coeff, delay)
        report["datasets"].append({
            "source": str(path.resolve()),
            "complete_trials": len(accepted),
            "rejected_trials": rejected,
            "linear": linear_metrics,
            "delayed_arx2": arx_metrics,
        })

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    print(f"External validation: {args.out}")


if __name__ == "__main__":
    main()
