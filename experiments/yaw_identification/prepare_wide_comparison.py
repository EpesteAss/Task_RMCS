#!/usr/bin/env python3
"""Build the ±15° original-gain comparison with the wide tuned test."""

from copy import deepcopy
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
PROFILE = (
    "tracking_test", "tracking_amplitude_deg", "tracking_frequency_hz",
    "tracking_frequencies_hz", "tracking_ramp_s", "duration",
)
# The larger trajectory approaches the software travel limit. Give both yaw
# candidates the same tested output envelope; compare original vs tuned gains
# and feedforward within that envelope.
SHARED_YAW_LIMITS = (
    "yaw_span_rad", "yaw_torque_limit", "yaw_torque_slew_Nm_s",
    "yaw_torque_release_slew_Nm_s",
    "yaw_angle_output_min", "yaw_angle_output_max",
    "yaw_velocity_output_min", "yaw_velocity_output_max",
    "yaw_velocity_integral_min", "yaw_velocity_integral_max",
)


def main() -> None:
    baseline = deepcopy(yaml.safe_load((HERE / "control_baseline_final.yaml").read_text()))
    tuned = yaml.safe_load((HERE / "control_tuned_wide.yaml").read_text())
    base_params = baseline["yaw_experiment"]["ros__parameters"]
    tuned_params = tuned["yaw_experiment"]["ros__parameters"]
    for key in PROFILE + SHARED_YAW_LIMITS:
        base_params[key] = deepcopy(tuned_params[key])
    baseline["yaw_collector"]["ros__parameters"]["csv_path"] = (
        "/tmp/c_yaw_control_baseline_wide_<timestamp>.csv"
    )
    destination = HERE / "control_baseline_wide.yaml"
    destination.write_text(yaml.safe_dump(baseline, sort_keys=False))
    print(f"Wrote {destination}")


if __name__ == "__main__":
    main()
