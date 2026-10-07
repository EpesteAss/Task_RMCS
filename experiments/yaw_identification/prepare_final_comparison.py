#!/usr/bin/env python3
"""Make matched ±5° comparison configs using the currently validated pitch setup."""

from copy import deepcopy
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
PITCH_KEYS = (
    "world_pitch_control", "world_pitch_target_deg", "pitch_target_deg",
    "pitch_ready_tolerance_deg", "min_world_pitch_deg", "pitch_min", "pitch_max",
    "pitch_ramp_rad_s", "pitch_target_lead_limit_deg", "pitch_target_soft_tau_s",
    "pitch_stiction_compensation_Nm", "pitch_precharge_s",
    "pitch_precharge_torque_slew_Nm_s", "pitch_start_ramp_s",
    "pitch_torque_limit", "pitch_torque_slew_Nm_s",
    "pitch_torque_release_slew_Nm_s", "pitch_off_slew_Nm_s",
    "pitch_velocity_limit", "pitch_velocity_filter_tau_s",
    "pitch_gravity_ff_gain", "pitch_gravity_ff_raise_gain",
    "pitch_gravity_ff_phase", "pitch_angle_output_min", "pitch_angle_output_max",
    "pitch_velocity_output_min", "pitch_velocity_output_max",
    "pitch_angle_kp", "pitch_angle_ki", "pitch_angle_kd",
    "pitch_angle_integral_min", "pitch_angle_integral_max",
    "pitch_velocity_kp", "pitch_velocity_ki", "pitch_velocity_kd",
)


def main() -> None:
    wide = yaml.safe_load((HERE / "control_tuned_wide.yaml").read_text())
    pitch = wide["yaw_experiment"]["ros__parameters"]
    for mode in ("baseline", "tuned"):
        config = deepcopy(yaml.safe_load((HERE / f"control_{mode}.yaml").read_text()))
        params = config["yaw_experiment"]["ros__parameters"]
        for key in PITCH_KEYS:
            params[key] = pitch[key]
        # Both runs share the same position command, pitch holder and velocity
        # measurement. The yaw PID, feedforward and limits remain the two
        # original experimental candidates to be compared.
        params["yaw_velocity_filter_tau_s"] = pitch["yaw_velocity_filter_tau_s"]
        params["tracking_amplitude_deg"] = 5.0
        params["tracking_frequency_hz"] = 0.25
        params["tracking_ramp_s"] = 1.0
        params.pop("tracking_frequencies_hz", None)
        params["yaw_span_rad"] = pitch["yaw_span_rad"]
        config["yaw_collector"]["ros__parameters"]["csv_path"] = (
            f"/tmp/c_yaw_control_{mode}_final_<timestamp>.csv"
        )
        signals = config["yaw_collector"]["ros__parameters"]["signals"]
        if "/yaw_experiment/tracking_frequency_hz" not in signals:
            signals.append("/yaw_experiment/tracking_frequency_hz")
        destination = HERE / f"control_{mode}_final.yaml"
        destination.write_text(yaml.safe_dump(config, sort_keys=False))
        print(f"Wrote {destination}")


if __name__ == "__main__":
    main()
