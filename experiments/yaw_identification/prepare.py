#!/usr/bin/env python3
"""Create isolated C-car yaw experiment parameter files from the upstream snapshot."""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
SOURCE = HERE / "vehicle-c.source.yaml"
HARDWARE = "rmcs_core::hardware::DeformableInfantryOmniC -> deformable_infantry"
GIMBAL = "rmcs_core::controller::gimbal::DeformableInfantryGimbalController -> gimbal_controller"
SWEEP = "yaw_identification::Controller -> yaw_experiment"
COLLECTOR = "rmcs_core::debug::ValueCollector -> yaw_collector"
REFEREE = "rmcs_core::referee::Status -> referee_status"


def write_config(path: Path, config: dict) -> None:
    path.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=SOURCE, help="actual C-car YAML if different")
    parser.add_argument("--tuned-angle-kp", type=float, default=None)
    parser.add_argument("--enable-arm", action="store_true", help="enable arm after checking this car's pitch direction")
    parser.add_argument("--pitch-gravity-ff-gain", type=float, default=None,
                        help="measured pitch gravity feedforward gain; defaults to the source YAML")
    parser.add_argument("--tuned-velocity-kp", type=float, default=None)
    args = parser.parse_args()
    base = yaml.safe_load(args.source.read_text(encoding="utf-8"))
    components = base["rmcs_executor"]["ros__parameters"]["components"]
    if HARDWARE not in components or GIMBAL not in components:
        raise SystemExit("Expected C-car hardware and gimbal controller are missing from source YAML")

    source_gimbal = base["gimbal_controller"]["ros__parameters"]
    experiment = {
        # Manual pitch-direction check must be completed before enabling motion.
        "allow_arm": args.enable_arm,
        "world_pitch_control": True,
        "world_pitch_target_deg": 5.0,
        "pitch_target_deg": -18.0,
        "pitch_ready_tolerance_deg": 1.5,
        "min_world_pitch_deg": 3.0,
        "pitch_min": source_gimbal["upper_limit"],
        "pitch_max": source_gimbal["lower_limit"],
        "pitch_ramp_rad_s": 0.0872665,
        # Below the MG4010Ei10 maximum torque reference (4.5 Nm) in RMCS.
        "pitch_torque_limit": 4.5,
        # Apply lifting effort gently, but release stale torque quickly enough
        # for the feedback loop to brake during a long raise.
        "pitch_torque_slew_Nm_s": 5.0,
        "pitch_torque_release_slew_Nm_s": 30.0,
        "pitch_velocity_limit": 6.0,
        "yaw_torque_limit": 3.6,
        "yaw_span_rad": 0.2617993877991494,
        "pitch_gravity_ff_gain": (source_gimbal["pitch_gravity_ff_gain"]
                                  if args.pitch_gravity_ff_gain is None
                                  else args.pitch_gravity_ff_gain),
        **({"pitch_gravity_ff_raise_gain": source_gimbal["pitch_gravity_ff_gain"]}
           if args.pitch_gravity_ff_gain is not None else {}),
        "pitch_gravity_ff_phase": source_gimbal["pitch_gravity_ff_phase"],
        "start_freq": 0.2, "end_freq": 3.0, "duration": 20.0,
        "sweep_amplitude": 0.96,
        "yaw_angle_kp": 2.0, "yaw_angle_ki": 0.0, "yaw_angle_kd": 0.0,
        "yaw_velocity_kp": 1.0, "yaw_velocity_ki": 0.0, "yaw_velocity_kd": 0.0,
        "yaw_angle_output_min": -0.2, "yaw_angle_output_max": 0.2,
        "yaw_velocity_output_min": -2.4, "yaw_velocity_output_max": 2.4,
        "pitch_angle_output_min": -6.0, "pitch_angle_output_max": 6.0,
        # Keep the original velocity PID effectively unbounded; final torque
        # is clamped and slew-limited by the experiment engine.
        "pitch_velocity_output_min": -20.0, "pitch_velocity_output_max": 20.0,
    }
    for prefix in ("pitch_angle", "pitch_velocity"):
        for suffix in ("kp", "ki", "kd", "integral_min", "integral_max"):
            key = f"{prefix}_{suffix}"
            if key in source_gimbal:
                experiment[key] = source_gimbal[key]
    # Match the source controller's pitch gains. The experiment separately
    # caps final torque at the motor's configured 4.5 Nm maximum reference.
    # The addon owns both torque outputs and has no remote-control dependency.
    identification = {
        "rmcs_executor": {"ros__parameters": {
            "update_rate": base["rmcs_executor"]["ros__parameters"]["update_rate"],
            # Supercap in the hardware command partner requires referee output_status.
            "components": [HARDWARE, REFEREE, SWEEP, COLLECTOR],
        }},
        "deformable_infantry": deepcopy(base["deformable_infantry"]),
        "yaw_experiment": {"ros__parameters": experiment},
    }
    identification["yaw_collector"] = {"ros__parameters": {
        "csv_path": "/tmp/c_yaw_ident_<timestamp>.csv",
        "signals": ["/gimbal/yaw/angle", "/gimbal/yaw/velocity",
                    "/gimbal/yaw/velocity_imu", "/gimbal/yaw/torque",
                    "/gimbal/yaw/control_torque", "/gimbal/pitch/angle",
                    "/gimbal/pitch/velocity_imu", "/gimbal/pitch/control_torque",
                    "/gimbal/pitch/velocity", "/gimbal/pitch/torque", "/gimbal/pitch/temperature",
                    "/yaw_experiment/state", "/yaw_experiment/fault",
                    "/yaw_experiment/completed",
                    "/yaw_experiment/time_s", "/yaw_experiment/excitation",
                    "/yaw_experiment/pitch_target", "/yaw_experiment/pitch_torque_unlimited",
                    "/yaw_experiment/pitch_world", "/gimbal/yaw/temperature"],
        "write_interval": 10, "flush_interval": 100,
    }}
    write_config(HERE / "identification.yaml", identification)

    for mode in ("baseline", "tuned"):
        config = deepcopy(base)
        selected = config["rmcs_executor"]["ros__parameters"]["components"]
        config["rmcs_executor"]["ros__parameters"]["components"] = [
            c for c in selected if not c.startswith("rmcs_core::debug::ValueCollector ->")
        ] + [COLLECTOR]
        config["yaw_collector"] = {"ros__parameters": {
            "csv_path": f"/tmp/c_yaw_{mode}_<timestamp>.csv",
            "signals": ["/gimbal/yaw/angle", "/gimbal/yaw/velocity",
                        "/gimbal/yaw/velocity_imu", "/gimbal/yaw/torque",
                        "/gimbal/yaw/control_torque", "/gimbal/yaw/control_angle_error"],
            "write_interval": 2, "flush_interval": 100,
        }}
        if mode == "tuned":
            params = config["gimbal_controller"]["ros__parameters"]
            if args.tuned_angle_kp is not None:
                params["yaw_angle_kp"] = args.tuned_angle_kp
            if args.tuned_velocity_kp is not None:
                params["yaw_velocity_kp"] = args.tuned_velocity_kp
        write_config(HERE / f"{mode}.yaml", config)
    print("Wrote identification.yaml, baseline.yaml, tuned.yaml")
    if args.tuned_angle_kp is None and args.tuned_velocity_kp is None:
        print("Tuned parameters equal baseline until --tuned-* values are provided")


if __name__ == "__main__":
    main()
