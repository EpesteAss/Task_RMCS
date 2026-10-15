#!/usr/bin/env python3
"""Preflight and launch one generated RMCS experiment configuration."""

from __future__ import annotations

import argparse
import subprocess
from datetime import datetime
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent
PHYSICAL_MODES = ("identification-physical", "identification-physical-temp130",
                  "identification-physical-temp130-x3", "identification-physical-low-pitch-temp130",
                  "identification-physical-raised-park-temp130-cycle",
                  "identification-physical-raised-park-temp130-trial",
                  "identification-physical-raised-release-temp130-cycle",
                  "identification-physical-raised-release-temp130-trial",
                  "identification-physical-fixed-pitch-temp130-trial",
                  "identification-physical-fixed-pitch-temp130-reverse-trial",
                  "identification-physical-fixed-pitch-temp130-1hz-trial",
                  "identification-physical-fixed-pitch-temp130-2hz-trial",
                  "identification-physical-fixed-pitch-temp130-reverse-1hz-trial",
                  "identification-physical-fixed-pitch-temp130-step-pos-trial",
                  "identification-physical-fixed-pitch-temp130-step-neg-trial",
                  "identification-physical-fixed-pitch-temp130-alternating-trial",
                  "identification-physical-fixed-pitch-temp130-alternating-05nm-trial",
                  "identification-physical-fixed-pitch-temp130-sine-05hz-trial",
                  "identification-physical-fixed-pitch-temp130-sine-1hz-trial",
                  "identification-physical-fixed-pitch-temp130-sine-2hz-trial",
                  "identification-physical-fixed-pitch-temp130-sine-reverse-05hz-trial",
                  "identification-physical-fixed-pitch-temp130-sine-pure-05hz-trial",
                  "identification-physical-fixed-pitch-temp130-sine-pure-125nm-05hz-trial",
                  "identification-physical-fixed-pitch-temp130-sine-pure-125nm-1hz-trial",
                  "identification-physical-fixed-pitch-temp130-sine-pure-125nm-2hz-trial",
                  "identification-physical-fixed-pitch-temp130-sine-pure-125nm-reverse-05hz-trial",
                  "identification-physical-fixed-pitch-temp130-sine-pure-reverse-05hz-trial")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("identification", "identification-rich", *PHYSICAL_MODES, "baseline", "tuned",
                                         "control-baseline", "control-tuned",
                                         "control-baseline-fast", "control-tuned-fast",
                                         "control-baseline-wide", "control-tuned-wide", "control-baseline-final",
                                         "control-tuned-final", "control-model-baseline-fixed",
                                         "control-model-ff-fixed"))
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    config_name = args.mode.replace("-", "_")
    config_path = HERE / f"{config_name}.yaml"
    if not config_path.exists():
        raise SystemExit("Run `python3 prepare.py` first")
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if args.mode in PHYSICAL_MODES:
        p = config["yaw_experiment"]["ros__parameters"]
        if p.get("physical_identification") is not True or p.get("tracking_test", False) or p.get("rich_identification", False):
            raise SystemExit("Physical mode requires physical_identification=true and tracking/rich modes disabled")
        expected_trip = 130.0 if "temp130" in args.mode else 65.0
        if float(p.get("yaw_temperature_trip_c", 65.0)) != expected_trip:
            raise SystemExit(f"This physical mode requires a Yaw temperature trip of {expected_trip:g} C")
        fixed_pitch = args.mode.startswith("identification-physical-fixed-pitch-temp130-")
        low_pose = args.mode == "identification-physical-low-pitch-temp130"
        if p.get("passive_pitch", False) is not (low_pose or fixed_pitch):
            raise SystemExit("Passive Pitch setting does not match the selected physical mode")
        if p.get("mechanically_fixed_pitch", False) is not fixed_pitch:
            raise SystemExit("Mechanically fixed Pitch setting does not match the selected physical mode")
        step_pulse = args.mode.endswith(("-step-pos-trial", "-step-neg-trial"))
        alternating = args.mode.endswith(("-alternating-trial", "-alternating-05nm-trial"))
        sine_profile = {"identification-physical-fixed-pitch-temp130-sine-05hz-trial": 0,
                        "identification-physical-fixed-pitch-temp130-sine-1hz-trial": 1,
                        "identification-physical-fixed-pitch-temp130-sine-2hz-trial": 2,
                        "identification-physical-fixed-pitch-temp130-sine-reverse-05hz-trial": 3,
                        "identification-physical-fixed-pitch-temp130-sine-pure-05hz-trial": 0,
                        "identification-physical-fixed-pitch-temp130-sine-pure-125nm-05hz-trial": 0,
                        "identification-physical-fixed-pitch-temp130-sine-pure-125nm-1hz-trial": 1,
                        "identification-physical-fixed-pitch-temp130-sine-pure-125nm-2hz-trial": 2,
                        "identification-physical-fixed-pitch-temp130-sine-pure-125nm-reverse-05hz-trial": 3,
                        "identification-physical-fixed-pitch-temp130-sine-pure-reverse-05hz-trial": 3}.get(args.mode)
        pure_sine = args.mode.endswith(("-sine-pure-05hz-trial", "-sine-pure-125nm-05hz-trial",
                                        "-sine-pure-125nm-1hz-trial", "-sine-pure-125nm-2hz-trial",
                                        "-sine-pure-125nm-reverse-05hz-trial",
                                        "-sine-pure-reverse-05hz-trial"))
        high_sine = "-sine-pure-125nm-" in args.mode
        trial_amplitude_limit = 1.25 if high_sine else 0.9
        trial_torque_limit = 1.35 if high_sine else 1.0
        if fixed_pitch and (p.get("allow_arm") is not True or p.get("allow_sweep") is not True
                            or float(p.get("sweep_amplitude", 0)) > (trial_amplitude_limit if step_pulse or alternating or sine_profile is not None else 0.75)
                            or float(p.get("yaw_torque_limit", 0)) > (trial_torque_limit if step_pulse or alternating or sine_profile is not None else 0.8)):
            raise SystemExit("Fixed Pitch trial requires one enabled, bounded Yaw sweep")
        if step_pulse and (p.get("physical_step_pulse") is not True
                           or p.get("physical_step_direction") != (1 if args.mode.endswith("-step-pos-trial") else -1)
                           or float(p.get("physical_step_start_s", -1)) != 0.5
                           or float(p.get("physical_step_end_s", -1)) != 0.9
                           or float(p.get("yaw_span_rad", 1)) > 0.785399
                           or float(p.get("yaw_speed_trip_rad_s", 1)) > 3.6):
            raise SystemExit("Step trial requires matching direction, bounded pulse timing and Yaw travel/speed")
        if alternating and (p.get("physical_alternating_pulses") is not True
                            or p.get("physical_step_pulse") is not False
                            or p.get("physical_alternating_first_direction") not in (-1, 1)
                            or p.get("physical_alternating_cycles") != 3
                            or float(p.get("physical_alternating_start_s", -1)) != 0.3
                            or float(p.get("physical_alternating_on_s", -1)) != 0.4
                            or float(p.get("physical_alternating_gap_s", -1)) != 0.1
                            or float(p.get("yaw_span_rad", 1)) > 0.785399
                            or float(p.get("yaw_speed_trip_rad_s", 1)) > 3.6):
            raise SystemExit("Alternating trial requires three bounded alternating pulse pairs")
        if args.mode.endswith("-alternating-05nm-trial") and float(p.get("sweep_amplitude", -1)) != 0.5:
            raise SystemExit("Low-amplitude alternating trial requires 0.5 Nm commanded pulses")
        if sine_profile is not None and (p.get("physical_alternating_pulses") is not False
                          or p.get("physical_step_pulse") is not False
                          or p.get("physical_pure_sine", False) is not pure_sine
                          or p.get("physical_start_profile") != sine_profile
                          or p.get("physical_frequencies_hz") != [0.5, 1.0, 2.0]
                          or float(p.get("duration", -1)) != 4.0
                          or float(p.get("sweep_amplitude", 0)) != (1.25 if high_sine else 0.9)
                          or float(p.get("yaw_span_rad", 1)) > (3.141593 if high_sine else 0.785399)
                          or float(p.get("yaw_speed_trip_rad_s", 1)) > 3.6):
            raise SystemExit("Sine trial requires the selected bounded physical profile")
        if args.mode.endswith("-reverse-trial") and (p.get("physical_start_profile") != 3
                                                       or float(p.get("yaw_span_rad", 1)) > 0.07854):
            raise SystemExit("Reverse trial requires profile 3 and a 4.5 degree Yaw span")
        if args.mode == "identification-physical-fixed-pitch-temp130-1hz-trial" and (p.get("physical_start_profile") != 1
                                                                                       or float(p.get("yaw_span_rad", 1)) > 0.07854):
            raise SystemExit("1 Hz trial requires profile 1 and a 4.5 degree Yaw span")
        if args.mode == "identification-physical-fixed-pitch-temp130-2hz-trial" and (p.get("physical_start_profile") != 2
                                                                                       or float(p.get("yaw_span_rad", 1)) > 0.07854):
            raise SystemExit("2 Hz trial requires profile 2 and a 4.5 degree Yaw span")
        if args.mode.endswith("-reverse-1hz-trial") and (p.get("physical_start_profile") != 4
                                                           or float(p.get("yaw_span_rad", 1)) > 0.07854):
            raise SystemExit("Reverse 1 Hz trial requires profile 4 and a 4.5 degree Yaw span")
        raised_park = args.mode.startswith("identification-physical-raised-park-")
        if p.get("controlled_pitch_parking", False) is not raised_park:
            raise SystemExit("Controlled Pitch parking setting does not match the selected physical mode")
        if raised_park and p.get("allow_sweep", True) is not args.mode.endswith("-trial"):
            raise SystemExit("Pitch cycle must disable Yaw Sweep; trial mode must enable it")
        raised_release = args.mode.startswith("identification-physical-raised-release-")
        if raised_release and (p.get("controlled_pitch_parking", False)
                               or float(p.get("pitch_off_slew_Nm_s", 0)) != 0.3
                               or p.get("allow_sweep", True) is not args.mode.endswith("-trial")):
            raise SystemExit("Raised release mode needs 0.3 Nm/s OFF ramp and matching Yaw Sweep gate")
    components = config["rmcs_executor"]["ros__parameters"]["components"]
    outputs = [component for component in components if " -> yaw_experiment" in component
               or " -> gimbal_controller" in component]
    if len(outputs) != 1:
        raise SystemExit(f"Expected exactly one yaw torque writer, got {outputs}")
    if not any(c.startswith("rmcs_core::referee::Status ->") for c in components):
        raise SystemExit("Missing referee Status: C-car hardware requires /referee/chassis/output_status. Run prepare.py again.")

    try:
        prefix = subprocess.check_output(
            ["ros2", "pkg", "prefix", "rmcs_core"], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        raise SystemExit("ROS 2 environment is not sourced; source the C-car workspace first")
    plugin_xml = Path(prefix) / "share" / "rmcs_core" / "plugins.xml"
    if not plugin_xml.exists() or "rmcs_core::hardware::DeformableInfantryOmniC" not in plugin_xml.read_text():
        raise SystemExit("Installed rmcs_core has no C-car hardware plugin; build/source the "
                         "assignment's merge/deformable C-car workspace first")
    if args.mode in PHYSICAL_MODES:
        try:
            bringup_prefix = subprocess.check_output(
                ["ros2", "pkg", "prefix", "rmcs_bringup"], text=True).strip()
        except (OSError, subprocess.CalledProcessError):
            raise SystemExit("Cannot locate installed C-car configuration; source the car workspace")
        installed_path = (Path(bringup_prefix) / "share" / "rmcs_bringup" / "config"
                          / "deformable-infantry-omni-c.yaml")
        if not installed_path.is_file():
            raise SystemExit(f"Installed C-car configuration missing: {installed_path}")
        installed = yaml.safe_load(installed_path.read_text(encoding="utf-8"))
        planned_hardware = config["deformable_infantry"]["ros__parameters"]
        installed_hardware = installed["deformable_infantry"]["ros__parameters"]
        if planned_hardware != installed_hardware:
            differences = sorted(key for key in planned_hardware.keys() | installed_hardware.keys()
                                 if planned_hardware.get(key) != installed_hardware.get(key))
            raise SystemExit("Physical experiment hardware config differs from the installed car config "
                             f"at {differences}; check zero points and wiring before starting")
    if args.mode.startswith("identification") or args.mode.startswith("control-"):
        try:
            subprocess.check_output(["ros2", "pkg", "prefix", "rmcs_yaw_identification"],
                                    text=True, stderr=subprocess.STDOUT)
        except subprocess.CalledProcessError:
            raise SystemExit("Build the addon with bash build-addon.bash, then source addon_install/setup.bash or setup.zsh")
    print(f"Static preflight passed (hardware dependency pairing occurs at startup): {config_path}")
    if args.check_only:
        return
    if args.mode in ("baseline", "tuned"):
        raise SystemExit("baseline/tuned are original-controller reference configs and still require remote inputs. "
                         "Do not use them for this no-DR16 experiment yet.")
    running = subprocess.run(["pgrep", "-x", "rmcs_executor"], capture_output=True, text=True)
    if running.returncode == 0:
        raise SystemExit("Another rmcs_executor is running. Stop it before starting the experiment.")
    root = HERE / ("physical_data" if args.mode in PHYSICAL_MODES
                   else "control_data" if args.mode.startswith("control-") else "data")
    folder = root / args.mode / datetime.now().strftime("%Y%m%d_%H%M%S_%f") \
        if args.mode.startswith("control-") else root / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    folder.mkdir(parents=True)
    config["yaw_collector"]["ros__parameters"]["csv_path"] = str(folder / "feedback.csv")
    config_path = folder / "config.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    print(f"Permanent recording directory: {folder}", flush=True)
    print(f"Starting {args.mode} OFF. Open a second SSH terminal and run the matching session; "
          + ("Pitch stays at zero torque under an external mechanical clamp; arm then sweep excites yaw. " if args.mode.startswith("identification-physical-fixed-pitch-temp130-")
             else "Pitch stays at zero torque; arm then sweep excites yaw. " if args.mode == "identification-physical-low-pitch-temp130"
             else "Arm raises Pitch; OFF parks it under control before zero torque. " if args.mode.startswith("identification-physical-raised-park-")
             else "Arm raises Pitch; OFF slowly reduces Pitch support torque. " if args.mode.startswith("identification-physical-raised-release-")
             else "arm raises pitch, sweep starts yaw. ")
          + "Heartbeat loss zeros both torques.", flush=True)
    raise SystemExit(subprocess.call([
        "ros2", "run", "rmcs_executor", "rmcs_executor", "--ros-args",
        "--params-file", str(config_path),
    ]))


if __name__ == "__main__":
    main()
