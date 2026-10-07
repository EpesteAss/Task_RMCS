#!/usr/bin/env python3
"""Preflight and launch one generated RMCS experiment configuration."""

from __future__ import annotations

import argparse
import subprocess
from datetime import datetime
from pathlib import Path

import yaml


HERE = Path(__file__).resolve().parent


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=("identification", "identification-rich", "baseline", "tuned",
                                         "control-baseline", "control-tuned",
                                         "control-baseline-fast", "control-tuned-fast",
                                         "control-baseline-wide", "control-tuned-wide", "control-baseline-final",
                                         "control-tuned-final"))
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args()
    config_name = args.mode.replace("-", "_")
    config_path = HERE / f"{config_name}.yaml"
    if not config_path.exists():
        raise SystemExit("Run `python3 prepare.py` first")
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
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
    root = HERE / ("control_data" if args.mode.startswith("control-") else "data")
    folder = root / args.mode / datetime.now().strftime("%Y%m%d_%H%M%S_%f") \
        if args.mode.startswith("control-") else root / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    folder.mkdir(parents=True)
    config["yaw_collector"]["ros__parameters"]["csv_path"] = str(folder / "feedback.csv")
    config_path = folder / "config.yaml"
    config_path.write_text(yaml.safe_dump(config, sort_keys=False), encoding="utf-8")
    print(f"Permanent recording directory: {folder}", flush=True)
    print(f"Starting {args.mode} OFF. Open a second SSH terminal and run the matching session; "
          "arm raises pitch, sweep starts yaw. "
          "Heartbeat loss zeros both torques; off stops yaw and slowly releases pitch.", flush=True)
    raise SystemExit(subprocess.call([
        "ros2", "run", "rmcs_executor", "rmcs_executor", "--ros-args",
        "--params-file", str(config_path),
    ]))


if __name__ == "__main__":
    main()
