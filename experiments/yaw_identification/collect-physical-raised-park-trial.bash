#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
exec python3 session.py --config identification_physical_raised_park_temp130_trial.yaml --trials 1 --max-start-temp 40 --stop-temp 55 --yaw-max-start-temp 126 --yaw-stop-temp 130 --require-off-pitch-world-deg -8 --off-pitch-tolerance-deg 2 --require-controlled-pitch-parking
