#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
if [ "$#" -ne 1 ]; then
  echo "Usage: bash collect-physical-fixed-pitch-temp130-reverse-trial.bash MEASURED_YAW_SHELL_TEMP_C" >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
exec python3 session.py --config identification_physical_fixed_pitch_temp130_reverse_trial.yaml \
  --trials 1 --max-start-temp 40 --stop-temp 55 \
  --yaw-max-start-temp 126 --yaw-stop-temp 130 \
  --require-passive-pitch --require-mechanically-fixed-pitch \
  --measured-yaw-shell-temp-c "$1"
