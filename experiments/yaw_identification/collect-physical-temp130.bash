#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
# Exactly one supervised trial. Pitch still stops at 55 C.
exec python3 session.py --config identification_physical_temp130.yaml \
  --trials 1 --max-start-temp 40 --stop-temp 55 \
  --yaw-max-start-temp 126 --yaw-stop-temp 130
