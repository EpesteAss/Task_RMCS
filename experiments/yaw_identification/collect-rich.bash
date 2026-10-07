#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
exec python3 session.py --config identification_rich.yaml \
  --max-start-temp 45 --stop-temp 64 "$@"
