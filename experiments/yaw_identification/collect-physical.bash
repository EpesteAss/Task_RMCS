#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
# Default to a single supervised probe. Repetition requires --trials explicitly.
exec python3 session.py --config identification_physical.yaml \
  --trials 1 --max-start-temp 40 --stop-temp 55 "$@"
