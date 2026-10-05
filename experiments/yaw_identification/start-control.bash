#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
mode="${1:-}"
if [[ "$mode" != "baseline" && "$mode" != "tuned" \
      && "$mode" != "baseline-fast" && "$mode" != "tuned-fast" ]]; then
  echo "Usage: bash start-control.bash baseline|tuned|baseline-fast|tuned-fast" >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
exec python3 run.py "control-$mode"
