#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
mode="${1:-}"
if [[ "$mode" != "baseline" && "$mode" != "tuned" \
      && "$mode" != "baseline-fast" && "$mode" != "tuned-fast" \
      && "$mode" != "baseline-wide" && "$mode" != "tuned-wide" && "$mode" != "baseline-final" \
      && "$mode" != "tuned-final" ]]; then
  echo "Usage: bash start-control.bash baseline|tuned|baseline-fast|tuned-fast|baseline-wide|tuned-wide|baseline-final|tuned-final" >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
exec python3 run.py "control-$mode"
