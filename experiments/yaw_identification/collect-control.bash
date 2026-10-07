#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
mode="${1:-}"
if [[ "$mode" != "baseline" && "$mode" != "tuned" \
      && "$mode" != "baseline-fast" && "$mode" != "tuned-fast" \
      && "$mode" != "baseline-wide" && "$mode" != "tuned-wide" && "$mode" != "baseline-final" \
      && "$mode" != "tuned-final" ]]; then
  echo "Usage: bash collect-control.bash baseline|tuned|baseline-fast|tuned-fast|baseline-wide|tuned-wide|baseline-final|tuned-final [trial count]" >&2
  exit 2
fi
trials="${2:-3}"
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
config_mode="${mode//-/_}"
exec python3 session.py --config "control_${config_mode}.yaml" --trials "$trials" \
  --max-start-temp 40 --stop-temp 55
