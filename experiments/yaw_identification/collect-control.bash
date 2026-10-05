#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
mode="${1:-}"
if [[ "$mode" != "baseline" && "$mode" != "tuned" \
      && "$mode" != "baseline-fast" && "$mode" != "tuned-fast" ]]; then
  echo "Usage: bash collect-control.bash baseline|tuned|baseline-fast|tuned-fast [trial count]" >&2
  exit 2
fi
trials="${2:-3}"
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
config_mode="${mode//-/_}"
exec python3 session.py --config "control_${config_mode}.yaml" --trials "$trials" \
  --max-start-temp 40 --stop-temp 55
