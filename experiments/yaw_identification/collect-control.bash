#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
mode="${1:-}"
if [[ "$mode" != "baseline" && "$mode" != "tuned" \
      && "$mode" != "baseline-fast" && "$mode" != "tuned-fast" \
      && "$mode" != "baseline-wide" && "$mode" != "tuned-wide" && "$mode" != "baseline-final" \
      && "$mode" != "tuned-final" && "$mode" != "model-baseline-fixed" \
      && "$mode" != "model-ff-fixed" ]]; then
  echo "Usage: bash collect-control.bash baseline|tuned|baseline-fast|tuned-fast|baseline-wide|tuned-wide|baseline-final|tuned-final|model-baseline-fixed|model-ff-fixed [trial count]" >&2
  exit 2
fi
trials="${2:-3}"
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
config_mode="${mode//-/_}"
if [[ "$mode" == "model-baseline-fixed" || "$mode" == "model-ff-fixed" ]]; then
  exec python3 session.py --config "control_${config_mode}.yaml" --trials "$trials" \
    --max-start-temp 40 --stop-temp 55 --yaw-max-start-temp 130 --yaw-stop-temp 130 \
    --require-passive-pitch --require-mechanically-fixed-pitch \
    --operator-monitored-yaw-shell-temperature
fi
exec python3 session.py --config "control_${config_mode}.yaml" --trials "$trials" \
  --max-start-temp 40 --stop-temp 55
