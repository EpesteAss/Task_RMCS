#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
source /opt/ros/jazzy/setup.bash
source /rmcs_install/setup.bash
source addon_install/setup.bash
python3 safe_off.py
echo "Outputs are zero. It is now safe to stop the experiment terminal or run:"
echo "  pkill -TERM -x rmcs_executor"
