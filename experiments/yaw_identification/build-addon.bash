#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
# Build only this addon. All output remains under the experiment directory.
colcon --log-base addon_log build --base-paths addon \
  --build-base addon_build --install-base addon_install \
  --cmake-args -DBUILD_TESTING=ON -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
ln -sfn ../addon_build/rmcs_yaw_identification/compile_commands.json addon/compile_commands.json
source addon_install/local_setup.bash
ctest --test-dir addon_build/rmcs_yaw_identification --output-on-failure
echo 'Build complete. Run: source addon_install/setup.bash (or setup.zsh)'
