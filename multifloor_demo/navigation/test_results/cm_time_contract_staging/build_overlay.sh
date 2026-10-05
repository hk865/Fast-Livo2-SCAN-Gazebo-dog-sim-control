#!/usr/bin/env bash
# Explicit isolated workspace required. Does not install to /opt or other workspaces.
set -eo pipefail
if [[ "$#" != 1 ]]; then
  echo "Usage: bash build_overlay.sh /absolute/isolated/workspace" >&2
  exit 2
fi
task_script_dir="$(cd -- "$(dirname -- "$0")" && pwd)"
task_cm_workspace="$1"
python3 "$task_script_dir/prepare_overlay.py" --workspace "$task_cm_workspace"
source /opt/ros/jazzy/setup.bash
set -u
cd -- "$task_cm_workspace"
nice -n 10 colcon --log-base log build --base-paths src/controller_manager \
  --build-base build --install-base install \
  --packages-select controller_manager --executor sequential \
  --allow-overriding controller_manager \
  --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo \
               -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
sha256sum install/controller_manager/lib/libcontroller_manager.so
