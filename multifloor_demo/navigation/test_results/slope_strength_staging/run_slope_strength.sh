#!/usr/bin/env bash
set -e
DEMO_DYNAMIC_STAGE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
DEMO_DYNAMIC_ROOT="${DEMO_TEST_ROOT:-$DEMO_DYNAMIC_STAGE}"
while [[ ! -f "$DEMO_DYNAMIC_ROOT/scripts/stack.launch.py" ]]; do
  if [[ "$DEMO_DYNAMIC_ROOT" == / ]]; then echo 'Demo root not found' >&2; exit 2; fi
  DEMO_DYNAMIC_ROOT="$(dirname -- "$DEMO_DYNAMIC_ROOT")"
done
export DEMO_TEST_ROOT="$DEMO_DYNAMIC_ROOT"
source /opt/ros/jazzy/setup.bash
source "$DEMO_DYNAMIC_ROOT/../go2_sim_control/install/setup.bash"
source "$DEMO_DYNAMIC_ROOT/../slam5_navigation/ros2_ws/install/setup.bash"
source "$DEMO_DYNAMIC_ROOT/../scan_multifloor/ros2_ws/install/setup.bash"
source "$DEMO_DYNAMIC_ROOT/slam/ros2_ws/install/local_setup.bash"
source "$DEMO_DYNAMIC_ROOT/navigation/ros2_ws/install/local_setup.bash"
export GZ_IP=127.0.0.1 ROS_LOCALHOST_ONLY=1 QT_QPA_PLATFORM=offscreen
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
exec python3 "$DEMO_DYNAMIC_STAGE/run_slope_strength.py" "$@"
