#!/usr/bin/env bash
set -e
DEMO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$DEMO_ROOT/../go2_sim_control/install/setup.bash"
source "$DEMO_ROOT/../slam5_navigation/ros2_ws/install/setup.bash"
source "$DEMO_ROOT/../scan_multifloor/ros2_ws/install/setup.bash"
if [[ "${1:-}" == "build" ]]; then
 cd "$DEMO_ROOT/simulation/ros2_control_ws"
 colcon build --packages-select controller_manager --executor sequential --allow-overriding controller_manager --cmake-args -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
 cd "$DEMO_ROOT/slam/ros2_ws"
 colcon build --packages-select fast_livo2_core fast_livo2_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
 cd "$DEMO_ROOT/navigation/ros2_ws"
 exec colcon build --packages-select plan_env path_searching bspline_opt scan_planner --parallel-workers 2 --allow-overriding bspline_opt --cmake-args -DCMAKE_BUILD_TYPE=Release
fi
source "$DEMO_ROOT/slam/ros2_ws/install/local_setup.bash"
source "$DEMO_ROOT/navigation/ros2_ws/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export GZ_PARTITION="${GZ_PARTITION:-go2_multifloor_demo}"
export GZ_IP=127.0.0.1
export ROS_LOCALHOST_ONLY=1
export QT_QPA_PLATFORM=offscreen
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-0}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export ROS_LOG_DIR="${DEMO_RUN_DIR:-$DEMO_ROOT/test_results}/ros_logs"
mkdir -p "$ROS_LOG_DIR"
mode="${1:-serve}"; if [[ $# -gt 0 ]]; then shift; fi
case "$mode" in
 serve) exec python3 "$DEMO_ROOT/scripts/mission_server.py" "$@" ;;
 stack) exec ros2 launch "$DEMO_ROOT/scripts/stack.launch.py" "$@" ;;
 prepare) exec python3 "$DEMO_ROOT/simulation/prepare.py" ;;
 unit) exec python3 "$DEMO_ROOT/tests/test_mission.py" ;;
 *) echo 'Usage: run.sh {build|prepare|serve|stack|unit}' >&2; exit 2 ;;
esac
