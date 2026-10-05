#!/usr/bin/env bash
set -eo pipefail
DEMO_ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
source /opt/ros/jazzy/setup.bash
source "$DEMO_ROOT/../scan_multifloor/ros2_ws/install/setup.bash"
if [[ -f "$DEMO_ROOT/navigation/ros2_ws/install/local_setup.bash" ]]; then
  source "$DEMO_ROOT/navigation/ros2_ws/install/local_setup.bash"
fi
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-72}
export ROS_LOG_DIR=${ROS_LOG_DIR:-$DEMO_ROOT/test_results/ros_logs}
mkdir -p "$ROS_LOG_DIR"
case "${1:-help}" in
  planner) shift; exec ros2 launch "$DEMO_ROOT/navigation/scan.launch.py" "$@" ;;
  controller) shift; exec python3 "$DEMO_ROOT/navigation/controller.py" "$@" ;;
  test) exec python3 "$DEMO_ROOT/navigation/test_control_core.py" ;;
  test-ros) exec python3 "$DEMO_ROOT/navigation/test_ros_contract.py" ;;
  test-scan) exec python3 "$DEMO_ROOT/navigation/test_scan_interface.py" ;;
  trace) shift; exec python3 "$DEMO_ROOT/navigation/record_feedback.py" "$@" ;;
  *) echo 'Usage: navigation/run.sh {planner|controller|test|test-ros|test-scan|trace}' ;;
esac
