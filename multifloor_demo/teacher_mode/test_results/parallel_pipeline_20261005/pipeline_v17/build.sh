#!/bin/bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /home/hyh001/projects/third_party/livox_ws/install/setup.bash
source /home/hyh001/projects/1.Project/Ros2_fastlivo2_/slam5_navigation/ros2_ws/install/setup.bash
export MAKEFLAGS='-j2 -l2'
export CMAKE_BUILD_PARALLEL_LEVEL=2
cd /home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/teacher_mode/navigation/ingress_pipeline_v17/slam_ws
colcon build --executor sequential --packages-select fast_livo2_core fast_livo2_ros --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
