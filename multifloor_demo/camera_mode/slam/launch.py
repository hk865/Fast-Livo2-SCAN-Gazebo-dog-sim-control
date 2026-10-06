"""Camera rig's genuine isolated FAST-LIVO2, frame adapter and RGB archive.

Include only after health_gate.py --phase sensors succeeds. The post-SLAM gate
is separate, because the SLAM core must initialize using actual sensor input.
"""
import json
import os
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from _shared import ROOT, snapshot_contract


def generate_launch_description():
    run_dir = LaunchConfiguration('run_dir')
    scenario_path = LaunchConfiguration('scenario')

    def start(context):
        directory = Path(run_dir.perform(context)).resolve()
        scenario = Path(scenario_path.perform(context)).resolve()
        if not run_dir.perform(context) or (directory / 'map_metadata.json').exists():
            raise RuntimeError('Camera SLAM requires a new run_dir without an old map')
        data = json.loads(scenario.read_text())
        if data.get('mode') != 'camera':
            raise RuntimeError('Camera SLAM requires an explicitly declared camera scenario')
        expected = ROOT / 'slam/ros2_ws/install/fast_livo2_core'
        if Path(get_package_prefix('fast_livo2_core')).resolve() != expected.resolve():
            raise RuntimeError('Camera SLAM requires the original isolated FAST-LIVO2 overlay')
        expected_wrapper = ROOT / 'slam/ros2_ws/install/fast_livo2_ros'
        if Path(get_package_prefix('fast_livo2_ros')).resolve() != expected_wrapper.resolve():
            raise RuntimeError('Camera SLAM requires the original isolated FAST-LIVO2 ROS wrapper')
        directory.mkdir(parents=True, exist_ok=True)
        snapshot_contract(directory)
        sim = {'use_sim_time': True}
        return [
            ExecuteProcess(cmd=['python3', str(HERE/'lidar_relay.py'), '--frame', data['sensors']['lidar']['frame'],
                                '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
            Node(package='demo_nodes_cpp', executable='parameter_blackboard', name='parameter_blackboard',
                 parameters=[str(HERE/'camera.yaml'), sim], output='screen'),
            Node(package='fast_livo2_ros', executable='fastlivo_mapping', name='laserMapping', output='screen',
                 additional_env={'DEMO_RUN_DIR': str(directory)},
                 parameters=[str(HERE/'fastlivo.yaml'), str(HERE/'camera.yaml'), sim]),
            ExecuteProcess(cmd=['python3', str(ROOT/'slam/odom_adapter.py'), '--scenario', str(scenario),
                                '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
            ExecuteProcess(cmd=['python3', str(ROOT/'slam/map_archive.py'), '--run-dir', str(directory),
                                '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('run_dir', default_value=os.environ.get('DEMO_RUN_DIR', '')),
        DeclareLaunchArgument('scenario', default_value=str(HERE.parents[0]/'simulation/scenario.json')),
        OpaqueFunction(function=start),
    ])
