"""FAST-LIVO2 sensor SLAM + explicit frame adapter + current-run RGB archive."""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix


def generate_launch_description():
    root = Path(__file__).resolve().parent
    run_dir = LaunchConfiguration('run_dir')
    use_sim_time = {'use_sim_time': True}
    def start(context):
        directory = run_dir.perform(context)
        if not directory or (Path(directory) / 'map_metadata.json').exists():
            raise RuntimeError('run_dir must be a new directory without previous map_metadata.json')
        expected = root / 'ros2_ws/install/fast_livo2_core'
        if Path(get_package_prefix('fast_livo2_core')).resolve() != expected.resolve():
            raise RuntimeError('Demo SLAM requires its isolated ros2_ws overlay; source its install/setup.bash after the stable workspace')
        return [
        ExecuteProcess(cmd=['python3', str(root / 'self_echo_filter.py'),
                            '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
        Node(package='demo_nodes_cpp', executable='parameter_blackboard',
             name='parameter_blackboard', output='screen',
             parameters=[str(root / 'camera.yaml'), use_sim_time]),
        Node(package='fast_livo2_ros', executable='fastlivo_mapping',
             name='laserMapping', output='screen',
             additional_env={'DEMO_RUN_DIR': directory},
             parameters=[str(root / 'fastlivo.yaml'), str(root / 'camera.yaml'), use_sim_time]),
        ExecuteProcess(cmd=['python3', str(root / 'odom_adapter.py'),
                            '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
        ExecuteProcess(cmd=['python3', str(root / 'map_archive.py'), '--run-dir', run_dir,
                            '--ros-args', '-p', 'use_sim_time:=true'], output='screen'),
        ]
    return LaunchDescription([
        DeclareLaunchArgument('run_dir', default_value=os.environ.get('DEMO_RUN_DIR', ''),
                              description='New unique directory for this run; must not contain an old map'),
        OpaqueFunction(function=start),
    ])
