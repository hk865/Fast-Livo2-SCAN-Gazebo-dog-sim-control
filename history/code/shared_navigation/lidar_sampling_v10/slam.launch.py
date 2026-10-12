"""Actual Teacher sensors to isolated FAST-LIVO2 only; no velocity publisher."""
import hashlib
import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here=Path(__file__).resolve().parent;teacher=here.parents[1];demo=teacher.parent
    run_dir=LaunchConfiguration('run_dir')
    def start(context):
        run=Path(run_dir.perform(context)).resolve()
        if not (run/'navigation_slam_contract.json').is_file():
            raise RuntimeError('Run scoped_profile.py --run <run> --slam-only before launching SLAM')
        if (run/'map_metadata.json').exists():
            raise RuntimeError('Reuse of an old SLAM map directory is forbidden')
        contract=json.loads((run/'navigation_slam_contract.json').read_text())
        for name,expected in contract['references'].items():
            if hashlib.sha256(Path(name).read_bytes()).hexdigest()!=expected:
                raise RuntimeError('Actual SLAM source/configuration hash changed: '+name)
        for package in ('fast_livo2_core','fast_livo2_ros'):
            expected=(here/'slam_ws/install'/package).resolve()
            if Path(get_package_prefix(package)).resolve()!=expected:
                raise RuntimeError('Source the V7 diagnostic SLAM overlay; package mismatch '+package)
        ros=['--ros-args','-p','use_sim_time:=true']
        processes=[
            ExecuteProcess(cmd=['python3',str(teacher/'navigation/sensor_relay.py'),*ros],output='screen'),
            ExecuteProcess(cmd=['python3',str(demo/'slam/self_echo_filter.py'),'--scenario',str(run/'navigation_scenario.json'),
                                *ros,'-r','/livox/lidar:=/demo/teacher/raw_lidar'],output='screen'),
            Node(package='demo_nodes_cpp',executable='parameter_blackboard',name='parameter_blackboard',
                 output='screen',parameters=[str(run/'navigation_camera.yaml'),{'use_sim_time':True}]),
            Node(package='fast_livo2_ros',executable='fastlivo_mapping',name='laserMapping',output='screen',
                 additional_env={'DEMO_RUN_DIR':str(run)},parameters=[str(run/'navigation_fastlivo.yaml'),
                    str(run/'navigation_camera.yaml'),{'use_sim_time':True}]),
            ExecuteProcess(cmd=['python3',str(demo/'slam/odom_adapter.py'),'--scenario',str(run/'navigation_scenario.json'),*ros],output='screen'),
            ExecuteProcess(cmd=['python3',str(demo/'slam/map_archive.py'),'--run-dir',str(run),*ros,
                                '-r','/livox/lidar:=/demo/teacher/raw_lidar','-r','/camera/image_color:=/demo/camera'],output='screen'),
        ]
        required=[RegisterEventHandler(OnProcessExit(target_action=p,on_exit=[EmitEvent(event=Shutdown(reason='Teacher sensor SLAM child ended'))]))for p in processes]
        return processes+required
    return LaunchDescription([DeclareLaunchArgument('run_dir',default_value=os.environ.get('DEMO_RUN_DIR','')),
                              OpaqueFunction(function=start)])
