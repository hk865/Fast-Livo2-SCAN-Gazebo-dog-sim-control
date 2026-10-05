"""Isolated physical sensor/SLAM fixture; no SCAN or mission is launched."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    root=Path(__file__).resolve().parents[1]
    run=Path(os.environ['DEMO_TURN_RUN'])
    gate=ExecuteProcess(cmd=['python3',str(root.parent/'scripts/wait_sensors.py')],output='screen')
    slam=[Node(package='demo_nodes_cpp',executable='parameter_blackboard',name='parameter_blackboard',
        parameters=[str(root/'camera.yaml'),{'use_sim_time':True}],output='screen'),
        Node(package='fast_livo2_ros',executable='fastlivo_mapping',name='laserMapping',
            parameters=[str(run/'fastlivo_probe.yaml'),str(root/'camera.yaml'),{'use_sim_time':True}],output='screen'),
        ExecuteProcess(cmd=['python3',str(root/'odom_adapter.py'),'--ros-args','-p','use_sim_time:=true'],output='screen'),
        ExecuteProcess(cmd=['python3',str(root/'map_archive.py'),'--run-dir',str(run),'--ros-args','-p','use_sim_time:=true'],output='screen')]
    if os.environ.get('DEMO_TURN_FILTER')=='1':
        slam.insert(0,ExecuteProcess(cmd=['python3',str(root/'self_echo_filter.py'),'--ros-args','-p','use_sim_time:=true'],output='screen'))
    def ready(event,context):
        if event.returncode!=0:raise RuntimeError('real sensor gate failed')
        return slam
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root.parent/'simulation/simulation.launch.py'))),
        gate,RegisterEventHandler(OnProcessExit(target_action=gate,on_exit=ready))])
