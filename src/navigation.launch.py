"""Daily simulation navigation nodes. No experiment mover or acceptance owner."""
import os,sys
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction,RegisterEventHandler,EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    here=Path(__file__).resolve().parent;sys.path.insert(0,str(here));root=here.parent
    output=LaunchConfiguration('run_dir')
    def start(context):
        run=Path(output.perform(context)).resolve()
        from runtime_context import load
        profile=load(run/'runtime_context.json')['profile']
        from project_config import path
        scan=path('scan_workspace')/'install/scan_planner'
        native=path('native_workspace')/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping'
        ros=['--ros-args','-p','use_sim_time:=true']
        def python(name,*arguments):return ['python3','-B',str(here/name),*map(str,arguments),*ros]
        commands=[
            python('sensor_relay.py','--run-dir',run,'--mode',profile.get('sensor_relay_publish_mode','async')),
            [*python('self_echo_filter.py','--scenario',run/'navigation_scenario.json'),'-r','/livox/lidar:=/demo/teacher/raw_lidar'],
            ['/opt/ros/jazzy/lib/demo_nodes_cpp/parameter_blackboard',*ros,'--params-file',str(run/'navigation_camera.yaml')],
            [str(native),*ros,'--params-file',str(run/'navigation_fastlivo.yaml'),'--params-file',str(run/'navigation_camera.yaml'),'-r','/cloud_registered_full:=/demo/slam/raw_registered_full'],
            [*python('odom_adapter.py','--scenario',run/'navigation_scenario.json'),'-r','__node:=demo_slam_raw_odom_adapter','-r','/demo/slam/body_odom:=/demo/slam/raw_body_odom','-r','/demo/slam/lidar_odom:=/demo/slam/raw_lidar_odom'],
            [*python('map_archive.py','--run-dir',run),'-r','/livox/lidar:=/demo/teacher/raw_lidar','-r','/camera/image_color:=/demo/camera','-r','/demo/slam/body_odom:=/demo/slam/raw_body_odom','-r','/cloud_registered_full:=/demo/slam/raw_registered_full'],
            python('scan_epoch_supervisor.py','--run',run,'--planner',scan/'lib/scan_planner/scan_planner_node','--config',scan/'share/scan_planner/config/planner.yaml','--params',run/'v34_scan_overrides.yaml'),
            python('localization_runtime.py','--run',run),python('sensor_gate.py','--run',run),
            python('controller.py','--run',run),python('bridge.py','--runtime-config',run/'runtime_context.json','--command-file',run/'navigation_command.json'),
            python('mission_runtime.py','--run',run)]
        processes=[ExecuteProcess(cmd=command,output='screen')for command in commands]
        return processes+[RegisterEventHandler(OnProcessExit(target_action=p,on_exit=[EmitEvent(event=Shutdown(reason='Navigation dependency exited'))]))for p in processes]
    return LaunchDescription([DeclareLaunchArgument('run_dir'),OpaqueFunction(function=start)])
