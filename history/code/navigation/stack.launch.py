"""Independent finite Teacher navigation stack; no Gazebo/CHAMP/camera carrier."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,IncludeLaunchDescription,OpaqueFunction,RegisterEventHandler,EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    import sys
    here=Path(__file__).resolve().parent;teacher=here.parents[1];demo=teacher.parent;nav=here.parent
    sys.path.insert(0,str(here))
    from pid_scope import verify_scope
    DYNAMIC_EXPERIMENTS=set()
    run_dir=LaunchConfiguration('run_dir')
    def start(context):
        run=Path(run_dir.perform(context)).resolve();receipt=verify_scope(run/'navigation_scope.json')
        from scan_workspace import scan_contract
        scan_workspace,_,_=scan_contract(receipt['profile'])
        expected=(scan_workspace/'install/scan_planner').resolve()
        if Path(get_package_prefix('scan_planner')).resolve()!=expected:
            raise RuntimeError('Source existing isolated SCAN overlay; package mismatch')
        profile=receipt['profile'];ros=['--ros-args','-p','use_sim_time:=true']
        planner=ExecuteProcess(cmd=['python3',str(here/'scan_epoch_supervisor.py'),'--run',str(run),
            '--planner',str(scan_workspace/'install/scan_planner/lib/scan_planner/scan_planner_node'),
            '--config',str(Path(get_package_share_directory('scan_planner'))/'config/planner.yaml'),
            '--params',str(run/'v34_scan_overrides.yaml'),*ros],output='screen')
        processes=[planner,
            ExecuteProcess(cmd=['python3',str(here/'known_scene_runtime.py'),'--run',str(run),*ros],output='screen'),
            ExecuteProcess(cmd=['python3',str(here/'sensor_gate.py'),'--run',str(run),*ros],output='screen'),
            ExecuteProcess(cmd=['python3',str(here/'controller.py'),'--run',str(run),*ros],output='screen',additional_env={'DEMO_RUN_DIR':str(run)}),
            ExecuteProcess(cmd=['python3',str(here/'bridge.py'),'--acceptance',str(run/'navigation_scope.json'),
                                '--command-file',str(run/'navigation_command.json'),*ros],output='screen'),
            ExecuteProcess(cmd=['python3',str(here/('mission46_runtime.py' if profile.get('mission46_required') else 'request.py')),'--run',str(run),*ros],output='screen')]
        if profile.get('mission46_required'):
            processes += [
                Node(package='ros_gz_bridge',executable='parameter_bridge',
                    arguments=['/world/teacher_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose'],output='screen'),
                ExecuteProcess(cmd=['python3',str(here/'mission46_obstacle_runtime.py'),'--run',str(run),*ros],output='screen')]
        if receipt['experiment'] in DYNAMIC_EXPERIMENTS:
            from dynamic.dynamic_obstacle import verify_scope as verify_dynamic_scope
            verify_dynamic_scope(run,run/'dynamic_protocol.json')
            dynamic=here/'dynamic'
            processes += [
                Node(package='ros_gz_bridge',executable='parameter_bridge',
                    arguments=['/world/teacher_demo/set_pose@ros_gz_interfaces/srv/SetEntityPose'],output='screen'),
                ExecuteProcess(cmd=[str(dynamic/'build/obstacle_pose_observer'),'/world/teacher_demo/pose/info',
                    str(run/'obstacle_actual_pose.jsonl'),'900'],output='screen'),
                ExecuteProcess(cmd=['python3',str(dynamic/'dynamic_obstacle.py'),'--execute','--run',str(run),
                    '--role','sensor_observer',*ros],output='screen'),
                ExecuteProcess(cmd=['python3',str(dynamic/'dynamic_obstacle.py'),'--execute','--run',str(run),
                    '--role','mover',*ros],output='screen')]
        required=[RegisterEventHandler(OnProcessExit(target_action=p,on_exit=[EmitEvent(event=Shutdown(reason='Teacher finite navigation child ended'))]))for p in processes]
        slam=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(here/'slam.launch.py')),
                                      launch_arguments={'run_dir':str(run)}.items())
        return [slam,*processes,*required]
    return LaunchDescription([DeclareLaunchArgument('run_dir',default_value=os.environ.get('DEMO_RUN_DIR','')),
                              OpaqueFunction(function=start)])
