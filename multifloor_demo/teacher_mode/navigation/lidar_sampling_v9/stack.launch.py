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
        expected=(demo/'navigation/ros2_ws/install/scan_planner').resolve()
        if Path(get_package_prefix('scan_planner')).resolve()!=expected:
            raise RuntimeError('Source existing isolated SCAN overlay; package mismatch')
        profile=receipt['profile'];ros=['--ros-args','-p','use_sim_time:=true']
        planner=Node(package='scan_planner',executable='scan_planner_node',name='scan_planner_node',output='screen',
            parameters=[str(Path(get_package_share_directory('scan_planner'))/'config/planner.yaml'),{
                'use_sim_time':True,'fsm.navi_mode':3,'fsm.planning_horizon':2.5,'fsm.thresh_replan':.35,
                'fsm.max_replan_fail_count':50,'fsm.measured_max_speed':profile['max_speed_mps'],
                'fsm.measured_velocity_filter_tau':.4,'grid_map.frame_id':'camera_init',
                'grid_map.sliding_map_frame_id':'teacher_scan_sliding_map','grid_map.sensor_type':'lidar',
                'grid_map.cloud_is_world':True,'grid_map.need_extrinsic':False,'grid_map.resolution':.08,
                'grid_map.body_height':.4,'grid_map.obstacles_inflation_z_down':.12,
                'grid_map.obstacles_inflation_z_up':.12,'grid_map.double_cylinder_radius':.25,
                'grid_map.double_cylinder_offset':.18,'grid_map.sliding_map_size_x':10.,
                'grid_map.sliding_map_size_y':10.,'grid_map.sliding_map_size_z':5.,
                'grid_map.local_update_range_x':5.,'grid_map.local_update_range_y':5.,'grid_map.local_update_range_z':2.5,
                'manager.max_vel':profile['max_speed_mps'],'manager.max_acc':.15,'manager.planning_horizon':2.5,
                'optimization.max_vel':profile['max_speed_mps'],'optimization.max_acc':.15}],
            remappings=[('body_pose','/demo/navigation/scan_body_odom'),('sensor_pose','/demo/slam/lidar_odom'),
                ('cloud','/demo/navigation/cloud'),('initial_path','/demo/navigation/scan_reference'),
                ('planning/bspline','/demo/navigation/bspline'),('planning/trajectory_metadata','/demo/navigation/trajectory_metadata'),
                ('planning/go2_execution_frozen','/demo/navigation/execution_frozen'),('grid_map/occupancy','/demo/navigation/occupancy')])
        processes=[planner,
            ExecuteProcess(cmd=['python3',str(here/'sensor_gate.py'),'--run',str(run),*ros],output='screen'),
            ExecuteProcess(cmd=['python3',str(here/'controller.py'),'--run',str(run),*ros],output='screen',additional_env={'DEMO_RUN_DIR':str(run)}),
            ExecuteProcess(cmd=['python3',str(here/'bridge.py'),'--acceptance',str(run/'navigation_scope.json'),
                                '--command-file',str(run/'navigation_command.json'),*ros],output='screen'),
            ExecuteProcess(cmd=['python3',str(here/'request.py'),'--run',str(run),*ros],output='screen')]
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
