"""Only CHAMP executors; parent owns Gazebo, command reader and sensor SLAM."""
import hashlib
import json
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,ExecuteProcess,OpaqueFunction,RegisterEventHandler,EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

def generate_launch_description():
    run_dir=LaunchConfiguration('run_dir')
    def start(context):
        run=Path(run_dir.perform(context)).resolve();stage=run/'champ';contract=json.loads((run/'champ_contract.json').read_text())
        if contract.get('controller_kind')!='champ':raise RuntimeError('CHAMP contract required')
        for path,digest in contract['artifacts'].items():
            if hashlib.sha256(Path(path).read_bytes()).hexdigest()!=digest:raise RuntimeError('CHAMP asset changed: '+path)
        selected=contract['selected_gait'];old=os.environ.get('LD_LIBRARY_PATH','')
        gait_env={'LD_LIBRARY_PATH':str(Path(selected['library']['path']).parent)+':'+contract['loader']['gait_messages_library_dir']+(':'+old if old else ''),
            'AMENT_PREFIX_PATH':contract['loader']['gait_messages_ament_prefix']+':'+os.environ.get('AMENT_PREFIX_PATH','')}
        urdf=(stage/'champ.urdf').read_text();params={'use_sim_time':True}
        gait=Node(executable=selected['executable']['path'],additional_env=gait_env,
            parameters=[params,{'urdf':urdf},str(stage/'joints.yaml'),str(stage/'links.yaml'),str(stage/'gait.yaml'),
                {'gazebo':True,'publish_joint_states':False,'publish_joint_control':True,'publish_foot_contacts':False,
                 'joint_controller_topic':'/demo/control/joint_reference/raw','hardware_connected':False,'close_loop_odom':True}],
            remappings=[('/cmd_vel/smooth','/demo/control/actuator_cmd_vel'),('/body_pose','/demo/champ/disabled_body_pose')],output='screen')
        adapter=ExecuteProcess(cmd=['python3',str(stage/'joint_reference_adapter.py'),'--ros-args','-p','use_sim_time:=true'],output='screen',
            additional_env={'DEMO_RUN_DIR':str(run),'DEMO_TEST_GAIT_CONFIG':str(stage/'gait.yaml'),'DEMO_TEST_ROBOT_URDF':str(stage/'champ.urdf'),'DEMO_JOINT_STOP_ADAPTER':'1'})
        rsp=Node(package='robot_state_publisher',executable='robot_state_publisher',name='robot_state_publisher',parameters=[params,{'robot_description':urdf}],output='screen')
        spawner=Node(package='controller_manager',executable='spawner',arguments=['joint_states_controller','joint_group_effort_controller','--controller-manager-timeout','120'],parameters=[params],output='screen')
        def spawned(event,context):
            (stage/'spawner_exit.json').write_text(json.dumps({'returncode':event.returncode,'success':event.returncode==0})+'\n')
            return []if event.returncode==0 else[EmitEvent(event=Shutdown(reason='CHAMP controller spawner failed'))]
        required=[RegisterEventHandler(OnProcessExit(target_action=p,on_exit=[EmitEvent(event=Shutdown(reason='Required CHAMP executor exited'))]))for p in (gait,adapter,rsp)]
        return [*required,RegisterEventHandler(OnProcessExit(target_action=spawner,on_exit=spawned)),rsp,gait,adapter,spawner]
    return LaunchDescription([DeclareLaunchArgument('run_dir',default_value=os.environ.get('DEMO_RUN_DIR','')),OpaqueFunction(function=start)])
