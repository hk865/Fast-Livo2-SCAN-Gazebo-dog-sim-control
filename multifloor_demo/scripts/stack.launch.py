"""Full mission startup: actual controllers, libraries and sensors before SLAM/NAV."""
import os,json,time
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    root=Path(__file__).resolve().parents[1];out=Path(os.environ['DEMO_RUN_DIR'])
    def include(path):return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))
    gate=ExecuteProcess(cmd=['python3',str(root/'scripts/wait_sensors.py')],output='screen')
    ready_gate=ExecuteProcess(cmd=['python3',str(root/'simulation/await_controller_ready.py'),
        '--receipt',str(out/'controller_spawner_exit.json'),'--output',str(out/'controller_ready_gate.json')],output='screen')
    parameter_gate=ExecuteProcess(cmd=['python3',str(root/'simulation/control_parameter_gate.py'),
        '--expected','true',
        '--profile',str(root/'simulation/config/ros_control.yaml'),'--output',str(out/'control_parameter_readback.json')],output='screen')
    runtime_gate=ExecuteProcess(cmd=['python3',str(root/'simulation/capture_control_runtime.py'),
        '--manifest',str(out/'runtime_manifest.json'),'--output',str(out/'startup_control_runtime.json')],output='screen')
    gait_runtime_gate=ExecuteProcess(cmd=['python3',str(root/'simulation/gait_runtime.py'),
        '--manifest',str(out/'runtime_manifest.json'),'--output',str(out/'startup_gait_runtime.json')],output='screen')
    sensor=ExecuteProcess(cmd=['python3',str(root/'scripts/record_sensors.py'),
        '--output',str(out/'sensor_audit.jsonl')],output='screen')
    trace=ExecuteProcess(cmd=['python3',str(root/'navigation/record_feedback.py'),
        '--output',str(out/'feedback_navigation.jsonl'),'--duration','7200'],output='screen')
    navigator=ExecuteProcess(cmd=['python3',str(root/'navigation/align_controller.py'),
        '--ros-args','-p','scenario_path:='+str(root/'simulation/scenario.json')],output='screen',
        additional_env={'DEMO_TEST_ALIGN_TRANSLATION_ENABLED':'1'})
    handlers=[RegisterEventHandler(OnProcessExit(target_action=action,on_exit=[
        EmitEvent(event=Shutdown(reason='Required full mission evidence/navigation process exited'))]))
        for action in (sensor,trace,navigator)]
    def after_gate(event,context):
        if event.returncode!=0:return [EmitEvent(event=Shutdown(reason='Actual sensor startup gate failed'))]
        return [include(root/'slam/launch.py'),include(root/'navigation/scan.launch.py'),navigator]
    def after_ready_gate(event,context):
        if event.returncode!=0:return [EmitEvent(event=Shutdown(reason='Actual controller startup failed'))]
        return [parameter_gate]
    def after_parameter_gate(event,context):
        if event.returncode!=0:return [EmitEvent(event=Shutdown(reason='Actual CM/JTC/PID readback failed'))]
        return [runtime_gate]
    def after_runtime_gate(event,context):
        if event.returncode!=0:return [EmitEvent(event=Shutdown(reason='Actual startup control runtime capture failed'))]
        return [gait_runtime_gate]
    def after_gait_runtime_gate(event,context):
        if event.returncode!=0:return [EmitEvent(event=Shutdown(reason='Actual private CHAMP runtime capture failed'))]
        return [gate]
    return LaunchDescription([*handlers,sensor,trace,
        RegisterEventHandler(OnProcessExit(target_action=gate,on_exit=after_gate)),
        RegisterEventHandler(OnProcessExit(target_action=ready_gate,on_exit=after_ready_gate)),
        RegisterEventHandler(OnProcessExit(target_action=parameter_gate,on_exit=after_parameter_gate)),
        RegisterEventHandler(OnProcessExit(target_action=runtime_gate,on_exit=after_runtime_gate)),
        RegisterEventHandler(OnProcessExit(target_action=gait_runtime_gate,on_exit=after_gait_runtime_gate)),
        include(root/'simulation/simulation.launch.py'),ready_gate])
