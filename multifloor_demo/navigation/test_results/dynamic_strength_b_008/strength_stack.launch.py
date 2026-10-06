"""Full sensor/SLAM/planning stack, isolated from historical examples."""
from pathlib import Path
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    root = Path(os.environ['DEMO_TEST_ROOT']).resolve()
    directory = os.environ.get('DEMO_RUN_DIR')
    if not directory:
        raise RuntimeError('DEMO_RUN_DIR is required for the full physical stack')
    def include(path):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/path)))
    gate = ExecuteProcess(cmd=['python3', str(root/'scripts/wait_sensors.py')], output='screen')
    sensor_audit = ExecuteProcess(cmd=['python3', str(root/'scripts/record_sensors.py'),
                                 '--output', str(Path(directory)/'sensor_audit.jsonl')], output='screen')
    feedback_audit = ExecuteProcess(cmd=['python3', str(root/'navigation/record_feedback.py'),
                                   '--output', str(Path(directory)/'feedback_navigation.jsonl'),
                                   '--duration', '7200'], output='screen')
    def after_audit(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='Required read-only audit recorder failed; stopping this simulation.'),
                    EmitEvent(event=Shutdown(reason='Required sensor/control evidence recorder failed.'))]
        return []
    def after_gate(event, context):
        if event.returncode != 0:
            return [LogInfo(msg='Sensor startup gate failed; stopping this simulation.'),
                    EmitEvent(event=Shutdown(reason='Required physical sensors did not become ready.'))]
        return [include('slam/launch.py'), include('navigation/scan.launch.py'),
                ExecuteProcess(cmd=['python3', str(Path(directory)/'controller_strength_wrapper.py')], output='screen')]
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=sensor_audit, on_exit=after_audit)),
        RegisterEventHandler(OnProcessExit(target_action=feedback_audit, on_exit=after_audit)),
        sensor_audit,
        feedback_audit,
        RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=after_gate)),
        include('simulation/simulation.launch.py'),
        gate,
    ])
