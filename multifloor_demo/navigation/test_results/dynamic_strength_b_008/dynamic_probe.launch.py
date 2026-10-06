"""Actual bounded two-goal probe, complete production sensors/SLAM/SCAN/NAV."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    root=Path(os.environ['DEMO_TEST_ROOT'])
    out=Path(os.environ['DEMO_RUN_DIR'])
    probe=ExecuteProcess(cmd=['python3',str(out/'probe_dynamic_navigation.py'),str(out/'dynamic_result.json')],output='screen')
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(target_action=probe,on_exit=[EmitEvent(event=Shutdown(reason='actual two-goal component finished'))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(out/'strength_stack.launch.py'))),probe])
