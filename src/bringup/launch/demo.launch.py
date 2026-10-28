from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from common.config import ROOT

def generate_launch_description():
    modules=[('localization','slam'),('planning','scan'),('control','control'),('simulation','simulation')]
    return LaunchDescription([DeclareLaunchArgument('run_dir'),*[
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(ROOT/'src'/module/'launch'/(name+'.launch.py'))),
            launch_arguments={'run_dir':LaunchConfiguration('run_dir')}.items())
        for module,name in modules]])
