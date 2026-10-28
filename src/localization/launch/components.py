from common.config import path
from common.runtime_context import load
from bringup.launching import python,owned,ROS
def components(run):
    profile=load(run/'runtime_context.json')['profile'];prefix=path('ros_prefix')
    commands=[
      ('sensor_relay',python('localization.sensors.sensor_relay','--run-dir',run,'--mode',profile.get('sensor_relay_publish_mode','async'))),
      ('self_filter',python('localization.sensors.self_echo_filter','--scenario',run/'navigation_scenario.json')+['-r','/livox/lidar:=/demo/teacher/raw_lidar']),
      ('camera_parameters',['/opt/ros/jazzy/lib/demo_nodes_cpp/parameter_blackboard',*ROS,'--params-file',str(run/'navigation_camera.yaml')]),
      ('slam',[str(prefix/'lib/fast_livo2_ros/fastlivo_mapping'),*ROS,'--params-file',str(run/'navigation_fastlivo.yaml'),'--params-file',str(run/'navigation_camera.yaml'),'-r','/cloud_registered_full:=/demo/slam/raw_registered_full']),
      ('odom_adapter',python('localization.odom_adapter','--scenario',run/'navigation_scenario.json')+['-r','__node:=demo_slam_raw_odom_adapter','-r','/demo/slam/body_odom:=/demo/slam/raw_body_odom','-r','/demo/slam/lidar_odom:=/demo/slam/raw_lidar_odom']),
      ('map_archive',python('localization.mapping.map_archive','--run-dir',run)+['-r','/livox/lidar:=/demo/teacher/raw_lidar','-r','/camera/image_color:=/demo/camera','-r','/demo/slam/body_odom:=/demo/slam/raw_body_odom','-r','/cloud_registered_full:=/demo/slam/raw_registered_full']),
      ('global_matching',python('localization.localization_runtime','--run',run)),
      ('sensor_gate',python('localization.sensors.sensor_gate','--run',run)),
    ]
    return [action for role,command in commands for action in owned(role,command,run)]
