"""SCAN using only measured SLAM cloud/pose, with no bundled simulator/controller."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    from ament_index_python.packages import get_package_prefix
    expected = os.path.realpath(os.path.join(os.path.dirname(__file__), 'ros2_ws', 'install', 'scan_planner'))
    actual = os.path.realpath(get_package_prefix('scan_planner'))
    if actual != expected:
        raise RuntimeError(f'Go2 Demo requires its isolated SCAN overlay: {expected}; resolved {actual}')
    base = os.path.join(get_package_share_directory("scan_planner"), "config", "planner.yaml")
    return LaunchDescription([Node(
        package="scan_planner", executable="scan_planner_node", name="scan_planner_node",
        output="screen", parameters=[base, {
            "use_sim_time": True,
            "fsm.navi_mode": 3,
            "fsm.planning_horizon": 2.5,
            "fsm.thresh_replan": 0.35,
            "fsm.max_replan_fail_count": 50,
            "fsm.measured_max_speed": 0.12,
            "fsm.measured_velocity_filter_tau": 0.4,
            "grid_map.frame_id": "camera_init",
            "grid_map.sliding_map_frame_id": "demo_scan_sliding_map",
            "grid_map.sensor_type": "lidar",
            "grid_map.cloud_is_world": True,
            "grid_map.need_extrinsic": False,
            "grid_map.resolution": 0.08,
            "grid_map.body_height": 0.4,
            # Go2 trunk is 0.114 m high. With 0.08 m voxels, 0.12 m rounds up
            # to a 0.16 m vertical margin around occupied cells. 0.18 m would
            # round to 0.24 m, causing support-floor cells to occupy the same
            # voxel as the base center near z=0 (especially negative small z).
            "grid_map.obstacles_inflation_z_down": 0.12,
            "grid_map.obstacles_inflation_z_up": 0.12,
            "grid_map.double_cylinder_radius": 0.25,
            "grid_map.double_cylinder_offset": 0.18,
            "grid_map.sliding_map_size_x": 10.0,
            "grid_map.sliding_map_size_y": 10.0,
            "grid_map.sliding_map_size_z": 5.0,
            "grid_map.local_update_range_x": 5.0,
            "grid_map.local_update_range_y": 5.0,
            "grid_map.local_update_range_z": 2.5,
            "manager.max_vel": 0.12,
            "manager.max_acc": 0.15,
            "manager.planning_horizon": 2.5,
            "optimization.max_vel": 0.12,
            "optimization.max_acc": 0.15,
        }], remappings=[
            ("body_pose", "/demo/navigation/scan_body_odom"),
            ("sensor_pose", "/demo/slam/lidar_odom"),
            ("cloud", "/demo/navigation/cloud"),
            ("initial_path", "/demo/navigation/scan_reference"),
            ("planning/bspline", "/demo/navigation/bspline"),
            ("planning/trajectory_metadata", "/demo/navigation/trajectory_metadata"),
            ("planning/go2_execution_frozen", "/demo/navigation/execution_frozen"),
            ("grid_map/occupancy", "/demo/navigation/occupancy"),
        ]
    )])
