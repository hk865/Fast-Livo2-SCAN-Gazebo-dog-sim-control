#!/usr/bin/env python3
"""ROS 75 real SCAN mapper; seeded component fixture, not physical acceptance."""
import hashlib
import json
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy
from std_msgs.msg import Bool, Header


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '75', 'isolated test domain 75 required'
    root = Path(__file__).resolve().parents[2]
    archive = root / 'runs/20260930_180318_efeddd/feedback_navigation_clouds/00000212.364'
    meta = json.loads(Path(str(archive) + '.json').read_text())
    points = np.load(str(archive) + '.npz')['filtered']
    origin = meta['topics']['filtered']['nearest_slam_lidar']['pose']
    body = meta['topics']['filtered']['nearest_slam_body']['pose']
    seeds = np.loadtxt(root / 'navigation/test_results/run8_ray_fixture.txt', skiprows=1, max_rows=10)
    # Outside the archived observed XY radius, still inside the physical mapper.
    hidden = np.array([.84, 4.04, .12])
    def cells(cloud):
        return {tuple(row) for row in np.floor(np.asarray(cloud) / .08 + 1e-6).astype(int)}
    seed_cells = cells(seeds)
    hidden_cell = next(iter(cells([hidden])))
    rclpy.init()
    node = Node('demo_ros_raycast_fixture')
    pubs = {topic: node.create_publisher(typ, topic, 10) for topic, typ in [
        ('/clock', Clock), ('/demo/navigation/scan_body_odom', Odometry),
        ('/demo/slam/lidar_odom', Odometry), ('/demo/navigation/cloud', PointCloud2),
        ('/demo/navigation/execution_frozen', Bool)]}
    maps = {}
    def receive(name, message):
        maps[name] = (time.monotonic(), cells(read_points_numpy(message, field_names=('x', 'y', 'z'), skip_nans=True)))
    for name, topic in [('occupied', '/demo/navigation/occupancy'), ('inflated', '/grid_map/occupancy_inflate')]:
        node.create_subscription(PointCloud2, topic, lambda m, n=name: receive(n, m), qos_profile_sensor_data)
    tick = 0
    def frame(cloud, duration=.22):
        nonlocal tick
        tick += 1
        clock = Clock()
        clock.clock.sec = 212 + tick // 5
        clock.clock.nanosec = (tick % 5) * 200000000
        header = Header(stamp=clock.clock, frame_id='camera_init')
        pubs['/clock'].publish(clock)
        pubs['/demo/navigation/execution_frozen'].publish(Bool(data=True))
        for topic, xyz, child in [('/demo/navigation/scan_body_odom', body, 'demo_scan_body_world_twist'),
                                  ('/demo/slam/lidar_odom', origin, 'velodyne')]:
            odom = Odometry(header=header, child_frame_id=child)
            odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = xyz
            odom.pose.pose.orientation.w = 1.
            pubs[topic].publish(odom)
        # Let the sensor callback precede this frame's cloud callback.
        rclpy.spin_once(node, timeout_sec=.02)
        pubs['/demo/navigation/cloud'].publish(create_cloud_xyz32(header, cloud))
        until = time.monotonic() + duration
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.02)
    try:
        until = time.monotonic() + 5
        while not pubs['/demo/navigation/cloud'].get_subscription_count() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.05)
        assert pubs['/demo/navigation/cloud'].get_subscription_count(), 'actual SCAN cloud subscriber missing'
        for _ in range(6):
            frame(np.vstack([seeds, hidden]))
        before = maps.get('occupied', (0, set()))[1]
        before_inflated = maps.get('inflated', (0, set()))[1]
        after_started = time.monotonic()
        for _ in range(10):
            frame(points)
        after_stamp, after = maps.get('occupied', (0, set()))
        inflated_stamp, after_inflated = maps.get('inflated', (0, set()))
        checks = {
            'all_10_seed_cells_observed_occupied_before_replay': seed_cells <= before,
            'all_10_observed_stale_cells_removed_after_replay': not (seed_cells & after),
            'unobserved_old_obstacle_kept': hidden_cell in before and hidden_cell in after,
            'unobserved_old_obstacle_inflation_kept': hidden_cell in before_inflated and hidden_cell in after_inflated,
            'fresh_actual_occupancy_messages': after_stamp > after_started,
            'fresh_actual_inflated_messages': inflated_stamp > after_started,
            'actual_measured_surfaces_still_occupied': len(after) > 100,
        }
        binary = Path(get_package_prefix('scan_planner')) / 'lib/scan_planner/scan_planner_node'
        result = {
            'scope': 'real ROS 75 SCAN mapper; manually seeded historical cells then actual archived rays; no Gazebo/GT/navigation commands',
            'snapshot': str(archive.relative_to(root)), 'ray_count': len(points),
            'source_lidar_stamp_gap_s': meta['topics']['filtered']['lidar_stamp_difference_s'],
            'seed_occupied_before': len(seed_cells & before), 'seed_occupied_after': len(seed_cells & after),
            'occupied_before': len(before), 'occupied_after': len(after),
            'inflated_after': len(after_inflated), 'checks': checks, 'passed': all(checks.values()),
            'scan_planner_prefix': get_package_prefix('scan_planner'),
            'scan_planner_binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest(),
        }
        (root / 'navigation/test_results/ros_raycast_result.json').write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps(result, indent=2))
        assert result['passed'], result
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
