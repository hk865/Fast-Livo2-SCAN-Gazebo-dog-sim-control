#!/usr/bin/env python3
"""Actual SCAN core + synthetic SLAM-interface fixture; not physical acceptance."""
import json
import os
import time
from pathlib import Path as FilePath
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Header, Bool
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from rosgraph_msgs.msg import Clock
from scan_planner_msgs.msg import Bspline

assert os.environ.get('ROS_DOMAIN_ID') == '75', 'requires isolated test domain 75'
rclpy.init()
node = Node('demo_scan_interface_probe')
odom_pub = node.create_publisher(Odometry, '/demo/navigation/scan_body_odom', 10)
lidar_pub = node.create_publisher(Odometry, '/demo/slam/lidar_odom', 10)
cloud_pub = node.create_publisher(PointCloud2, '/demo/navigation/cloud', 10)
clock_pub = node.create_publisher(Clock, '/clock', 10)
path_pub = node.create_publisher(Path, '/demo/navigation/scan_reference', 1)
freeze_pub = node.create_publisher(Bool, '/demo/navigation/execution_frozen', 10)
splines, occupancy = [], []
node.create_subscription(Bspline, '/demo/navigation/bspline', lambda m:splines.append(m), 10)
node.create_subscription(PointCloud2, '/demo/navigation/occupancy', lambda m:occupancy.append(m.width*m.height), qos_profile_sensor_data)
start = time.monotonic()
sent = False
try:
    while time.monotonic()-start < 8.:
        elapsed = time.monotonic()-start
        clock = Clock()
        clock.clock.sec = int(100+elapsed)
        clock.clock.nanosec = int((elapsed-int(elapsed))*1e9)
        clock_pub.publish(clock)
        freeze_pub.publish(Bool(data=True))  # Fixture is stationary; do not advance SCAN execution time.
        header = Header(frame_id='camera_init',stamp=clock.clock)
        odom = Odometry(header=header,child_frame_id='demo_scan_body_world_twist')
        odom.pose.pose.orientation.w = 1.
        odom_pub.publish(odom)
        lidar = Odometry(header=header,child_frame_id='velodyne')
        lidar.pose.pose.position.x = .2
        lidar.pose.pose.position.z = .1177
        lidar.pose.pose.orientation.w = 1.
        lidar_pub.publish(lidar)
        points = [[x*.1,y*.1,-.3] for x in range(-20,41) for y in range(-20,21)]
        points += [[x*.1,y,z*.1] for x in range(-20,41) for y in [-2.,2.] for z in range(-3,15)]
        cloud_pub.publish(create_cloud_xyz32(header,points))
        if not sent and elapsed > 1.5 and path_pub.get_subscription_count():
            route = Path(header=header)
            pose = PoseStamped(header=header)
            pose.pose.position.x = 2.
            pose.pose.position.z = -.2  # public body goal +.2 minus SCAN's internal .4
            pose.pose.orientation.w = 1.
            route.poses = [pose]
            path_pub.publish(route)
            sent = True
        rclpy.spin_once(node,timeout_sec=.025)
        time.sleep(.025)
    result = dict(scope='actual SCAN core with synthetic SLAM-interface fixture',
                  sent=sent, bsplines=len(splines), occupancy_messages=len(occupancy),
                  occupancy_points=max(occupancy or [0]),
                  first_trajectory_z_range=None if not splines else [min(p.z for p in splines[0].pos_pts),max(p.z for p in splines[0].pos_pts)])
    result['passed'] = sent and len(splines)>0 and max(occupancy or [0])>0
    FilePath(__file__).with_name('scan_interface_result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    assert result['passed'],result
finally:
    node.destroy_node()
    rclpy.shutdown()
