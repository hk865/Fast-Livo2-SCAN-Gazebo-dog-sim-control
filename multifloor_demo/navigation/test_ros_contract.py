#!/usr/bin/env python3
"""Isolated component test, using synthetic ROS messages (NOT SLAM acceptance).

Run a controller in ROS_DOMAIN_ID=75 first. The test must not share a simulator
domain. It checks the real message paths, watchdog and stop/resume behavior.
"""
import json
import math
import os
import time
from pathlib import Path as FilePath
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import String, Bool, Header
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import PointCloud2, Imu
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from rosgraph_msgs.msg import Clock
from scan_planner_msgs.msg import Bspline
from trajectory_contract import payload


assert os.environ.get('ROS_DOMAIN_ID') == '75', 'Synthetic component test requires isolated ROS_DOMAIN_ID=75'
rclpy.init()
node = Node('demo_navigation_contract_probe')
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
state, cmds, references = {}, [], []
node.create_subscription(String, '/demo/navigation/status', lambda m: state.update(json.loads(m.data)), qos)
node.create_subscription(Twist, '/demo/cmd_vel', lambda m: cmds.append([m.linear.x,m.linear.y,m.angular.z]), 10)
node.create_subscription(Path, '/demo/navigation/scan_reference', lambda m: references.append(m), 1)
clock_pub = node.create_publisher(Clock, '/clock', 10)
odom_pub = node.create_publisher(Odometry, '/demo/slam/body_odom', 10)
cloud_pub = node.create_publisher(PointCloud2, '/cloud_registered_full', 10)
request_pub = node.create_publisher(String, '/demo/navigation/request', 10)
stop_pub = node.create_publisher(Bool, '/demo/navigation/stop', 10)
spline_pub = node.create_publisher(Bspline, '/demo/navigation/bspline', 10)
metadata_pub = node.create_publisher(String, '/demo/navigation/trajectory_metadata', 10)
imu_pub = node.create_publisher(Imu, '/livox/imu', 10)
bridge_safety_pub = node.create_publisher(String, '/demo/control/safety', 10)
sim_time = 10.
fixed_stamp = None
fixed_imu_stamp = None
next_traj_id = 0


def pump(duration, *, pose=(0.,0.,0.), obstacle=False, odom=True, fresh_stamp=True, roll=0.,
         imu=True, imu_roll=0., imu_fresh=True):
    global sim_time, fixed_stamp, fixed_imu_stamp
    end = time.monotonic()+duration
    while time.monotonic() < end:
        sim_time += .04
        clock = Clock()
        clock.clock.sec = int(sim_time)
        clock.clock.nanosec = int((sim_time-int(sim_time))*1e9)
        clock_pub.publish(clock)
        if fresh_stamp or fixed_stamp is None:
            fixed_stamp = clock.clock
        header = Header(frame_id='camera_init', stamp=fixed_stamp)
        if imu:
            if imu_fresh or fixed_imu_stamp is None:
                fixed_imu_stamp = clock.clock
            raw = Imu(header=Header(frame_id='imu_link', stamp=fixed_imu_stamp))
            raw.orientation.x = math.sin(imu_roll/2)
            raw.orientation.w = math.cos(imu_roll/2)
            imu_pub.publish(raw)
        if odom:
            msg = Odometry(header=header, child_frame_id='base')
            msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = pose
            msg.pose.pose.orientation.x = math.sin(roll/2)
            msg.pose.pose.orientation.w = math.cos(roll/2)
            odom_pub.publish(msg)
        points = [[x,y,-.3] for x in np.arange(.4,1,.1) for y in [-.2,0,.2]]
        if obstacle:
            points += [[.65,y,z] for y in [-.1,0,.1] for z in [0.,.1,.2]]
        cloud_pub.publish(create_cloud_xyz32(header, points))
        rclpy.spin_once(node, timeout_sec=.02)
        time.sleep(.02)


def publish_trajectory(msg):
    global next_traj_id
    next_traj_id += 1
    msg.traj_id=next_traj_id
    ref=references[-1]
    goal=ref.poses[-1].pose.position
    metadata=dict(schema=1,reference_stamp=[ref.header.stamp.sec,ref.header.stamp.nanosec],
                  body_goal=[goal.x,goal.y,goal.z+.4],trajectory=payload(msg))
    spline_pub.publish(msg)
    metadata_pub.publish(String(data=json.dumps(metadata)))


def trajectory():
    msg = Bspline(order=3, traj_id=1, knots=[0.,0.,0.,0.,5.,5.,5.,5.])
    msg.pos_pts = [Point(x=x,y=0.,z=0.) for x in [0.,1/3,2/3,1.]]
    publish_trajectory(msg)


results = []
def check(name, condition):
    results.append({'name':name,'passed':bool(condition)})
    if not condition:
        raise AssertionError(f'{name}: state={state}, last_commands={cmds[-5:]}')


try:
    pump(1.2)
    check('controller discovered and idle', state.get('state') == 'idle')
    request = dict(request_id='contract-1', frame_id='camera_init', waypoints=[[1.,0.,0.]])
    request_pub.publish(String(data=json.dumps(request)))
    pump(.8)
    check('SCAN reference emitted', len(references) > 0)
    check('SCAN internal body height offset cancelled', abs(references[-1].poses[0].pose.position.z + .4) < 1e-9)
    check('no SCAN trajectory means no translation', all(abs(v[0])+abs(v[1]) < 1e-8 for v in cmds[-5:]))
    unmatched=Bspline(order=3,traj_id=100,knots=[0.,0.,0.,0.,5.,5.,5.,5.])
    unmatched.pos_pts=[Point(x=x,y=0.,z=0.) for x in [0.,1/3,2/3,1.]]
    spline_pub.publish(unmatched)
    pump(.4)
    check('B-spline without committed metadata cannot drive',state.get('replans')==0 and all(sum(abs(x) for x in v)<1e-8 for v in cmds[-5:]))
    oldmeta=dict(schema=1,reference_stamp=[1,0],body_goal=[1.,0.,0.],trajectory=payload(unmatched))
    metadata_pub.publish(String(data=json.dumps(oldmeta)))
    pump(.3)
    check('old reference cannot be relabelled by fresh B-spline time',state.get('replans')==0 and state.get('trajectory_association_rejected',0)>0 and all(sum(abs(x) for x in v)<1e-8 for v in cmds[-5:]))
    trajectory()
    pump(3.0)
    check('valid SCAN trajectory drives feedback motion', any(v[0] > .05 for v in cmds[-8:]))
    request_pub.publish(String(data=json.dumps(request)))
    pump(.3)
    check('idempotent request preserves trajectory', state.get('replans') == 1)
    parked = Bspline(order=3, traj_id=2, knots=[0.,0.,0.,0.,5.,5.,5.,5.])
    parked.pos_pts = [Point(x=0.,y=0.,z=0.) for _ in range(4)]
    publish_trajectory(parked)
    pump(.4)
    pump(.8,imu_fresh=False)
    check('repeated raw IMU stamps cannot bypass freshness', state.get('tilt_source')=='raw_imu_stale'
          and state.get('raw_imu_last_rejected')=='repeated or out-of-order IMU stamp'
          and all(sum(abs(x) for x in v)<1e-8 for v in cmds[-5:]))
    pump(.4)
    check('SCAN emergency parking spline clears previous motion',
          state.get('degenerate_splines') == 1 and state.get('last_spline_rejected')
          and all(sum(abs(x) for x in v)<1e-8 for v in cmds[-5:]))
    trajectory()
    pump(.8)
    check('valid SCAN route recovers after parking rejection', any(v[0] > .05 for v in cmds[-8:]))
    pump(.6, obstacle=True)
    check('registered obstacle causes stop', state.get('obstacle_hold') and all(sum(abs(x) for x in v) < 1e-8 for v in cmds[-5:]))
    pump(1.3)
    check('cleared obstacle requests replan', state.get('obstacle_resumes') == 1 and state.get('reference_requests',0) >= 2)
    trajectory()
    pump(3.0)
    check('new trajectory resumes translation', any(v[0] > .05 for v in cmds[-8:]))
    pump(.4, imu_roll=.35)
    check('raw IMU stops while SLAM remains level', state.get('tilt_hold') and state.get('tilt_source')=='raw_imu'
          and state.get('raw_imu_max_tilt_rad',0) >= .35-1e-9
          and all(sum(abs(x) for x in v)<1e-8 for v in cmds[-5:]))
    pump(.3)
    check('raw IMU hold cannot resume before stable window', state.get('tilt_hold'))
    pump(1.0)
    check('both stable attitudes release raw IMU hold', not state.get('tilt_hold'))
    trajectory()
    pump(3.0)
    pump(.8, imu=False)
    check('missing raw IMU stops despite fresh SLAM and cloud', state.get('tilt_source')=='raw_imu_stale'
          and all(sum(abs(x) for x in v)<1e-8 for v in cmds[-5:]))
    pump(.4)
    pump(1.1, odom=False)
    check('odom loss stops without waiting for failure', all(sum(abs(x) for x in v) < 1e-8 for v in cmds[-5:]))
    pump(.4)
    trajectory()
    pump(1.2, fresh_stamp=False)
    check('repeated old stamps do not bypass watchdog', all(sum(abs(x) for x in v) < 1e-8 for v in cmds[-5:]))
    pump(.4)
    trajectory()
    pump(.5,roll=.35)
    check('SLAM tilt protection stops motion', state.get('tilt_hold') and all(sum(abs(x) for x in v)<1e-8 for v in cmds[-5:]))
    pump(1.2)
    check('stable body pose releases tilt hold and replans', not state.get('tilt_hold') and state.get('tilt_stops')==2)
    stop_pub.publish(Bool(data=True))
    pump(.4)
    check('stop cancels and publishes zero', state.get('state') == 'stopped' and all(sum(abs(x) for x in v) < 1e-8 for v in cmds[-5:]))
    request['request_id']='contract-tilt-failure'
    request_pub.publish(String(data=json.dumps(request)))
    pump(.5,roll=.55)
    check('severe SLAM tilt fails task without translating', state.get('state')=='failed' and all(sum(abs(x) for x in v)<1e-8 for v in cmds[-5:]))
    pump(.4)
    request['request_id']='contract-raw-imu-failure'
    request_pub.publish(String(data=json.dumps(request)))
    pump(.5,imu_roll=.55)
    check('severe raw IMU fails task while SLAM level', state.get('state')=='failed' and state.get('tilt_source')=='raw_imu'
          and all(sum(abs(x) for x in v)<1e-8 for v in cmds[-5:]))
    pump(.4)
    request['request_id']='contract-execution-bridge-failure'
    request_pub.publish(String(data=json.dumps(request)))
    pump(.3)
    bridge_safety_pub.publish(String(data=json.dumps({'state':'failed','reason':'independent raw IMU peak'})))
    pump(.4)
    check('execution bridge fail latch propagates to task with level SLAM and IMU',
          state.get('state')=='failed' and state.get('tilt_source')=='execution_bridge'
          and all(sum(abs(x) for x in v)<1e-8 for v in cmds[-5:]))
    result = {'scope':'synthetic ROS component contract; not end-to-end SLAM', 'passed':True, 'checks':results}
    output = FilePath(__file__).with_name('ros_contract_result.json')
    output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result, indent=2))
finally:
    stop_pub.publish(Bool(data=True))
    rclpy.spin_once(node, timeout_sec=.1)
    node.destroy_node()
    rclpy.shutdown()
