#!/usr/bin/env python3
"""Actual isolated SCAN node, synthetic measured interface, run11 handoff case.

No simulator or actuator is involved. This verifies state initialization and
published SCAN geometry/metadata; it is not physical demo acceptance.
"""
import json
import os
import time
from pathlib import Path as FilePath
import numpy as np
from scipy.interpolate import BSpline
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Bool, String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from scan_planner_msgs.msg import Bspline

assert os.environ.get('ROS_DOMAIN_ID') == '75'
rclpy.init()
node=Node('scan_actual_handoff_fixture')
pubs={topic:node.create_publisher(kind,topic,10) for topic,kind in [
    ('/clock',Clock),('/demo/navigation/scan_body_odom',Odometry),
    ('/demo/slam/lidar_odom',Odometry),('/demo/navigation/cloud',PointCloud2),
    ('/demo/navigation/scan_reference',Path),('/demo/navigation/execution_frozen',Bool)]}
splines,metadata={},[]
node.create_subscription(Bspline,'/demo/navigation/bspline',lambda m:splines.update({
    (m.traj_id,m.start_time.sec,m.start_time.nanosec):m}),10)
node.create_subscription(String,'/demo/navigation/trajectory_metadata',lambda m:metadata.append(json.loads(m.data)),10)
sim=100.
points=[[x*.1,y*.1,-.30] for x in range(-25,46) for y in range(-25,36)]
points += [[x*.1,y,z*.1] for x in range(-25,46) for y in [-2.4,3.4] for z in range(-3,12)]


def pump(duration,pose,velocity,frozen=True,reference=None):
    global sim
    until=time.monotonic()+duration
    sent=None
    while time.monotonic()<until:
        sim+=.04
        clock=Clock()
        clock.clock.sec=int(sim);clock.clock.nanosec=int((sim-int(sim))*1e9)
        pubs['/clock'].publish(clock)
        header=Header(frame_id='camera_init',stamp=clock.clock)
        pubs['/demo/navigation/execution_frozen'].publish(Bool(data=frozen))
        odom=Odometry(header=header,child_frame_id='demo_scan_body_world_twist')
        odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z=map(float,pose)
        odom.pose.pose.orientation.w=1.
        odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z=map(float,velocity)
        pubs['/demo/navigation/scan_body_odom'].publish(odom)
        lidar=Odometry(header=header,child_frame_id='velodyne')
        lidar.pose.pose.position.x=pose[0]+.2;lidar.pose.pose.position.y=pose[1]
        lidar.pose.pose.position.z=pose[2]+.1177;lidar.pose.pose.orientation.w=1.
        pubs['/demo/slam/lidar_odom'].publish(lidar)
        pubs['/demo/navigation/cloud'].publish(create_cloud_xyz32(header,points))
        if reference is not None and sent is None and pubs['/demo/navigation/scan_reference'].get_subscription_count():
            route=Path(header=header)
            waypoint=PoseStamped(header=header)
            waypoint.pose.position.x,waypoint.pose.position.y=map(float,reference[:2])
            waypoint.pose.position.z=float(reference[2]-.4)
            waypoint.pose.orientation.w=1.
            route.poses=[waypoint]
            pubs['/demo/navigation/scan_reference'].publish(route)
            sent=[clock.clock.sec,clock.clock.nanosec]
        rclpy.spin_once(node,timeout_sec=.01)
        time.sleep(.02)
    return sent


checks=[]
def check(name,passed,**details):
    checks.append(dict(name=name,passed=bool(passed),**details))
    if not passed:raise AssertionError(checks[-1])


try:
    discovery=time.monotonic()+10.
    while time.monotonic()<discovery:
        pump(.5,[.9442,-.1549,-.00399],[0.,0.,0.])
        if all(pubs[t].get_subscription_count() for t in ['/demo/navigation/scan_body_odom','/demo/navigation/scan_reference']):break
    check('actual planner subscriptions discovered',all(pubs[t].get_subscription_count() for t in ['/demo/navigation/scan_body_odom','/demo/navigation/scan_reference']))
    pump(1.,[.9442,-.1549,-.00399],[0.,0.,0.])
    previous=pump(3.,[.9442,-.1549,-.00399],[0.,0.,0.],reference=[0.,0.,0.])
    check('old origin reference genuinely generated a SCAN payload',any(m['reference_stamp']==previous for m in metadata))
    actual=[-.01874,-.007816,-.011466]
    pose=[.304,-.152,-.00305]
    goal=[-.0534,2.,-.00136]
    stamp=pump(3.,pose,actual,reference=goal)
    selected=[m for m in metadata if m['reference_stamp']==stamp]
    check('fresh north reference committed and generated actual SCAN payload',bool(selected),count=len(selected))
    geometries=[]
    for meta in selected:
        tr=meta['trajectory'];identity=(tr['traj_id'],*tr['start_time'])
        check('metadata pairs exact emitted B-spline',identity in splines)
        msg=splines[identity]
        check('full control points and knots pair',tr['pos_pts']==[[p.x,p.y,p.z] for p in msg.pos_pts] and tr['knots']==list(msg.knots))
        state=meta['start_state']
        check('raw measured velocity retained in diagnostics',np.allclose(state['measured_velocity'],actual))
        check('freeze explicitly uses stopped reference boundary, not measured zero',state['frozen_reference_boundary'] and np.linalg.norm(state['used_velocity'])<1e-12 and np.linalg.norm(state['used_acceleration'])<1e-12)
        check('raw measured start position and requested body goal retained',np.allclose(state['position'],pose) and np.allclose(meta['body_goal'],goal))
        curve=BSpline(msg.knots,[[p.x,p.y,p.z] for p in msg.pos_pts],msg.order)
        ts=np.linspace(msg.knots[msg.order],msg.knots[-msg.order-1],400)
        samples=curve(ts)
        peak=float(np.linalg.norm(curve.derivative()(ts),axis=1).max())
        peakacc=float(np.linalg.norm(curve.derivative(2)(ts),axis=1).max())
        maximum_x=float(samples[:,0].max())
        end=samples[-1].tolist()
        geometries.append(dict(peak_speed=peak,peak_acceleration=peakacc,maximum_x=maximum_x,end=end))
        check('new SCAN route progresses north without run11 east excursion',end[1]>1.5 and maximum_x<.45,geometry=geometries[-1])
        check('actual B-spline respects mechanical translational bounds',peak<=.132 and peakacc<=.165,peak_speed=peak,peak_acceleration=peakacc)
    # The filter advances on measured stamps, not wall time after publication.
    # Let actual moving measurements converge before requesting the next plan.
    pump(1.,pose,[-.021,-.004,0.],frozen=False)
    stamp_moving=pump(2.,pose,[-.021,-.004,0.],frozen=False,reference=goal)
    moving=[m for m in metadata if m['reference_stamp']==stamp_moving]
    check('unfrozen measured-velocity reference generates a SCAN payload',bool(moving))
    used=np.array(moving[-1]['start_state']['used_velocity'])
    check('unfrozen plan uses actual filtered bounded velocity',not moving[-1]['start_state']['frozen_reference_boundary'] and 0.<np.linalg.norm(used)<=.12 and np.linalg.norm(used-[-.021,-.004,0.])<.003,used_velocity=used.tolist())
    result=dict(scope='actual isolated SCAN planner with synthetic measured-state handoff; not Gazebo/SLAM acceptance',passed=True,checks=checks,geometries=geometries)
finally:
    if 'result' not in globals():result=dict(scope='isolated actual SCAN node',passed=False,checks=checks,metadata_tail=metadata[-3:])
    output=FilePath(__file__).resolve().parent.parent/'test_results/planner_handoff.json'
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    node.destroy_node();rclpy.shutdown()
