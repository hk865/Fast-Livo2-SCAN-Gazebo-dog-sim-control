#!/usr/bin/env python3
"""Synthetic ROS-only exporter fixture. Starts no world, actuator or simulation."""
import hashlib
import json
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String


def main():
    root=Path(__file__).resolve().parent
    executable=root/'install/scan_planner/lib/scan_planner/scan_planner_node'
    log=(root/'snapshot_transport_node.log').open('w')
    proc=subprocess.Popen([str(executable),'--ros-args','--params-file',
        str(root/'install/scan_planner/share/scan_planner/config/planner.yaml'),
        '-p','grid_map.frame_id:=camera_init','-p','grid_map.resolution:=0.08'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    rclpy.init();node=rclpy.create_node('corridor_exporter_synthetic_test')
    snapshots=[]
    subscription=node.create_subscription(String,'/demo/teacher/corridor/snapshot',lambda m:snapshots.append(json.loads(m.data)),1)
    request_pub=node.create_publisher(RosPath,'/demo/teacher/corridor/request',1)
    pose_pub=node.create_publisher(Odometry,'sensor_pose',10)
    cloud_pub=node.create_publisher(PointCloud2,'cloud',10)
    start=time.monotonic();request_sent=start;request_stamp=None;sent_stamps=set()
    try:
        while time.monotonic()-start<8 and not any(x.get('complete')for x in snapshots):
            if proc.poll()is not None:raise RuntimeError('exporter_process_exited')
            stamp=node.get_clock().now().to_msg();ns=stamp.sec*1_000_000_000+stamp.nanosec;sent_stamps.add(ns)
            pose=Odometry();pose.header.frame_id='camera_init';pose.header.stamp=stamp
            pose.pose.pose.position.z=.7;pose.pose.pose.orientation.w=1.;pose_pub.publish(pose)
            request=RosPath();request.header.frame_id='camera_init'
            if request_stamp is None or time.monotonic()-request_sent>2:
                request_stamp=stamp;request_sent=time.monotonic()
            request.header.stamp=request_stamp
            for x in (0.,.6):
                p=PoseStamped();p.pose.position.x=x;p.pose.position.z=.7;p.pose.orientation.w=1.;request.poses.append(p)
            request_pub.publish(request)
            points=[(ix*.08,iy*.08,.3)for ix in range(-9,16)for iy in range(-9,10)]
            points += [(1.2,iy*.08,.4+iz*.08)for iy in range(-8,9)for iz in range(8)]
            cloud=PointCloud2();cloud.header.frame_id='camera_init';cloud.header.stamp=stamp
            cloud.height=1;cloud.width=len(points);cloud.is_dense=True;cloud.point_step=12;cloud.row_step=len(points)*12
            cloud.fields=[PointField(name=name,offset=i*4,datatype=PointField.FLOAT32,count=1)for i,name in enumerate(('x','y','z'))]
            cloud.data=b''.join(struct.pack('<fff',*p)for p in points);cloud_pub.publish(cloud)
            rclpy.spin_once(node,timeout_sec=.08)
        complete=[x for x in snapshots if x.get('complete')]
        if not complete:raise RuntimeError('no_complete_snapshot_received')
        snapshot=complete[-1]
        assert snapshot['schema']=='teacher_scan_local_snapshot/v1'
        assert snapshot['source']=='actual_registered_lidar_raycast'
        assert snapshot['source_cloud_stamp_ns']in sent_stamps
        assert snapshot['source_sensor_pose_stamp_ns']in sent_stamps
        assert snapshot['surface_points_complete']
        assert 0<len(snapshot['surface_points_xyz'])<=12000
        assert all(c in snapshot['states']for c in '012'), {c:snapshot['states'].count(c)for c in '012'}
        assert len(snapshot['cell_observation_age_ms'])==len(snapshot['states'])<=65536
        assert all(age>=0 for state,age in zip(snapshot['states'],snapshot['cell_observation_age_ms'])if state=='1')
        assert snapshot['navigation_ground_truth_used']is False
        counts={c:snapshot['states'].count(c)for c in '012'}
        report=dict(schema='teacher_corridor_exporter_transport_test/v1',passed=True,
            fixture='synthetic ROS messages only; no Gazebo, Teacher or physical actuators',
            executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
            revision=snapshot['revision'],voxel_counts=counts,measured_surface_points=len(snapshot['surface_points_xyz']),
            source_stamp_matched=True,finite_messages=len(snapshots),navigation_acceptance='unverified')
        (root/'SNAPSHOT_TRANSPORT_TEST.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report))
    finally:
        node.destroy_node();rclpy.shutdown()
        if proc.poll()is None:
            os.killpg(proc.pid,signal.SIGINT)
            try:proc.wait(timeout=4)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=4)
        log.close()


if __name__=='__main__':main()
