#!/usr/bin/env python3
"""Read-only synchronized command/SLAM/independent physics diagnostic recorder.

Ground truth is only written to the diagnostic file. This program has no ROS
publisher and cannot influence navigation, sensors, robot or obstacle behavior.
"""
import argparse,json,math,time
from collections import deque
from pathlib import Path as FilePath
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry,Path
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from visualization_msgs.msg import Marker
from control_core import rotation_xyzw,follow_trajectory

parser=argparse.ArgumentParser()
parser.add_argument('--output',required=True)
parser.add_argument('--duration',type=float,default=90.)
parser.add_argument('--cloud-interval',type=float,default=3.,help='simulated seconds between cloud archives; 0 disables')
parser.add_argument('--cloud-radius',type=float,default=3.,help='XY radius around measured body for sparse cloud archive')
parser.add_argument('--cloud-voxel',type=float,default=.08,help='archive-only voxel sampling; never alters navigation input')
args=parser.parse_args()
output=FilePath(args.output);output.parent.mkdir(parents=True,exist_ok=True)
rclpy.init();node=Node('demo_readonly_feedback_recorder')
latest={};previous={};path=None;trajectory_stream=None
clouds={};pose_history=deque(maxlen=100);lidar_pose_history=deque(maxlen=100);last_cloud_snapshot=None
cloud_archive=output.with_name(output.stem+'_clouds')
if args.cloud_interval>0:cloud_archive.mkdir(parents=True,exist_ok=True)

def twist_cb(key,msg):
    latest[key]=[msg.linear.x,msg.linear.y,msg.angular.z]

def odom_cb(key,msg):
    p=msg.pose.pose.position;q=msg.pose.pose.orientation
    xyz=np.array([p.x,p.y,p.z]);R=rotation_xyzw([q.x,q.y,q.z,q.w])
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
    record={'stamp':stamp,'pose':xyz.tolist(),'yaw':math.atan2(R[1,0],R[0,0]),
            'quaternion':[q.x,q.y,q.z,q.w],
            'twist_body':[msg.twist.twist.linear.x,msg.twist.twist.linear.y,msg.twist.twist.linear.z],
            'angular_body':[msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z]}
    if key in previous:
        t,old=previous[key]
        if stamp>t:record['finite_difference_world']=((xyz-old)/(stamp-t)).tolist()
    previous[key]=(stamp,xyz)
    if key=='slam':
        pose_history.append(record)
        record['twist_world']=(R@record['twist_body']).tolist()
        if path is not None and len(path)>1:
            _,_,target=follow_trajectory(xyz,R,path,path[-1])
            direction=target[:2]-xyz[:2]
            desired=math.atan2(direction[1],direction[0])
            record['lookahead']=target.tolist()
            record['heading_error']=math.atan2(math.sin(desired-record['yaw']),math.cos(desired-record['yaw']))
    if key=='lidar':
        lidar_pose_history.append(record)
    latest[key]=record

def path_cb(msg):
    global path
    path=np.array([[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in msg.poses])
    if trajectory_stream is not None:
        trajectory_stream.write(json.dumps({'kind':'accepted_path','receipt_sim_time':latest.get('sim_time'),
            'header_stamp':[msg.header.stamp.sec,msg.header.stamp.nanosec],
            'points':path.tolist()},ensure_ascii=False)+'\n')
        trajectory_stream.flush()

def metadata_cb(msg):
    # Once per newly published trajectory, preserve its complete numeric payload
    # for exact future replay. This is the same owned, read-only stack recorder.
    try:metadata=json.loads(msg.data)
    except (ValueError,TypeError):return
    if trajectory_stream is not None:
        trajectory_stream.write(json.dumps({'kind':'scan_metadata','receipt_sim_time':latest.get('sim_time'),
            'metadata':metadata},ensure_ascii=False)+'\n')
        trajectory_stream.flush()

def cloud_cb(key,msg):
    # Diagnostic sampling is independent of the controller and its publishers.
    # Keep current messages; the expensive clipping happens only once per archive.
    clouds[key]=msg

def bounds_cb(msg):
    if not msg.points:
        return
    points=np.array([[p.x,p.y,p.z] for p in msg.points])
    latest['scan_bounds']={'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec/1e9,
                           'frame_id':msg.header.frame_id,'min':points.min(axis=0).tolist(),
                           'max':points.max(axis=0).tolist()}

def archive_clouds(sim_time):
    if not pose_history or not clouds:
        return None
    body=pose_history[-1];center=np.asarray(body['pose']);arrays={};metadata={}
    for key,msg in clouds.items():
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        points=np.asarray(read_points_numpy(msg,field_names=('x','y','z'),skip_nans=True)).reshape(-1,3)
        total=len(points)
        points=points[(np.linalg.norm(points[:,:2]-center[:2],axis=1)<=args.cloud_radius)
                      &(points[:,2]>center[2]-.7)&(points[:,2]<center[2]+.8)]
        if len(points):
            _,indices=np.unique(np.floor(points/args.cloud_voxel).astype(np.int32),axis=0,return_index=True)
            points=points[indices]
        arrays[key]=points.astype(np.float32)
        matched=min(pose_history,key=lambda p:abs(p['stamp']-stamp))
        metadata[key]={'stamp':stamp,'frame_id':msg.header.frame_id,'source_points':total,
                       'archived_points':len(points),'nearest_slam_body':matched,
                       'body_stamp_difference_s':abs(matched['stamp']-stamp)}
        if lidar_pose_history:
            lidar=min(lidar_pose_history,key=lambda p:abs(p['stamp']-stamp))
            metadata[key]['nearest_slam_lidar']=lidar
            metadata[key]['lidar_stamp_difference_s']=abs(lidar['stamp']-stamp)
    basename=f'{sim_time:012.3f}'
    cloud_tmp=cloud_archive/(basename+'.tmp.npz')
    json_tmp=cloud_archive/(basename+'.tmp.json')
    np.savez_compressed(cloud_tmp,**arrays)
    json_tmp.write_text(json.dumps({
        'scope':'read-only sparse snapshots of actual ROS clouds; no publishers',
        'sim_time':sim_time,'body_for_archive_crop':body,'cloud_radius_m':args.cloud_radius,
        'z_limits_relative_body_m':[-.7,.8],'voxel_size_m':args.cloud_voxel,
        'navigation_status':latest.get('navigation'),'scan_bounds':latest.get('scan_bounds'),
        'topics':metadata},ensure_ascii=False)+'\n')
    cloud_tmp.replace(cloud_archive/(basename+'.npz'))
    json_tmp.replace(cloud_archive/(basename+'.json'))
    return str((cloud_archive/(basename+'.npz')).resolve())

node.create_subscription(Twist,'/demo/cmd_vel',lambda m:twist_cb('nav_command',m),10)
node.create_subscription(Twist,'/demo/control/safe_cmd_vel',lambda m:twist_cb('actuator_command',m),10)
node.create_subscription(Odometry,'/demo/slam/body_odom',lambda m:odom_cb('slam',m),qos_profile_sensor_data)
node.create_subscription(Odometry,'/demo/slam/lidar_odom',lambda m:odom_cb('lidar',m),qos_profile_sensor_data)
node.create_subscription(Odometry,'/demo/ground_truth',lambda m:odom_cb('ground_truth',m),qos_profile_sensor_data)
node.create_subscription(String,'/demo/navigation/status',lambda m:latest.update(navigation=json.loads(m.data)),10)
node.create_subscription(Path,'/demo/navigation/path',path_cb,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(String,'/demo/navigation/trajectory_metadata',metadata_cb,10)
if args.cloud_interval>0:
    for key,topic in {'full':'/cloud_registered_full','filtered':'/demo/navigation/cloud',
                      'occupied':'/demo/navigation/occupancy','inflated':'/grid_map/occupancy_inflate'}.items():
        node.create_subscription(PointCloud2,topic,lambda m,k=key:cloud_cb(k,m),qos_profile_sensor_data)
node.create_subscription(Clock,'/clock',lambda m:latest.update(sim_time=m.clock.sec+m.clock.nanosec/1e9),10)
node.create_subscription(Marker,'/grid_map/sliding_map_bbox',bounds_cb,10)
start=time.monotonic();last_write=0.;rows=0
try:
    with output.open('w') as stream, output.with_name(output.stem+'_trajectories.jsonl').open('w') as trajectory_stream:
        while rclpy.ok() and time.monotonic()-start<args.duration:
            rclpy.spin_once(node,timeout_sec=.025)
            now=time.monotonic()
            if now-last_write>=.1:
                sim_time=latest.get('sim_time')
                if (args.cloud_interval>0 and sim_time is not None
                        and (last_cloud_snapshot is None or sim_time-last_cloud_snapshot>=args.cloud_interval)):
                    latest['cloud_snapshot']=archive_clouds(sim_time)
                    last_cloud_snapshot=sim_time
                stream.write(json.dumps({'wall_elapsed':now-start,**latest},ensure_ascii=False)+'\n')
                stream.flush();rows+=1;last_write=now
except (KeyboardInterrupt, ExternalShutdownException):
    pass
finally:
    node.destroy_node();rclpy.try_shutdown()
print(json.dumps({'file':str(output.resolve()),'rows':rows,'scope':'read-only commands/SLAM/physics diagnostics'}))
