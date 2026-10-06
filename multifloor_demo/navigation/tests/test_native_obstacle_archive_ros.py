#!/usr/bin/env python3
"""Actual ROS75 controller + production safety bridge; synthetic sensors only."""
import hashlib
import json
import os
from pathlib import Path
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32,read_points_numpy
from std_msgs.msg import Header, String, Bool

NAV=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(NAV));sys.path.insert(0,str(NAV.parent/'simulation'))
from controller import Navigation
from control_bridge import Bridge
from control_core import steering_obstacle_ahead
from trajectory_contract import payload
from scan_planner_msgs.msg import Bspline

assert os.environ.get('ROS_DOMAIN_ID')=='75','isolated synthetic ROS domain75 only'
OUT=NAV/'test_results/native_obstacle_ros75'
OUT.mkdir(parents=True,exist_ok=True)
os.environ['DEMO_RUN_DIR']=str(OUT)
rclpy.init();nav=Navigation();bridge=Bridge();probe=rclpy.create_node('native_obstacle_archive_probe')
executor=SingleThreadedExecutor()
for n in (nav,bridge,probe):executor.add_node(n)
references=[];safe=[];filtered=[]
probe.create_subscription(RosPath,'/demo/navigation/scan_reference',references.append,1)
probe.create_subscription(Twist,'/demo/control/safe_cmd_vel',lambda m:safe.append((time.monotonic(),[m.linear.x,m.linear.y,m.angular.z])),10)
probe.create_subscription(PointCloud2,'/demo/navigation/cloud',lambda m:filtered.append((m.header.stamp.sec*1000000000+m.header.stamp.nanosec,np.asarray(read_points_numpy(m,field_names=('x','y','z'))).reshape(-1,3).copy())),qos_profile_sensor_data)
clock=probe.create_publisher(Clock,'/clock',10);odom=probe.create_publisher(Odometry,'/demo/slam/body_odom',10)
imu=probe.create_publisher(Imu,'/livox/imu',10);cloud=probe.create_publisher(PointCloud2,'/cloud_registered_full',10)
request=probe.create_publisher(String,'/demo/navigation/request',10)
stop_request=probe.create_publisher(Bool,'/demo/navigation/stop',10)
bspline=probe.create_publisher(Bspline,'/demo/navigation/bspline',10)
meta=probe.create_publisher(String,'/demo/navigation/trajectory_metadata',10)
sim=10.;trajectory_id=0;checks={};release=threading.Event();writer_entered=threading.Event();commit=[];queued=[]
write_original=nav.event_archive._write;enqueue_original=nav.event_archive.enqueue

def delayed_write(*args):
    writer_entered.set()
    if not release.wait(5.):raise TimeoutError('test did not release writer')
    result=write_original(*args);commit.append(time.monotonic());return result

def observe_enqueue(arrays,metadata):
    queued.append(dict(command=list(nav.command),time=time.monotonic(),cloud=arrays['cloud'],metadata=metadata))
    return enqueue_original(arrays,metadata)

nav.event_archive._write=delayed_write;nav.event_archive.enqueue=observe_enqueue

def pump(seconds,blocked=False,position=(.12,.04,.02)):
    global sim
    stop=time.monotonic()+seconds
    while time.monotonic()<stop:
        sim+=.03;tick=Clock();tick.clock.sec=int(sim);tick.clock.nanosec=int((sim-int(sim))*1e9)
        clock.publish(tick);header=Header(stamp=tick.clock,frame_id='camera_init')
        m=Odometry(header=header,child_frame_id='base');m.pose.pose.orientation.w=1.
        m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=position;odom.publish(m)
        raw=Imu(header=Header(stamp=tick.clock,frame_id='imu_link'));raw.orientation.w=1.;imu.publish(raw)
        points=[[x,y,-.28] for x in [.55,.65,.75,.85] for y in [-.16,.04,.24]]
        points += [[.20,.04,.02],[.46,.04,-.13]] # Actual self-volume filtering.
        if blocked:points += [[.77,y,z] for y in [-.06,.04,.14] for z in [.02,.12,.22]]
        cloud.publish(create_cloud_xyz32(header,points))
        for _ in range(12):executor.spin_once(timeout_sec=.002)
        time.sleep(.007)

def trajectory():
    global trajectory_id
    trajectory_id+=1
    msg=Bspline(order=3,traj_id=trajectory_id,knots=[0.,0.,0.,0.,5.,5.,5.,5.])
    msg.pos_pts=[Point(x=x,y=.04,z=.02) for x in [.12,.68,1.24,1.8]]
    ref=references[-1];rs=[ref.header.stamp.sec,ref.header.stamp.nanosec]
    bspline.publish(msg);meta.publish(String(data=json.dumps(dict(schema=1,reference_stamp=rs,body_goal=[1.8,.04,.02],trajectory=payload(msg)))))

def check(label,value):
    checks[label]=bool(value)
    if not value:raise AssertionError(label)

try:
    pump(1.)
    request.publish(String(data=json.dumps(dict(request_id='native-obstacle-ros75',frame_id='camera_init',waypoints=[[1.8,.04,.02]]))))
    pump(.5);check('actual reference transport',bool(references));trajectory();pump(2.6)
    check('production bridge safe movement before obstacle',any(v[1][0]>.05 for v in safe[-20:]))
    before=len(safe);pump(.5,blocked=True)
    check('native guard actually holds navigation',nav.obstacle_hold and nav.state=='running')
    check('zero publication precedes native queue',len(queued)==1 and queued[0]['command']==[0.,0.,0.])
    check('production bridge delivered safe zero',any(t>=queued[0]['time'] and v==[0.,0.,0.] for t,v in safe[before:]))
    check('writer has not committed before safe zero',writer_entered.is_set() and not commit)
    # Replace the newest callback input before allowing the old event to serialize.
    pump(.15,blocked=False,position=(.22,.04,.02))
    check('later sensor callback cannot replace captured cloud',nav.cloud is not queued[0]['cloud'] and nav.cloud_input_context['message_stamp_ns']!=queued[0]['metadata']['cloud_input']['message_stamp_ns'])
    release.set();pump(.3,blocked=True)
    check('native event committed',nav.event_archive.completed==1)
    document=json.loads(Path(nav.event_archive.status()['path']).read_text())
    evidence=np.load(Path(nav.event_archive.status()['path']).with_name(document['cloud_file']))
    check('captured actual cloud is byte-exact',np.array_equal(evidence['cloud'],queued[0]['cloud']))
    matching=[v for stamp,v in filtered if stamp==document['cloud_input']['message_stamp_ns']]
    check('actual filtered ROS cloud equals native guard cloud',any(np.array_equal(v,evidence['cloud']) for v in matching))
    check('matched body and cloud stamps preserved',document['cloud_input']['message_stamp_ns']==document['cloud_input']['filtering_body_stamp_ns'] and document['odom_stamp_ns']==document['cloud_input']['filtering_body_stamp_ns'])
    check('raw guard body pose preserved',np.allclose(evidence['pose'],[.12,.04,.02]) and np.allclose(document['cloud_input']['filtering_body_pose'],[.12,.04,.02]))
    check('native guard result independently reproduces',list(steering_obstacle_ahead(evidence['cloud'],evidence['pose'],evidence['checked_target'],evidence['steering_direction'],evidence['route']))==document['guard_result'])
    check('archive contains actual self-filter and guard counts',document['cloud_input']['self_filtered_points']==5 and document['guard_result'][2]==9)
    check('actual safe zero received before commit',any(queued[0]['time']<=t<commit[0] and v==[0.,0.,0.] for t,v in safe))
    pump(1.3);trajectory();pump(.6)
    def broken(*args):raise PermissionError('deliberate archive failure')
    nav.event_archive._write=broken
    pump(.7,blocked=True)
    check('archive failure retains protection and running task',nav.state=='running' and nav.obstacle_hold and nav.command==[0.,0.,0.] and all(v==[0.,0.,0.] for _,v in safe[-5:]))
    check('archive failure exposed as diagnostic', 'PermissionError' in nav.event_archive.status().get('error',''))
    event_count=len(list((OUT/'navigation_events').glob('*.json')))
    stop_request.publish(Bool(data=True));pump(.25)
    request.publish(String(data=json.dumps(dict(request_id='native-obstacle-second-request',frame_id='camera_init',waypoints=[[1.8,.04,.02]]))))
    pump(.35)
    check('new request clears per-request event context',nav.request_id=='native-obstacle-second-request' and nav.pending_obstacle_event is None and nav.obstacle_guard_context is None and nav.last_obstacle_event_id is None and nav.obstacle_event_capture_error is None)
    check('old archive diagnosis cannot appear as new-request result','error' not in nav.event_archive.status(request_id=nav.request_id))
    check('new request preserves existing event files',len(list((OUT/'navigation_events').glob('*.json')))==event_count)
    result=dict(passed=all(checks.values()),scope='actual ROS75 native controller and production bridge with synthetic sensors; no Gazebo/GT/SLAM acceptance',checks=checks,first_event=document,source_sha256={f:hashlib.sha256((NAV/f).read_bytes()).hexdigest() for f in ['controller.py','event_archive.py','control_core.py','trajectory_contract.py']})
    (OUT/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'passed':result['passed'],'checks':checks},indent=2))
except Exception as exc:
    (OUT/'result_failed.json').write_text(json.dumps(dict(passed=False,checks=checks,error=f'{type(exc).__name__}: {exc}'),indent=2)+'\n')
    raise
finally:
    release.set();nav.publish_command();nav.event_archive.close()
    for n in (nav,bridge,probe):executor.remove_node(n);n.destroy_node()
    executor.shutdown();rclpy.try_shutdown()
