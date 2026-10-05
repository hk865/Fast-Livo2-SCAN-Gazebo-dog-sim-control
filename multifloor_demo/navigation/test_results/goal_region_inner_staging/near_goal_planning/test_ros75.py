#!/usr/bin/env python3
"""Actual isolated SCAN, short-center positive/point-cloud blocking negative.

All sensor messages are synthetic; no simulator, actuator, GT, or physical PASS.
The real planner/map/optimizer and its actual emitted Bspline are exercised.
"""
import argparse, hashlib, json, os, signal, subprocess, time
from pathlib import Path
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from rosgraph_msgs.msg import Clock
from scan_planner_msgs.msg import Bspline
from scipy.interpolate import BSpline
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32, read_points_numpy
from std_msgs.msg import Bool, Header, String

HERE=Path(__file__).parent;NAV=HERE.parents[2]
assert os.environ.get('ROS_DOMAIN_ID')=='75'
parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--baseline',action='store_true');args=parser.parse_args()
out=HERE/args.output;out.mkdir(exist_ok=False)
(out/'test_ros75.py').write_bytes(Path(__file__).read_bytes())
expected=NAV/'ros2_ws/install/scan_planner'
prefix=Path(get_package_prefix('scan_planner')).resolve()
assert prefix==expected.resolve(),(prefix,expected)
binary=prefix/'lib/scan_planner/scan_planner_node'
checks=[];cases=[]
def check(name,condition,**details):
 checks.append(dict(name=name,passed=bool(condition),**details))
 if not condition:raise AssertionError(checks[-1])

def run_case(blocked):
 name='blocked' if blocked else 'open'
 log=(out/(name+'.log')).open('w')
 env=dict(os.environ,ROS_LOG_DIR=str(out/(name+'_roslog')))
 proc=subprocess.Popen(['bash',str(NAV/'run.sh'),'planner'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
 rclpy.init();node=Node('near_goal_actual_scan_'+name)
 pubs={topic:node.create_publisher(kind,topic,10) for topic,kind in [('/clock',Clock),('/demo/navigation/scan_body_odom',Odometry),('/demo/slam/lidar_odom',Odometry),('/demo/navigation/cloud',PointCloud2),('/demo/navigation/scan_reference',RosPath),('/demo/navigation/execution_frozen',Bool)]}
 metadata=[];splines={};occupied=[];inflated=[];commands=[];sim_ns=200000000000;parameters={}
 node.create_subscription(String,'/demo/navigation/trajectory_metadata',lambda m:metadata.append(json.loads(m.data)),10)
 node.create_subscription(Bspline,'/demo/navigation/bspline',lambda m:splines.update({(m.traj_id,m.start_time.sec,m.start_time.nanosec):m}),10)
 node.create_subscription(PointCloud2,'/demo/navigation/occupancy',lambda m:occupied.append(read_points_numpy(m,field_names=('x','y','z'),skip_nans=True)),qos_profile_sensor_data)
 node.create_subscription(PointCloud2,'/grid_map/occupancy_inflate',lambda m:inflated.append(read_points_numpy(m,field_names=('x','y','z'),skip_nans=True)),qos_profile_sensor_data)
 node.create_subscription(Twist,'/demo/cmd_vel',lambda m:commands.append([m.linear.x,m.linear.y,m.angular.z]),10)
 floor=[[x*.1,y*.1,-.30] for x in range(-25,46) for y in range(-25,36)]
 wall=[[.08,y*.04,z*.04] for y in range(-15,16) for z in range(-3,16)] if blocked else []
 points=floor+wall;reference=None
 def pump(duration,send=False):
  nonlocal sim_ns,reference
  until=time.monotonic()+duration
  while time.monotonic()<until:
   sim_ns+=40000000;clock=Clock();clock.clock.sec,clock.clock.nanosec=divmod(sim_ns,1000000000);pubs['/clock'].publish(clock)
   h=Header(frame_id='camera_init',stamp=clock.clock)
   pubs['/demo/navigation/execution_frozen'].publish(Bool(data=True))
   m=Odometry(header=h,child_frame_id='demo_scan_body_world_twist');m.pose.pose.position.x=.19;m.pose.pose.orientation.w=1.;pubs['/demo/navigation/scan_body_odom'].publish(m)
   lidar=Odometry(header=h,child_frame_id='velodyne');lidar.pose.pose.position.x=.39;lidar.pose.pose.position.z=.1177;lidar.pose.pose.orientation.w=1.;pubs['/demo/slam/lidar_odom'].publish(lidar)
   pubs['/demo/navigation/cloud'].publish(create_cloud_xyz32(h,points))
   if send and reference is None and pubs['/demo/navigation/scan_reference'].get_subscription_count():
    p=PoseStamped(header=h);p.pose.position.z=-.4;p.pose.orientation.w=1.;route=RosPath(header=h,poses=[p]);pubs['/demo/navigation/scan_reference'].publish(route);reference=[h.stamp.sec,h.stamp.nanosec]
   rclpy.spin_once(node,timeout_sec=.01);time.sleep(.02)
 try:
  until=time.monotonic()+12
  while time.monotonic()<until and not all(pubs[t].get_subscription_count() for t in ['/demo/navigation/scan_body_odom','/demo/navigation/scan_reference','/demo/navigation/cloud']):pump(.25)
  check(name+'_actual_subscribers',all(pubs[t].get_subscription_count() for t in ['/demo/navigation/scan_body_odom','/demo/navigation/scan_reference','/demo/navigation/cloud']))
  pump(1.5)
  client=node.create_client(GetParameters,'/scan_planner_node/get_parameters')
  check(name+'_actual_parameter_service',client.wait_for_service(timeout_sec=3.))
  names=['manager.max_vel','manager.max_acc','manager.feasibility_tolerance','optimization.vel_tolerance','optimization.acc_tolerance']
  future=client.call_async(GetParameters.Request(names=names));until=time.monotonic()+5.
  while not future.done() and time.monotonic()<until:pump(.1)
  check(name+'_actual_parameter_reply',future.done() and future.exception() is None)
  parameters={k:v.double_value for k,v in zip(names,future.result().values)}
  check(name+'_nominal_and_inherited_tolerances_unmodified',parameters==dict(zip(names,[.12,.15,.5,1.,1.])),parameters=parameters)
  check(name+'_actual_occupancy_received',bool(occupied) and len(occupied[-1])>100)
  if blocked:
   check('actual_wall_endpoint_cells_occupied',any(len(c) and np.any((abs(c[:,0]-.08)<.081)&(abs(c[:,1])<.3)&(abs(c[:,2])<.12)) for c in occupied))
  pump(3.5,send=True)
  selected=[m for m in metadata if m['reference_stamp']==reference]
  geometries=[]
  for meta in selected:
   tr=meta['trajectory'];key=(tr['traj_id'],*tr['start_time']);check(name+'_actual_full_payload_pairs',key in splines)
   msg=splines[key];check(name+'_emitted_payload_equal_metadata',tr['pos_pts']==[[p.x,p.y,p.z] for p in msg.pos_pts] and tr['knots']==list(msg.knots))
   c=BSpline(msg.knots,[[p.x,p.y,p.z] for p in msg.pos_pts],msg.order);ts=np.linspace(msg.knots[msg.order],msg.knots[-msg.order-1],500);p=c(ts)
   geometries.append(dict(start=p[0].tolist(),end=p[-1].tolist(),arc_length_m=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum()),peak_speed=float(np.linalg.norm(c.derivative()(ts),axis=1).max()),peak_acceleration=float(np.linalg.norm(c.derivative(2)(ts),axis=1).max()),velocity_controlpoint_max_abs=float(abs(c.derivative().c).max()),acceleration_controlpoint_max_abs=float(abs(c.derivative(2).c).max())))
  detail=dict(name=name,reference_stamp=reference,metadata=selected,geometries=geometries,actual_parameters=parameters,occupancy_count=len(occupied[-1]),inflated_count=len(inflated[-1]) if inflated else 0)
  (out/(name+'_observations.json')).write_text(json.dumps(detail,indent=2)+'\n')
  moving=[g for g in geometries if g['arc_length_m']>=.05]
  if blocked:check('point_cloud_blocking_short_goal_not_published_as_free_route',not moving)
  elif args.baseline:check('original20_gate_reproduces_no19_moving_route',not moving)
  else:
   check('new_short19_request_emits_checked_center_route',bool(selected))
   for meta,g in zip(selected,geometries):
    check('short_route_keeps_raw_start_and_original_center',np.allclose(meta['start_state']['position'],[.19,0,0]) and np.allclose(meta['body_goal'],[0,0,0]) and np.allclose(meta['adjusted_body_goal'],[0,0,0]))
    check('short_checked_path_reaches_inner17_with_actual_progress',np.linalg.norm(g['end'])<.05 and g['arc_length_m']>=.1)
    v,a,tol=[parameters[k] for k in ['manager.max_vel','manager.max_acc','manager.feasibility_tolerance']]
    controlpoint_limits=dict(velocity=v*(1+tol)+1e-4,acceleration=a*(1+tol)+1e-4)
    dynamic_limits=dict(velocity=v+parameters['optimization.vel_tolerance'],acceleration=a+parameters['optimization.acc_tolerance'])
    check('short_route_satisfies_unchanged_original_controlpoint_feasibility',g['velocity_controlpoint_max_abs']<=controlpoint_limits['velocity'] and g['acceleration_controlpoint_max_abs']<=controlpoint_limits['acceleration'],original_limits=controlpoint_limits,geometry=g)
    check('short_route_satisfies_unchanged_original_dynamic_check',g['peak_speed']<=dynamic_limits['velocity'] and g['peak_acceleration']<=dynamic_limits['acceleration'],original_limits=dynamic_limits,nominal_limits=dict(velocity=v,acceleration=a),nominal_speed_exceeded=g['peak_speed']>v,nominal_acceleration_exceeded=g['peak_acceleration']>a)
  check(name+'_planner_never_publishes_actuator_command',not commands)
  return detail
 finally:
  node.destroy_node();rclpy.try_shutdown()
  if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
  try:proc.wait(timeout=8)
  except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
  log.close();check(name+'_owned_planner_clean_exit',proc.returncode==0,returncode=proc.returncode)

try:
 cases.append(run_case(False));cases.append(run_case(True));passed=True
finally:
 result=dict(scope=__doc__,passed=globals().get('passed',False),baseline=args.baseline,checks=checks,cases=cases,source_sha256=hashlib.sha256((NAV/'ros2_ws/src/plan_manage/src/planner_manager.cpp').read_bytes()).hexdigest(),binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(),test_script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),scan_planner_prefix=str(prefix),limits_interpretation='Original SCAN tolerances are read from actual node, not a newly invented 10% limit. Curve time derivatives can exceed nominal .12/.15 within original tolerances and are NOT NAV actuator commands. NAV actual forward/yaw caps remain .12/.08; no physics or exact motion tracking claim.')
 (out/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
