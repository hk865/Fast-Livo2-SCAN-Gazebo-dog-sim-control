#!/usr/bin/env python3
"""Actual DDS/controller synthetic-interface contracts, never physics/SLAM PASS."""
import copy,hashlib,json,os,signal,subprocess,sys,time
from pathlib import Path
NAV=Path(__file__).resolve().parents[2];sys.path.insert(0,str(NAV))
from goal_regions import parse_request,definitions_sha256
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import String,Bool,Header
from rosgraph_msgs.msg import Clock

assert os.environ.get('ROS_DOMAIN_ID')=='75'
out=Path(__file__).with_name('ros75_inner_v1');out.mkdir(exist_ok=False)
env=os.environ.copy();env.pop('DEMO_RUN_DIR',None)
checks=[];status={};commands=[];history=[];references=[];sim_ns=10000000000;fixed_stamp=None

def check(name,condition):
    checks.append(dict(name=name,passed=bool(condition)))
    if not condition:raise AssertionError(name+': '+json.dumps(status))

def stamp(ns):
    from builtin_interfaces.msg import Time
    s,n=divmod(ns,1000000000);return Time(sec=s,nanosec=n)

def state(m):
    status.clear();status.update(json.loads(m.data));history.append(copy.deepcopy(status))

log=(out/'controller.log').open('w')
proc=subprocess.Popen([sys.executable,str(NAV/'controller.py')],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();node=Node('region_contract_probe')
node.create_subscription(String,'/demo/navigation/status',state,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
node.create_subscription(Twist,'/demo/cmd_vel',lambda m:commands.append([m.linear.x,m.linear.y,m.angular.z]),10)
node.create_subscription(RosPath,'/demo/navigation/scan_reference',lambda m:references.append(m),10)
clock=node.create_publisher(Clock,'/clock',10);odom=node.create_publisher(Odometry,'/demo/slam/body_odom',10)
imu=node.create_publisher(Imu,'/livox/imu',10);cloud=node.create_publisher(__import__('sensor_msgs.msg',fromlist=['PointCloud2']).PointCloud2,'/cloud_registered_full',10)
bridge=node.create_publisher(String,'/demo/control/safety',10);requests=node.create_publisher(String,'/demo/navigation/request',10)
stop=node.create_publisher(Bool,'/demo/navigation/stop',10)

def pump(seconds,p=(.5,0,0),fresh=True,safety='ready',odom_stride=1):
    global sim_ns,fixed_stamp
    until=time.monotonic()+seconds
    iteration=0
    while time.monotonic()<until:
        iteration+=1
        sim_ns+=25000000;clock.publish(Clock(clock=stamp(sim_ns)))
        if fresh or fixed_stamp is None:fixed_stamp=stamp(sim_ns)
        h=Header(frame_id='camera_init',stamp=fixed_stamp)
        m=Odometry(header=h,child_frame_id='demo_base');m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,p)
        m.pose.pose.orientation.w=1.
        if iteration%odom_stride==0:odom.publish(m)
        raw=Imu(header=Header(frame_id='imu_link',stamp=stamp(sim_ns)));raw.orientation.w=1.;imu.publish(raw)
        # Synthetic floor returns remain outside the real Go2 self filter;
        # the v2 fixture accidentally put every point on the filtered body.
        cloud.publish(create_cloud_xyz32(h,[[5.,5.,-.5],[5.,6.,-.5],[6.,5.,-.5]]))
        bridge.publish(String(data=json.dumps(dict(state=safety,reason='synthetic_interface_fixture'))))
        rclpy.spin_once(node,timeout_sec=.006);time.sleep(.014)

def send(data):requests.publish(String(data=json.dumps(data)))
def region_request(r=.35,control_r=.25):return dict(schema_version=2,request_id='region-contract',frame_id='camera_init',goals=[
    dict(goal_id='flat',center=[0,0,0],arrival=dict(type='disc_prism',radius_m=r,height_half_span_m=.1,dwell_sim_s=.4,control_band=dict(radius_m=control_r,height_half_span_m=.1)),timeout_sim_s=90),
    dict(goal_id='origin',center=[1,0,0],arrival=dict(type='sphere',radius_m=.22,dwell_sim_s=.4,control_band=dict(radius_m=.17)),timeout_sim_s=90)])

try:
    pump(1.0);check('actual_controller_discovered',status.get('state')=='idle')
    request=region_request();send(request);pump(.4)
    expected=definitions_sha256(parse_request(request)[1]);check('actual_v2_hash_and_definitions',status.get('goals_definition_sha256')==expected and len(status.get('goals_definitions',[]))==2)
    check('SCAN_reference_uses_original_center_and_height_boundary',bool(references) and references[-1].poses[-1].pose.position.x==0. and references[-1].poses[-1].pose.position.y==0. and references[-1].poses[-1].pose.position.z==-.4)
    send(region_request(.4));pump(.3)
    check('same_ID_changed_region_rejected_without_mutation',status.get('goals_definition_sha256')==expected and status.get('waypoint_index')==0 and bool(status.get('last_rejected')))
    send(region_request(control_r=.20));pump(.25)
    check('same_ID_changed_only_control_band_rejected',status.get('goals_definition_sha256')==expected and status['goals_definitions'][0]['arrival']['control_band']['radius_m']==.25 and bool(status.get('last_rejected')))
    send(request);pump(.5,p=(.30,0,0))
    check('raw_outer_region_inside_alone_cannot_start_inner_dwell',status.get('waypoint_index')==0 and not status.get('region_arrivals') and (status.get('region_arrival_evidence') or {}).get('region_inside') is True and status['region_arrival_evidence']['control_region_inside'] is False and status['region_arrival_evidence']['dwell_ns']==0)
    send(request);pump(.2,p=(0,0,1.2))
    check('wrong_floor_same_XY_does_not_arrive',status.get('state')=='running' and status.get('waypoint_index')==0)
    pump(.1,p=(0,0,0));pump(.9,p=(0,0,0),fresh=False)
    check('repeated_odom_stamps_do_not_create_dwell',status.get('state')=='running' and status.get('waypoint_index')==0 and not status.get('region_arrivals'))
    pump(.12,p=(0,0,0));pump(.4,p=(0,0,0),safety='hold')
    check('actual_bridge_hold_resets_region_dwell',status.get('waypoint_index')==0 and status.get('tilt_hold') and (status.get('region_arrival_evidence') or {}).get('dwell_ns',0)==0)
    pump(.9,p=(0,0,0))
    check('actual_first_region_receipt',status.get('waypoint_index')==1 and len(status.get('region_arrivals',[]))==1)
    receipt=status['region_arrivals'][0]
    check('receipt_has_exact_integer_stamp_raw_position_and_hash',type(receipt['stamp_ns']) is int and type(receipt['start_stamp_ns']) is int and receipt['dwell_ns']==receipt['stamp_ns']-receipt['start_stamp_ns']>=400000000 and receipt['raw_position']==[0.,0.,0.] and receipt['goals_definition_sha256']==expected and receipt['goal_id']=='flat')
    pump(.4,p=(1.25,0,0));check('new_sphere_retains_strict_point_bound',status.get('waypoint_index')==1)
    pump(.5,p=(1.2,0,0));check('origin_outer22_does_not_override_declared_inner17',status.get('waypoint_index')==1 and status['region_arrival_evidence']['region_inside'] is True and status['region_arrival_evidence']['control_region_inside'] is False)
    pump(.7,p=(1.15,0,0));check('ordered_two_goal_receipts_with_no_fabricated_index',status.get('state')=='succeeded' and [r['goal_id'] for r in status.get('region_arrivals',[])]==['flat','origin'])
    check('inner_and_outer_receipt_definitions_are_both_explicit',all(e['region_inside'] and e['control_region_inside'] and e['control_arrival_definition']==e['arrival_definition']['control_band'] for e in status['region_arrivals']))
    old=dict(request_id='old-contract',frame_id='camera_init',waypoints=[[0,0,0]])
    send(old);pump(.4,p=(.25,0,0));check('legacy_point_not_replaced_by_flat_region_radius',status.get('state')=='running' and status.get('waypoint_index')==0 and status['goals_definitions'][0]['legacy'])
    pump(.7,p=(.2,0,0));check('legacy_timer_contract_remains_and_no_region_receipt',status.get('state')=='succeeded' and not status['region_arrivals'])
    overlap=region_request();overlap['request_id']='overlapping-regions'
    overlap['goals'][1]=copy.deepcopy(overlap['goals'][0]);overlap['goals'][1]['goal_id']='flat-next'
    send(overlap);pump(1.6,p=(.2,0,0),odom_stride=5)
    check('overlapping_goals_need_strictly_new_odom_for_next_window',status.get('state')=='succeeded' and len(status.get('region_arrivals',[]))==2 and status['region_arrivals'][1]['start_stamp_ns']>status['region_arrivals'][0]['stamp_ns'])
    check('synthetic_arrival_tests_never_drive_without_SCAN',all(v==[0.,0.,0.] for v in commands))
    timeout_request=region_request();timeout_request['request_id']='region-deadline'
    timeout_request['goals']=timeout_request['goals'][:1]
    send(timeout_request);pump(.10,p=(.2,0,0))
    # No dwell can complete yet. Declared timeout must also bound in-region
    # waits and protected arrival resets, with exact zero and no fake index.
    sim_ns+=91_000_000_000
    pump(.20,p=(.2,0,0),safety='hold')
    check('v2_inside_protected_wait_obeys_declared_90s_deadline',status.get('state')=='failed' and status.get('waypoint_index')==0 and not status.get('region_arrivals') and '仿真时间限制' in status.get('message',''))
    check('v2_deadline_failure_never_sends_motion',all(v==[0.,0.,0.] for v in commands))
finally:
    stop.publish(Bool(data=True));rclpy.spin_once(node,timeout_sec=.05)
    node.destroy_node();rclpy.try_shutdown()
    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
    try:proc.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
    log.close()
    result=dict(scope=__doc__,passed=bool(checks) and all(c['passed'] for c in checks) and proc.returncode==0,
        checks=checks,controller_exit_code=proc.returncode,owned_controller_clean=proc.poll() is not None,
        status_history=history,actual_commands=commands,source_sha256={name:hashlib.sha256((NAV/name).read_bytes()).hexdigest() for name in ['controller.py','goal_regions.py','control_core.py']})
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['status_history','actual_commands']},indent=2))
