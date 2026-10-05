#!/usr/bin/env python3
"""Actual DDS/candidate Navigation, synthetic interfaces only; no physics.

Bridge/Adapter status are deliberate fixture messages, not real hardware ACK.
No Gazebo, CHAMP, GT, SLAM estimator or real SCAN planner runs. The actual NAV
subscribers, raw input acceptance, checked-source association and emitted Twist
are exercised with precisely identified synthetic source payloads.
"""
import hashlib,json,os,sys,time
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).resolve().parent
ROOT=Path(os.environ['DEMO_TEST_ROOT']).resolve()
sys.path.insert(0,str(HERE));sys.path.insert(0,str(ROOT/'navigation'))
from nav_align_controller import NavigationAlign
from trajectory_contract import payload
import rclpy
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry,Path as RosPath
from sensor_msgs.msg import Imu,PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from geometry_msgs.msg import Twist,Point
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String,Header,Bool
from builtin_interfaces.msg import Time
from scan_planner_msgs.msg import Bspline

assert os.environ.get('ROS_DOMAIN_ID')=='77'
out=HERE/'ros77';out.mkdir(exist_ok=False)
os.environ['DEMO_RUN_DIR']=str(out)
rclpy.init(args=['--ros-args','-p','scenario_path:='+str(ROOT/'simulation/scenario.json')])
nav=NavigationAlign();probe=rclpy.create_node('align_translation_interface_probe')
ex=SingleThreadedExecutor();ex.add_node(nav);ex.add_node(probe)
clocks=probe.create_publisher(Clock,'/clock',10)
odom=probe.create_publisher(Odometry,'/demo/slam/body_odom',10)
cloud=probe.create_publisher(PointCloud2,'/cloud_registered_full',10)
imu=probe.create_publisher(Imu,'/livox/imu',10)
bridge=probe.create_publisher(String,'/demo/control/safety',10)
request=probe.create_publisher(String,'/demo/navigation/request',10)
stop=probe.create_publisher(Bool,'/demo/navigation/stop',10)
bsp=probe.create_publisher(Bspline,'/demo/navigation/bspline',10)
meta=probe.create_publisher(String,'/demo/navigation/trajectory_metadata',10)
commands=[];refs=[]
probe.create_subscription(Twist,'/demo/cmd_vel',lambda m:commands.append(dict(sim_ns=sim,
    value=[m.linear.x,m.linear.y,m.angular.z])),100)
probe.create_subscription(RosPath,'/demo/navigation/scan_reference',refs.append,10)
sim=10000000000;rawstamp=sim;traj=0;checks={};counter=0;stepcount=0;events=[]
def stamp(ns):s,n=divmod(ns,1000000000);return Time(sec=s,nanosec=n)
def check(k,v):
    checks[k]=bool(v)
    if not v:raise AssertionError(k)
def spins(n=12):
    for _ in range(n):ex.spin_once(timeout_sec=.001)
def send_odom(t,p,q=None,frame='camera_init'):
    q=Rotation.from_euler('z',.5).as_quat() if q is None else q
    m=Odometry(header=Header(frame_id=frame,stamp=stamp(t)),child_frame_id='demo_slam_body')
    m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,p)
    m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,q)
    odom.publish(m)
def pump(ns,p=(0.,0.,0.),*,fresh=True,points=None,cloud_stamp=None,adapter='walk',stops=None):
    global sim,rawstamp,stepcount
    limit=sim+ns
    while sim<limit:
        sim+=25000000;stepcount+=1;clocks.publish(Clock(clock=stamp(sim)))
        if stepcount%4==0:
            if fresh:rawstamp=sim
            send_odom(rawstamp,p)
        raw=Imu(header=Header(frame_id='imu_link',stamp=stamp(sim)));raw.orientation.w=1.;imu.publish(raw)
        pts=points if points is not None else [[5.,5.,-.5],[5.,6.,-.5],[6.,5.,-.5]]
        cs=sim if cloud_stamp is None else cloud_stamp
        cloud.publish(create_cloud_xyz32(Header(frame_id='camera_init',stamp=stamp(cs)),pts))
        value=dict(state='ready',safe=list(nav.command),requested=list(nav.command),
            joint_adapter=dict(state=adapter,sim=sim/1e9,nominal_calibrated=True,
                counters=dict(stops=counter if stops is None else stops)),joint_adapter_wall_age=0.)
        bridge.publish(String(data=json.dumps(value)))
        spins();time.sleep(.003)
def new_request(name):
    global counter
    stop.publish(Bool(data=True));pump(200000000,adapter='idle')
    request.publish(String(data=json.dumps(dict(schema_version=2,request_id=name,frame_id='camera_init',goals=[
        dict(goal_id='test-center',center=[5.,0.,0.],arrival=dict(type='sphere',radius_m=.3,dwell_sim_s=.4,
            control_band=dict(type='sphere',radius_m=.22,dwell_sim_s=.4)),timeout_sim_s=90.)]))))
    pump(250000000)
    check(name+'_actual_request_reference',nav.request_id==name and bool(refs))
    return nav.segment_started_ros
def trajectory():
    global traj
    traj+=1;msg=Bspline(order=3,traj_id=traj,knots=[0.,0.,0.,0.,10.,10.,10.,10.])
    p=nav.region_raw_pose
    msg.pos_pts=[Point(x=float(x),y=0.,z=0.) for x in [p[0],p[0]+1.,p[0]+3.,5.]]
    ref=refs[-1];rs=[ref.header.stamp.sec,ref.header.stamp.nanosec]
    bsp.publish(msg);meta.publish(String(data=json.dumps(dict(schema=1,reference_stamp=rs,
        body_goal=[5.,0.,0.],trajectory=payload(msg)))))
    pump(250000000);check('actual_checked_source_'+str(traj),nav.active_trajectory_id==traj and nav.samples is not None)
def retreat_fit(sign=-1):
    pump(1100000000)
    origin=sim
    while sim-origin<800000000:
        p=(sign*.08*(sim-origin)/1e9,0.,0.)
        pump(100000000,p)
    return p
try:
    pump(500000000,adapter='idle');check('actual_nav_idle',nav.state=='idle')
    new_request('align-forward');trajectory()
    p=retreat_fit()
    check('actual_new_raw_supported_fit',nav.align_pd.latest.get('valid') and nav.align_pd.latest['sample_count']>=4)
    check('positive_longitudinal_emit_from_measured_retreat',any(r['value'][0]>0 for r in commands))
    check('every_actual_mixed_emit_has_vy0_and_yaw_cap',all(r['value'][1]==0. and abs(r['value'][2])<=.08+1e-12 for r in commands if r['value'][0]!=0.))
    raw=dict(nav.align_raw);oldwall=nav.align_pd.last_wall
    send_odom(raw['stamp'],[7.,0.,0.],Rotation.from_euler('z',2.).as_quat());spins(20)
    check('duplicate_real_DDS_pose_cannot_replace_PD_rotation_or_age',nav.align_raw['stamp']==raw['stamp']
        and np.array_equal(nav.align_raw['R'],raw['R']) and nav.align_pd.last_wall==oldwall)
    send_odom(sim+100000000,[4.,0.,0.],frame='world');spins(20)
    check('foreign_frame_real_DDS_rejected_for_PD',nav.align_raw['stamp']==raw['stamp'])
    pump(350000000,p,fresh=False)
    check('duplicate_stamps_become_stale_and_emit_exactzero',nav.command==[0.,0.,0.] and nav.align_pd.context is None)
    new_request('align-backward');trajectory();p=retreat_fit(1)
    check('negative_compensation_supported_by_raw_forward_error',any(r['value'][0]<0 for r in commands))
    # Raw body heading is .5rad; build three returns in its BACKWARD corridor.
    direction=nav.align_raw['R'][:2,0];center=np.array(p[:2])-.65*direction
    lateral=np.array([-direction[1],direction[0]])
    pts=[[*list(center+x*lateral),.1] for x in [-.1,0.,.1]]
    deadline=nav.segment_started_ros;before=nav.reference_requests;counter=0
    pump(250000000,p,points=pts)
    check('real_backward_body_corridor_causes_exact_STOP',nav.drift_pending and nav.command==[0.,0.,0.])
    check('STOP_preserves_original_region_and_deadline',nav.waypoint_index==0 and nav.segment_started_ros==deadline and not nav.region_arrivals)
    old_count=nav.drift_stop_count
    pump(400000000,p,adapter='returning',stops=old_count+1)
    check('zero_returning_status_does_not_authorize_fresh_reference',nav.reference_requests==before)
    pump(400000000,p,adapter='idle',stops=old_count)
    check('old_idle_count_does_not_authorize_fresh_reference',nav.reference_requests==before)
    pump(1200000000,p,adapter='idle',stops=old_count+1)
    check('fresh_controlled_idle_ACK_after_one_second_declares_new_reference',nav.reference_requests==before+1 and nav.drift_pending and nav.command==[0.,0.,0.])
    counter=old_count+1
    trajectory();check('fresh_matched_payload_only_releases_original_preturn',not nav.drift_pending and nav.heading_gate.phase=='pre_turn' and nav.command==[0.,0.,0.])
    # Correct-signed clear inputs; an old cloud buffer with a new wall callback
    # must not borrow the newest cloud_stamp to authorize compensation.
    p=retreat_fit(-1);check('fresh_source_can_enter_measured_compensation',nav.align_pd.latest.get('valid'))
    cs=nav.cloud_stamp-100000000
    pump(150000000,p,cloud_stamp=cs)
    check('old_header_real_cloud_buffer_fails_closed_despite_fresh_global_age',nav.drift_pending and nav.command==[0.,0.,0.])
    check('original_source_identity_and_timeout_not_replaced_by_feedback',nav.goals[0].timeout_sim_s==90. and nav.goals[0].radius==.3 and nav.segment_started_ros==deadline)
    events=list(nav.drift_log)
except Exception as e:
    failure=f'{type(e).__name__}: {e}';raise
finally:
    nav.publish_command();nav.close_drift();nav.event_archive.close()
    for n in [nav,probe]:ex.remove_node(n);n.destroy_node()
    ex.shutdown();rclpy.try_shutdown()
    result=dict(passed=bool(checks) and all(checks.values()) and 'failure' not in globals(),
        checks=checks,failure=globals().get('failure'),owned_nodes_destroyed=True,domain=77,
        scope=__doc__,commands=commands,events=events,
        sources={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in ['nav_align_controller.py','turn_drift.py','align_translation.py','test_ros77.py']})
    (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['events','commands']},indent=2))
