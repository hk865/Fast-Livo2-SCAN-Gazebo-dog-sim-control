#!/usr/bin/env python3
"""Prospective turn20/actual-native-guard regression checks, no ROS init."""
import copy,hashlib,importlib.util,json,math,sys,time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np

ROOT=Path(__file__).resolve().parents[2];NAV=ROOT/'navigation'
sys.path.insert(0,str(ROOT.parent/'navigation'));sys.path.insert(0,str(NAV))
from teacher_transition import TeacherHeadingGate
from guard_audit import NativeGuardAudit,result_json
from control_core import steering_obstacle_ahead
from controller import TeacherNavigation,base
from runtime_io import EvidenceWriter

oldfile=ROOT/'runs/20261004_092935_navigation_slam_scan_dynamic_flat_v42_cleanup_r1_10c7/sources/navigation/teacher_transition.py'
spec=importlib.util.spec_from_file_location('frozen_v42_gate',oldfile);old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
profile=json.loads((NAV/'flat_relative_roundtrip.json').read_text())
dynamic=json.loads((NAV/'dynamic/flat_dynamic_profile.json').read_text())
turn20=json.loads((NAV/'dynamic/flat_dynamic_profile_turn20.json').read_text())
checks={}
def check(name,ok,evidence=None):
    checks[name]={'passed':bool(ok),'evidence':evidence}
    if not ok:raise AssertionError(name)

check('turn20_only_cap_and_explicit_variant_change',
    {k:v for k,v in turn20.items()if k not in ('experiment','variant','max_yaw_rate_radps')}==
    {k:v for k,v in dynamic.items()if k not in ('experiment','variant','max_yaw_rate_radps')})
check('unchanged_reviewed_old_profiles',hashlib.sha256((NAV/'flat_relative_roundtrip.json').read_bytes()).hexdigest()=='38154e14a7ba9735c9f6be15d4e6f3bce02a227706a9e41388c748cce3aa2b7b'
    and hashlib.sha256((NAV/'dynamic/flat_dynamic_profile.json').read_bytes()).hexdigest()=='c4f198a923279943a607ffa9b5c1f81af690028e3a048f952f922df62f46b49e')
a=old.TeacherHeadingGate(profile);b=TeacherHeadingGate(profile);rng=np.random.default_rng(20261004)
count=0;phases=set()
for i in range(6000):
    stamp=1+i*.05;wall=100+stamp
    if i%97==0:a.reset();b.reset()
    vel=[.001,0,0]if i%31<20 else [.06,0,0]
    gyro=[0,0,.001]if i%41<31 else [0,0,.08]
    zero=i%53<43
    args=(stamp,wall,vel,gyro[2],gyro,stamp-.01,zero)
    assert a.observe(*args)==b.observe(*args)
    yaw=.005*math.sin(i*.05);target=0. if i%101<75 else float(rng.uniform(-3.14,3.14))
    aa=a.update(stamp,yaw,target,wall);bb=b.update(stamp,yaw,target,wall)
    assert aa==bb and a.evidence(stamp,wall)==b.evidence(stamp,wall)
    assert (a.phase,a.heading,a.settle_until,a.outside_since,list(a.history),a.last_stamp)==(b.phase,b.heading,b.settle_until,b.outside_since,list(b.history),b.last_stamp)
    count+=1;phases.add(a.phase)
check('flat_6000_full_state_and_command_regression_exact',True,{'steps':count,'phases':sorted(phases),'frozen_source':str(oldfile)})
def stopped_gate(p):
    gate=TeacherHeadingGate(p)
    for t in (1.,1.1,1.2,1.3):gate.observe(t,t,[0,0,0],0,[0,0,0],t,True)
    return gate
for p,cap,label in ((profile,.12,'flat'),(dynamic,.12,'old_dynamic'),(turn20,.2,'turn20')):
    g=stopped_gate(p);check(label+'_positive_align_cap',g.update(1.3,0.,2.,1.3)==(False,cap))
    g=stopped_gate(p);check(label+'_negative_align_cap',g.update(1.3,0.,-2.,1.3)==(False,-cap))
    g=TeacherHeadingGate(p);check(label+'_no_sensor_stop_no_turn',g.update(5.,0.,2.,5.)==(False,0.))
for bad in (0.,-.1,.3000001,math.inf,math.nan):
    badp=copy.deepcopy(turn20);badp['max_yaw_rate_radps']=bad
    rejected=False
    try:TeacherHeadingGate(badp)
    except ValueError:rejected=True
    check('rejects_invalid_turn_cap_'+str(bad),rejected)

capture=NativeGuardAudit(steering_obstacle_ahead)
native_obstacle=steering_obstacle_ahead.__globals__['obstacle_ahead'];counts=0
for i in range(800):
    pose=np.array([0.,0.,.27]);target=np.array([.2+float(rng.random()),float(rng.uniform(-1,1)),.27])
    direction=rng.uniform(-1,1,2);route=np.array([[0,0,.27],[1,0,.27]])
    cloud=rng.uniform([-1,-1,-.2],[1.5,1,1.2],size=(i%71,3))
    expected=steering_obstacle_ahead(cloud,pose,target,direction,route)
    actual=capture(cloud,pose,target,direction,route)
    assert actual==expected and len(capture.calls)==2
    assert tuple(type(x)for x in actual)==tuple(type(x)for x in expected)
    counts+=1
check('800_native_guard_returns_exact_no_extra_geometry',True,{'native_calls_per_guard':2,'cases':counts})
check('original_native_code_object_retained',capture.instrumented.__code__ is steering_obstacle_ahead.__code__)
check('shared_native_geometry_namespace_not_mutated',steering_obstacle_ahead.__globals__['obstacle_ahead'] is native_obstacle)

cloud=np.array([[.6,-.1,.45],[.65,0,.5],[.7,.1,.55]],dtype=float);pose=np.array([0,0,.27]);route=np.array([[0,0,.27],[1,0,.27]])
f=NS(profile=profile,last_actual_guard_clock_ns=None,guard_cloud=cloud,cloud_stamp=2_000_000_000,pose_quaternion=[0,0,0,1],guard_sequence=0,
    get_clock=lambda:NS(now=lambda:NS(nanoseconds=2_100_000_000)),native_guard_audit=capture,
    request_id='actual-slam-relative',waypoint_index=0,active_trajectory_id=4,
    trajectory_association=NS(reference_stamp=[2,0]),trajectory_archive_reference='navigation_trajectories/000001_trajectory_4.npz',
    pose_stamp=2_000_000_000,rotation=np.eye(3),tracking_pose=pose.copy(),pose_updated=100.,
    waypoints=np.array([[1,0,.27]]),obstacle_hold=False,obstacle_clear_since=None,
    guard_cloud_receipt={'stamp_ns':2_000_000_000,'frame_id':'camera_init','received_monotonic_wall':100.,
        'callback_ros_clock_ns':2_010_000_000,'nearest_slam_pose_stamp_ns':2_000_000_000,
        'filtering_body_pose':pose.tolist(),'filtering_body_rotation':np.eye(3).tolist(),
        'filtered_xyz_float64_sha256':hashlib.sha256(cloud.astype('<f8').tobytes()).hexdigest(),
        'filtered_points':3,'self_filtered_points':0})
actual=TeacherNavigation.record_native_guard(f,cloud,pose,np.array([1,0,.27]),np.array([1,0]),route)
row=f.pending_guard_row
check('actual_controller_hook_preserves_native_result',actual==steering_obstacle_ahead(cloud,pose,np.array([1,0,.27]),np.array([1,0]),route))
check('hook_exact_integer_pose_cloud_and_filtered_hash',row['control_pose_stamp_ns']==2_000_000_000 and row['cloud_header_stamp_ns']==2_000_000_000 and row['filtered_xyz_float64_sha256']==f.guard_cloud_receipt['filtered_xyz_float64_sha256'])
check('hook_exact_route_sha_and_two_results',row['route_float64_sha256']==hashlib.sha256(route.astype('<f8').tobytes()).hexdigest()and row['goal_corridor_result']['blocked']and row['motion_corridor_result']['blocked'])
check('hook_small_record_json_finite',len(json.dumps(row,allow_nan=False))<5000)
bad=False
try:TeacherNavigation.record_native_guard(f,cloud.copy(),pose,np.array([1,0,.27]),np.array([1,0]),route)
except RuntimeError:bad=True
check('unpaired_cloud_identity_fails_closed',bad)
f.pose_quaternion=None;bad=False
try:TeacherNavigation.record_native_guard(f,cloud,pose,np.array([1,0,.27]),np.array([1,0]),route)
except RuntimeError:bad=True
check('missing_exact_slam_receipt_fails_closed',bad)

# Actual control method finalizes the pending guard after native hold/recovery
# mutations. Patch only the offline class method; never instantiate a Node.
original=base.Navigation.control
writer=EvidenceWriter();tmp=Path(__file__).with_name('mock_guard_history.jsonl')
if tmp.exists():tmp.unlink()
mock_fields=dict(profile=profile,evidence=writer,state='running',obstacle_hold=False,heading_gate=NS(phase='drive'),
     get_clock=lambda:NS(now=lambda:NS(nanoseconds=3_000_000_000)),run=tmp.parent,
     command=[0.,0.,0.],last_command_time=3.,obstacle_clear_since=2.,obstacle_resumes=1)
f=TeacherNavigation.__new__(TeacherNavigation);f.__dict__.update(mock_fields)
def fake_native_control(self,now):
    self.pending_guard_row={'schema':'mock_actual_control_finalization','sequence':1}
    self.obstacle_hold=False;self.command=[.1,0,0]
try:
    base.Navigation.control=fake_native_control;TeacherNavigation.control(f,100.)
finally:base.Navigation.control=original;writer.close()
written=json.loads((tmp.parent/'navigation_guard_history.jsonl').read_text().splitlines()[-1])
check('guard_finalization_records_actual_post_control_hold_command',not written['obstacle_hold_after']and written['command_after_control']==[.1,0,0]and not written['zero_requested_after_control']and written['clear_elapsed_after_s']==1.)
check('native_base_control_restored_after_offline_mock',base.Navigation.control is original)

receipt={'schema':1,'status':'passed','checks':checks,'starts_ROS':False,'rclpy_init_called':False,
    'starts_Gazebo':False,'navigation_runtime_status':'unverified',
    'mock_record_is_not_actual_sensor_evidence':True,
    'source_hashes':{str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()for p in
        [NAV/'teacher_transition.py',NAV/'guard_audit.py',NAV/'controller.py',NAV/'dynamic/flat_dynamic_profile_turn20.json',Path(__file__)]}}
out=Path(__file__).with_name('v43_regression_checks.json');out.write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'status':'passed','checks':len(checks),'receipt':str(out)}))
