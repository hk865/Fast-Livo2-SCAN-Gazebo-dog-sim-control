#!/usr/bin/env python3
"""Actual scheduled native guard/control tests on offline synthetic state."""
from pathlib import Path
from types import SimpleNamespace as NS
import hashlib,json,math,runpy,sys
import numpy as np
HERE=Path(__file__).resolve().parent
# Construct the same actual controller test state; no Node initialization.
ns=runpy.run_path(str(HERE/'offline_checks.py'))
TeacherNavigation=ns['TeacherNavigation'];base=ns['base'];control_node=ns['control_node'];ROOT=ns['ROOT'];NAV=ns['NAV']
sys.path.insert(0,str(NAV))
from controller import NATIVE_STEERING_GUARD
from guard_audit import NativeGuardAudit
checks={}
def check(n,c,e=None):
 checks[n]={'passed':bool(c),'evidence':e}
 if not c:raise AssertionError(n)
def make():
 f=control_node();f.records=[];f.evidence=NS(error=None,append=lambda path,row:f.records.append((Path(path).name,row)))
 f.run=HERE;f.obstacle_hold=True;f.obstacle_hold_started_ros=24.;f.obstacle_clear_since=25.69
 f.native_guard_audit=NativeGuardAudit(NATIVE_STEERING_GUARD);f.guard_sequence=0;f.pose_quaternion=[0,0,0,1]
 f.cloud=np.array([[.6,0.,0.],[.7,.1,0.]],dtype=float);f.guard_cloud=f.cloud;f.trajectory_archive_reference='mock_no_actual_trajectory.npz'
 return f
def clock(f,sim,wall,due):
 f.clock_ns=round(sim*1e9);f.pose_stamp=f.cloud_stamp=f.clock_ns;f.pose_updated=f.cloud_updated=wall
 f.last_obstacle_check=wall-.11 if due else wall-.05
 f.guard_cloud_receipt={'stamp_ns':f.cloud_stamp,'frame_id':'camera_init','received_monotonic_wall':wall,
  'callback_ros_clock_ns':f.clock_ns,'nearest_slam_pose_stamp_ns':f.pose_stamp,
  'filtering_body_pose':f.pose.tolist(),'filtering_body_rotation':f.rotation.tolist(),
  'filtered_xyz_float64_sha256':hashlib.sha256(f.cloud.astype('<f8').tobytes()).hexdigest(),'filtered_points':2,'self_filtered_points':0}
old=base.steering_obstacle_ahead
try:
 f=make();base.steering_obstacle_ahead=f.record_native_guard;f.last_actual_guard_clock_ns=26_090_000_000
 clock(f,26.64,100.,True);TeacherNavigation.control(f,100.)
 check('actual_550ms_guard_gap_resets_original_clear_timer',f.obstacle_hold and f.obstacle_clear_since==26.64 and f.guard_sequence==1)
 check('gap_reset_has_no_extra_geometry_call',len(f.native_guard_audit.calls)==2)
 check('reset_history_records_actual_source_gap',any(p=='navigation_guard_continuity_history.jsonl'and r['actual_guard_gap_ns']==550_000_000 for p,r in f.records))
 for i in range(1,20):
  sim=26.64+i*.05;wall=100.+i*.05;due=i%3==0
  clock(f,sim,wall,due);TeacherNavigation.control(f,wall)
  assert f.obstacle_hold and f.obstacle_resumes==0
 check('healthy_clear_guard_span_below_one_second_never_releases',True)
 before=f.guard_sequence;clock(f,27.64,101.,False);TeacherNavigation.control(f,101.)
 check('cached_elapsed_one_second_does_not_release_or_replan',f.obstacle_hold and f.obstacle_resumes==0 and f.plans==0 and f.command==[0,0,0])
 check('cached_release_wait_computes_no_extra_geometry',f.guard_sequence==before)
 clock(f,27.69,101.05,True);TeacherNavigation.control(f,101.05)
 check('next_original_scheduled_actual_guard_confirms_and_releases',not f.obstacle_hold and f.obstacle_resumes==1 and f.plans==1)
 check('actual_release_keeps_zero_until_new_scan',f.command==[0,0,0]and f.samples is None)
 guards=[r for p,r in f.records if p=='navigation_guard_history.jsonl']
 check('all_actual_guard_calls_have_complete_contiguous_trace',[r['sequence']for r in guards]==list(range(1,f.guard_sequence+1)))
 check('release_exact_actual_guard_elapsed_at_least_one_second',guards[-1]['compute_sim_time_s']-guards[-1]['clear_start_before_s']>=1. and not guards[-1]['obstacle_hold_after'])
 check('native_guard_inputs_results_unchanged',all(not r['union_result']['blocked']for r in guards))
 f=make();f.last_actual_guard_clock_ns=10_000_000_000;f.clock_ns=10_500_000_000;f.pose_stamp=f.cloud_stamp=f.clock_ns
 clock(f,10.5,100.,False);f.last_actual_guard_clock_ns=10_000_000_000
 TeacherNavigation.control(f,100.)
 check('cached_gap_over_300ms_resets_and_cannot_release',f.obstacle_hold and f.obstacle_resumes==0 and f.command==[0,0,0]and f.obstacle_clear_since==10.5)
 f=make();clock(f,26.7,100.,True);f.last_actual_guard_clock_ns=26_600_000_000;f.cloud_stamp=26_400_000_000
 TeacherNavigation.control(f,100.)
 check('exact_300ms_original_cloud_source_age_stops_and_resets',f.command==[0,0,0]and f.obstacle_hold and f.obstacle_clear_since is None and f.guard_sequence==0)
 f=make();clock(f,26.7,100.,True);f.last_actual_guard_clock_ns=26_800_000_000
 TeacherNavigation.control(f,100.)
 check('backward_actual_clock_fails_closed_before_release',f.state=='failed'and f.command==[0,0,0]and f.obstacle_hold)
 f=make();clock(f,26.7,100.,True);f.last_actual_guard_clock_ns=f.clock_ns
 rejected=False
 try:f.record_native_guard(f.cloud,f.pose,np.array([1,0,.3]),np.array([1,0]),np.array([[0,0,.3],[1,0,.3]]))
 except RuntimeError:rejected=True
 check('duplicate_actual_guard_clock_fails_closed_without_geometry',rejected and f.state=='failed'and f.guard_sequence==0)
 f=make();clock(f,27.,100.,False);f.last_actual_guard_clock_ns=26_900_000_000;f.obstacle_clear_since=26.
 seen=[];f.apply_tilt_guard=lambda *a:seen.append(a)or True
 TeacherNavigation.control(f,100.)
 check('cached_wait_keeps_actual_physical_protection_check',bool(seen)and f.command==[0,0,0]and f.obstacle_clear_since is None)
finally:base.steering_obstacle_ahead=old
receipt={'schema':1,'status':'passed','checks':checks,'ROS_init_called':False,'ROS_nodes_started':False,'Gazebo_started':False,
 'mock_records_are_not_actual_sensor_evidence':True,'runtime_navigation':'unverified',
 'source_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in [NAV/'controller.py',NAV/'guard_audit.py',Path(__file__)]}}
out=HERE/'continuity_checks.json';out.write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'status':'passed','checks':len(checks),'receipt':str(out)}))
