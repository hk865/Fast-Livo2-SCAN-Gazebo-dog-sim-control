#!/usr/bin/env python3
"""Actual controller arrival/control methods on synthetic state; no ROS init."""
from pathlib import Path
from types import SimpleNamespace as NS
import copy,hashlib,json,math,sys
import numpy as np
ROOT=Path(__file__).resolve().parents[2];NAV=ROOT/'navigation'
sys.path.insert(0,str(ROOT.parent/'navigation'));sys.path.insert(0,str(NAV))
from controller import TeacherNavigation,base
from goal_regions import parse_request,ArrivalWindow,definitions_sha256
checks={}
def check(n,c,e=None):
    checks[n]={'passed':bool(c),'evidence':e}
    if not c:raise AssertionError(n)
profiles={name:json.loads((NAV/name).read_text())for name in ('flat_relative_roundtrip.json','dynamic/flat_dynamic_profile.json','dynamic/flat_dynamic_profile_turn20.json','dynamic/flat_dynamic_profile_dwell.json')}
new=profiles['dynamic/flat_dynamic_profile_dwell.json'];old=profiles['dynamic/flat_dynamic_profile_turn20.json']
check('new_profile_only_selector_variant_and_stop_policy_change',
 {k:v for k,v in new.items()if k not in ('experiment','variant','arrival_stop_policy','clear_guard_gap_max_sim_s')}==
 {k:v for k,v in old.items()if k not in ('experiment','variant','arrival_stop_policy')})
request={'schema_version':2,'request_id':'offline_not_actual_route','frame_id':'camera_init',
 'goals':[{'goal_id':name,'center':center,'timeout_sim_s':90.,'arrival':{'type':'disc_prism','radius_m':.22,
 'height_half_span_m':.1,'dwell_sim_s':.6,'control_band':{'type':'disc_prism','radius_m':.17,'height_half_span_m':.1}}}
 for name,center in [('out',[1.,0.,.3]),('return',[0.,0.,.3])]]}
rid,goals=parse_request(request)
def node(profile):
 f=TeacherNavigation.__new__(TeacherNavigation)
 f.profile=profile;f.goals=goals;f.waypoint_index=0;f.region_raw_pose=np.array([.85,0,.3]);f.pose_stamp=10_000_000_000
 f.obstacle_hold=False;f.tilt_hold=False;f.bridge_safety={'state':'ready'};f.region_arrival=ArrivalWindow(goals[0]);f.region_arrival_evidence=None
 f.request_id=rid;f.goals_definition_sha256=definitions_sha256(goals)
 return f
# All legacy/current profiles retain the original return and evidence exactly.
steps=[(.85,False,'ready'),(.85,False,'ready'),(.9,True,'ready'),(.9,False,'hold'),(.7,False,'ready')]+[(.85,False,'ready')]*8
for name,p in profiles.items():
 if p is new:continue
 a=node(p);b=node(p)
 for i,(x,hold,bridge)in enumerate(steps):
  for f in (a,b):f.region_raw_pose=np.array([x,0,.3]);f.pose_stamp=10_000_000_000+i*100_000_000;f.obstacle_hold=hold;f.bridge_safety={'state':bridge}
  aa=base.Navigation.measured_region_arrival(a);bb=TeacherNavigation.measured_region_arrival(b)
  assert aa==bb and a.region_arrival_evidence==b.region_arrival_evidence
 check('old_return_and_evidence_exact_'+name,True,{'actual_method_cases':len(steps)})
f=node(new);result=[]
for i in range(7):
 f.pose_stamp=10_000_000_000+i*100_000_000;result.append(TeacherNavigation.measured_region_arrival(f))
check('first_control_entry_does_not_start_zero_parking',result[0]==(False,False))
check('less_than_original_600ms_never_arrives',all(x==(False,False)for x in result[:-1]))
check('original_raw_600ms_dwell_starts_zero_and_arrives',result[-1]==(True,True)and f.region_arrival_evidence['dwell_ns']==600_000_000)
check('duplicate_stamp_does_not_forge_arrival',TeacherNavigation.measured_region_arrival(f)==(False,False))
for kind in ('obstacle','tilt','bridge','outside','gap'):
 f=node(new)
 for i in range(5):f.pose_stamp=10_000_000_000+i*100_000_000;TeacherNavigation.measured_region_arrival(f)
 f.pose_stamp+=100_000_000
 if kind=='obstacle':f.obstacle_hold=True
 if kind=='tilt':f.tilt_hold=True
 if kind=='bridge':f.bridge_safety={'state':'hold'}
 if kind=='outside':f.region_raw_pose=np.array([.829,0,.3])
 if kind=='gap':f.pose_stamp+=300_000_001
 result=TeacherNavigation.measured_region_arrival(f)
 check(kind+'_cannot_reuse_prior_dwell',result==(False,False)and(f.region_arrival.since is None or f.region_arrival.since==f.pose_stamp))
f=node(new);f.region_raw_pose=np.array([.99,0,.401])
check('original_height_band_remains_required',TeacherNavigation.measured_region_arrival(f)==(False,False))
f=node(new);f.region_raw_pose=np.array([.829,0,.3]);f.tracking_pose=np.array([.99,0,.3])
check('tracking_pose_cannot_substitute_raw_slam_arrival',TeacherNavigation.measured_region_arrival(f)==(False,False))

def control_node():
 f=node(new);f.clock_ns=10_000_000_000
 f.get_clock=lambda:NS(now=lambda:NS(nanoseconds=f.clock_ns))
 f.state='running';f.pose=f.region_raw_pose.copy();f.rotation=np.eye(3);f.tracking_pose=f.pose.copy()
 f.pose_updated=f.cloud_updated=100.;f.pose_timeout=f.cloud_timeout=.3;f.raw_imu=NS(fresh=lambda *a:True)
 f.evidence=NS(error=None);f.heading_gate=NS(phase='drive',stopped=lambda *a:True,update=lambda *a:(True,0.),reset=lambda:None,resume_after_stop=lambda *a:False)
 f.apply_tilt_guard=lambda *a:False;f.segment_start=np.array([0.,0.,.3]);f.segment_started_ros=10.
 f.waypoints=np.array([g.center for g in goals]);f.samples=np.column_stack((np.linspace(0,1,101),np.zeros(101),np.full(101,.3)))
 f.max_speed=.2;f.arrival_radius=.22;f.last_reference=0.;f.last_spline_rejected=None;f.alignment_hold=False
 f.last_obstacle_check=math.inf;f.obstacle_result=(False,None,0);f.obstacle_resume_pending=False
 f.obstacle_stops=f.obstacle_resumes=f.aligned_obstacle_resumes=0;f.obstacle_hold_started_ros=None;f.obstacle_stopped_duration=0.
 f.last_actual_guard_clock_ns=f.clock_ns-100_000_000;f.cloud_stamp=f.clock_ns;f.guard_continuity_resets=0
 f.min_obstacle_clearance=None;f.pending_obstacle_event=None;f.obstacle_clear_since=None;f.active_trajectory_id=3
 f.trajectory_association=NS(reference_stamp=[10,0],reset=lambda:None);f.planning_start_state=None;f.steering=None
 f.region_arrivals=[];f.arrival_since=f.stale_since=None;f.command=[0.,0.,0.];f.last_command_time=10.;f.plans=0
 def publish(velocity=None,yaw_rate=0.):f.command=[0.,0.,0.]if velocity is None else [float(velocity[0]),float(velocity[1]),float(yaw_rate)];f.last_command_time=f.clock_ns/1e9
 f.publish_command=publish;f.publish_status=lambda:None
 def plan():f.plans+=1;f.last_reference=100.
 f.request_plan=plan
 return f
f=control_node();TeacherNavigation.control(f,100.)
check('actual_control_keeps_checked_forward_until_dwell',f.command[0]>.01 and f.waypoint_index==0)
for i in range(1,7):
 f.clock_ns=10_000_000_000+i*100_000_000;f.pose_stamp=f.clock_ns;f.pose_updated=f.cloud_updated=100.+i*.1
 TeacherNavigation.control(f,100.+i*.1)
check('actual_control_zero_and_goal_transition_only_after_dwell',f.command==[0.,0.,0.]and f.waypoint_index==1 and len(f.region_arrivals)==1)
check('new_goal_dwell_does_not_inherit_prior_goal',f.region_arrival.since is None and f.region_arrival.last_stamp==f.pose_stamp)
f.clock_ns+=100_000_000;f.pose_stamp=f.clock_ns;f.pose_updated=f.cloud_updated=100.7
TeacherNavigation.control(f,100.7)
check('replan_for_next_goal_keeps_zero_without_path',f.command==[0.,0.,0.]and f.waypoint_index==1 and f.plans==1)
for failure in ('stale','protected','obstacle','deadline','path_exhausted'):
 f=control_node()
 if failure=='stale':f.pose_updated=99.69
 if failure=='protected':f.apply_tilt_guard=lambda *a:True
 if failure=='obstacle':f.obstacle_result=(True,.6,4);f.obstacle_hold=True;f.obstacle_hold_started_ros=9.
 if failure=='deadline':f.clock_ns=100_100_000_000;f.pose_stamp=f.clock_ns;f.pose_updated=f.cloud_updated=100.
 if failure=='path_exhausted':
  f.pose=f.region_raw_pose=f.tracking_pose=np.array([.7,0,.3]);f.samples=np.array([[.6,0,.3],[.7,0,.3]])
 TeacherNavigation.control(f,100.)
 check('actual_control_'+failure+'_zero_no_region_progress',f.command==[0.,0.,0.]and f.waypoint_index==0 and len(f.region_arrivals)==0)

receipt={'schema':1,'status':'passed','checks':checks,'actual_controller_methods_on_synthetic_state':True,
 'ROS_init_called':False,'ROS_nodes_started':False,'Gazebo_started':False,'runtime_navigation':'unverified',
 'original_arrival_requirements':{'raw_control_radius_m':.17,'dwell_sim_s':.6,'max_pose_gap_s':.2,'goal_deadline_sim_s':90.},
 'source_hashes':{str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in
 [NAV/'controller.py',NAV/'scoped_profile.py',NAV/'dynamic/flat_dynamic_profile_dwell.json',ROOT.parent/'navigation/controller.py',ROOT.parent/'navigation/goal_regions.py',Path(__file__)]}}
out=Path(__file__).with_name('offline_checks.json');out.write_text(json.dumps(receipt,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'status':'passed','checks':len(checks),'receipt':str(out)}))
