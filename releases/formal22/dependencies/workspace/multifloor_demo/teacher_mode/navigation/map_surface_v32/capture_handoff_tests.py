#!/usr/bin/env python3
"""ROS-free real-production AST regression with immutable r2 SCAN173/174.

Fresh source flags are controlled fixtures, not claims of new runtime observations.
Exercises both actual shared exhausted branches and actual accepted-archive layers.
"""
import argparse,ast,copy,hashlib,json,math,sys,tempfile,time
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import path_admission_tests as base
from cascade_core import Controller
from replan_policy import low_command_is_exhausted_path
HERE=Path(__file__).resolve().parent;TOP=HERE.parent.parent
RUN=TOP/'runs/20261006_225041_closed_loop_cascade_clock_hold_V27_steady_original46_r2_ce93'
DIAG=TOP/'test_results/path_admission_v27_20261006/r2_scan_audit/R2_CAPTURE_AND_FENCE_DIAGNOSIS.json'
REPORT=dict(schema='V28_capture_handoff_production_AST/v1',simulation_launched=False,navigation_verified=False,
    freshness_fixture_scope='Current actual source receipts are mocked fresh for controlled regression. No physical replay/closed-loop pass inferred.',tests=[],bindings={})

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p):REPORT['bindings'][str(Path(p).resolve())]=sha(p);return json.loads(Path(p).read_text())
def check(name,test,**evidence):
 REPORT['tests'].append(dict(name=name,passed=bool(test),**base.replayable(evidence)))
 if not test:raise AssertionError(name+':'+str(evidence))

def exhausted_branch(which):
 tree=ast.parse((HERE/'shared_controller.py').read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='Navigation');method=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='control')
 if which=='steering_exhausted':branch=next(n for n in method.body if isinstance(n,ast.If)and ast.unparse(n.test).startswith("self.steering['exhausted']"))
 else:branch=next(n for n in method.body if isinstance(n,ast.If)and any(isinstance(c,ast.Call)and isinstance(c.func,ast.Name)and c.func.id=='low_command_is_exhausted_path'for c in ast.walk(n.test)))
 f=ast.FunctionDef(name='execute',args=ast.arguments(posonlyargs=[],args=[ast.arg(arg=k)for k in ['self','now','ros_now','velocity','yaw_rate','goal']],kwonlyargs=[],kw_defaults=[],defaults=[]),body=[copy.deepcopy(branch)],decorator_list=[])
 ns=dict(np=np,low_command_is_exhausted_path=low_command_is_exhausted_path,trajectory_has_progress=base.trajectory_has_progress)
 exec(compile(ast.fix_missing_locations(ast.Module(body=[f],type_ignores=[])),str(HERE/'shared_controller.py'),'exec'),ns);return ns['execute']

def fixed_state(c):
 return dict(goal_record=copy.deepcopy(c.cascade.goal_record),goal_sha=c.cascade.goal_sha,
  segment_started_ros=c.segment_started_ros,segment_started_ros_ns=c.segment_started_ros_ns,
  segment_start=c.segment_start.tolist(),velocity_PI=c.cascade.velocity_integral.tolist(),
  filtered=c.cascade.filtered.tolist(),inner_filtered=c.cascade.inner_filtered.tolist(),command=c.cascade.command.tolist(),
  last_math_stamp=c.cascade.last_math_stamp,last_math_wall=c.cascade.last_math_wall,
  waypoint_index=c.waypoint_index,request_id=c.request_id,goals_definition_sha256=c.goals_definition_sha256)

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--output',type=Path,required=True);a=ap.parse_args();output=a.output.resolve();output.parent.mkdir(parents=True,exist_ok=True)
 try:
  old=load(RUN/'navigation_trajectories/000160_trajectory_173.json');new=load(RUN/'path_admission_candidates/000172_trajectory_174.json');diag=load(DIAG)
  profile=load(HERE/'profiles/repair_original46.json');original_profile=load(RUN/'navigation_profile.json');anchor=load(RUN/'navigation_anchor.json')
  oldarray=RUN/'navigation_trajectories'/old['array_file'];assert sha(oldarray)==old['array_sha256'];REPORT['bindings'][str(oldarray.resolve())]=sha(oldarray)
  check('profile_controller_math_and_geometry_unchanged',all(profile[k]==original_profile[k]for k in ['cascade','pid','original_scenario','registered_route_fence','pose_cloud_timeout_s','teacher_transition','arrival_stop_policy','max_speed_mps','mission46_final_hold_sim_s']))
  check('shared_only_two_exhaustion_branches_mark', (HERE/'shared_controller.py').read_text().count("self.mark_execution_path_exhausted(")==2)
  cls=base.production_class();branches={k:exhausted_branch(k)for k in ['steering_exhausted','low_command_exhausted_path']}
  with tempfile.TemporaryDirectory(prefix='V28_capture_AST_')as temp:
   parent=Path(temp);counter=[0]
   def setup():
    counter[0]+=1;run=parent/str(counter[0]);run.mkdir();a0=copy.deepcopy(anchor);a0['run_dir']=str(run)
    (run/'navigation_anchor.json').write_text(json.dumps(a0)+'\n');(run/'navigation_scene_axis_registration.json').write_bytes((RUN/'navigation_scene_axis_registration.json').read_bytes())
    c=base.setup_controller(cls,run,profile,old);c.waypoint_index=16;c.waypoints=np.tile(np.asarray(old['metadata']['body_goal']),(18,1));c.goals=[NS(legacy=False,timeout_sim_s=90)]*18
    c.pose_stamp=old['accepted_slam_stamp_ns'];c.get_clock=lambda:NS(now=lambda:NS(nanoseconds=new['decision']['control_stamp_ns']))
    c.pose=np.asarray(old['metadata']['trajectory']['pos_pts'][0]);c.execution_path_exhaustion=None
    c.accept_spline(base.message(old['metadata']['trajectory']),old['metadata']);assert c.active_trajectory_id==173 and c.samples is not None
    with np.load(oldarray,allow_pickle=False)as z:assert np.array_equal(c.samples,z['samples'])
    c.pose=np.asarray(diag['actual_pose_at_first_capture_rejection']['position']);c.pose_stamp=new['decision']['source_pose_stamp_ns']
    c.trajectory_association.reference_stamp=tuple(new['metadata']['reference_stamp']);c.trajectory_association.accepted_highwater_id=174
    c.segment_start=np.array([4.,7.,2.4]);c.segment_started_ros=310.;c.segment_started_ros_ns=310_000_000_000
    c.goals_definition_sha256=diag['goal17_geometry']['goal'].get('goal_id') # immutable sentinel, not a fabricated runtime hash
    c.cascade=Controller(profile['cascade'],c.waypoints[16].tolist(),3.13,'actual_same_goal_fixture')
    r=c.path_receipt;c.cascade.set_path(r['points_xyz'],r['path_id'],r['stamp_ns'],r['received_wall_ns'],position=c.pose)
    c.cascade_key=(c.request_id,16);c.cascade_path_id=r['path_id'];c.cascade.protected=False;c.cascade.failure_latched=None
    c.cascade.capture_stamp=331584999999;c.cascade.capture_wall=time.monotonic_ns()-12_000_000_000;c.cascade.capture_clock=331600000000
    c.cascade.arrival={'pose_stamp_ns':331584999999};c.cascade.capture_dwell=331584999999;c.cascade.hold_stamp=331684999999;c.cascade.hold_wall=11;c.cascade.hold_clock=331700000000
    c.cascade.velocity_integral[:]=[.021,-.012,.034];c.cascade.hold_integral[:]=[.01,.02,.03]
    c.cascade.filtered=np.array([.03,.02,.01]);c.cascade.inner_filtered=np.array([.05,.04,.03]);c.cascade.command[:]=[.1,.02,.01];c.cascade.last_math_stamp=342400000000;c.cascade.last_math_wall=123
    c.steering={'exhausted':True};c.alignment_hold=False;c.cascade_last_row={'mode':'path_end_hold'};c.last_reference=time.monotonic()-4.;c.request_count=0
    c.request_plan=lambda:setattr(c,'request_count',c.request_count+1);c.pose_updated=c.cloud_updated=c.bridge_updated=time.monotonic()
    return c
   def mark(c,which='steering_exhausted'):
    c.pose_updated=c.cloud_updated=c.bridge_updated=time.monotonic()
    branches[which](c,time.monotonic(),new['decision']['control_stamp_ns']/1e9,np.zeros(2),0.,c.waypoints[16])
    assert c.samples is None and c.execution_path_exhaustion is not None and c.request_count==1
   def accept(c):c.accept_spline(base.message(new['payload']),new['metadata']);return c.path_admission_last_decision
   c=setup();before=base.snapshot(c);fixed=fixed_state(c);capture=c.cascade.capture_stamp;decision=accept(c)
   check('actual174_capture_old_samples_kept_certified',base.unchanged(before,base.snapshot(c))and fixed==fixed_state(c)and c.cascade.capture_stamp==capture and not decision['admitted']and decision['old_path_retention']['allowed']and decision['reason']=='terminal_capture_kept_checked_path',decision=decision)
   for which in branches:
    c=setup();fixed=fixed_state(c);mark(c,which);marker=copy.deepcopy(c.execution_path_exhaustion);decision=accept(c)
    check('actual174_'+which+'_safe_committed_recovery',decision['admitted']and c.active_trajectory_id==174 and c.samples is not None and c.cascade.capture_stamp is None and c.execution_path_exhaustion is None and c.accepted_archives==[173,174]and not c.path_admission_hold,decision=decision,marker=marker)
    check(which+'_fixed_goal_timeout_PI_filters_preserved',fixed==fixed_state(c)and c.cascade.hold_stamp is None and c.cascade.capture_dwell is None and np.all(c.cascade.hold_integral==0),fixed_before=fixed,fixed_after=fixed_state(c))
    c.ensure_cascade()
    check(which+'_actual_cascade_rebinds_new_active_reference',c.cascade.path_id==c.path_receipt['path_id']and c.cascade_path_id==c.path_receipt['path_id']and c.cascade.path_record['stamp_ns']==new['metadata']['reference_stamp'][0]*1_000_000_000+new['metadata']['reference_stamp'][1]and c.active_reference_stamp==tuple(new['metadata']['reference_stamp']),path_record=c.cascade.path_record)
   cases={
    'missing_marker':lambda c:setattr(c,'execution_path_exhaustion',None),
    'spoof_marker_schema':lambda c:c.execution_path_exhaustion.update(schema='external'),
    'spoof_marker_source':lambda c:c.execution_path_exhaustion.update(source='planner_payload'),
    'marker_old_request':lambda c:c.execution_path_exhaustion.update(request_id='other'),
    'marker_old_waypoint':lambda c:c.execution_path_exhaustion.update(waypoint_index=15),
    'marker_changed_goal':lambda c:c.execution_path_exhaustion.update(goal_xyz=[0,0,0]),
    'marker_old_active_id':lambda c:c.execution_path_exhaustion.update(active_trajectory_id=172),
    'marker_old_reference':lambda c:c.execution_path_exhaustion.update(active_reference_stamp=[1,2]),
    'marker_bad_samples_sha':lambda c:c.execution_path_exhaustion.update(previous_samples_sha256='0'*64),
    'marker_bad_samples_count':lambda c:c.execution_path_exhaustion.update(previous_samples_count=1),
    'marker_future_clock':lambda c:c.execution_path_exhaustion.update(control_stamp_ns=2**60),
    'marker_zero_not_exhaustion':lambda c:c.execution_path_exhaustion.update(reason='command_zero'),
    'stale_pose':lambda c:setattr(c,'pose_updated',time.monotonic()-.301),
    'stale_cloud':lambda c:setattr(c,'cloud_updated',time.monotonic()-.301),
    'stale_bridge':lambda c:setattr(c,'bridge_updated',time.monotonic()-.301),
    'stale_raw_imu':lambda c:setattr(c,'raw_imu',NS(fresh=lambda now,ros:False)),
    'bridge_hold':lambda c:c.bridge_safety.update(state='hold'),
    'obstacle_hold':lambda c:setattr(c,'obstacle_hold',True),
    'tilt_hold':lambda c:setattr(c,'tilt_hold',True),
    'cascade_protected':lambda c:setattr(c.cascade,'protected',True),
    'cascade_fatal':lambda c:setattr(c.cascade,'failure_latched','test'),
    'cross_request_active':lambda c:setattr(c,'request_id','other'),
    'cross_waypoint_active':lambda c:setattr(c,'waypoint_index',15),
    'cross_goal_active':lambda c:c.path_admission_active['identity'].update(goal_xyz=[0,0,0])}
   for name,mutate in cases.items():
    c=setup();mark(c);mutate(c);before=base.snapshot(c);capture=c.cascade.capture_stamp;decision=accept(c)
    check(name+'_cannot_clear_capture_or_commit',not decision['admitted']and c.cascade.capture_stamp==capture and base.unchanged(before,base.snapshot(c))and c.path_admission_hold and c.zero_count>0,decision=decision)
   c=setup();c.samples=None;before=base.snapshot(c);decision=accept(c)
   check('samples_absent_without_actual_exhaustion_rejected',not decision['admitted']and base.unchanged(before,base.snapshot(c))and c.cascade.capture_stamp is not None,decision=decision)
   # Force the actual lower shared cubic dimension gate to refuse after the
   # top proof, testing the production failure branch, not a copied guard.
   c=setup();mark(c);before=base.snapshot(c);capture=c.cascade.capture_stamp
   original=base.FakeBase.accept_spline;called=[]
   try:
    def reject_actual_lower(self,msg,metadata):
     called.append(msg.traj_id);msg.order=2;return original(self,msg,metadata)
    base.FakeBase.accept_spline=reject_actual_lower
    decision=accept(c)
   finally:base.FakeBase.accept_spline=original
   check('super_execution_gate_rejection_does_not_clear_capture',called==[174]and not decision['admitted']and decision['reason']=='existing_execution_gate_rejected'and c.cascade.capture_stamp==capture and base.unchanged({**before,'super_calls':c.super_calls},base.snapshot(c)),decision=decision)
   # Actual shared degenerate rejection additionally clears active identity;
   # recovery transaction restores the previous identity/marker and stays zero.
   c=setup();mark(c);before=base.snapshot(c);capture=c.cascade.capture_stamp;layer=base.actual_acceptance_layers();shared_global=layer.__mro__[1].accept_spline.__globals__
   # FakeBase was constructed from a distinct AST class; patch its actual parent.
   shared_global=base.FakeBase.__mro__[2].accept_spline.__globals__;progress=shared_global['trajectory_has_progress']
   try:
    shared_global['trajectory_has_progress']=lambda *args:False
    decision=accept(c)
   finally:shared_global['trajectory_has_progress']=progress
   check('actual_lower_degenerate_gate_restores_old_identity_and_marker',not decision['admitted']and c.cascade.capture_stamp==capture and c.execution_path_exhaustion is not None and base.unchanged({**before,'super_calls':c.super_calls},base.snapshot(c)),decision=decision)
   c=setup();mark(c);v=copy.deepcopy(new['payload']);v['pos_pts']=[[5e307]*3]*len(v['pos_pts']);m=copy.deepcopy(new['metadata']);m['trajectory']=v;before=base.snapshot(c);capture=c.cascade.capture_stamp
   c.accept_spline(base.message(v),m);decision=c.path_admission_last_decision
   check('invalid_new_proof_does_not_recover_capture',not decision['admitted']and c.cascade.capture_stamp==capture and base.unchanged(before,base.snapshot(c)),decision=decision)
   c=setup();c.cascade_last_row={'mode':'reference_constraint_hold'};branches['low_command_exhausted_path'](c,time.monotonic(),342.47,np.zeros(2),0.,c.waypoints[16])
   check('protected_low_command_never_marked_exhausted',c.samples is not None and c.execution_path_exhaustion is None and c.request_count==0)
   # Force elapsed callback wall time across100ms after eligibility, while
   # real source predicates are still valid inside the original300ms TTL.
   c=setup();mark(c);c.pose_updated=c.cloud_updated=c.bridge_updated=0.;before=base.snapshot(c);capture=c.cascade.capture_stamp
   globals_=cls.accept_spline.__globals__;real_time=globals_['time'];clock_values=iter([0.,.05,.08,.09,.101,.102,.103,.104])
   globals_['time']=NS(monotonic=lambda:next(clock_values,.105),monotonic_ns=lambda:105_000_000)
   try:decision=accept(c)
   finally:globals_['time']=real_time
   check('recovery_callback_100ms_budget_remains_failclosed',not decision['admitted']and decision['proof']['reason']=='admission_callback_budget_exhausted'and c.cascade.capture_stamp==capture and base.unchanged(before,base.snapshot(c)),decision=decision)
   # All ordinary samples-None assignments must invalidate, not create markers.
   tree=ast.parse((HERE/'shared_controller.py').read_text());ordinary=[]
   for method in [n for cl in tree.body if isinstance(cl,ast.ClassDef)for n in cl.body if isinstance(n,ast.FunctionDef)]:
    for parentnode in ast.walk(method):
     for attr in ['body','orelse','finalbody']:
      seq=getattr(parentnode,attr,[])
      if not isinstance(seq,list):continue
      for i,node in enumerate(seq):
       if isinstance(node,ast.Assign)and ast.unparse(node)=='self.samples = None':
        prior=ast.unparse(seq[i-1])if i else'';after=ast.unparse(seq[i+1])if i+1<len(seq)else''
        if 'mark_execution_path_exhausted' not in prior:ordinary.append(dict(method=method.name,line=node.lineno,cleared=after=='self.execution_path_exhaustion = None'))
   check('all_nonexhaustion_clears_invalidate_receipt',len(ordinary)==7 and all(x['cleared']for x in ordinary),assignments=ordinary)
  REPORT['production_source_sha256']={name:sha(HERE/name)for name in ['controller.py','shared_controller.py','path_admission.py','cascade_core.py','teacher_wrapper.py','capture_handoff_tests.py','path_admission_tests.py']}
  REPORT['passed']=True
 except Exception as error:REPORT.update(passed=False,error=type(error).__name__+':'+str(error))
 output.write_text(json.dumps(REPORT,ensure_ascii=False,allow_nan=False,indent=2)+'\n');print(json.dumps(dict(passed=REPORT['passed'],checks=len(REPORT['tests']),error=REPORT.get('error'),output=str(output))))
 if not REPORT['passed']:raise SystemExit(1)
if __name__=='__main__':main()
