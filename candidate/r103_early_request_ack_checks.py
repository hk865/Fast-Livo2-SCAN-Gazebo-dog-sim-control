"""Finite real-method OFF/ON replay of saved early receipts; no ROS or DDS."""
import ast
import copy
import gc
import hashlib
import json
import os
from pathlib import Path
import statistics
import subprocess
import sys
import time
from types import SimpleNamespace as NS
from unittest.mock import patch

HERE=Path(__file__).resolve().parent
DEMO=Path('/home/hyh001/projects/1.Project/Ros2_fastlivo2_/multifloor_demo/development/r86_feedback_isolation/navigation_acceptance_round27_20261024/worktrees/published_formal22_repro/releases/formal22/dependencies/workspace/multifloor_demo')
sys.path[:0]=[str(DEMO/'navigation'),str(DEMO/'teacher_mode/navigation')]
import numpy as np
import path_handoff as handoff
import automatic_reference_runtime as auto
import pending_path_commit as pending
from mode3_trajectory_contract import R85TrajectoryAssociation
from trajectory_contract import TrajectoryAssociation

BASE='d72dbaeca14e1ad1576b48d3e07e11215515f847'
RUN=Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261023T105532Z_closed_loop_R85_R103_imu_round26_004f')
OUT=HERE.parent.parent.parent/'handoff'
INPUTS={}
def read(name,lines=False):
 p=RUN/name;raw=p.read_bytes();INPUTS[str(p)]=dict(bytes=len(raw),sha256=hashlib.sha256(raw).hexdigest())
 return [json.loads(x) for x in raw.splitlines()] if lines else json.loads(raw)
EARLY=read('path_handoff_early_requests.jsonl',True)
LOCAL=read('r85_local_replan_requests.jsonl',True)
REFERENCE=read('r85_reference_publications.jsonl',True)[0]
PROFILE=read('navigation_scope.json')['profile']
DOC=read(PROFILE['r85_auto_reference_contract'].get('reference_file','r85_global_reference.json'))
STATUS=read('navigation_status.json')
REQUEST=STATUS['request_id']
checks=[]
def check(name,condition,**evidence):
 assert condition,(name,evidence)
 checks.append(dict(name=name,passed=True,**evidence))


def method(path,cls,name,source=None):
 tree=ast.parse(source if source is not None else path.read_text())
 c=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name==cls)
 return copy.deepcopy(next(x for x in c.body if isinstance(x,ast.FunctionDef) and x.name==name))


def owner_type():
 shared=method(HERE/'shared_controller.py','Navigation','request_plan')
 pid=method(HERE/'controller.py','PIDNavigation','request_plan')
 known=method(HERE/'known_scene_inputs.py','KnownSceneInputMixin','request_plan')
 retry=method(HERE/'controller.py','PIDNavigation','request_admission_replan_if_due')
 classes=[ast.ClassDef(name='Shared',bases=[],keywords=[],body=[shared],decorator_list=[]),
  ast.ClassDef(name='PID',bases=[ast.Name(id='Shared',ctx=ast.Load())],keywords=[],body=[pid,retry],decorator_list=[]),
  ast.ClassDef(name='Owner',bases=[ast.Name(id='PID',ctx=ast.Load())],keywords=[],body=[known],decorator_list=[])]
 scope=dict(time=time,pending_path_waiting=pending.waiting)
 exec(compile(ast.fix_missing_locations(ast.Module(body=classes,type_ignores=[])),str(HERE/'controller.py'),'exec'),scope)
 return scope['Owner']
Owner=owner_type()


class Evidence:
 def __init__(self):self.rows=[];self.fail_local_once=False
 def append(self,path,value):
  if path.name=='r85_local_replan_requests.jsonl' and self.fail_local_once:
   self.fail_local_once=False;raise ValueError('fixture_after_publish_evidence_failure')
  self.rows.append((path.name,copy.deepcopy(value)))


class Publisher:
 def __init__(self):self.messages=[];self.fail_once=False;self.subscribers=1
 def get_subscription_count(self):return self.subscribers
 def publish(self,msg):
  if self.fail_once:self.fail_once=False;raise ValueError('fixture_publish_failure')
  self.messages.append(copy.deepcopy(msg))


def owner():
 n=Owner();n.run=Path('/tmp/r103_early_request_ack_fixture_no_files')
 a=R85TrajectoryAssociation(n.run.name);a.configure_epoch(REFERENCE['planner_epoch'],REFERENCE['planner_process_id'],REFERENCE['localization_generation'])
 a.request(REFERENCE['sent_reference']['stamp'],REFERENCE['planner_body_goal'],REFERENCE['sent_reference'])
 a.reference_revision=1;a.accepted={'fixture_existing_checked_trajectory':True}
 n.trajectory_association=a;n.profile=PROFILE;n.reference_requests=1;n.last_reference=0.;n.last_reference_stamp=14_730_000_000
 n.reference_pub=Publisher();n.local_replan_pub=Publisher();n.evidence=Evidence();n.zero_count=0
 n.publish_command=lambda:setattr(n,'zero_count',n.zero_count+1)
 n.pose=np.asarray(REFERENCE['actual_connector_certificate']['actual_start_body_xyz'])
 n.pose_stamp=27_560_000_000;n.cloud_stamp=n.pose_stamp;n.clock_ns=n.pose_stamp+50_000_000
 n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=n.clock_ns))
 n.ready=True;n._known_generation_ready=lambda:n.ready
 n.known_scene_inputs={'generation':1};n.known_scene_reference_generation=1;n.scan_reference_coverage_index=None
 n.known_scene_scan_epoch=dict(generation=1,planner_epoch=REFERENCE['planner_epoch'],planner_identity={'pid':REFERENCE['planner_process_id']},run_id=n.run.name,ready=True)
 n.state='running';n.pending_path_commit=None;n.heading_gate=NS(phase='drive')
 n.request_id=REQUEST;n.waypoint_index=0;n.continuous_route=None;n.cascade_key=(REQUEST,0)
 n.cascade=NS(capture_stamp=None,path_id=EARLY[0]['path_id'],spatial_config={'arc_length_m':.2})
 n.path_receipt={'path_id':n.cascade.path_id,'trajectory_id':3};n.active_trajectory_id=3
 n.path_admission_active={'correction_generation':1,'ordered_coverage_index':None}
 n.path_execution_eligibility=lambda:{'conditions':{'fixture_original_execution_allowed':True}}
 n.path_handoff_early_requested_for=None;n.path_handoff_last_check=None
 n.path_admission_hold=True;n.path_admission_replan_pending=True;n.reference_replan_pending=True
 n.path_admission_last_decision={'reason':'fixture_checked_path_end_hold'};n.global_reference_settings={'enabled':False}
 n.admission_identity=lambda:{'request_id':n.request_id,'waypoint':n.waypoint_index}
 n.planner_goal=lambda:DOC['body_points'][-1];n.planner_reference_points=lambda goal:[goal]
 def make_path(points,height_offset):
  stamp=NS(sec=n.clock_ns//1_000_000_000,nanosec=n.clock_ns%1_000_000_000)
  return NS(header=NS(frame_id='camera_init',stamp=stamp),poses=[NS(header=NS(frame_id='camera_init',stamp=copy.deepcopy(stamp)),pose=NS(position=NS(x=p[0],y=p[1],z=p[2]-height_offset),orientation=NS(x=0.,y=0.,z=0.,w=1.))) for p in points])
 n.make_path=make_path;n.stamp=lambda msg:msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
 n.make_local_replan_message=lambda epoch,revision,seq,stamp:dict(epoch=epoch,revision=revision,sequence=seq,stamp=stamp)
 r=auto.AutomaticReferenceRuntime(n.run,PROFILE,a);r.scope=(REFERENCE['planner_epoch'],REFERENCE['planner_process_id'],1)
 r.reference=copy.deepcopy(REFERENCE);r.file_sha=REFERENCE['reference_file_sha256'];r.load=lambda *args:DOC
 n.automatic_reference=r
 return n


def call(n,row=None):
 row=EARLY[0] if row is None else row
 n.pose_stamp=row['source_pose_stamp_ns'];n.clock_ns=n.pose_stamp+50_000_000
 n.cascade.path_id=row['path_id'];n.cascade.spatial_config['arc_length_m']=row['original_preview_arc_m']
 n.path_receipt={'path_id':row['path_id'],'trajectory_id':int(row['path_id'].rsplit(':',1)[1])};n.active_trajectory_id=n.path_receipt['trajectory_id']
 return handoff.early_request(n,dict(remaining_m=row['remaining_m'],endpoint_is_fixed_goal=False))


def switches(value):
 return patch.multiple(handoff,EARLY_REQUEST_ACK_ENABLED=value),patch.multiple(auto,EARLY_REQUEST_ACK_ENABLED=value)


# Baseline functions independently extracted, to establish exact OFF parity.
old_handoff=subprocess.check_output(['git','show',BASE+':candidate/path_handoff.py'],cwd=HERE.parent,text=True)
old_auto=subprocess.check_output(['git','show',BASE+':candidate/automatic_reference_runtime.py'],cwd=HERE.parent,text=True)
old_func=next(x for x in ast.parse(old_handoff).body if isinstance(x,ast.FunctionDef) and x.name=='early_request')
ns=dict(context=handoff.context);exec(compile(ast.Module(body=[old_func],type_ignores=[]),'baseline_early_request','exec'),ns)
old_request=method(HERE/'automatic_reference_runtime.py','AutomaticReferenceRuntime','request_plan',old_auto)
nsauto=dict(auto.__dict__);exec(compile(ast.Module(body=[old_request],type_ignores=[]),'baseline_automatic_request','exec'),nsauto)
replays={};gc_before=(gc.isenabled(),gc.get_threshold())
for label,value in [('baseline',False),('OFF',False),('ON',True)]:
 n=owner();samples=[];returns=[]
 p1,p2=switches(value)
 with p1,p2:
  for row in EARLY:
   if label=='baseline':
    n.automatic_reference.request_plan=nsauto['request_plan'].__get__(n.automatic_reference)
    # Reuse the same recorded-key/source setup, then invoke the old function.
    saved=handoff.early_request;handoff.early_request=ns['early_request']
   start,cpu=time.monotonic_ns(),time.thread_time_ns()
   try:receipt=call(n,row)
   finally:
    if label=='baseline':handoff.early_request=saved
   samples.append((time.monotonic_ns()-start,time.thread_time_ns()-cpu))
   returns.append(receipt)
 replays[label]=dict(input_events=len(EARLY),local_publications=len(n.local_replan_pub.messages),global_reference_requests=n.reference_requests,
  receipt_count=sum(x is not None for x in returns),requested_true=sum(x is not None and x['requested'] is True for x in returns),
  receipts=returns,median_wall_ns=statistics.median(x[0] for x in samples),median_thread_cpu_ns=statistics.median(x[1] for x in samples),
  total_wall_ns=sum(x[0] for x in samples),total_thread_cpu_ns=sum(x[1] for x in samples))
check('same_actual_614_sequence_baseline_OFF_parity',replays['baseline']['receipts']==replays['OFF']['receipts']==EARLY,local_publications=replays['OFF']['local_publications'])
check('same_actual_614_sequence_ON_one_success_per_actual_key',replays['ON']['local_publications']==3 and replays['ON']['requested_true']==3 and replays['ON']['receipt_count']==3 and replays['ON']['global_reference_requests']==1,
 actual_distinct_paths=len(set(r['path_id'] for r in EARLY)),trajectory36_input_count=sum(r['path_id'].endswith(':36') for r in EARLY))

p1,p2=switches(True)
with p1,p2:
 # Use the real catch/return chain. Sequence advances before publish; no false ACK.
 for phase in ('publish','after_publish_note','after_publish_evidence'):
  n=owner()
  note=n.trajectory_association.note_local_replan_request
  if phase=='publish':n.local_replan_pub.fail_once=True
  elif phase=='after_publish_note':
   n.trajectory_association.note_local_replan_request=lambda seq:(_ for _ in ()).throw(ValueError('fixture_note_failure'))
  else:n.evidence.fail_local_once=True
  first=call(n);seq_after=n.automatic_reference.replan_sequence
  check(phase+'_failure_not_latched',first['requested'] is False and n.path_handoff_early_requested_for is None and seq_after==1)
  n.trajectory_association.note_local_replan_request=note
  second=call(n)
  check(phase+'_next_tick_retries_successfully',second['requested'] is True and n.automatic_reference.replan_sequence==2 and n.reference_requests==1)
 # Real revision waiting path, using a protocol-disabled fixture to yield None.
 n=owner();n.trajectory_association.reference_revision=None;n.automatic_reference.profile={}
 first=call(n)
 check('revision_wait_no_publication_or_latch',first['requested'] is False and not n.local_replan_pub.messages and n.path_handoff_early_requested_for is None)
 n.trajectory_association.reference_revision=1
 check('revision_ready_retry_succeeds',call(n)['requested'] is True)
 # Real pending context and higher-layer generation readiness both stay retryable.
 n=owner();n.pending_path_commit={'context':pending.context(n)}
 check('pending_wait_no_request_or_latch',call(n) is None and not n.local_replan_pub.messages and n.path_handoff_early_requested_for is None)
 n.pending_path_commit=None;n.ready=False
 check('generation_wait_no_request_or_latch',call(n) is None and not n.local_replan_pub.messages)
 n.ready=True;check('waiting_cleared_success',call(n)['requested'] is True)
 # Independent normal retry still runs after successful early-request one-shot.
 n=owner();call(n);before=len(n.local_replan_pub.messages)
 n.request_admission_replan_if_due(n.last_reference+2.99)
 check('ordinary_retry_original_3s_gate_retained',len(n.local_replan_pub.messages)==before)
 n.request_admission_replan_if_due(n.last_reference+3.01)
 check('ordinary_retry_allowed_after_early_latch',len(n.local_replan_pub.messages)==before+1 and n.reference_requests==1 and n.path_admission_replan_pending is True)
 # Generation key changes allow a new request once original readiness is valid.
 n.known_scene_inputs['generation']=2;n.path_admission_active['correction_generation']=2;n.known_scene_scan_epoch['generation']=2
 n.known_scene_reference_generation=2;n.trajectory_association.localization_generation=2;n.automatic_reference.scope=(REFERENCE['planner_epoch'],REFERENCE['planner_process_id'],2)
 check('new_generation_allows_new_early_request',call(n)['requested'] is True)
 # Ordinary publisher returns None and only increments the existing global counter.
 n=owner();n.automatic_reference=None;n.trajectory_association=TrajectoryAssociation()
 check('ordinary_None_plus_global_counter_compatible',call(n)['requested'] is True and n.reference_requests==2 and len(n.reference_pub.messages)==1)
 # Actual automatic first-global branch still returns True after complete work.
 n=owner();n.automatic_reference.reference=None
 n.automatic_reference._physical_connector=lambda *args:dict(actual_start_body_xyz=n.pose.tolist())
 check('automatic_first_global_success_True',n.request_plan() is True and n.reference_requests==2 and len(n.reference_pub.messages)==1)
 # Failure after global counter increment must not acknowledge automatic completion.
 n=owner();n.automatic_reference.reference=None
 n.automatic_reference._physical_connector=lambda *args:dict(actual_start_body_xyz=n.pose.tolist())
 append=n.evidence.append
 def fail_global(path,row):
  if path.name=='r85_reference_publications.jsonl':raise ValueError('fixture_global_evidence_failure')
  return append(path,row)
 n.evidence.append=fail_global
 check('automatic_global_post_counter_failure_not_latched',call(n)['requested'] is False and n.reference_requests==2 and n.path_handoff_early_requested_for is None)

# Actual source gates and ordinary retry implementation are unmodified.
check('ordinary_retry_AST_identical',ast.dump(method(HERE/'controller.py','PIDNavigation','request_admission_replan_if_due'))==ast.dump(method(HERE/'controller.py','PIDNavigation','request_admission_replan_if_due',subprocess.check_output(['git','show',BASE+':candidate/controller.py'],cwd=HERE.parent,text=True))))
check('shared_controller_entire_file_identical', (HERE/'shared_controller.py').read_bytes()==subprocess.check_output(['git','show',BASE+':candidate/shared_controller.py'],cwd=HERE.parent))
check('natural_GC_policy_unchanged',gc_before==(gc.isenabled(),gc.get_threshold()))
for value,expected in [(None,False),('0',False),('1',True),('invalid','rejected')]:
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1');env.pop('R103_EARLY_REQUEST_ACK',None)
 if value is not None:env['R103_EARLY_REQUEST_ACK']=value
 code="import sys;sys.path[:0]="+repr(sys.path[:2])+";import path_handoff as p,automatic_reference_runtime as a,r103_early_request_ack as selector,os;before=(p.EARLY_REQUEST_ACK_ENABLED,a.EARLY_REQUEST_ACK_ENABLED,selector.enabled());os.environ['R103_EARLY_REQUEST_ACK']='0' if before[0] else '1';assert before==(p.EARLY_REQUEST_ACK_ENABLED,a.EARLY_REQUEST_ACK_ENABLED,selector.enabled());print(p.EARLY_REQUEST_ACK_ENABLED,a.EARLY_REQUEST_ACK_ENABLED)"
 proc=subprocess.run([sys.executable,'-c',code],cwd=HERE,env=env,capture_output=True,text=True)
 check('process_fixed_selector_'+str(value),(proc.returncode!=0 if expected=='rejected' else proc.returncode==0 and proc.stdout.strip()==f'{expected} {expected}'))

# Keep scalar summaries; exact per-event receipts form the replay evidence.
result=dict(baseline=BASE,checks=checks,replays=replays,inputs=INPUTS,
 source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [HERE/'path_handoff.py',HERE/'automatic_reference_runtime.py',HERE/'r103_early_request_ack.py',Path(__file__)]},
 limitations=['Recorded early receipt order, path ids, remaining distances, preview arcs and source stamps are real inputs.',
 'Known generation1, waypoint0 and no continuous ledger are taken from saved run context; unrecorded gates/ROS delivery are controlled fixtures.',
 'Real AutomaticReferenceRuntime and R85TrajectoryAssociation plus exact Shared/PID/Known request methods are exercised; source-file load and ROS publishers are fixtures.',
 'This is a contract replay, not a reconstruction of unknown system state, DDS, planner output, command timing or physical motion.',
 'Timing is finite fixture work, not physical navigation benefit. Natural GC unchanged; no collect.'],
 simulation_or_ROS_started=False,protected_modules_changed=False)
OUT.mkdir(parents=True,exist_ok=True);(OUT/'CHECK_RESULTS.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(checks=len(checks),replays={k:{x:y for x,y in v.items() if x!='receipts'} for k,v in replays.items()}),indent=2))
