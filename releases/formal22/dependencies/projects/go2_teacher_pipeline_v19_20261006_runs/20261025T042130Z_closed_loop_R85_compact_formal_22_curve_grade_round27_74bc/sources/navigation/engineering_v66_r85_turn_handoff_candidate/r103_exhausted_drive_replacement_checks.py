"""Finite saved-data contract projection; no ROS, source refresh or DDS replay."""
import ast,copy,gc,hashlib,json,math,os,statistics,subprocess,sys,time
from contextlib import nullcontext
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from scipy.interpolate import BSpline
HERE=Path(__file__).resolve().parent
DEMO=Path('/opt/experiment/workspace/multifloor_demo')
sys.path.extend([str(DEMO/'navigation'),str(DEMO/'teacher_mode/navigation')])
import path_handoff as handoff
from global_route_reference import StableProjectionController
from path_commit import staged_cascade_binding
from control_core import trajectory_has_progress
from handoff_stop_policy import tail_stop_request
BASE='95c60f0836c65123585d54e33116ee74fedd319a'
RUN=Path('/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/20261023T112551Z_closed_loop_R85_R103_handoff_round26_5537')
OUT=HERE.parent.parent.parent/'replacement'
inputs={};checks=[]
def read(name,lines=False):
 p=RUN/name;raw=p.read_bytes();inputs[name]=dict(sha256=hashlib.sha256(raw).hexdigest(),bytes=len(raw))
 return [json.loads(x) for x in raw.splitlines()] if lines else json.loads(raw)
def check(name,value):
 assert value,name
 checks.append(dict(name=name,passed=True))
receipt=next(x for x in read('cascade_path_history.jsonl',True) if x['trajectory_id']==27)
accepted=read('path_admission_candidates/000026_trajectory_27.json')
rejected=read('path_admission_candidates/000027_trajectory_28.json')
profile=read('navigation_scope.json')['profile']
array=RUN/receipt['array_file'];inputs[receipt['array_file']]=dict(sha256=hashlib.sha256(array.read_bytes()).hexdigest(),bytes=array.stat().st_size)
with np.load(array,allow_pickle=False) as z:samples=z['samples'].copy()
check('actual_27_samples_match_accepted_receipt',len(samples)==receipt['source_samples_count'] and hashlib.sha256(np.asarray(samples,dtype='<f8').tobytes()).hexdigest()==receipt['source_samples_float64_sha256'])
def method(file,cls,name):
 tree=ast.parse((HERE/file).read_text());c=next(x for x in tree.body if isinstance(x,ast.ClassDef) and x.name==cls)
 f=copy.deepcopy(next(x for x in c.body if isinstance(x,ast.FunctionDef) and x.name==name));f.decorator_list=[];return f
mark=method('shared_controller.py','Navigation','mark_execution_path_exhausted')
scope=dict(np=np,hashlib=hashlib)
exec(compile(ast.fix_missing_locations(ast.Module(body=[mark],type_ignores=[])),'real_mark','exec'),scope)
mark=scope['mark_execution_path_exhausted']
baseline={};exec(compile(subprocess.check_output(['git','show',BASE+':candidate/path_handoff.py'],text=True),'<baseline_path_handoff>','exec'),baseline)
value=rejected['payload'];metadata=rejected['metadata']
new_samples=BSpline(value['knots'],value['pos_pts'],3)(np.linspace(value['knots'][3],value['knots'][-4],min(2000,max(2,int((value['knots'][-4]-value['knots'][3])/.08)+1))))
recorded_conditions=rejected['decision']['handoff']['eligibility']['conditions']
class Owner:
 def admission_identity(self):return dict(request_id=self.request_id,waypoint_index=self.waypoint_index,goal_xyz=self.waypoints[0].tolist())
 def _known_generation_ready(self):return True
 def path_execution_eligibility(self):
  self.execution_checks+=1
  c={k:v for k,v in recorded_conditions.items() if k not in ('source_generation_ready','same_correction_generation','same_coverage','same_cascade_goal','receipt_present','same_path','same_trajectory')}
  c.update(self.source_overrides);c['samples_present']=self.samples is not None
  return dict(conditions=c,clock_ns=rejected['decision']['control_stamp_ns'],checked_monotonic_wall=time.monotonic())
def owner():
 n=Owner();n.execution_checks=0;n.source_overrides={};n.path_receipt=copy.deepcopy(receipt)
 n.request_id=receipt['request_id'];n.waypoint_index=0;n.waypoints=[np.array(metadata['body_goal'])]
 n.active_trajectory_id=27;n.active_reference_stamp=tuple(metadata['reference_stamp'])
 n.known_scene_inputs=dict(generation=1);n.continuous_route=None;n.state='running'
 n.cascade_key=(n.request_id,0);n.path_admission_active=dict(identity=n.admission_identity(),trajectory_id=27,active_reference_stamp=n.active_reference_stamp,correction_generation=1,ordered_coverage_index=None)
 n.profile=profile;n.heading_gate=NS(phase='drive',stopped=lambda *args:False,evidence=lambda *args:{})
 n.cascade=StableProjectionController(profile['cascade'],n.waypoints[0].tolist(),0.,n.request_id+':0')
 n.cascade.protected=False # Explicit original drive-ready fixture, not replayed feedback state.
 n.global_reference_settings=dict(enabled=False)
 n.cascade.set_path(receipt['points_xyz'],receipt['path_id'],receipt['stamp_ns'],receipt['received_wall_ns'],position=samples[0])
 # Known real geometry, projected endpoint pose and aligned heading are fixtures.
 n.pose=samples[-1].copy();yaw=n.cascade.preview_reference(n.pose)['heading_rad']
 n.rotation=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
 n.pose_stamp=rejected['decision']['source_pose_stamp_ns'];n.cloud_stamp=n.pose_stamp
 n.samples=samples.copy();n.steering=dict(exhausted=True);n.cascade_last_row=dict(mode='path_end_hold')
 n.path_admission_hold=True;n.command=[0.,0.,0.]
 n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=rejected['decision']['control_stamp_ns']))
 # Actual method with explicit synthetic causal boundary, NOT archived live marker.
 mark(n,'steering_exhausted',time.monotonic(),n.pose_stamp/1e9);n.samples=None
 return n
n=owner();marker=copy.deepcopy(n.execution_path_exhaustion)
with patch.object(handoff,'EXHAUSTED_DRIVE_REPLACEMENT_ENABLED',False):
 off=handoff.context(n);old=baseline['context'](n)
 check('baseline_OFF_exact_recorded_context',off==old==rejected['decision']['handoff']['eligibility'])
 check('OFF_actual_preview_still_rejects_missing_samples',handoff.preview(n,new_samples,value,metadata)['reason']=='handoff_active_context_not_ready')
with patch.object(handoff,'EXHAUSTED_DRIVE_REPLACEMENT_ENABLED',True):
 n=owner();before=n.execution_checks;on=handoff.context(n)
 check('one_source_check_only',n.execution_checks==before+1)
 check('replacement_only_old_execution_stays_false',on['replacement_allowed'] and not on['allowed'] and on['conditions']['samples_present'] is False and n.samples is None)
 original_path=n.cascade.path_id;before_samples=n.samples;preview=handoff.preview(n,new_samples,value,metadata)
 check('real_clone_heading_preview_runs_without_old_execution',preview['required'] and 'heading_compatible' in preview and n.samples is before_samples and n.cascade.path_id==original_path)
 bad=owner();bad.rotation=np.diag([-1.,-1.,1.]);heading=handoff.preview(bad,new_samples,value,metadata)
 check('original_heading_limit_still_rejects',heading['allowed'] is False and heading['reason']=='handoff_not_ready')
 check('original_measured_stop_still_required',handoff.stopped_rebind(bad,heading,dict(allowed=True))['allowed'] is False)
 variants={
  'marker_missing':lambda x:setattr(x,'execution_path_exhaustion',None),
  'marker_old_trajectory':lambda x:x.execution_path_exhaustion.update(active_trajectory_id=26),
  'wrong_path':lambda x:x.path_receipt.update(path_id='other'),
  'wrong_source':lambda x:x.execution_path_exhaustion.update(source='timer_zero'),
  'wrong_reason':lambda x:x.execution_path_exhaustion.update(reason='no_samples'),
  'wrong_goal':lambda x:x.execution_path_exhaustion.update(goal_xyz=[0.,0.,0.]),
  'wrong_reference':lambda x:x.execution_path_exhaustion.update(active_reference_stamp=[0,0]),
  'wrong_samples_hash':lambda x:x.execution_path_exhaustion.update(previous_samples_sha256='0'*64),
  'wrong_samples_count':lambda x:x.execution_path_exhaustion.update(previous_samples_count=2),
  'future_marker':lambda x:x.execution_path_exhaustion.update(control_stamp_ns=999999999999),
  'pre_reference_marker':lambda x:x.execution_path_exhaustion.update(control_stamp_ns=1),
  'pose_source_stale':lambda x:x.source_overrides.update(pose_source_fresh=False),
  'imu_source_stale':lambda x:x.source_overrides.update(raw_imu_source_fresh=False),
  'protected':lambda x:setattr(x.cascade,'protected',True),
  'capture':lambda x:setattr(x.cascade,'capture_stamp',1),
  'failure':lambda x:setattr(x.cascade,'failure_latched','fixture'),
  'non_drive':lambda x:setattr(x.heading_gate,'phase','settle'),
 }
 for name,change in variants.items():
  n=owner();change(n);check(name+'_does_not_authorize_replacement',not handoff.context(n)['replacement_allowed'])
 n=owner();n.samples=samples.copy()
 check('ordinary_samples_present_exact_baseline_context',handoff.context(n)==baseline['context'](n))
 # Finite natural-GC cost; only context prerequisite, no cloud/sample copy/hash.
 policy=(gc.isenabled(),gc.get_threshold());cost={}
 for enabled in (False,True):
  n=owner();wall=[];cpu=[]
  with patch.object(handoff,'EXHAUSTED_DRIVE_REPLACEMENT_ENABLED',enabled):
   for _ in range(1000):
    a=time.perf_counter_ns();b=time.thread_time_ns();handoff.context(n)
    cpu.append(time.thread_time_ns()-b);wall.append(time.perf_counter_ns()-a)
  cost[str(enabled)]=dict(calls=1000,median_wall_ns=statistics.median(wall),median_thread_cpu_ns=statistics.median(cpu),total_wall_ns=sum(wall))
 check('natural_GC_policy_unchanged',policy==(gc.isenabled(),gc.get_threshold()))
# Gate orchestration below uses actual original admission method, controlled gates.
class LowerBoundary:
 def accept_spline(self,msg,meta):
  self.events.append('lower')
  if self.fail=='lower':return
  self.samples=new_samples.copy();self.active_trajectory_id=28;self.active_reference_stamp=tuple(meta['reference_stamp'])
  self.execution_path_exhaustion=None;self.replans+=1;self.trajectory_archive_reference='fixture_28.npz'
class PhysicalBoundary:
 def certify(self,n,*args):
  n.events.append('physical');return dict(allowed=n.fail!='physical',reason='fixture_physical_boundary')
 def fresh(self,n,*args):
  stage='final_source' if 'enqueue' in n.events else 'before_lower_source';n.events.append(stage)
  return dict(allowed=n.fail!=stage,source_binding={},checked_monotonic_wall=time.monotonic())
class BatchBoundary:
 def __init__(self,n,value,metadata,decision,*args):
  n.events.append('prepare');self.node=n;self.source_binding={};self.sequence=1;self.decision=decision;self.enqueue_attempted=False
 def enqueue(self,evidence):
  self.enqueue_attempted=True;self.node.events.append('enqueue')
  if self.node.fail=='enqueue':raise ValueError('fixture_enqueue_failure')
 def settle(self,allowed,*args):
  self.node.events.append('settle_'+str(allowed));self.node.settled.append(allowed)
  self.decision=dict(self.decision,admitted=allowed)
class AdmissionFixture(Owner,LowerBoundary):pass
admit=method('controller.py','PIDNavigation','_admit_matched_spline')
klass=ast.ClassDef(name='PIDNavigation',bases=[ast.Name(id='AdmissionFixture',ctx=ast.Load())],keywords=[],body=[admit],decorator_list=[])
events=[]
def certificate(*args,**kwargs):
 events.append('geometry');return dict(allowed=gate_failure[0]!='geometry',reason='fixture_geometry_boundary')
gate_failure=['']
scope=dict(AdmissionFixture=AdmissionFixture,time=time,np=np,copy=copy,hashlib=hashlib,json=json,
 BSpline=BSpline,_r87_diag=NS(span=lambda *args,**kw:nullcontext()),
 certify_curve=certificate,observe_business_gate=lambda n,key,proof,*args:proof,
 trajectory_has_progress=trajectory_has_progress,
 admission_direction_check=lambda *args:dict(allowed=gate_failure[0]!='direction',reason='fixture_direction_boundary'),
 admission_callback_rejected=lambda *args:False,preview_path_handoff=handoff.preview,
 stopped_path_rebind=handoff.stopped_rebind,handoff_context=handoff.context,
 tail_stop_request=tail_stop_request,defer_future_path=lambda *args,**kw:False,
 staged_cascade_binding=staged_cascade_binding,PreparedAdmissionBatch=BatchBoundary,
 _r85_intake_outer=lambda *args:None)
exec(compile(ast.fix_missing_locations(ast.Module(body=[klass],type_ignores=[])),'actual_admission_method','exec'),scope)
Admission=scope['PIDNavigation']
msg=NS(**{k:v for k,v in value.items() if k not in ('start_time','pos_pts')},
 start_time=NS(sec=value['start_time'][0],nanosec=value['start_time'][1]),pos_pts=[NS(x=x,y=y,z=z) for x,y,z in value['pos_pts']])
binding=rejected['decision']['anchor_binding']
def admission_owner(fail=''):
 n=Admission();n.__dict__.update(owner().__dict__);n.fail=fail;n.events=[];n.settled=[];n.decisions=[]
 n._r88_admission=None;n.run=Path('/tmp/R103_replacement_in_memory_only');n.arrival_radius=profile['arrival_radius_m']
 n.automatic_reference=NS(file_sha=binding['reference_file_sha256'],config=dict(max_reference_points=4096))
 n.trajectory_association=NS(accepted=value,reference_stamp=metadata['reference_stamp'])
 n.path_admission_fence=NS(load=lambda:(np.array(receipt['points_xyz']),{},binding))
 n.r85_physical_gate=PhysicalBoundary();n.path_admission_replan_pending=True
 n.global_reference_settings=dict(enabled=True);n.segment_start=samples[0].copy();n.fixed_terminal_heading=lambda *args:0.
 n.planning_start_state={};n.replans=21;n.cascade_path_id=receipt['path_id']
 n.reference_replan_pending=False;n.reference_consistency_hold=False;n.reference_consistency_check=None
 n.pid_guard_needs_evaluation=False;n.evidence=NS(error=None)
 n.publish_command=lambda:n.events.append('zero')
 n.archive_admission=lambda value,metadata,decision,started:n.decisions.append(decision)
 n.retain_admitted_path=lambda *args,**kw:dict(allowed=False,reason='fixture_old_path_absent',samples_present=False)
 return n
flows={}
for enabled,fail in [(False,''),(True,''),(True,'geometry'),(True,'direction'),(True,'physical'),(True,'before_lower_source'),(True,'lower'),(True,'enqueue'),(True,'final_source')]:
 n=admission_owner(fail);old_path=n.cascade;old_receipt=n.path_receipt;old_active=n.path_admission_active;old_marker=copy.deepcopy(n.execution_path_exhaustion)
 events.clear();gate_failure[0]=fail
 with patch.object(handoff,'EXHAUSTED_DRIVE_REPLACEMENT_ENABLED',enabled):n._admit_matched_spline(msg,metadata)
 flows[str(enabled)+':'+(fail or 'success')]=dict(gates=events+list(n.events),settled=n.settled,
  decisions=[dict(admitted=d['admitted'],reason=d['reason'],binding_error=d.get('binding_error')) for d in n.decisions],
  samples_present=n.samples is not None,active_id=n.active_trajectory_id,path_id=n.path_receipt['path_id'])
 if enabled and not fail:
  check('actual_admission_full_control_flow_success',n.settled==[True] and n.active_trajectory_id==28 and n.samples is not None and not n.path_admission_hold and n.execution_path_exhaustion is None and n.path_receipt['trajectory_id']==28)
  check('success_passed_all_original_gate_boundaries',all(x in n.events for x in ('physical','before_lower_source','lower','prepare','enqueue','final_source','settle_True')) and 'geometry'in events)
 else:
  check(('OFF' if not enabled else fail)+'_still_rejects_and_preserves_old_absent_execution',n.samples is None and n.active_trajectory_id==27 and n.cascade is old_path and n.path_receipt is old_receipt and n.path_admission_active is old_active and n.execution_path_exhaustion==old_marker and n.path_admission_hold and True not in n.settled and 'zero' in n.events)
  check(('OFF' if not enabled else fail)+'_no_success_admission_receipt',all(d['admitted'] is False for d in n.decisions))
# Entire unchanged execution/transaction/capture files, and preview/stop methods.
for file in ('controller.py','shared_controller.py','path_commit.py'):
 check(file+'_byte_identical_to_baseline',(HERE/file).read_bytes()==subprocess.check_output(['git','show',BASE+':candidate/'+file]))
base_tree=ast.parse(subprocess.check_output(['git','show',BASE+':candidate/path_handoff.py'],text=True));new_tree=ast.parse((HERE/'path_handoff.py').read_text())
for name in ('preview','stopped_rebind','early_request','retention_reference_eligibility'):
 b=next(x for x in base_tree.body if isinstance(x,ast.FunctionDef) and x.name==name);a=next(x for x in new_tree.body if isinstance(x,ast.FunctionDef) and x.name==name)
 check(name+'_AST_identical',ast.dump(a)==ast.dump(b))
for option,expected in [(None,False),('0',False),('1',True),('bad',None)]:
 env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1');env.pop('R103_EXHAUSTED_DRIVE_REPLACEMENT',None)
 if option is not None:env['R103_EXHAUSTED_DRIVE_REPLACEMENT']=option
 code="import os,path_handoff as h,r103_exhausted_drive_replacement as s;v=h.EXHAUSTED_DRIVE_REPLACEMENT_ENABLED;os.environ['R103_EXHAUSTED_DRIVE_REPLACEMENT']='0' if v else '1';assert v==s.enabled();print(v)"
 p=subprocess.run([sys.executable,'-c',code],cwd=HERE,env=env,capture_output=True,text=True)
 check('selector_'+str(option),p.returncode!=0 if expected is None else p.returncode==0 and p.stdout.strip()==str(expected))
result=dict(baseline=BASE,checks=checks,inputs=inputs,marker_fixture=marker,context_OFF=off,context_ON=on,real_preview=preview,flows=flows,cost=cost,
 limitations=['Actual archived trajectory27 receipt and 54 samples plus candidate28 payload/metadata/context are input evidence.',
 'The live exhaustion marker was not archived. It is produced by the unchanged original mark method using explicit synthetic causal boundary stamps, steering exhausted and path_end_hold.',
 'Pose is the real archived27 endpoint projected as fixture; aligned heading, drive-ready nonprotected state, and fresh source booleans are controlled conditions, not reconstructed whole-system state.',
 'Original context, preview, real cascade clone/binding and original admission orchestration execute. Geometry/direction/physical/source/lower acceptance/writer boundaries are controlled test outcomes; no physical validity or complete DDS replay is claimed.',
 'No ROS/Gazebo/build, no GC collect/threshold modification, no safety threshold change. Timing measures only the context fixture.'],
 source_sha256={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in ('path_handoff.py','r103_exhausted_drive_replacement.py',Path(__file__).name)})
OUT.mkdir(exist_ok=True,parents=True);(OUT/'CHECK_RESULTS.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(dict(checks=len(checks),flows=flows,cost=cost),indent=2))
