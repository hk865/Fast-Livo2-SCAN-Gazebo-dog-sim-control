"""Bounded real-method speed-tier contracts; synthetic feedback/inference, no physics.

Model inference completion is an explicit test boundary. No weights are loaded.
"""
import ast,copy,hashlib,json,os,subprocess,tempfile,unittest
from contextlib import ExitStack
from pathlib import Path
from unittest.mock import patch
import numpy as np
import r109_speed_tiers as tiers
import student_actor_adapter as adapter
import student_lower_loop_ack as lower
from executor_command import CommandReader
from cascade_core import Controller
from scan_runtime_parameters import parameters
HERE=Path(__file__).resolve().parent
BASE=json.loads((HERE/'profiles/r73_minimal_baseline_20261017.json').read_text())
WORKER=Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261023T092511Z_closed_loop_R85_R102_B_round25_41cb/sources/navigation/engineering_v63_r83_parking_hysteresis_candidate/policy_candidate/worker.py')
function=next(n for n in ast.parse(WORKER.read_text()).body if isinstance(n,ast.FunctionDef)and n.name=='prepare_velocity_input')
ns={'np':np};exec(compile(ast.Module(body=[function],type_ignores=[]),str(WORKER),'exec'),ns)
PREPARE=ns['prepare_velocity_input']
OLD={'__file__':str(HERE/'student_actor_adapter.py')}
exec(compile(subprocess.check_output(['git','show','64a6dfe:candidate/student_actor_adapter.py'],text=True),'<base-adapter>','exec'),OLD)
CONTRACT=json.loads((HERE/'newstudent_pid_contract.json').read_text())
LIMITS=[.6,.2,.6]

class Flow:
 def __init__(self,speed):
  self.profile=copy.deepcopy(BASE)if speed is None else tiers.select_profile(BASE,speed)
  self.config=self.profile['cascade']['r85_student_lower_loop_ack']
  with patch.dict(os.environ,GO2_R73_MINIMAL_SIMULATION='1'):
   self.student=adapter.StudentRuntime('/tmp/r109_nonexistent',{'profile':self.profile},
    dict(student_contract_snapshot=str(HERE/'newstudent_pid_contract.json'),student_export_snapshot='/tmp/not-loaded'),LIMITS)
   self.reader=CommandReader('run','a'*64,LIMITS,self.profile)
  self.cmd=np.zeros(3);self.serial=0
 def step(self,request,measured=(0.,0.,0.),stamp=None):
  self.serial+=1;stamp=1_000_000_000+self.serial*20_000_000 if stamp is None else stamp
  source=dict(navigation_ground_truth_used=False,request_id='run:route',waypoint_index=0,
   path_sha256='b'*64,goal_sha256='c'*64,source_pose_stamp_ns=stamp-5_000_000,
   paired_imu_stamp_ns=stamp-10_000_000,control_stamp_ns=stamp-5_000_000,
   source_pose_received_monotonic_wall=stamp/1e9,paired_imu_received_monotonic_wall=stamp/1e9,
   command_after_slew=list(request))
  envelope=dict(schema_version=1,mode='scan_slam',source='scan_slam',acceptance={'sha256':'a'*64},
   sequence=self.serial,command=list(request),sim_time=stamp/1e9,monotonic_wall=stamp/1e9,
   healthy=True,stop_requested=False,controller_source=source)
  requested,rejected,_=self.reader.read(json.dumps(envelope).encode(),stamp,stamp/1e9)
  if rejected:raise AssertionError('Real CommandReader rejected fixture')
  state=np.zeros(64);state[0]=stamp/1e9;state[4]=1.;state[14:26]=adapter.QDEFAULT;state[[8,9,13]]=measured
  s=self.student;s.native_frame(state)
  proof=s.prepare_command(self.cmd,requested,stamp,self.reader.before_observation,
   dict(rejected=False,actual_original_envelope=envelope),'run',PREPARE)
  s.observation_frame(state,self.cmd,np.zeros(12),np.zeros(247),False)
  # Model inference is not performed: supply completion only to test receipt plumbing.
  s.inferred=True;s.inference_count=self.serial;s.actual_action=np.zeros(12)
  s.input_receipt.update(actor_inferred_this_frame=True,action_finite=True,hidden_finite=True,inference_sequence=self.serial)
  row=dict(world_sim_time=stamp/1e9,command=self.cmd.tolist(),action=s.actual_action.tolist())
  s.telemetry(row)
  ack=dict(stamp_ns=stamp,ack_native_clock_ns=stamp+20_000_000,sequence=self.serial,
   received_wall_ns=10_000_000_000+stamp,applied_command_body=self.cmd.tolist(),requested_command_body=list(request),
   actor_kind='student',actual_checkpoint_sha256=lower.CHECKPOINT_SHA,actual_export_sha256=lower.EXPORT_SHA,
   student_contract_sha256=lower.CONTRACT_SHA,actual_actor_input=copy.deepcopy(s.input_receipt),source_scope_sha256='a'*64)
  lower.annotate_pending(ack,s.pid.last_diagnostic,self.config,LIMITS);lower.bind_causal_ack(ack,self.config)
  residual,_=lower.residual(ack,self.config,LIMITS)
  return ack,copy.deepcopy(s.pid.last_diagnostic),residual

class SpeedTierChecks(unittest.TestCase):
 def test_two_exact_profiles_and_only_required_difference(self):
  for speed in(.2,.5):
   p=tiers.select_profile(BASE,speed);self.assertEqual(tiers.configuration(p),tiers.choice(speed))
   selection=json.loads((HERE/'profiles'/('r109_speed_020.json'if speed==.2 else'r109_speed_050.json')).read_text())
   self.assertEqual(tiers.validate(selection),tiers.configuration(p))
   params=parameters(p);original=parameters(BASE)
   for k in ('fsm.measured_max_speed','manager.max_vel','optimization.max_vel'):
    self.assertEqual(params.pop(k),speed);original.pop(k)
   self.assertEqual(params,original);self.assertEqual(p['continuous_route_contract'],BASE['continuous_route_contract'])
   p.pop(tiers.KEY);p['cascade']['r85_student_lower_loop_ack'].pop(tiers.KEY)
   p['max_speed_mps']=BASE['max_speed_mps'];p['cascade']['desired_speed']=BASE['cascade']['desired_speed']
   self.assertEqual(p,BASE)
  for v in(1.,.3,1,True,'0.5'):
   with self.assertRaises(ValueError):tiers.choice(v)
  for field,value in [('simulation_only',1),('student_longitudinal_limit_mps',1.),('planning_horizon_m',1.625)]:
   selected=tiers.choice(.5);selected[field]=value
   with self.assertRaises(ValueError):tiers.validate(selected)
  bad=tiers.select_profile(BASE,.5);bad['cascade']['desired_speed']=.2
  with self.assertRaises(ValueError):tiers.configuration(bad)
 def test_original_and_explicit_point_two_pid_equal(self):
  old=OLD['CorrectionOnlyPID'](CONTRACT,LIMITS);plain=adapter.CorrectionOnlyPID(CONTRACT,LIMITS)
  slow=adapter.CorrectionOnlyPID(CONTRACT,LIMITS,speed_tier=tiers.choice(.2))
  for i in range(100):
   request=np.array([.2,.18,.5])if i<60 else np.zeros(3)
   measured=np.array([.13,.05,.15]);args=(1+i*.02,request,measured,'run:0',not bool(np.any(request)))
   ref,row=old.update(*args)
   for pid in(plain,slow):
    out,actual=pid.update(*args);actual=dict(actual);actual.pop(tiers.KEY,None)
    np.testing.assert_array_equal(out,ref);self.assertEqual(actual,row)
 def test_reader_actor_actual45_ack_chain_and_unchanged_slew(self):
  for speed,expected in[(.2,.28),(.5,.58)]:
   flow=Flow(speed);previous=np.zeros(3)
   for _ in range(80):
    ack,pid,residual=flow.step([speed,.2,.6])
    self.assertTrue(np.all(abs(flow.cmd-previous)<=[.01200000001,.01200000001,.01600000001]));previous=flow.cmd.copy()
    np.testing.assert_array_equal(np.asarray(flow.cmd,np.float32),flow.student.obs45[6:9])
    self.assertEqual(pid['effective_command_bounds'],[.3 if speed==.2 else .6,.2,.3])
   self.assertAlmostEqual(flow.cmd[0],expected);self.assertEqual(flow.cmd[1:].tolist(),[.2,.3])
   self.assertEqual(ack['r85_student_lower_loop_ack'][tiers.KEY],tiers.choice(speed))
   if speed==.5:self.assertGreater(flow.student.obs45[6],.3)
   # A .6 outer request still clips at .6 after the original bounded correction.
   for _ in range(10):flow.step([.6,0.,0.])
   self.assertAlmostEqual(flow.cmd[0],.3 if speed==.2 else .6)
   for _ in range(60):ack,pid,_=flow.step([0.,0.,0.])
   np.testing.assert_array_equal(flow.cmd,np.zeros(3));self.assertEqual(pid['pid_integral'],[0.,0.,0.])
 def test_default_full_flow_equals_explicit_point_two_except_typed_evidence(self):
  old=Flow(None);new=Flow(.2)
  for i in range(60):
   request=[.2,0.,.1]if i<35 else[0.,0.,0.]
   a,pa,ra=old.step(request);b,pb,rb=new.step(request)
   pb.pop(tiers.KEY);self.assertEqual(pa,pb);np.testing.assert_array_equal(ra,rb)
   b['actual_actor_input'].pop(tiers.KEY);b['r85_student_lower_loop_ack'].pop(tiers.KEY)
   a['r85_student_lower_loop_ack'].pop('ledger_sha256');b['r85_student_lower_loop_ack'].pop('ledger_sha256')
   self.assertEqual(a,b)
 def test_cross_tier_missing_forged_and_causal_ack_rejected(self):
  f=Flow(.5);ack,pid,_=f.step([.5,0.,0.])
  for config in(lower.CONFIG,tiers.select_profile(BASE,.2)['cascade']['r85_student_lower_loop_ack']):
   with self.assertRaises(ValueError):lower.residual(ack,config,LIMITS)
  for mutate in('missing','forged','native_interval'):
   bad=copy.deepcopy(ack)
   if mutate=='missing':bad['actual_actor_input'].pop(tiers.KEY)
   elif mutate=='forged':bad['actual_actor_input'][tiers.KEY]['simulation_only']=1
   else:bad['ack_native_clock_ns']+=1
   with self.assertRaises(ValueError):lower.residual(bad,f.config,LIMITS)
 def test_real_preflight_and_guardian_speed_consumers(self):
  # External resource renewal, binary and model authorization are explicit
  # boundaries, not granted here. Keep all actual business profile validation.
  from r85_preflight import validate_profile
  from guardian_archive_shutdown import _r85_limits
  with ExitStack()as stack:
   for name,value in [('r85_resource_renewal.retained_hard_bytes',BASE['r85_session']['maximum_retained_bytes']),
    ('r85_session_guard.session_deadline_utc',BASE['r85_session']['deadline_utc']),
    ('scan_workspace.scan_contract',{}),('policy_selection.selection','student'),('policy_selection.student_sources',{})]:
    stack.enter_context(patch(name,return_value=value))
   for speed in(.2,.5):
    p=tiers.select_profile(BASE,speed);validate_profile(p,HERE)
    plan=dict(schema='student_pct_scan_closed_loop_runtime_plan/v1',profile=p,duration_s=p['duration_s'],wall_budget_s=p['duration_s']+120)
    with tempfile.TemporaryDirectory(prefix='r109_guardian_')as temp:
     folder=Path(temp)
     for name,key in [('source_manifest.json','source_manifest_sha256'),('r85_task_contract.json','r85_task_contract_sha256')]:
      raw=b'{}';(folder/name).write_bytes(raw);plan[key]=hashlib.sha256(raw).hexdigest()
     (folder/'input_profile.json').write_text(json.dumps(p))
     raw=json.dumps(plan).encode();(folder/'runtime_plan.json').write_bytes(raw)
     owner={'runtime_plan_sha256':hashlib.sha256(raw).hexdigest()}
     _r85_limits(folder,owner,plan)
     # Profile mismatch must still fail both independent readers.
     p['cascade']['desired_speed']=.19
     with self.assertRaises(ValueError):validate_profile(p,HERE)
     with self.assertRaises(ValueError):_r85_limits(folder,owner,plan)
 def test_real_cascade_uses_selected_drive_speed_with_actual_lower_ledger(self):
  for speed in(.2,.5):
   f=Flow(speed);c=Controller(f.profile['cascade'],[4.,0.,.32],0.,'run:0')
   c.set_path([[0.,0.,.32],[4.,0.,.32]],'checked-fixture',0,0)
   for n in range(1,41):
    ack,_,_=f.step([speed,0.,0.]);stamp=ack['stamp_ns']+20_000_000;wall=10_000_000_000+stamp
    pose=dict(stamp_ns=stamp,received_wall_ns=wall,frame_id='camera_init',position_world_xyz=[0.,0.,.32],quaternion_wxyz=[1.,0.,0.,0.],origin_velocity_body=[0.,0.,0.])
    imu=dict(stamp_ns=stamp-10_000_000,received_wall_ns=wall-10_000_000,frame_id='body',angular_velocity_body=[0.,0.,0.])
    command,row=c.update(pose,imu,ack,stamp+5_000_000,wall)
   self.assertEqual(row['mode'],'drive');self.assertAlmostEqual(c.last_reference_speed,speed)
   # Same untouched causal guard rejects a future gyro, for both tiers.
   pose['stamp_ns']+=20_000_000;pose['received_wall_ns']+=20_000_000;imu['stamp_ns']=pose['stamp_ns']+1
   _,row=c.update(pose,imu,ack,pose['stamp_ns']+5_000_000,wall+20_000_000)
   self.assertEqual(row['mode'],'protect')

if __name__=='__main__':unittest.main(verbosity=2)
