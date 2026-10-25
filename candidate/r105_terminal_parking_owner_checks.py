"""Finite real-method terminal capture/typed-zero flows. No node/model/physics.

Archived region facts are reused. Source motion and Actor inference completion
are explicit synthetic boundaries; these checks are not a DDS/physical replay.
"""
import ast,copy,gc,hashlib,json,math,os,subprocess,time,unittest,tempfile
from contextlib import nullcontext,contextmanager
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from cascade_core import Controller
from parking_contract import qualification
from parking_zero_contract import make_event,validate_zero_ack,validate_event
from r105_terminal_reference import digest,fields,validate_fields
import r105_terminal_parking_owner as owner
from bridge import validate_controller_source
from executor_command import CommandReader
from student_actor_adapter import StudentRuntime,CorrectionOnlyPID,PID_SPEC,RESET_SCHEMA,QDEFAULT
from student_lower_loop_ack import annotate_pending,bind_causal_ack,CONFIG,CONTRACT_SHA,EXPORT_SHA,CHECKPOINT_SHA
HERE=Path(__file__).resolve().parent;OUT=HERE.parent.parent.parent/'terminal_owner'
RUN=Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261023T122842Z_closed_loop_R85_R104_causal_join_round26_c865')
STATUS=json.loads((RUN/'navigation_status.json').read_text());PROFILE=json.loads((RUN/'navigation_scope.json').read_text())['profile']
ARRIVAL=STATUS['region_arrivals'][-1];GOAL=STATUS['goals_definitions'][-1]['center'];RID=STATUS['request_id']
START=385_700_000_000;WALL=100_000_000_000;FRAME=[456,np.eye(4).tolist(),np.eye(4).tolist()]
BASE={};exec(compile(subprocess.check_output(['git','show','84392fa:candidate/cascade_core.py'],text=True),'<baseline_core>','exec'),BASE)
OLDZERO={};exec(compile(subprocess.check_output(['git','show','84392fa:candidate/parking_zero_contract.py'],text=True),'<baseline_zero>','exec'),OLDZERO)

def reference(core):
 return dict(schema='terminal_fixed_goal/v1',run_id=RUN.name,request_id=RID,waypoint_index=0,
  goals_definition_sha256=STATUS['goals_definition_sha256'],region_receipt=copy.deepcopy(ARRIVAL),
  fixed_goal=copy.deepcopy(core.goal_record),fixed_goal_sha256=core.goal_sha,generation=456,
  coordinate_frame=copy.deepcopy(FRAME),coordinate_frame_sha256=digest(FRAME),
  source_pose_stamp_ns=START,source_cloud_stamp_ns=START,paired_imu_stamp_ns=START,
  created_control_clock_ns=START,created_wall_ns=WALL,source_pose_received_wall_ns=WALL,
  source_cloud_received_wall_ns=WALL,source_imu_received_wall_ns=WALL,
  deadline_clock_ns=783_550_000_000,path_geometry_present=False,navigation_ground_truth_used=False)

def student(stamp,serial,reader=None,envelope=None):
 # Real adapter/observation functions, with model inference completion explicitly
 # supplied by the test. No model load, no action or actuator owner is invoked.
 st=StudentRuntime.__new__(StudentRuntime);st.native_state=None;st.frame_clock_ns=None;st.frame_serial=serial-1
 st.integral_unload=st.unload_helper=st.xy_stop_takeover=None
 st.pid=CorrectionOnlyPID(dict(pid=PID_SPEC,reset_schema=RESET_SCHEMA,command_bounds=[.3,.2,.3],command_slew_acceleration=[.6,.6,.8]),PROFILE['cascade']['command_limits'])
 state=np.zeros(64);state[0]=stamp/1e9;state[4]=1.;state[14:26]=QDEFAULT
 st.native_frame(state);cmd=np.zeros(3);request=np.zeros(3)
 evidence=dict(rejected=False,actual_original_envelope=envelope or {})
 proof=st.prepare_command(cmd,request,stamp,None if reader is None else reader.before_observation,
  evidence,RUN.name,lambda c,r,t,a:c.__setitem__(slice(None),r))
 st.observation_frame(state,cmd,np.zeros(12),np.zeros(247),False)
 st.inferred=True;st.inference_count=serial;st.actual_action=np.zeros(12)
 st.input_receipt.update(actor_inferred_this_frame=True,action_finite=True,hidden_finite=True,inference_sequence=serial)
 row=dict(world_sim_time=stamp/1e9,command=cmd.tolist(),requested=request.tolist(),action=st.actual_action.tolist(),
  ordinary_zero_before_actor_observation=proof,fault=None,command_expired=False)
 st.telemetry(row)
 ack=dict(stamp_ns=stamp,ack_native_clock_ns=stamp+20_000_000,received_wall_ns=WALL+(stamp-START),sequence=serial,
  applied_command_body=cmd.tolist(),requested_command_body=request.tolist(),actor_kind='student',
  actual_checkpoint_sha256=CHECKPOINT_SHA,actual_export_sha256=EXPORT_SHA,student_contract_sha256=CONTRACT_SHA,
  actual_actor_input=copy.deepcopy(st.input_receipt),source_scope_sha256='a'*64,navigation_ground_truth_used=False)
 annotate_pending(ack,st.pid.last_diagnostic,CONFIG,PROFILE['cascade']['command_limits'])
 bind_causal_ack(ack,CONFIG)
 if proof is not None:ack['ordinary_zero_before_actor_observation']=copy.deepcopy(row['ordinary_zero_before_actor_observation'])
 return ack,st,row

def inputs(stamp,serial,offset=.0):
 wall=WALL+(stamp-START);p=np.array(GOAL);p[0]+=offset
 f=dict(stamp_ns=stamp,received_wall_ns=wall,frame_id='camera_init',position_world_xyz=p.tolist(),
  quaternion_wxyz=[0.,0.,0.,1.],origin_velocity_body=[0.,0.,0.])
 i=dict(stamp_ns=stamp,received_wall_ns=wall,frame_id='body',angular_velocity_body=[0.,0.,0.])
 a,_,_=student(stamp-10_000_000,serial)
 return f,i,a,stamp+10_000_000,wall+10_000_000

def core(terminal=True,base=False,path_variant=0):
 cls=BASE['Controller'] if base else Controller
 c=cls(PROFILE['cascade'],GOAL,math.pi,RID+':0')
 if terminal:c.configure_terminal_reference(reference(c))
 else:
  goal=np.array(GOAL);p=goal.copy();p[path_variant]=p[path_variant]+1.
  c.set_path([p.tolist(),goal.tolist()],'real_fixture_path_'+str(path_variant),START-100_000_000,WALL-100_000_000)
 c.begin_ordinary_terminal(START,770_000_000_000)
 return c

def zero_source(c,stamp,wall,seq):
 r=c.row
 src=dict(schema='actual_SLAM_terminal_fixed_goal_command_source/v1',navigation_ground_truth_used=False,
  source_pose_stamp_ns=stamp,paired_imu_stamp_ns=stamp,control_stamp_ns=stamp+10_000_000,
  source_pose_received_wall_ns=wall,source_imu_received_wall_ns=wall,
  source_pose_received_monotonic_wall=wall/1e9,paired_imu_received_monotonic_wall=wall/1e9,
  command_after_slew=[0.,0.,0.],cascade_sequence=seq,request_id=RID,waypoint_index=0,
  goal_sha256=c.goal_sha,path_sha256=None,cascade_mode='ordinary_zero_hold',ordinary_parking_stage='zero_hold',**fields(c.terminal_reference))
 c.ordinary_zero_event=make_event(RUN.name,RID,c.goal_sha,None,seq,c.ordinary_parking.ready_stamp,
  c.ordinary_parking.handoff_clock,seq,terminal_reference=c.terminal_reference)
 src.update(ordinary_terminal_zero_event=c.ordinary_zero_event,ordinary_zero_event_id=c.ordinary_zero_event['event_id'],ordinary_zero_event_sequence=seq)
 return src

from r86_expired_prefix_tests import ProductionChainTests
import controller as controller_module
from known_scene_inputs import KnownSceneInputMixin
from teacher_wrapper import TeacherNavigation,base as shared,NativeGuardAudit,NATIVE_STEERING_GUARD

class NodeFixture(ProductionChainTests.Fixture):
 """Real wrappers/core/guard/publication; DDS and archived accepted inputs are boundaries."""
 def __init__(self,folder,on=True):
  super().__init__(expired=False)
  self.run=Path(folder)/RUN.name;self.run.mkdir()
  self.profile=copy.deepcopy(PROFILE);self.request_id=RID;self._r105_terminal_owner_enabled=on
  self._r105_terminal_context=None;self._r104_parking_causal_join=None
  self.state='running';self.waypoint_index=0;self.waypoints=np.array([GOAL])
  self.goals=[NS(timeout_sim_s=460.,legacy=False)];self.goals_definition_sha256=STATUS['goals_definition_sha256']
  self.region_arrivals=[];self.region_arrival=NS(last_stamp=None);self.region_arrival_evidence=copy.deepcopy(ARRIVAL)
  self.measured_region_arrival=lambda:(True,True) # actual archived region decision, not a new geometry claim
  self.continuous_before_region=lambda *a:False;self.continuous_record_hard=lambda *a:None
  self.segment_start=None;self.segment_started_ros=323.55;self.final_bundle=None
  self.trajectory_association=NS(reset=lambda:None);self.continuous_route=None
  self.samples=self.path_receipt=self.path_admission_active=self.active_trajectory_id=None
  self.active_reference_stamp=self.trajectory_archive_reference=None
  self.path_admission_hold=True;self.path_admission_replan_pending=True
  self.cascade_key=self.cascade_path_id=None;self.terminal_parking_owner=None
  self.terminal_parking_request=None;self.ordinary_terminal_failure_latched=None
  self.automatic_reference=NS(scope=(11,1234,456),terminal_heading=lambda:math.pi,
    points=lambda:[(np.array(GOAL)+[1.,0.,0.]).tolist(),GOAL])
  self.evidence.error=None;self.obstacle_hold=self.tilt_hold=self.alignment_hold=False
  self.obstacle_stops=self.tilt_stops=0;self.obstacle_clear_since=self.tilt_clear_since=None
  self.obstacle_resume_pending=False;self.max_tilt=0.;self.stale_since=None
  self.bridge_safety=dict(state='ready');self.last_command_time=(START-50_000_000)/1e9
  self.heading_gate.phase='drive';self.heading_gate.heading=None;self.heading_gate.reset=lambda:None
  self.pid=NS(resets=0,reset=lambda why:None);self.pid_row=self.pid_prepared_output=None
  self.pid_records=0;self.pid_guard_needs_evaluation=False;self.pending_guard_row=None
  self.control_clock_hold=NS(before_control=lambda *a:False,guard_required=False,holding=False,guard_completed=lambda:None)
  self.poll_corridor=lambda *a:None
  self._known_generation_ready=lambda:True # accepted same-generation ready receipt boundary
  self.known_scene_transition=False;self.known_scene_status={}
  self.cmds=[];self.cmd_pub=NS(publish=lambda m:self.cmds.append([m.linear.x,m.linear.y,m.angular.z]))
  self.freeze_pub=NS(publish=lambda m:None);self.native_guard_audit=NativeGuardAudit(NATIVE_STEERING_GUARD)
  self.last_actual_guard_clock_ns=None;self.guard_sequence=0;self.guard_continuity_resets=0
  self.cloud=np.array([[100.,100.,10.]],dtype=float);self.guard_cloud=self.cloud
  self.update(START,1)
 def update(self,stamp,serial,offset=.0,ack=None):
  f,i,a,clock,wall=inputs(stamp,serial,offset)
  self.clock.nanoseconds=clock;self.feedback=f;self.paired_imu=i;self.ack=a if ack is None else ack
  self.pose=np.array(f['position_world_xyz']);self.tracking_pose=self.pose.copy()
  self.pose_quaternion=[0.,0.,1.,0.];self.rotation=np.diag([-1.,-1.,1.])
  self.pose_stamp=self.cloud_stamp=stamp;self.pose_updated=self.cloud_updated=(wall-10_000_000)/1e9
  self.bridge_updated=wall/1e9;self.wall=wall
  self.known_scene_inputs=dict(generation=self.automatic_reference.scope[2],C_world_odom=FRAME[1],T_navigation_world=FRAME[2],source_ns=stamp)
  self.known_scene_scan_epoch=dict(schema='known_scene_SCAN_epoch_ready/v1',run_id=self.run.name,
    ready=True,generation=self.automatic_reference.scope[2],localization_generation=self.automatic_reference.scope[2],planner_epoch=11,planner_identity=dict(pid=1234))
  self.raw_imu.update([0,0,0,1],stamp/1e9,(wall-10_000_000)/1e9,clock/1e9)
  self.guard_cloud_receipt=dict(stamp_ns=stamp,frame_id='camera_init',received_monotonic_wall=self.cloud_updated,
    callback_ros_clock_ns=clock,nearest_slam_pose_stamp_ns=stamp,filtering_body_pose=self.pose.tolist(),
    filtering_body_rotation=self.rotation.tolist(),filtered_xyz_float64_sha256=hashlib.sha256(self.cloud.tobytes()).hexdigest(),
    filtered_points=1,self_filtered_points=0,cloud_array_file='synthetic-cloud-boundary',cloud_raw_payload_file='synthetic-cloud-boundary',cloud_archive_sequence=serial)
 def get_ack(self):return self.ack
 def finish_region(self):
  with patch('time.monotonic_ns',return_value=self.wall),patch('time.monotonic',return_value=self.wall/1e9):
   shared.Navigation.control(self,self.wall/1e9)
 def step(self,stamp,serial,offset=.0,ack=None):
  self.update(stamp,serial,offset,ack)
  with patch('time.monotonic_ns',return_value=self.wall),patch('time.monotonic',return_value=self.wall/1e9):
   controller_module.PIDNavigation._control_candidate(self,self.wall/1e9)
  return copy.deepcopy(self.cascade.row) if self.cascade else None

class Checks(unittest.TestCase):
 def setUp(self):
  self.env=patch.dict(os.environ,{"GO2_R73_MINIMAL_SIMULATION":"1"});self.env.start()
 def tearDown(self):self.env.stop()
 def test_selector_strict_fixed_across_real_modules(self):
  code="import os,json,controller,known_scene_inputs,r105_terminal_parking_owner as h; before=[m.TERMINAL_PARKING_OWNER_ENABLED for m in (h,controller,known_scene_inputs)]; os.environ['R105_TERMINAL_PARKING_OWNER']='1' if not before[0] else '0'; assert before==[m.TERMINAL_PARKING_OWNER_ENABLED for m in (h,controller,known_scene_inputs)]; print(json.dumps(before))"
  for option,expected in ((None,False),('0',False),('1',True),('yes',None)):
   env=dict(os.environ);env.pop('R105_TERMINAL_PARKING_OWNER',None)
   if option is not None:env['R105_TERMINAL_PARKING_OWNER']=option
   result=subprocess.run([__import__('sys').executable,'-c',code],env=env,text=True,capture_output=True)
   if expected is None:self.assertNotEqual(result.returncode,0)
   else:self.assertEqual(result.returncode,0,result.stderr);self.assertEqual(json.loads(result.stdout),[expected]*3)
 def test_actual_shared_success_to_terminal_owner(self):
  with tempfile.TemporaryDirectory() as folder:
   n=NodeFixture(folder);n.finish_region()
   self.assertEqual((n.state,n.waypoint_index),('succeeded',1));self.assertIsNone(n.final_bundle)
   self.assertEqual(n.region_arrivals,[ARRIVAL]);phases=[]
   for k in range(20):
    row=n.step(START+(k+1)*50_000_000,k+2,.035 if k<4 else 0.);phases.append(row['mode'])
   self.assertEqual((n.state,n.waypoint_index),('succeeded',1));self.assertEqual(n.region_arrivals,[ARRIVAL])
   self.assertIn('capture',phases);self.assertIn('ordinary_zero_hold',phases)
   self.assertIsNone(n.samples);self.assertIsNone(n.cascade.path);self.assertIsNone(n.active_trajectory_id)
   self.assertTrue(n.path_admission_hold) # original SCAN state is not overwritten to gain terminal authority
   self.assertGreater(n.guard_sequence,0)
   src=json.loads((n.run/'cascade_command_source.json').read_text())
   self.assertEqual(src['ordinary_terminal_zero_event']['schema'],'ordinary_terminal_zero_event/v2')
   validate_controller_source(src,[0.,0.,0.],n.clock.nanoseconds,n.wall,n.run.name)
   self.assertIsNone(n.cascade.ordinary_parking.first_actor_zero_ack)
 def test_node_generation_before_ready_rebuild_after_ready_and_T0_fail(self):
  from std_msgs.msg import String
  for entry in ('committed_frame','correction_transaction'):
   for qualified in (False,True):
    with self.subTest(entry=entry,qualified=qualified),tempfile.TemporaryDirectory() as folder:
     n=NodeFixture(folder);n.finish_region();n.step(START+50_000_000,2)
     before=n._r105_terminal_context['sha256'];before_core=n.cascade
     with patch('time.monotonic_ns',return_value=n.wall),patch('time.monotonic',return_value=n.wall/1e9):
      n._minimal_discard_prior_frame_path()
     self.assertIsNone(n._r105_terminal_context);self.assertEqual(n.state,'succeeded')
     n.automatic_reference.scope=(12,1235,457)
     n.step(START+100_000_000,3)
     self.assertNotEqual(before,n._r105_terminal_context['sha256']);self.assertIsNot(n.cascade,before_core)
     self.assertIsNone(n.cascade.ordinary_parking.ready_stamp)
     for k in range(3,22):n.step(START+k*50_000_000,k+1)
     self.assertIsNotNone(n.cascade.ordinary_parking.ready_stamp)
     if qualified:self.qualify_node(n)
     prior_owner=n.terminal_parking_owner;priorproof=copy.deepcopy(prior_owner.first_actor_zero_ack)
     with patch('time.monotonic_ns',return_value=n.wall),patch('time.monotonic',return_value=n.wall/1e9):
      if entry=='committed_frame':n._minimal_discard_prior_frame_path()
      else:n.on_localization_status(String(data=json.dumps(dict(schema='known_scene_navigation_status/v1',run_id=n.run.name,correction_transition=True,generation=458))))
     self.assertEqual(n.state,'failed');self.assertIsNotNone(n.ordinary_terminal_failure_latched)
     self.assertIsNone(n._r105_terminal_context);self.assertIsNone(n.cascade)
     self.assertEqual(prior_owner.first_actor_zero_ack,priorproof)
     self.assertFalse(owner.control(n,n.wall/1e9));self.assertIsNone(n._r105_terminal_context)
 def qualify_node(self,n):
  src=json.loads((n.run/'cascade_command_source.json').read_text());stamp=n.pose_stamp+20_000_000
  wall=WALL+(stamp-START)
  env=dict(schema_version=1,mode='scan_slam',source='scan_slam',acceptance=dict(sha256='a'*64),sequence=1,
   command=[0.,0.,0.],sim_time=stamp/1e9,monotonic_wall=wall/1e9,healthy=True,controller_source=src,
   ordinary_terminal_zero_event=src['ordinary_terminal_zero_event'])
  reader=CommandReader(n.run.name,'a'*64,[.6,.2,.6]);_,rejected,_=reader.read(json.dumps(env).encode(),stamp,wall/1e9)
  self.assertFalse(rejected);ack,_,_=student(stamp,100,reader,env)
  n.step(n.pose_stamp+50_000_000,101,ack=ack)
  self.assertIsNotNone(n.terminal_parking_owner.first_actor_zero_ack)
  return qualification(dict(cascade_parking=n.cascade.row,known_scene_localization=n.known_scene_inputs),n.run.name,n.request_id,n.clock.nanoseconds)
 def test_node_OFF_stale_future_bridge_and_danger_do_not_create(self):
  for kind in ('OFF','future','stale','missing_pair','bridge','tilt'):
   with self.subTest(kind=kind),tempfile.TemporaryDirectory() as folder:
    n=NodeFixture(folder,on=kind!='OFF');n.finish_region();n.update(START+50_000_000,2)
    if kind=='future':n.pose_stamp=n.cloud_stamp=n.clock.nanoseconds+1;n.known_scene_inputs['source_ns']=n.pose_stamp
    elif kind=='stale':n.pose_updated-=.4
    elif kind=='missing_pair':n.paired_imu=None;n.imu_source_buffer.history.clear()
    elif kind=='bridge':n.bridge_safety['state']='failed'
    elif kind=='tilt':n.raw_imu.tilt=.6
    with patch('time.monotonic_ns',return_value=n.wall),patch('time.monotonic',return_value=n.wall/1e9):owned=owner.control(n,n.wall/1e9)
    self.assertEqual(owned,kind!='OFF');self.assertIsNone(n._r105_terminal_context);self.assertIsNone(n.cascade)
    self.assertFalse(any(n.command));self.assertEqual(n.region_arrivals,[ARRIVAL])
    if kind in ('bridge','tilt'):self.assertEqual(n.state,'failed')
 def test_v2_duplicate_and_R104_reference_bound(self):
  from r85_duplicate_ordinary_source import preserve_existing_source,CONTEXT_SCHEMA
  from r104_parking_causal_join import ParkingCausalJoin
  with tempfile.TemporaryDirectory() as folder:
   n=NodeFixture(folder);n.finish_region()
   for k in range(1,20):n.step(START+k*50_000_000,k+1)
   self.qualify_node(n)
   src=json.loads((n.run/'cascade_command_source.json').read_text());raw=(n.run/'cascade_command_source.json').read_bytes()
   clock=n.clock.nanoseconds;wall=n.wall
   n.control_clock_hold.holding=True;n.control_clock_hold.guard_required=True
   n.control_clock_hold.last_clock_ns=clock;n.control_clock_hold.last_wall_ns=wall
   n.control_clock_hold.last_progress_wall_ns=wall-10_000_000
   context=dict(schema=CONTEXT_SCHEMA,decision='duplicate',protected_or_stale_reset=False,source_stale_reasons=[],command_before=[0.,0.,0.],compute_ros_clock_ns=clock,compute_monotonic_wall_ns=wall)
   with patch('time.monotonic_ns',return_value=wall),patch('time.monotonic',return_value=wall/1e9):
    result=preserve_existing_source(n,context,clock,command_before=[0.,0.,0.])
   self.assertTrue(result['allowed'],result);self.assertEqual((n.run/'cascade_command_source.json').read_bytes(),raw)
   join=ParkingCausalJoin();join.published(src)
   self.assertEqual(join.published_source['reference_sha256'],src['reference_sha256'])
   self.assertEqual(join.owner(n)[-2:],('terminal_fixed_goal/v1',src['reference_sha256']))
   with patch('time.monotonic_ns',return_value=wall),patch('time.monotonic',return_value=wall/1e9):join.health(n,clock,wall)
   join.published_source['reference_sha256']='0'*64
   with patch('time.monotonic_ns',return_value=wall),patch('time.monotonic',return_value=wall/1e9),self.assertRaises(ValueError):join.health(n,clock,wall)
   with patch('time.monotonic_ns',return_value=wall),patch('time.monotonic',return_value=wall/1e9):
    ordinary=n.terminal_parking_owner
    self.assertTrue(ordinary.observe_business_rejection('ordinary_zero_hold_actual_motion_or_drift_exceeded',n.pose_stamp,clock,{},dict(run=n.run,profile=n.profile)))
    observed=preserve_existing_source(n,context,clock,command_before=[0.,0.,0.])
   self.assertTrue(observed['allowed'],observed);self.assertIsNone(observed['active_physical_raw_payload_sha256'])
   self.assertEqual(observed['reference_sha256'],src['reference_sha256'])
   self.assertEqual((n.run/'cascade_command_source.json').read_bytes(),raw)

 def test_original_capture_math_equal_with_two_paths_and_terminal(self):
  values=[core(True),core(False,True,0),core(False,True,1)]
  phases=[]
  for k in range(25):
   stamp=START+k*50_000_000;args=inputs(stamp,k+1,.035 if k<4 else 0.)
   results=[c.update(*copy.deepcopy(args),mode_override='capture') for c in values]
   for cmd,row in results[1:]:
    np.testing.assert_array_equal(cmd,results[0][0]);self.assertEqual(row['mode'],results[0][1]['mode'])
    for name in ('position_terms','yaw_PD','velocity_PI','reference_velocity_world','integral_dt_s','capture_fresh_dwell_start_pose_stamp_ns'):
     self.assertEqual(row.get(name),results[0][1].get(name),name)
   phases.append(results[0][1]['mode'])
  self.assertIn('capture',phases);self.assertIn('controlled_deceleration',phases);self.assertIn('ordinary_zero_hold',phases)
  self.assertIsNone(values[0].path);self.assertIsNone(values[0].path_record);self.assertIsNone(values[0].path_sha)
  self.assertIsNone(values[0].ordinary_parking.first_actor_zero_ack)
 def test_off_scan_math_and_v1_exact_baseline(self):
  new=core(False);old=core(False,True)
  for k in range(18):
   args=inputs(START+k*50_000_000,k+1)
   a,ar=new.update(*copy.deepcopy(args),mode_override='capture');b,br=old.update(*copy.deepcopy(args),mode_override='capture')
   np.testing.assert_array_equal(a,b);self.assertEqual(ar,br)
  params=(RUN.name,RID,new.goal_sha,new.path_sha,7,START,START,7)
  self.assertEqual(make_event(*params),OLDZERO['make_event'](*params))
 def test_v2_real_command_reader_student_ack_qualification(self):
  c=core();last=None
  for k in range(17):
   last=inputs(START+k*50_000_000,k+1);c.update(*copy.deepcopy(last),mode_override='capture')
  stamp=last[0]['stamp_ns'];wall=last[0]['received_wall_ns'];src=zero_source(c,stamp,wall,19)
  input_ns=stamp+20_000_000;input_wall=wall+20_000_000
  validate_controller_source(src,[0.,0.,0.],input_ns,input_wall,RUN.name)
  env=dict(schema_version=1,mode='scan_slam',source='scan_slam',acceptance=dict(sha256='a'*64),
   sequence=1,command=[0.,0.,0.],sim_time=input_ns/1e9,monotonic_wall=input_wall/1e9,healthy=True,
   controller_source=src,ordinary_terminal_zero_event=c.ordinary_zero_event)
  reader=CommandReader(RUN.name,'a'*64,[.6,.2,.6]);raw=json.dumps(env).encode()
  _,rejected,_=reader.read(raw,input_ns,input_wall/1e9);self.assertFalse(rejected)
  ack,st,row=student(input_ns,30,reader,env)
  validate_zero_ack(ack,c.ordinary_zero_event,input_ns+30_000_000)
  # Worker uses original dict-forwarding of the same typed observation proof;
  # independently execute its actual nested dumps method and round-trip archive.
  tree=ast.parse((HERE/'worker.py').read_text());cls=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='IsolatedJSON')
  latest=dict(evidence=dict(rejected=False,actual_original_envelope=env))
  scope=dict(json=json,latest=latest,student=st,actor_kind='student',scope_sha='a'*64,scope={'checkpoint_sha256':CHECKPOINT_SHA},
   student_export=EXPORT_SHA,sha=lambda p:CONTRACT_SHA,lower_ack_config=CONFIG,limits=[.6,.2,.6],annotate_pending=annotate_pending,
   provider=None,recording=NS(project_telemetry=lambda x:x),timer=NS(measure=lambda x:nullcontext()))
  st.contract_path=Path('/synthetic-contract-binding')
  exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),'<real_worker_json>','exec'),scope)
  archived=json.loads(scope['IsolatedJSON']().dumps(row))
  self.assertEqual(archived['ordinary_zero_before_actor_observation'],ack['ordinary_zero_before_actor_observation'])
  self.assertEqual(latest['pending_ack']['ordinary_zero_before_actor_observation'],ack['ordinary_zero_before_actor_observation'])
  self.assertEqual(archived['command_source'],'actual SLAM/IMU fixed-goal terminal reference')
  newer=inputs(stamp+50_000_000,31);newer=(newer[0],newer[1],ack,newer[3],newer[4])
  c.update(*copy.deepcopy(newer),mode_override='capture')
  self.assertIsNotNone(c.ordinary_parking.first_actor_zero_ack)
  status=dict(cascade_parking=c.row,known_scene_localization=dict(generation=456,C_world_odom=FRAME[1],T_navigation_world=FRAME[2]))
  anchor=qualification(status,RUN.name,RID,newer[3]);self.assertEqual(anchor['start_stamp_ns'],newer[3])
  self.assertEqual(anchor['event']['schema'],'ordinary_terminal_zero_event/v2')
  for change in ('reference','generation','schema','path','kind','run','trajectory','geometry','path_id'):
   bad=copy.deepcopy(src)
   if change=='reference':bad['reference_sha256']='0'*64
   elif change=='generation':bad['generation']+=1
   elif change=='schema':bad['ordinary_terminal_zero_event']['schema']='ordinary_terminal_zero_event/v1'
   elif change=='path':bad['path_sha256']='0'*64
   elif change=='kind':bad.pop('reference_kind')
   elif change=='run':bad['terminal_reference']['run_id']='other'
   elif change=='trajectory':bad['trajectory_id']=3
   elif change=='geometry':bad['path_geometry_present']=True
   else:bad['path_id']='borrowed'
   with self.subTest(change=change),self.assertRaises(ValueError):validate_controller_source(bad,[0.,0.,0.],input_ns,input_wall,RUN.name)
  v1=OLDZERO['make_event'](RUN.name,RID,c.goal_sha,'a'*64,19,c.ordinary_parking.ready_stamp,c.ordinary_parking.handoff_clock,19)
  with self.assertRaises(ValueError):validate_zero_ack(ack,v1,input_ns+30_000_000)
 def test_terminal_forbids_route_motion_and_original_source_failures(self):
  for mode in ('drive','turn','brake'):
   c=core();_,row=c.update(*inputs(START,1),mode_override=mode);self.assertEqual(row['mode'],'protect')
  for bad in ('future','pair','wall','attitude','ack'):
   c=core();args=list(inputs(START,1))
   if bad=='future':args[0]['stamp_ns']+=400_000_000
   elif bad=='pair':args[1]['stamp_ns']-=21_000_000
   elif bad=='wall':args[0]['received_wall_ns']-=400_000_000
   elif bad=='attitude':args[0]['quaternion_wxyz']=[math.cos(.4),math.sin(.4),0.,0.]
   else:args[2]['actual_actor_input']['actor_inferred_this_frame']=False
   _,row=c.update(*args,mode_override='capture');self.assertEqual(row['mode'],'protect',bad)
   self.assertIsNone(c.ordinary_parking.first_actor_zero_ack)

@contextmanager
def temporary_function_code(function,code):
 original=function.__code__
 try:
  function.__code__=code;yield
 finally:function.__code__=original

class StartupLoadedChecks(unittest.TestCase):
 """Actual initialization function; entry identity/scope are finite file fixtures."""
 def test_actual_reader_initialization_once_and_wrong_bindings_fail(self):
  import worker,executor_command,sys
  from ipc_request_window import observed_path,RequestWindow
  for kind in ('valid','wrong_module_path','wrong_function_path','wrong_plan','wrong_snapshot','wrong_scope_map'):
   with self.subTest(kind=kind),tempfile.TemporaryDirectory() as folder:
    run=Path(folder)/'bounded_executor_startup_fixture';run.mkdir();sources=run/'sources';sources.mkdir()
    entry=HERE/'ipc_request_window.py';refs={};snapshots={};manifest={}
    for source in (entry,HERE/'worker.py',HERE/'executor_command.py'):
     dest=sources/'navigation'/'engineering_v66_r85_turn_handoff_candidate'/source.name;dest.parent.mkdir(parents=True,exist_ok=True)
     dest.write_bytes(source.read_bytes());value=worker.sha(source)
     refs[str(source)]=refs[str(dest)]=value;snapshots[str(source)]=dict(snapshot=str(dest),sha256=value)
     manifest[str(source)]=manifest[str(dest.relative_to(sources))]=value
    for name,data in (('navigation_source_snapshots.json',snapshots),('source_manifest.json',manifest)):
     path=run/name;path.write_text(json.dumps(data));refs[str(path)]=worker.sha(path)
    scope=dict(run_dir=str(run),references=refs);path=run/'navigation_scope.json';path.write_text(json.dumps(scope));scope_sha=worker.sha(path)
    plan=dict(run=str(run),scope_sha256=scope_sha,source_manifest_sha256=worker.sha(run/'source_manifest.json'),
     runtime_commands=dict(worker=[sys.executable,'-B',str(entry),'--run',str(run)]))
    if kind=='wrong_plan':plan['runtime_commands']['worker'][2]=str(HERE/'worker.py')
    (run/'runtime_plan.json').write_text(json.dumps(plan))
    if kind=='wrong_snapshot':Path(snapshots[str(HERE/'executor_command.py')]['snapshot']).write_text('not the frozen code')
    if kind=='wrong_scope_map':(run/'navigation_source_snapshots.json').write_text('{}')
    code=worker.main.__code__
    if kind=='wrong_function_path':code=code.replace(co_filename='/wrong-loaded-worker.py')
    module_file='/wrong-loaded-executor_command.py' if kind=='wrong_module_path' else executor_command.__file__
    with patch.object(sys.modules['__main__'],'__file__',str(entry)),patch.object(sys,'argv',[str(entry),'--run',str(run)]), \
         patch.object(executor_command,'__file__',module_file),temporary_function_code(worker.main,code), \
         patch.object(worker,'Path',observed_path(Path,RequestWindow())):
     if kind=='valid':
      reader=worker.initialize_command_reader(run,scope,scope_sha,np.array([.6,.2,.6]),{})
      self.assertIs(type(reader),executor_command.CommandReader)
      receipt=json.loads((run/'worker_executor_sources_loaded.json').read_text())
      self.assertEqual(receipt['modules']['worker']['actual_module_file'],str(HERE/'worker.py'))
      self.assertEqual(receipt['modules']['executor_command']['actual_module_file'],str(HERE/'executor_command.py'))
      self.assertTrue(receipt['before_legacy_main']);self.assertTrue(receipt['actual_reader_constructed'])
      self.assertFalse(receipt['Actor_model_or_physical_PASS_claimed'])
      with self.assertRaises(FileExistsError):worker.initialize_command_reader(run,scope,scope_sha,np.array([.6,.2,.6]),{})
     else:
      with self.assertRaises(RuntimeError):worker.initialize_command_reader(run,scope,scope_sha,np.array([.6,.2,.6]),{})
      self.assertFalse((run/'worker_executor_sources_loaded.json').exists())
  # The real production main calls this exact initialization before legacy.main;
  # no parallel/mirrored source-verification implementation is exercised.
  tree=ast.parse((HERE/'worker.py').read_text());main=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
  calls=[n for n in ast.walk(main) if isinstance(n,ast.Call)]
  init=next(n for n in calls if isinstance(n.func,ast.Name) and n.func.id=='initialize_command_reader')
  legacy=next(n for n in calls if isinstance(n.func,ast.Attribute) and isinstance(n.func.value,ast.Name) and n.func.value.id=='legacy' and n.func.attr=='main')
  self.assertLess(init.lineno,legacy.lineno)

def finite_cost():
 # Finite in-memory core runs only; include original capture math and added
 # reference validation/row fields. No inference/DDS/IO, no forced GC/thresholds.
 cases={};args=[inputs(START+k*50_000_000,k+1,.035 if k<4 else 0.) for k in range(25)]
 for kind in ('baseline_scan','candidate_scan','terminal_reference'):
  costs=[];before=gc.get_stats();cpu=time.process_time_ns();wall=time.perf_counter_ns()
  for repeat in range(30):
   c=core(kind=='terminal_reference',kind=='baseline_scan')
   tick=time.perf_counter_ns()
   for item in args:c.update(*copy.deepcopy(item),mode_override='capture')
   costs.append((time.perf_counter_ns()-tick)/len(args))
  costs.sort();cases[kind]=dict(ticks=750,median_ns_per_tick=costs[len(costs)//2],p95_ns_per_tick=costs[int(len(costs)*.95)],
   cpu_ns=time.process_time_ns()-cpu,wall_ns=time.perf_counter_ns()-wall,
   natural_gc_collections=[b['collections']-a['collections'] for a,b in zip(before,gc.get_stats())])
 cases['limits']=dict(simulation=False,forced_collection=False,GC_thresholds_changed=False,external_inference_DDS_IO_included=False,
  fixture='same finite capture/deceleration/zero-hold sequence; not live callback CPU or causality proof')
 OUT.mkdir(parents=True,exist_ok=True);(OUT/'FINITE_COST.json').write_text(json.dumps(cases,indent=2)+'\n')
 return cases

if __name__=='__main__':
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Checks))
 if result.wasSuccessful():finite_cost()
 OUT.mkdir(parents=True,exist_ok=True)
 (OUT/'CHECKS.json').write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,errors=len(result.errors),failures=len(result.failures),
  original_run=RUN.name,baseline='84392fa',actual_region_stamp_ns=ARRIVAL['stamp_ns'],
  simulation_started=False,model_loaded=False,synthetic_source_sequence=True,synthetic_inference_completion_boundary=True),indent=2)+'\n')
 raise SystemExit(0 if result.wasSuccessful() else 1)
