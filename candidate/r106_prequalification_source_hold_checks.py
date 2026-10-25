"""Real methods and a8bc negative measurements; finite synthetic causal sequence.

No simulation or claim to replay unknown DDS arrival, pose vector, or physics.
"""
import ast,copy,gc,hashlib,json,math,os,subprocess,sys,tempfile,time,types,unittest
from contextlib import contextmanager
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
import ordinary_parking as ordinary_module
from cascade_core import Controller
from r86_prepared_feedback import check_negative_parking
from r106_prequalification_source_hold import eligible
from r105_terminal_parking_owner_checks import student
HERE=Path(__file__).resolve().parent;OUT=HERE.parent.parent.parent/'prequalification_hold'
BASE='59f96ab341fa63045e02d97baa30edacd0a5a0ef'
RUN=Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261023T133356Z_closed_loop_R85_R105_terminal_owner_round26_a8bc')
SCOPE=json.loads((RUN/'navigation_scope.json').read_text());STATUS=json.loads((RUN/'navigation_status.json').read_text())
GOAL=STATUS['cascade_parking']['fixed_goal'];PROFILE=SCOPE['profile']
NEGATIVE=next(json.loads(line) for line in (RUN/'R85_DIAGNOSTIC_BUSINESS_OBSERVATIONS.jsonl').open()
 if 'ordinary_terminal_parking_business' in line and 'parking_fixed_target_lost' in line)
MEASURED=NEGATIVE['evidence']['measurements'];BADSTAMP=NEGATIVE['evidence']['source_pose_stamp_ns'];BADCLOCK=NEGATIVE['evidence']['control_stamp_ns']
START=338_730_000_000;DEADLINE=1_108_730_000_000
old_ordinary=types.ModuleType('ordinary_parking')
exec(compile(subprocess.check_output(['git','show',BASE+':candidate/ordinary_parking.py'],text=True),'<baseline_ordinary>','exec'),old_ordinary.__dict__)
OLD={}
with patch.dict(sys.modules,{'ordinary_parking':old_ordinary}):
 exec(compile(subprocess.check_output(['git','show',BASE+':candidate/cascade_core.py'],text=True),'<baseline_core>','exec'),OLD)
old_tree=ast.parse(subprocess.check_output(['git','show',BASE+':candidate/r86_prepared_feedback.py'],text=True))
old_check=next(n for n in old_tree.body if isinstance(n,ast.FunctionDef) and n.name=='check_negative_parking')
OLD_NEGATIVE=dict(copy=copy,math=math)
exec(compile(ast.fix_missing_locations(ast.Module(body=[old_check],type_ignores=[])),'<baseline_queued>','exec'),OLD_NEGATIVE)

def inputs(stamp,serial,bad=False):
 # Exact archived negative xy/z/yaw errors; direction, attitude remainder,
 # velocity, pair/receipt/ACK sequence are explicit synthetic boundaries.
 p=np.array(GOAL['position_world_xyz']);yaw=GOAL['heading_rad']
 if bad:
  p[0]+=MEASURED['goal_distance_xy_m'];p[2]-=MEASURED['goal_z_error_m'];yaw-=MEASURED['goal_yaw_error_rad']
 wall=100_000_000_000+(stamp-385_700_000_000)
 f=dict(stamp_ns=stamp,received_wall_ns=wall,frame_id='camera_init',position_world_xyz=p.tolist(),
  quaternion_wxyz=[math.cos(yaw/2),0.,0.,math.sin(yaw/2)],origin_velocity_body=[0.,0.,0.])
 i=dict(stamp_ns=stamp,received_wall_ns=wall,frame_id='body',angular_velocity_body=[0.,0.,0.])
 ack,_,_=student(stamp-10_000_000,serial)
 return f,i,ack,stamp+(BADCLOCK-BADSTAMP),wall+(BADCLOCK-BADSTAMP)

def make_core(folder,*,old=False,diagnostic=True,on=False):
 c=(OLD['Controller'] if old else Controller)(PROFILE['cascade'],GOAL['position_world_xyz'],GOAL['heading_rad'],GOAL['goal_id'])
 goal=np.array(GOAL['position_world_xyz']);p=goal+[1.,0.,0.]
 c.set_path([p.tolist(),goal.tolist()],'explicit_synthetic_checked_path_fixture',START-100_000_000,1)
 c.begin_ordinary_terminal(START,DEADLINE-START)
 if not old:c.ordinary_parking.prequalification_source_hold_enabled=on
 c.diagnostic_observe_only=dict(run=Path(folder),profile={'diagnostic_observe_only':{'enabled':diagnostic}})
 for k in range(2):c.update(*inputs(BADSTAMP-100_000_000+k*50_000_000,k+1),mode_override='capture')
 return c

def state(c):
 e=copy.deepcopy(c.ordinary_parking.evidence());e.pop('r106_prequalification_source_hold',None)
 return dict(protected=c.protected,failure_latched=c.failure_latched,ordinary=e,command=c.command.tolist(),
  recovery=list(c.recovery),last_math_stamp=c.last_math_stamp,last_seen_pose_stamp=c.last_seen_pose_stamp)

def queued(c,stamp=BADSTAMP,old=False,bad=True):
 f,i,_,clock,_=inputs(stamp,3,bad)
 n=NS(cascade=c,feedback=f,paired_imu=i,get_clock=lambda:NS(now=lambda:NS(nanoseconds=clock)))
 (OLD_NEGATIVE['check_negative_parking'] if old else check_negative_parking)(n)
 return n

class Checks(unittest.TestCase):
 def test_actual_negative_three_way_queued_and_core(self):
  for entry in ('queued','core'):
   with self.subTest(entry=entry),tempfile.TemporaryDirectory() as folder:
    strict=make_core(folder,old=True,diagnostic=False)
    original_diag=make_core(folder,old=True);off=make_core(folder);on=make_core(folder,on=True)
    for c,old in ((strict,True),(original_diag,True),(off,False),(on,False)):
     if entry=='queued':queued(c,old=old)
     else:c.update(*inputs(BADSTAMP,3,True),mode_override='capture')
    self.assertEqual(state(off),state(original_diag));self.assertEqual(state(on),state(strict))
    self.assertEqual(off.ordinary_parking.failure,'parking_fixed_target_lost');self.assertFalse(off.protected)
    self.assertIsNone(on.ordinary_parking.failure);self.assertTrue(on.protected);self.assertFalse(any(on.command))
    note=on.ordinary_parking.evidence()['r106_prequalification_source_hold']
    self.assertEqual(note['count'],1);self.assertEqual(note['first']['source_pose_stamp_ns'],BADSTAMP)
    self.assertEqual(note['first']['queued_pose_checked'],entry=='queued')
    self.assertAlmostEqual(note['first']['goal_z_error_m'],MEASURED['goal_z_error_m'])
 def test_every_negative_retained_then_original_new_source_recovery_and_dwell(self):
  with tempfile.TemporaryDirectory() as folder:
   c=make_core(folder,on=True);owner=c.ordinary_parking
   queued(c);first=copy.deepcopy(owner.prequalification_source_hold_first)
   queued(c,BADSTAMP+50_000_000)
   self.assertEqual(owner.prequalification_source_hold_count,2);self.assertEqual(owner.prequalification_source_hold_first,first)
   self.assertEqual(owner.prequalification_source_hold_last['source_pose_stamp_ns'],BADSTAMP+50_000_000)
   self.assertEqual(c.recovery,[]);self.assertIsNone(owner.dwell)
   good=inputs(BADSTAMP+100_000_000,4);_,row=c.update(*copy.deepcopy(good),mode_override='capture')
   self.assertEqual(row['mode'],'recovering');self.assertEqual(len(c.recovery),1);self.assertIsNone(owner.ready_stamp)
   c.update(*copy.deepcopy(good),mode_override='capture')
   self.assertEqual(len(c.recovery),1);self.assertIsNone(owner.dwell)
   stages=[]
   for k in range(1,16):
    _,row=c.update(*inputs(BADSTAMP+100_000_000+k*50_000_000,4+k),mode_override='capture');stages.append(row['mode'])
    if owner.ready_stamp is None:self.assertIsNone(owner.first_actor_zero_ack)
   self.assertIn('controlled_deceleration',stages);self.assertIn('ordinary_zero_hold',stages)
   self.assertGreaterEqual(owner.ready_stamp-owner.dwell,600_000_000)
   self.assertEqual((owner.start_clock,owner.deadline),(START,DEADLINE));self.assertIsNone(owner.failure)
   self.assertIsNone(owner.first_actor_zero_ack);self.assertIsNone(c.ordinary_zero_event)
 def test_qualified_or_failed_owner_and_unknown_event_never_take_exception(self):
  for boundary in ('ready','ready_clock','handoff','publication','ACK','typed_zero','unknown_zero','existing_failure','prior_diagnostic'):
   with self.subTest(boundary=boundary),tempfile.TemporaryDirectory() as folder:
    c=make_core(folder,on=True);o=c.ordinary_parking;event=None
    if boundary=='ready':o.ready_stamp=BADSTAMP-1
    elif boundary=='ready_clock':o.ready_clock=BADSTAMP-1
    elif boundary=='handoff':o.handoff_clock=BADSTAMP-1
    elif boundary=='publication':o.first_zero_publication_clock=BADSTAMP-1
    elif boundary=='ACK':o.first_actor_zero_ack={'qualification_clock_ns':BADSTAMP-1}
    elif boundary=='typed_zero':event={'event_id':'existing'}
    elif boundary=='unknown_zero':event=__import__('r106_prequalification_source_hold').UNSPECIFIED_ZERO_EVENT
    elif boundary=='existing_failure':o.failure='previous_failure'
    else:o.diagnostic_business_failure_reason='previous_failure';o.diagnostic_business_observation_count=1
    before=copy.deepcopy((o.ready_stamp,o.ready_clock,o.handoff_clock,o.first_zero_publication_clock,o.first_actor_zero_ack,o.deadline,o.failure))
    self.assertFalse(eligible(o,'parking_fixed_target_lost',event))
    o.observe_business_rejection('parking_fixed_target_lost',BADSTAMP,BADCLOCK,MEASURED,c.diagnostic_observe_only,terminal_zero_event=event)
    self.assertEqual(o.prequalification_source_hold_count,0)
    self.assertEqual((o.ready_stamp,o.ready_clock,o.handoff_clock,o.first_zero_publication_clock,o.first_actor_zero_ack,o.deadline),before[:-1])
    if before[-1] is not None:self.assertEqual(o.failure,before[-1])
    if boundary=='ready':
     o.guard('real_source_protection');self.assertEqual(o.stage,'failed');self.assertIsNotNone(o.failure)
 def test_only_target_reasons_and_enabled_capture_owner(self):
  with tempfile.TemporaryDirectory() as folder:
   for stage in ('capture','controlled_deceleration'):
    for reason in ('parking_fixed_target_lost','capture_fixed_goal_out_of_range'):
     c=make_core(folder,on=True);o=c.ordinary_parking;o.stage=stage
     before=(o.dwell,o.failure,o.start_clock,o.deadline)
     self.assertFalse(o.observe_business_rejection(reason,BADSTAMP,BADCLOCK,MEASURED,c.diagnostic_observe_only,terminal_zero_event=None))
     self.assertEqual((o.dwell,o.failure,o.start_clock,o.deadline),before);self.assertEqual(o.prequalification_source_hold_count,1)
   for mutation in ('disabled','different_reason','zero_hold'):
    c=make_core(folder,on=True);o=c.ordinary_parking
    if mutation=='disabled':o.enabled=False
    if mutation=='zero_hold':o.stage='zero_hold'
    reason='ordinary_precise_capture_deceleration_timeout' if mutation=='different_reason' else 'parking_fixed_target_lost'
    self.assertFalse(eligible(o,reason,None))
 def test_off_full_core_rows_equal_and_qualifier_unchanged(self):
  with tempfile.TemporaryDirectory() as folder:
   old=make_core(folder,old=True);new=make_core(folder)
   for k in range(18):
    args=inputs(BADSTAMP+k*50_000_000,k+3,k<2)
    a,ar=old.update(*copy.deepcopy(args),mode_override='capture');b,br=new.update(*copy.deepcopy(args),mode_override='capture')
    np.testing.assert_array_equal(a,b);self.assertEqual(ar,br)
  for file in ('parking_contract.py','parking_zero_contract.py','control_core.py'):
   if (HERE/file).exists():self.assertEqual((HERE/file).read_bytes(),subprocess.check_output(['git','show',BASE+':candidate/'+file]))
 def test_process_fixed_strict_selector(self):
  code="import os,json,r106_prequalification_source_hold as h,ordinary_parking as p; x=p.OrdinaryParking(); v=h.PREQUALIFICATION_SOURCE_HOLD_ENABLED; os.environ['R106_PREQUALIFICATION_SOURCE_HOLD']='0' if v else '1'; print(json.dumps([v,p.PREQUALIFICATION_SOURCE_HOLD_ENABLED,x.prequalification_source_hold_enabled,p.OrdinaryParking().prequalification_source_hold_enabled]))"
  for value,expected in ((None,False),('0',False),('1',True),('true',None)):
   env=dict(os.environ);env.pop('R106_PREQUALIFICATION_SOURCE_HOLD',None)
   if value is not None:env['R106_PREQUALIFICATION_SOURCE_HOLD']=value
   result=subprocess.run([sys.executable,'-c',code],env=env,capture_output=True,text=True)
   if expected is None:self.assertNotEqual(result.returncode,0)
   else:self.assertEqual(result.returncode,0,result.stderr);self.assertEqual(json.loads(result.stdout),[expected]*4)

def finite_cost():
 with tempfile.TemporaryDirectory() as folder:
  c=make_core(folder,on=True);o=c.ordinary_parking;before=gc.get_stats();cpu=time.process_time_ns();wall=time.perf_counter_ns()
  for _ in range(1000):o.observe_business_rejection('parking_fixed_target_lost',BADSTAMP,BADCLOCK,MEASURED,c.diagnostic_observe_only,terminal_zero_event=None)
  elapsed=time.perf_counter_ns()-wall;used=time.process_time_ns()-cpu
 return dict(calls=1000,wall_ns_per_call=elapsed/1000,cpu_ns_per_call=used/1000,
  natural_gc_collections=[b['collections']-a['collections'] for a,b in zip(before,gc.get_stats())],
  forced_collect=False,GC_threshold_changed=False,hot_disk_writes=False,
  limitation='finite actual observe_business_rejection decision+bounded scalar record; excludes simulator/DDS/inference')

if __name__=='__main__':
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Checks));OUT.mkdir(parents=True,exist_ok=True)
 document=dict(passed=result.wasSuccessful(),tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
  base=BASE,run=RUN.name,actual_first_negative=NEGATIVE,synthetic_remaining_causal_inputs=True,
  complete_DDS_or_physics_replay=False,simulation_started=False)
 if result.wasSuccessful():document['finite_cost']=finite_cost()
 (OUT/'CHECKS.json').write_text(json.dumps(document,indent=2)+'\n')
 raise SystemExit(0 if result.wasSuccessful() else 1)
