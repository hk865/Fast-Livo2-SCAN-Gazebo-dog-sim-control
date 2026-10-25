"""Saved first-proof + real R86/PID/Teacher flow; synthetic FIFO receipt order.

No ROS node, DDS arrival replay, simulator, GC forcing or changed time source.
Patches of monotonic are confined to finite offline fixtures.
"""
import copy,json,math,time,tempfile,unittest,hashlib,gc,statistics,ast,subprocess,os
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from sensor_msgs.msg import Imu
from r86_expired_prefix_tests import ProductionChainTests
from r86_prepared_feedback_tests import packet,feed
from point_provenance import message_sha
from ordinary_parking import OrdinaryParking
import r104_parking_causal_join as joinmod
import controller
from r104_parking_causal_join import ParkingCausalJoin
from known_scene_inputs import KnownSceneInputMixin
from teacher_wrapper import TeacherNavigation
HERE=Path(__file__).resolve().parent
RUN=Path('/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs/20261023T114455Z_closed_loop_R85_R103_replacement_round26_590e')
OUT=HERE.parent.parent.parent/'causal_join'
STATUS=json.loads((RUN/'navigation_status.json').read_text())
ANCHOR=json.loads((RUN/'r85_fixed5_parking_evidence.json').read_text())['T0_origin_anchor']
POSE=334_425_000_000;OLD=334_325_000_000;CLOCK=334_460_000_000;WALL=60_000_000_000

class Fixture(ProductionChainTests.Fixture):
 def __init__(self,on=True):
  super().__init__(expired=False)
  self.run=RUN;self.request_id=STATUS['request_id'];self.state='succeeded'
  self.clock.nanoseconds=CLOCK;self._r104_parking_causal_join=ParkingCausalJoin() if on else None
  self.evidence.error=None;self.automatic_reference=NS(scope=(self.request_id,0))
  self._known_generation_ready=lambda:True # external generation readiness is a fixed fixture boundary
  self.obstacle_hold=self.tilt_hold=False;self.max_tilt=0.;self.tilt_stops=0;self.tilt_clear_since=None
  self.bridge_updated=(WALL-10_000_000)/1e9;self.bridge_safety=dict(state='ready')
  self.pose_stamp=self.cloud_stamp=OLD;self.pose_updated=self.cloud_updated=(WALL-100_000_000)/1e9
  origin=ANCHOR['origin'];self.pose=np.array(origin['position']);self.rotation=np.eye(3)
  self.raw_imu.update([0,0,0,1],334.390,(WALL-5_000_000)/1e9,CLOCK/1e9)
  row=copy.deepcopy(STATUS['cascade_parking']);row.update(protection_active=False,failure_latched=None)
  ordinary=OrdinaryParking();ordinary.begin(322_000_000_000,60_000_000_000)
  ordinary.stage='zero_hold';ordinary.ready_stamp=ANCHOR['event']['ready_pose_stamp_ns']
  ordinary.ready_clock=ordinary.handoff_clock=ANCHOR['event']['handoff_clock_ns']
  ordinary.first_actor_zero_ack=copy.deepcopy(ANCHOR['proof'])
  ordinary.origin=np.array(origin['position']);ordinary.origin_yaw=math.pi-.0344
  self.cascade=NS(ordinary_parking=ordinary,ordinary_zero_event=copy.deepcopy(ANCHOR['event']),
   goal_sha=row['fixed_goal_sha256'],path_sha=row['path_sha256'],command=np.zeros(3),row=row,
   protected=False,failure_latched=None,goal=np.array(row['fixed_goal']['position_world_xyz']),
   goal_yaw=row['fixed_goal']['heading_rad'],goal_z_tolerance=.15,diagnostic_observe_only=None)
  def protect(reason,fatal=False):
   self.cascade.protected=True;self.cascade.failure_latched=reason;ordinary.guard(reason)
  self.cascade._protect=protect
  self.terminal_parking_owner=ordinary;self.terminal_parking_request=self.request_id
  self.known_scene_inputs=dict(generation=1,C_world_odom=None,T_navigation_world=None)
  self.known_scene_commit_token=('a'*32,0);self._r86_update_scope()
  source=dict(navigation_ground_truth_used=False,command_after_slew=[0.,0.,0.],
   source_pose_stamp_ns=OLD,paired_imu_stamp_ns=OLD,control_stamp_ns=334_365_000_000,
   source_pose_received_wall_ns=WALL-100_000_000,source_imu_received_wall_ns=WALL-110_000_000,
   cascade_sequence=7000,ordinary_terminal_zero_event=copy.deepcopy(ANCHOR['event']),
   ordinary_zero_event_id=ANCHOR['event']['event_id'],ordinary_zero_event_sequence=ANCHOR['event']['event_sequence'],
   request_id=self.request_id,goal_sha256=self.cascade.goal_sha,path_sha256=self.cascade.path_sha,
   cascade_mode='ordinary_zero_hold',ordinary_parking_stage='zero_hold')
  if on:self._r104_parking_causal_join.published(source)
  self.imu_history.append(dict(stamp_ns=334_385_000_000,received_wall_ns=WALL-5_000_000,angular_velocity_body=[0.,0.,0.]))
  self.fifo(334_390_000_000)
 def fifo(self,stamp,gyro=0.):
  m=Imu();m.header.stamp.sec,m.header.stamp.nanosec=divmod(stamp,10**9)
  m.header.frame_id='livox_frame';m.orientation.w=1.;m.angular_velocity.z=gyro
  self.imu_source_buffer.callback(m,{})
 def queue(self,stamp=POSE,commit=1,bad=False):
  data=packet(stamp,commit)
  p=data['body'].pose.pose.position;q=data['body'].pose.pose.orientation
  p.x,p.y,p.z=map(float,ANCHOR['origin']['position']);q.x,q.y,q.z,q.w=ANCHOR['origin']['quaternion_xyzw']
  if bad:data['body'].twist.twist.linear.x=.2
  data['metadata']['run_id']=RUN.name
  data['metadata']['corrected_body_message_sha256']=message_sha(data['body'])
  feed(self._r86_prepared_mailbox,data,arrival=WALL-2_000_000)
  return self._r86_prepared_mailbox.peek_ready()
 def invoke(self,entry='commit'):
  if entry=='ready':self._r92_commit_ready()
  elif entry=='tick':
   with patch.object(TeacherNavigation,'tick',side_effect=AssertionError('positive tick during wait')):
    self.tick()
  else:self._r86_commit_feedback()
 def finish(self):
  j=self._r104_parking_causal_join
  with tempfile.TemporaryDirectory() as folder:
   old=self.run;self.run=Path(folder)
   if j is not None:j.finish(self)
   self.run=old
  return self._r86_prepared_mailbox.snapshot(),self._r86_prepared_mailbox.accounting()

class Checks(unittest.TestCase):
 def setUp(self):
  self.clockpatcher=patch('time.monotonic_ns',return_value=WALL);self.clockpatch=self.clockpatcher.start()
  self.secondpatch=patch('time.monotonic',return_value=WALL/1e9);self.secondpatch.start()
 def tearDown(self):self.secondpatch.stop();self.clockpatcher.stop()
 def test_off_original_partial_commit_failure(self):
  n=Fixture(False);n.queue();n.invoke()
  self.assertEqual((n.pose_stamp,n.cloud_stamp),(POSE,POSE));self.assertIn('queued_parking_pose_without_causal_IMU',n._r86_prepared_mailbox.fault_reason())
  self.assertTrue(n._r86_prepared_mailbox.accounting()['closes'])
 def test_on_wait_ready_tick_recover_every_real_callback(self):
  n=Fixture();b=n.queue();j=n._r104_parking_causal_join;before=copy.deepcopy(n.cascade.ordinary_parking.first_actor_zero_ack)
  n.invoke('ready');n.invoke('tick')
  self.assertIs(n._r86_prepared_mailbox.peek_ready(),b)
  self.assertEqual((n.pose_stamp,n.cloud_stamp,n.known_scene_commit_token),(OLD,OLD,('a'*32,0)))
  self.assertFalse(n.scan_poses);self.assertFalse(n.scan_clouds);self.assertEqual(j.counts['wait'],1)
  self.assertEqual(n._r86_prepared_mailbox.retained_bytes(),b.size)
  for stamp in range(334_405_000_000,334_425_000_001,5_000_000):n.fifo(stamp)
  n.invoke('ready');self.assertIsNone(n._r86_prepared_mailbox.fault_reason())
  self.assertEqual((n.pose_stamp,n.cloud_stamp),(POSE,POSE));self.assertEqual(len(n.scan_poses),1)
  self.assertEqual(n.paired_imu['stamp_ns'],POSE);self.assertEqual(n.feedback['received_wall_ns'],b.arrivals_ns[0])
  self.assertEqual(n.cascade.ordinary_parking.first_actor_zero_ack,before)
  self.assertIsNone(j.waiting);self.assertEqual(j.counts['commit'],1)
  s,a=n.finish();self.assertEqual(s['retained_bytes'],0);self.assertTrue(a['closes'])
 def test_expired_owner_nonzero_danger_scope_failclosed(self):
  for kind in ('wall','sim','owner','command','core_command','danger','scope','proof'):
   with self.subTest(kind=kind):
    n=Fixture();n.queue();n.invoke();j=n._r104_parking_causal_join;self.assertIsNotNone(j.waiting)
    if kind=='wall':self.clockpatch.return_value=WALL+300_000_000
    elif kind=='sim':n.clock.nanoseconds+=300_000_000
    elif kind=='owner':n.request_id+=':other'
    elif kind=='command':n.command[0]=.01
    elif kind=='core_command':n.cascade.command[0]=.01
    elif kind=='danger':n.raw_imu.tilt=.6
    elif kind=='scope':n.waypoint_index+=1
    else:n.cascade.ordinary_parking.first_actor_zero_ack['qualification_clock_ns']+=1
    n.invoke();self.assertIsNotNone(n._r86_prepared_mailbox.fault_reason());self.assertEqual(n.pose_stamp,OLD)
    s,a=n.finish();self.assertEqual(s['retained_bytes'],0);self.assertTrue(a['closes'])
    self.clockpatch.return_value=WALL
 def test_future_suffix_not_hidden_or_new_wait(self):
  n=Fixture();n.queue();n.queue(CLOCK+1_000_000,2);n.invoke()
  self.assertIsNone(n._r104_parking_causal_join.waiting)
  self.assertIn('queued_parking_pose_without_causal_IMU',n._r86_prepared_mailbox.fault_reason())
 def test_no_ack_and_window_boundary_original_behavior(self):
  for kind in ('missing_proof','boundary','within'):
   n=Fixture()
   if kind=='missing_proof':n.cascade.ordinary_parking.first_actor_zero_ack=None
   else:
    target=ANCHOR['end_stamp_ns']-(1 if kind=='within' else 0)
    # Eligibility boundary unit projection uses original proof and source only;
    # moving the complete historical source epoch is not asserted as replay.
    b=n.queue();self.assertFalse(n._r104_parking_causal_join.after_fixed_window(n,NS(source_ns=target)));continue
   n.queue();n.invoke();self.assertIn('queued_parking_pose_without_causal_IMU',n._r86_prepared_mailbox.fault_reason())
 def test_snapshot_survives_eviction_and_prefix_append(self):
  n=Fixture();b=n.queue();n.invoke();n.fifo(POSE);j=n._r104_parking_causal_join
  original=n._r86_prepared_mailbox.take_ready_prefix
  def take(expected):
   n.queue(POSE+10_000_000,2);n.imu_source_buffer.history.clear();return original(expected)
  with patch.object(n._r86_prepared_mailbox,'take_ready_prefix',side_effect=take):n.invoke()
  self.assertIsNone(n._r86_prepared_mailbox.fault_reason());self.assertEqual(n.paired_imu['stamp_ns'],POSE)
  self.assertEqual(n._r86_prepared_mailbox.accounting()['ready'],1);self.assertTrue(n._r86_prepared_mailbox.accounting()['closes'])
 def test_negative_bad_before_good_never_washed(self):
  n=Fixture();n.queue(bad=True);n.invoke();n.fifo(POSE);n.queue(POSE+10_000_000,2);n.fifo(POSE+10_000_000)
  n.invoke();self.assertIsNotNone(n.cascade.ordinary_parking.failure);self.assertTrue(n.cascade.protected)
  self.assertTrue(n._r86_prepared_mailbox.accounting()['closes'])
 def test_generation_frame_and_proof_identity(self):
  n=Fixture();n.queue();n.invoke();n.known_scene_inputs['generation']=2;n.invoke()
  self.assertIsNotNone(n._r86_prepared_mailbox.fault_reason());self.assertEqual(n.pose_stamp,OLD)
  n=Fixture();n.queue();n.invoke();n.fifo(POSE);n._last_authorized_hold_frame=(2,None,None);n.invoke()
  self.assertIn('r104_control_frame_changed',n._r86_prepared_mailbox.fault_reason());self.assertEqual(n.pose_stamp,OLD)
 def test_future_arrives_while_waiting_and_bounded_exit_diagnostics(self):
  n=Fixture();n.queue();n.invoke();n.queue(CLOCK+1_000_000,2);n.invoke()
  self.assertIn('future_pose_original_rejection',n._r86_prepared_mailbox.fault_reason())
  j=n._r104_parking_causal_join
  for _ in range(300):j.event('bounded_probe')
  self.assertEqual(len(j.events),joinmod.EVENT_CAP);self.assertGreater(j.counts['events_omitted'],0)
  s,a=n.finish();self.assertEqual(s['retained_bytes'],0);self.assertTrue(a['closes'])

 def test_published_copy_and_bound(self):
  j=ParkingCausalJoin();source={'ordinary_terminal_zero_event':{'event_id':'a'},'command_after_slew':[0.,0.,0.]}
  j.published(source);source['command_after_slew'][0]=1.;self.assertEqual(j.published_source['command_after_slew'][0],0.)
  with self.assertRaises(ValueError):joinmod.scalar_snapshot(list(range(513)))
 def test_complete_pair_and_pre_ack_remain_original_both_entries(self):
  for entry in ('ready','tick'):
   for before_ack in (False,True):
    n=Fixture();n.queue();n.fifo(POSE)
    if before_ack:n.cascade.ordinary_parking.first_actor_zero_ack=None
    with patch.object(n,'apply_tilt_guard',side_effect=AssertionError('new guard outside join')), patch.object(TeacherNavigation,'tick',return_value=None):
     if entry=='ready':n._r92_commit_ready()
     else:n.tick()
    self.assertIsNone(n._r86_prepared_mailbox.fault_reason());self.assertEqual(n.pose_stamp,POSE)
    self.assertEqual(n._r104_parking_causal_join.counts['wait'],0)
 def test_publication_replace_then_failure_invalidates_cache(self):
  for failed in ('evidence','dds','after_dds'):
   n=Fixture();j=n._r104_parking_causal_join
   n.heading_gate.phase='drive';n.heading_gate.heading=None;n.alignment_hold=False
   n.pid_prepared_output=np.zeros(3);n.pid=NS(resets=0)
   n.pid_row=dict(sequence=7001,source_pose_stamp_ns=OLD,paired_imu_stamp_ns=OLD,
    control_stamp_ns=334_365_000_000,feedback=dict(received_wall_ns=WALL-100_000_000),
    imu=dict(received_wall_ns=WALL-110_000_000),request_id=n.request_id,trajectory_id=5,
    waypoint_index=0,cascade=copy.deepcopy(n.cascade.row))
   def dds(msg):
    self.assertIsNone(j.published_source)
    if failed=='dds':raise RuntimeError('injected DDS failure')
   n.cmd_pub=NS(publish=dds)
   original_mark=controller._r47_mark
   def mark(*args,**kwargs):
    if failed=='after_dds' and len(args)>2 and args[2]=='DDS_publish_return':raise RuntimeError('stop after real publish boundary')
    return original_mark(*args,**kwargs)
   with tempfile.TemporaryDirectory() as folder:
    n.run=Path(folder)
    with patch.object(controller,'_append_pid_row_evidence',side_effect=RuntimeError('injected append failure') if failed=='evidence' else None),patch.object(controller,'_r47_mark',side_effect=mark):
     with self.assertRaises(RuntimeError):controller.PIDNavigation.publish_command(n,np.zeros(2),0.)
    self.assertTrue((n.run/'cascade_command_source.json').exists())
    if failed=='after_dds':
     self.assertIsNotNone(j.published_source);self.assertEqual(j.published_source['cascade_sequence'],7001)
    else:self.assertIsNone(j.published_source)

 def test_fixed_deadline_never_refreshes(self):
  n=Fixture();n.queue();n.invoke();j=n._r104_parking_causal_join;before=(j.waiting['deadline_clock_ns'],j.waiting['deadline_wall_ns'])
  n.raw_imu.stamp+=.01;n.invoke()
  self.assertEqual((j.waiting['deadline_clock_ns'],j.waiting['deadline_wall_ns']),before)

def finite_cost():
 rows={}
 for kind in ('complete_pair_original','retained_wait'):
  with patch('time.monotonic_ns',return_value=WALL),patch('time.monotonic',return_value=WALL/1e9):
   n=Fixture();n.queue()
   if kind=='complete_pair_original':n.fifo(POSE)
   else:n.invoke()
   before=gc.get_stats();durations=[];cpu=time.process_time_ns()
   for _ in range(500):
    start=time.perf_counter_ns();n._r104_parking_causal_join.prepare(n,n._r86_prepared_mailbox)
    durations.append(time.perf_counter_ns()-start)
   cpu=time.process_time_ns()-cpu;after=gc.get_stats()
   rows[kind]=dict(iterations=500,median_wall_ns=statistics.median(durations),
    p95_wall_ns=sorted(durations)[474],max_wall_ns=max(durations),total_cpu_ns=cpu,
    natural_gc_collections=[a['collections']-b['collections'] for a,b in zip(after,before)],
    retained_bytes=n._r86_prepared_mailbox.retained_bytes(),
    accounted=n._r86_prepared_mailbox.accounting()['closes'])
 return rows

if __name__=='__main__':
 suite=unittest.defaultTestLoader.loadTestsFromTestCase(Checks)
 result=unittest.TextTestRunner(verbosity=2).run(suite)
 OUT.mkdir(parents=True,exist_ok=True)
 records={p.name:dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in (RUN/'navigation_status.json',RUN/'r85_fixed5_parking_evidence.json')}
 costs=finite_cost() if result.wasSuccessful() else None
 (OUT/'CHECKS.json').write_text(json.dumps(dict(passed=result.wasSuccessful(),tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),finite_cost=costs,inputs=records,source_pose_ns=POSE,first_T0_ns=ANCHOR['start_stamp_ns'],window_end_ns=ANCHOR['end_stamp_ns'],fifo_receipt_schedule='synthetic; archived SLAM headers do not prove navigation FIFO arrival'),indent=2)+'\n')
 raise SystemExit(0 if result.wasSuccessful() else 1)
