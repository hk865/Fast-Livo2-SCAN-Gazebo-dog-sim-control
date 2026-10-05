import copy,json,pathlib,unittest
from types import SimpleNamespace as S
from turn_drift import TurnDriftSupervisor
C=('request','goal',5,(1,0))
def pair(ref=(3,0),goal=(1,0,0),traj=6):
 m=S(order=3,traj_id=traj,start_time=S(sec=4,nanosec=0),pos_pts=[S(x=0.,y=0.,z=0.),S(x=1.,y=0.,z=0.)],knots=[0.,1.,2.])
 meta=dict(schema=1,reference_stamp=list(ref),body_goal=list(goal),trajectory=dict(order=3,traj_id=traj,start_time=[4,0],pos_pts=[[0.,0.,0.],[1.,0.,0.]],knots=[0.,1.,2.]))
 return m,meta
class Test(unittest.TestCase):
 def fresh(self):
  x=TurnDriftSupervisor();x.begin(C,0,[0,0,0],0,[1,0,0]);return x
 def sample(self,x,t,p=(.3,0,0),head=.3,**kw):return x.observe(t,p,head,C,phase=kw.get('phase','align'),actual_command=kw.get('command',[0,0,.12]),adapter_state=kw.get('adapter','walk'),protected=kw.get('protected',False),pose_age_ns=kw.get('age',0))
 def trigger(self):
  x=self.fresh()
  for t in range(100_000_000,600_000_000,100_000_000):r=self.sample(x,t)
  self.assertEqual(r['event']['event'],'stop_for_turn_drift');return x
 def idle(self,x,t=1_500_000_000,**kw):return x.execution(t,kw.get('cmd',[0,0,0]),adapter_state=kw.get('state','idle'),nominal_calibrated=kw.get('nominal',True),adapter_stamp_ns=kw.get('stamp',t),adapter_age_ns=kw.get('age',0),protected=kw.get('protected',False))
 def ready(self):
  x=self.trigger();self.idle(x,500_000_000);self.assertIsNotNone(self.idle(x));x.declare_reference([3,0],[1,0,0]);return x
 def test_displacement_profile_does_not_require_route_angle(self):
  x=TurnDriftSupervisor(require_path_offset=False);x.begin(C,0,[0,0,0],0,[1,0,0])
  for t in range(100_000_000,600_000_000,100_000_000):r=self.sample(x,t,p=[.3,0,0],head=0)
  self.assertEqual(r['event']['event'],'stop_for_turn_drift')
 def test_latched_single_event(self):
  x=self.trigger();r=self.sample(x,600_000_000);self.assertIsNone(r['event']);self.assertEqual(len(x.events),1)
 def test_no_motion_or_stable_path_not_trigger(self):
  for p,h in [([.01,0,0],1),([1,0,0],.01)]:
   x=self.fresh()
   for t in range(100_000_000,2_000_000_000,100_000_000):r=self.sample(x,t,p,h)
   self.assertIsNone(r['event'])
 def test_alternating_noise_no_trigger(self):
  x=self.fresh()
  for i in range(1,30):r=self.sample(x,i*100_000_000,head=.3*(-1)**i)
  self.assertIsNone(r['event'])
 def test_duplicate_does_not_advance(self):
  x=self.fresh()
  for _ in range(10):r=self.sample(x,100_000_000)
  self.assertIsNone(r['event'])
 def test_gap_resets_dwell(self):
  x=self.fresh();self.sample(x,100_000_000);self.sample(x,400_000_000)
  for t in [500_000_000,600_000_000,700_000_000,800_000_000]:r=self.sample(x,t)
  self.assertIsNone(r['event']);self.assertIsNotNone(self.sample(x,900_000_000)['event'])
 def test_hold_stale_drive_returning_reset(self):
  for kw in [dict(protected=True),dict(age=300_000_000),dict(phase='drive'),dict(adapter='returning'),dict(command=[.12,0,.1])]:
   x=self.fresh()
   for t in range(100_000_000,2_000_000_000,100_000_000):r=self.sample(x,t,**kw)
   self.assertIsNone(r['event'])
 def test_geometry_jump_must_persist(self):
  x=self.fresh();self.sample(x,100_000_000);self.sample(x,200_000_000,p=[0,0,0]);self.assertIsNone(self.sample(x,300_000_000)['event'])
 def test_source_change_stops_not_resume(self):
  x=self.fresh();r=x.observe(100_000_000,[0,0,0],0,('new','goal',5,(1,0)),phase='align',actual_command=[0,0,.1],adapter_state='walk');self.assertEqual(r['event']['event'],'stop_for_source_change')
 def test_idle_before_stop_and_stale_never_ready(self):
  for kw in [dict(stamp=0),dict(age=300_000_000),dict(state='returning'),dict(protected=True),dict(nominal=False)]:
   x=self.trigger();self.idle(x,500_000_000);self.assertIsNone(self.idle(x,**kw))
 def test_nonzero_resets_real_zero_window(self):
  x=self.trigger();self.idle(x,500_000_000);self.idle(x,1_000_000_000,cmd=[0,0,.01]);self.assertIsNone(self.idle(x));self.assertIsNone(self.idle(x,2_000_000_000));self.assertIsNotNone(self.idle(x,2_500_000_000))
 def test_reference_before_idle_rejected(self):
  x=self.trigger()
  with self.assertRaises(ValueError):x.declare_reference([3,0],[1,0,0])
 def test_same_reference_or_wrong_goal_rejected(self):
  for ref,goal in [([1,0],[1,0,0]),([3,0],[2,0,0])]:
   x=self.trigger();self.idle(x,500_000_000);self.idle(x)
   with self.assertRaises(ValueError):x.declare_reference(ref,goal)
 def test_two_real_payload_orders(self):
  for order in ['meta','spline']:
   x=self.ready();m,d=pair()
   first=x.metadata(d) if order=='meta' else x.spline(m);self.assertIsNone(first)
   r=x.spline(m) if order=='meta' else x.metadata(d);self.assertEqual(r['event'],'fresh_source_identity_valid')
 def test_old_metadata_fresh_execution_time_not_source(self):
  x=self.ready();m,d=pair(ref=(1,0));self.assertIsNone(x.metadata(d));self.assertIsNone(x.spline(m));self.assertNotEqual(x.state,'ready_for_existing_gate')
 def test_same_id_fake_payload_rejected(self):
  x=self.ready();m,d=pair();d['trajectory']['pos_pts'][0][0]=.1;x.metadata(d);self.assertIsNone(x.spline(m))
 def test_new_reference_no_execute_permission(self):
  x=self.ready();self.assertEqual(x.state,'await_paired_source')
 def test_nonfinite_reject_and_wrap(self):
  x=self.fresh()
  with self.assertRaises(ValueError):self.sample(x,100_000_000,p=[float('nan'),0,0])
  x=TurnDriftSupervisor();x.begin(C,0,[0,0,0],3.13,[1,0,0])
  for t in range(100_000_000,1_000_000_000,100_000_000):r=x.observe(t,[.3,0,0],-3.13,C,phase='align',actual_command=[0,0,.1],adapter_state='walk')
  self.assertIsNone(r['event'])
if __name__=='__main__':
 suite=unittest.defaultTestLoader.loadTestsFromTestCase(Test);r=unittest.TextTestRunner(verbosity=1).run(suite)
 pathlib.Path(__file__).with_name('test_result.json').write_text(json.dumps(dict(tests=r.testsRun,failures=len(r.failures),errors=len(r.errors),passed=r.wasSuccessful(),scope='pure event/association contracts, not physical test'),indent=2)+'\n')
 raise SystemExit(not r.wasSuccessful())
