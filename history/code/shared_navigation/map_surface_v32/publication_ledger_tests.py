"""Focused execution of actual production publish AST; no ROS or simulator."""
import ast,copy,hashlib,json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import patch
import numpy as np
from pid_core import slew
HERE=Path(__file__).parent
class Twist:
 def __init__(self):self.linear=NS(x=0.,y=0.,z=0.);self.angular=NS(x=0.,y=0.,z=0.)
class Publisher:
 def __init__(self):self.messages=[];self.fail=False
 def publish(self,msg):
  if self.fail:raise RuntimeError('test-only DDS publish failure')
  self.messages.append(msg)
class Evidence:
 def __init__(self):self.rows=[];self.error=None;self.fail=False
 def append(self,path,row):
  if self.fail:self.error='test-only evidence queue overflow';raise RuntimeError(self.error)
  self.rows.append((path.name,copy.deepcopy(row)))
def actual():
 cls=next(n for n in ast.parse((HERE/'controller.py').read_text()).body if isinstance(n,ast.ClassDef)and n.name=='PIDNavigation')
 fn=next(n for n in cls.body if isinstance(n,ast.FunctionDef)and n.name=='publish_command');scope={'np':np,'json':json}
 exec(compile(ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[])),str(HERE/'controller.py'),'exec'),scope)
 return scope['publish_command']
def strip(text):
 lines=[];active=False
 for line in text.splitlines(keepends=True):
  if '# PUBLICATION_OBSERVER_BEGIN 'in line:assert not active;active=True
  elif '# PUBLICATION_OBSERVER_END 'in line:assert active;active=False
  elif not active:lines.append(line)
 assert not active;return ''.join(lines)
class Checks(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
  self.n=NS(run=self.root,state='running',command=[.1,-.02,.03],last_command_time=1.,last_publication_ros_clock_ns=1_000_000_000,clock_ns=1_050_000_000,pid_row=None,pid_prepared_output=None,counts={'commands':0},cmd_pub=Publisher(),freeze_pub=Publisher(),evidence=Evidence(),publication_records=0,pid=NS(resets=0),obstacle_hold=False,alignment_hold=False,tilt_hold=False,tilt_source=None,bridge_safety={'state':'ready'},heading_gate=NS(phase='drive',heading=.1),control_clock_hold=NS(holding=False,guard_required=False),pid_guard_needs_evaluation=False)
  self.n.pid.reset=lambda reason:setattr(self.n.pid,'resets',self.n.pid.resets+1)
  self.n.get_clock=lambda:NS(now=lambda:NS(nanoseconds=self.n.clock_ns));self.publish=actual()
  self.modules={'geometry_msgs.msg':NS(Twist=Twist),'std_msgs.msg':NS(Bool=lambda **kw:NS(**kw))}
 def tearDown(self):self.temp.cleanup()
 def invoke(self,trigger,velocity=None,yaw=0.):
  namespace={'publish':self.publish};exec('def '+trigger+'(node,velocity,yaw):\n return publish(node,velocity,yaw)\n',namespace)
  with patch.dict('sys.modules',self.modules):namespace[trigger](self.n,velocity,yaw)
 def records(self):return [r for name,r in self.n.evidence.rows if name=='navigation_command_publications.jsonl']
 def test_zero_IMU_bridge_timer_parking_stop_hold_cleanup_paths_each_record_actual_pub(self):
  names=['on_imu','on_bridge_safety','control','control_parking','on_stop','before_control','main']
  for name in names:self.invoke(name);self.n.clock_ns+=10_000_000
  rows=self.records();self.assertEqual([r['trigger']for r in rows],names);self.assertEqual([r['sequence']for r in rows],list(range(1,8)))
  self.assertEqual(self.n.counts['commands'],self.n.publication_records);self.assertEqual(len(self.n.cmd_pub.messages),7)
  for r in rows:
   self.assertEqual(r['command_after'],[0.,0.,0.]);self.assertIsNone(r['associated_pid_sequence']);self.assertTrue(r['stopped_argument'])
   self.assertLessEqual(r['entry_monotonic_wall_ns'],r['publish_started_monotonic_wall_ns']);self.assertLessEqual(r['publish_started_monotonic_wall_ns'],r['monotonic_wall_ns']);self.assertEqual(r['last_command_time_after'],r['publish_ros_clock_ns']/1e9)
 def test_nonzero_guarded_output_not_relimited_or_reshaped(self):
  value=[.03,-.01,.04];self.n.pid_prepared_output=np.array(value);self.invoke('control',np.array(value[:2]),value[2]);r=self.records()[0]
  self.assertEqual(r['command_before'],[.1,-.02,.03]);self.assertEqual(r['command_after'],value);self.assertEqual(r['last_command_time_before'],1.);self.assertEqual(r['publish_ros_clock_ns'],1_050_000_000);self.assertEqual(self.n.pid.resets,0)
 def test_zero_output_from_PID_branch_keeps_branch_identity(self):
  self.n.pid_prepared_output=np.zeros(3);self.invoke('control',np.zeros(2));r=self.records()[0];self.assertEqual(r['command_after'],[0.,0.,0.]);self.assertFalse(r['stopped_argument']);self.assertEqual(self.n.pid.resets,0)
 def test_actual_PID_row_sequence_retained_after_original_flush(self):
  self.n.pid_row=dict(sequence=42,feedback=None,imu=None,source_pose_stamp_ns=1_000_000_000,paired_imu_stamp_ns=990_000_000,control_stamp_ns=1_050_000_000,request_id='test',trajectory_id=1,waypoint_index=0,cascade=dict(fixed_goal_sha256='goal',path_sha256='path',mode='drive',parking_hold_declared_pose_stamp_ns=None))
  self.n.pid_prepared_output=np.array([.03,0.,0.]);self.invoke('control',np.array([.03,0.]));self.assertEqual(self.records()[0]['associated_pid_sequence'],42);self.assertIsNone(self.n.pid_row)
  source=json.loads((self.root/'cascade_command_source.json').read_text());self.assertEqual(source['cascade_sequence'],42);self.assertEqual(source['control_stamp_ns'],1_050_000_000)
 def test_callback_zero_anchor_controls_following_slew_and_chain(self):
  self.n.clock_ns=1_100_000_000;self.invoke('on_bridge_safety');self.n.clock_ns=1_150_000_000
  desired=[.3,.1,.2];value=slew(self.n.command,desired,self.n.clock_ns/1e9-self.n.last_command_time,[.6,.6,.8]);self.n.pid_prepared_output=np.array(value);self.invoke('control',np.array(value[:2]),value[2]);a,b=self.records()
  self.assertEqual(b['command_before'],a['command_after']);self.assertEqual(b['last_command_time_before'],a['last_command_time_after']);self.assertEqual(b['last_publication_ros_clock_ns_before'],a['publish_ros_clock_ns'])
  self.assertEqual(list(slew(b['command_before'],desired,b['publish_ros_clock_ns']/1e9-b['last_command_time_before'],[.6,.6,.8])),b['command_after'])
 def test_publish_exception_has_no_false_success_record(self):
  self.n.cmd_pub.fail=True
  with self.assertRaisesRegex(RuntimeError,'DDS publish failure'):self.invoke('on_imu')
  self.assertEqual(self.n.publication_records,0);self.assertEqual(self.n.counts['commands'],0);self.assertFalse(self.records())
 def test_writer_failure_after_pub_is_visible_and_propagates(self):
  self.n.evidence.fail=True
  with self.assertRaisesRegex(RuntimeError,'evidence queue overflow'):self.invoke('on_imu')
  self.assertEqual(len(self.n.cmd_pub.messages),1);self.assertEqual(self.n.publication_records,1);self.assertEqual(self.n.counts['commands'],1);self.assertFalse(self.records());self.assertIsNotNone(self.n.evidence.error)
  self.assertIn("if self.evidence.error:severe.append('evidence_writer')",(HERE/'teacher_wrapper.py').read_text())
 def test_invalid_guarded_candidate_never_publishes(self):
  self.n.pid_prepared_output=np.zeros(3)
  with self.assertRaisesRegex(RuntimeError,'changed after native guard'):self.invoke('control',np.array([.1,0.]))
  self.assertEqual(self.n.publication_records,0);self.assertFalse(self.records());self.assertFalse(self.n.cmd_pub.messages)
 def test_strip_observer_restores_original_V9_production_bytes(self):
  expected={'controller.py':'a2500ef645fca32e8f20ed74c35ba403f3844a190caa6a2afea9154ea2883bed','clock_hold.py':'3503750c26381557f2aa379949910296f814c132e6741ae00189e9bca0e0c8f7'}
  for name,sha in expected.items():self.assertEqual(hashlib.sha256(strip((HERE/name).read_text()).encode()).hexdigest(),sha)
 def test_all_actual_cmd_calls_go_through_one_recorded_production_boundary(self):
  tree=ast.parse((HERE/'controller.py').read_text());cls=next(n for n in tree.body if isinstance(n,ast.ClassDef)and n.name=='PIDNavigation');owners=[]
  for fn in cls.body:
   if isinstance(fn,ast.FunctionDef):
    for n in ast.walk(fn):
     if isinstance(n,ast.Call)and isinstance(n.func,ast.Attribute)and n.func.attr=='publish'and isinstance(n.func.value,ast.Attribute)and n.func.value.attr=='cmd_pub':owners.append(fn.name)
  self.assertEqual(owners,['publish_command'])
  for file,trigger in [('teacher_wrapper.py','on_imu'),('teacher_wrapper.py','on_bridge_safety'),('shared_controller.py','on_stop')]:
   methods=[n for n in ast.walk(ast.parse((HERE/file).read_text()))if isinstance(n,ast.FunctionDef)and n.name==trigger]
   self.assertTrue(any(isinstance(n,ast.Call)and isinstance(n.func,ast.Attribute)and n.func.attr=='publish_command'for fn in methods for n in ast.walk(fn)),trigger)
if __name__=='__main__':unittest.main()
