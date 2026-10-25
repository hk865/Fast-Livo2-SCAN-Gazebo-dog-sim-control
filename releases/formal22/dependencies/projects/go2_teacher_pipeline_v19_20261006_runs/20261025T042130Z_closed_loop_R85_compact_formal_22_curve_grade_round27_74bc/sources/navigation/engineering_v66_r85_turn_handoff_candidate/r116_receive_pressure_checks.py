"""Actual Admission + association, bounded closed-run envelope projection.

FakeWorker/geometry/commit sink and association initial scope are explicit fixtures.
No ROS Node, benchmark or simulation; original401 String contents are unknown.
"""
from pathlib import Path
import copy,json,os,subprocess,sys,types,unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
import r88_async_admission as service
from r88_async_admission_tests import FakeWorker,MinNode,message
from mode3_trajectory_contract import R85TrajectoryAssociation
HERE=Path(__file__).resolve().parent
FIXTURE_PATH=Path(os.environ['R116_CHECK_FIXTURE']).resolve()
FIXTURE=json.loads(FIXTURE_PATH.read_text())
BASELINE=os.environ.get('R116_CHECK_BASELINE','191582d537fc991fb86fa1cf557af475bafa4664')
BASE=types.ModuleType('r116_base');BASE.__file__=str(HERE/'r88_async_admission.py')
exec(compile(subprocess.check_output(['git','show',BASELINE+':candidate/r88_async_admission.py'],cwd=HERE,text=True),'<baseline>','exec'),BASE.__dict__)
OUTCOMES={}
class ReadyWorker(FakeWorker):
 def result_readable(self):return bool(self.results)
def make(on=True,poll=True,base=False):
 n=MinNode();n._r86_fail_closed=lambda:False;w=ReadyWorker();m=BASE if base else service
 with patch.dict(os.environ,R94_SCAN_READY_POLL='1'),patch.object(m,'POLL_YIELD_ENABLED',poll):
  if base:a=m.Admission(n,w)
  else:
   with patch.object(m,'RECEIVE_PRESSURE_ENABLED',on):a=m.Admission(n,w)
 n._r88_admission=a
 return n,w,a

def envmsg(row):
 if row['kind']==2:
  return NS(order=row['order'],traj_id=row['traj_id'],start_time=NS(sec=row['start_sec'],nanosec=row['start_nsec']),
    pos_pts=[NS(x=float(i),y=0.,z=0.)for i in range(row['pos_count'])],
    knots=list(range(row['knot_count'])),yaw_pts=[0.]*row['yaw_count'],yaw_dt=0.)
 return NS(data='x'*row['size'])
def real_envelopes(n,w,a):
 a.sequence=125;a.start(message(16),{'fixture':'completed job126'});a.event_sequence=400
 for row in FIXTURE['envelope_rows']:
  receipt=FIXTURE['receive_rows'][str(row['event_index'])];n.clock.nanoseconds=receipt['original_received_ros_ns']
  with patch('time.monotonic_ns',return_value=receipt['original_received_wall_ns']):
   a.receive({1:'attempt',2:'spline',3:'metadata'}[row['kind']],envmsg(row),lambda msg,i=row['event_index']:n.order.append(i))
 assert len(a.events)==32 and a.queue_bytes==1452591

def incoming(n,a):
 row=FIXTURE['incoming'];r=FIXTURE['receive_rows']['433'];n.clock.nanoseconds=r['original_received_ros_ns']
 msg=envmsg(row)
 with patch('time.monotonic_ns',return_value=r['original_received_wall_ns']):a.receive('spline',msg,lambda msg:n.order.append(433))
 return msg

def snapshot(n,w,a):
 return dict(order=n.order,counts=dict(a.counts),fault=a.fault,state=n.state,queue_bytes=a.queue_bytes,
  queue=[{k:v for k,v in e.items()if k not in('msg','callback')}for e in a.events],
  submissions=w.submissions,rows=n.rows,close=a.close())

def seed(n,w,a,sizes,callback=None):
 a.start(message(16),{})
 for i,size in enumerate(sizes):a.receive('attempt',NS(data='x'*size),callback if i==0 and callback else lambda msg,i=i:n.order.append(i))

def actual_association(n):
 data=FIXTURE['actual_payload'];meta=data['metadata'];a=R85TrajectoryAssociation(meta['run_id'])
 a.configure_epoch(meta['reference_epoch'],meta['planner_process_id'],meta['localization_generation'])
 a.request(meta['reference_stamp'],meta['body_goal'],meta['reference_content'])
 a.note_local_replan_request(meta['attempt']['local_replan_request_sequence'])
 n.trajectory_association=a
 return a

def payload_msg(p):
 return NS(order=p['order'],traj_id=p['traj_id'],start_time=NS(sec=p['start_time'][0],nanosec=p['start_time'][1]),
  pos_pts=[NS(x=v[0],y=v[1],z=v[2])for v in p['pos_pts']],knots=p['knots'],yaw_pts=p['yaw_pts'],yaw_dt=p['yaw_dt'])

class ReceivePressureChecks(unittest.TestCase):
 def test_off_exact_original_business_and_exit(self):
  outputs=[]
  for base in (True,False):
   with patch('time.monotonic',return_value=42.),patch('time.monotonic_ns',return_value=42_000_000_000):
    n,w,a=make(False,base=base);real_envelopes(n,w,a);w.finish();incoming(n,a);outputs.append(snapshot(n,w,a))
  self.assertEqual(outputs[0],outputs[1]);self.assertEqual(outputs[0]['fault'],'r88_scan_event_queue_bound')
 def test_original_433_boundary_and_every_event_disposition(self):
  for on in (False,True):
   n,w,a=make(on);real_envelopes(n,w,a);original=list(a.events);old_receipts=[(e['received_wall_ns'],e['received_ros_ns'])for e in original];w.finish();source=incoming(n,a)
   if not on:
    self.assertEqual(a.fault,'r88_scan_event_queue_bound');self.assertEqual(a.counts['consumed'],0)
    self.assertEqual(a.counts['discarded_events'],32);self.assertEqual(a.queue_bytes,0)
    OUTCOMES['OFF']=dict(rejected=433,discarded=list(range(401,433)),consumed=[],yield_count=a.poll_yield_count)
   else:
    self.assertIsNone(a.fault);self.assertEqual(n.order,[('admission',16),401])
    self.assertEqual([e['index']for e in a.events],list(range(402,434)))
    self.assertEqual(a.queue_bytes,1452591);self.assertEqual(a.receive_pressure_consumed,1);self.assertEqual(a.poll_yield_count,0)
    self.assertEqual(list(a.events)[:-1],original[1:])
    self.assertIsNot(a.events[-1]['msg'],source);self.assertEqual(a.events[-1]['msg'],source)
    self.assertTrue(all(x is y for x,y in zip(list(a.events)[:-1],original[1:])))
    self.assertEqual([(e['received_wall_ns'],e['received_ros_ns'])for e in list(a.events)[:-1]],old_receipts[1:])
    while a.events:a.poll()
    self.assertEqual(n.order[1:],list(range(401,434)));self.assertEqual(a.queue_bytes,0)
    OUTCOMES['ON']=dict(consumed=n.order[1:],rejected=[],final_queue_bytes=0,pressure_calls=a.receive_pressure_calls)
 def test_tick_nonpressure_and_noresult_original_three(self):
  n,w,a=make();real_envelopes(n,w,a);w.finish();a.poll()
  self.assertEqual(a.counts['consumed'],0);self.assertEqual(a.poll_yield_count,1);self.assertEqual(len(a.events),32)
  a.poll();self.assertEqual(a.counts['consumed'],3);self.assertEqual(len(a.events),29)
  n,w,a=make();seed(n,w,a,[1,1]);w.finish();a.receive('attempt',NS(data='n'),lambda msg:n.order.append('new'))
  self.assertEqual(a.counts['consumed'],0);self.assertEqual(len(a.events),3);self.assertEqual(a.poll_yield_count,1)
  n,w,a=make(poll=False);seed(n,w,a,[1]*32);w.finish();a.receive('attempt',NS(data='n'),lambda msg:None)
  self.assertEqual(a.counts['consumed'],3);self.assertEqual(a.receive_pressure_calls,0)
 def test_byte_pressure_one_enough_or_still_original_reject(self):
  for sizes,enough in [([800000,800000],True),([1,800000,800000],False)]:
   n,w,a=make();seed(n,w,a,sizes);w.finish();a.receive('metadata',NS(data='z'*800000),lambda msg:n.order.append('incoming'))
   self.assertEqual(a.receive_pressure_consumed,1);self.assertEqual(a.poll_yield_count,0)
   if enough:self.assertIsNone(a.fault);self.assertEqual(a.queue_bytes,1600000);self.assertEqual(len(a.events),2)
   else:self.assertEqual(a.fault,'r88_scan_event_queue_bound');self.assertEqual(a.receive_pressure_still_rejected,1);self.assertEqual(a.queue_bytes,0)
 def test_worker_error_orphan_owner_failure_and_commit_clear(self):
  for case in ('error','orphan','owner','commit_fault','commit_clear'):
   n,w,a=make();seed(n,w,a,[1]*32)
   if case=='owner':n.known_scene_inputs['generation']=2
   if case=='commit_fault':n._admit_matched_spline=lambda *x,**kw:a.fail('fixture_live_failure')
   if case=='commit_clear':n._admit_matched_spline=lambda *x,**kw:a.invalidate('fixture_live_clear')
   w.finish(token=999 if case=='orphan'else None,error='worker_error'if case=='error'else None)
   a.receive('attempt',NS(data='n'),lambda msg:n.order.append('incoming'))
   self.assertEqual(a.receive_pressure_calls,0);self.assertEqual(a.counts['consumed'],0)
   if case=='error':self.assertEqual(a.fault,'r88_physical_worker_failed');self.assertEqual(a.poll_yield_count,0)
   elif case=='orphan':self.assertEqual(a.fault,'r88_scan_event_queue_bound')
   elif case=='commit_fault':self.assertEqual(a.fault,'fixture_live_failure')
   else:self.assertIsNone(a.fault);self.assertEqual(len(a.events),1);self.assertEqual(a.queue_bytes,1)
 def test_queue_head_error_and_fault_are_not_swallowed(self):
  for kind in ('raise','fail','close'):
   n,w,a=make()
   def callback(msg):
    if kind=='raise':raise ValueError('fixture_head_failure')
    if kind=='fail':a.fail('fixture_head_failure')
    if kind=='close':a.close()
   seed(n,w,a,[1]*32,callback);w.finish()
   if kind=='raise':
    with self.assertRaisesRegex(ValueError,'fixture_head_failure'):a.receive('attempt',NS(data='n'),lambda msg:None)
    self.assertEqual(n.state,'failed');self.assertEqual(len(a.events),31);self.assertEqual(a.queue_bytes,31)
   else:a.receive('attempt',NS(data='n'),lambda msg:None);self.assertEqual(a.queue_bytes,0);self.assertEqual(a.counts['discarded_events'],32)
   self.assertEqual(a.receive_pressure_calls,1);self.assertEqual(a.receive_pressure_consumed,1);self.assertEqual(a.poll_yield_count,0)
 def test_actual_association_negative_duplicate_version_and_conflict(self):
  for case in ('negative','duplicate','version','conflict'):
   views=[]
   for on in(False,True):
    n,w,a=make(on);assoc=actual_association(n);attempt=copy.deepcopy(FIXTURE['actual_payload']['attempt'])
    if case=='negative':attempt['attempt']['plan_success']=False
    if case in ('duplicate','conflict'):assoc.add_attempt(attempt)
    if case=='conflict':attempt['attempt']['plan_success']=not attempt['attempt']['plan_success']
    if case=='version':attempt['committed_reference_revision']+=1;assoc.add_attempt(FIXTURE['actual_payload']['attempt'])
    callback=lambda msg:assoc.add_attempt(json.loads(msg.data))
    a.start(message(16),{});a.receive('attempt',NS(data=json.dumps(attempt)),callback)
    for _ in range(31):a.receive('attempt',NS(data='x'),lambda msg:None)
    w.finish()
    # Explicit direct receive-event projection avoids OFF capacity failure, so
    # compare after its next original poll against ON's one callback result.
    if on:a.poll(receive_event={'size':1})
    else:a.poll();a.poll()
    views.append(assoc.snapshot());self.assertEqual(a.fault,None)
   self.assertEqual(views[0],views[1])
   if case=='negative':self.assertIsNotNone(views[1]['observed_reference_revision']);self.assertEqual(views[1]['accepted_highwater_id'],-1)
   if case in ('version','conflict'):self.assertEqual(views[1]['rejected'],1)
 def test_actual_payload_association_match_submits_once_and_incoming_deferred(self):
  n,w,a=make();assoc=actual_association(n);data=FIXTURE['actual_payload'];msg=payload_msg(data['payload'])
  self.assertIsNone(assoc.add_attempt(data['attempt']));self.assertIsNone(assoc.add_spline(msg))
  def callback(value):
   matched=assoc.add_metadata(json.loads(value.data))
   self.assertIsNotNone(matched);a.start(*matched)
  a.start(message(15),{})
  a.receive('metadata',NS(data=json.dumps(data['metadata'])),callback)
  for _ in range(31):a.receive('attempt',NS(data='x'),lambda msg:None)
  w.finish();a.receive('attempt',NS(data='incoming'),lambda msg:n.order.append('incoming'))
  self.assertEqual(a.receive_pressure_consumed,1);self.assertEqual(len(w.submissions),2);self.assertTrue(w.busy)
  self.assertEqual(assoc.accepted_highwater_id,16);self.assertEqual(len(a.events),32)
  self.assertEqual(a.events[-1]['msg'].data,'incoming');self.assertNotIn('incoming',n.order)
 def test_selector_and_exit_contract(self):
  n,w,a=make();seed(n,w,a,[1]*32);w.finish();a.receive('attempt',NS(data='n'),lambda msg:None);r=a.close()
  self.assertEqual(r['r116_receive_pressure']['consumed'],1);self.assertEqual(r['r110_poll_yield']['result_yields'],0)
  code="import os,r88_async_admission as m; a=m.RECEIVE_PRESSURE_ENABLED; os.environ['R116_RECEIVE_PRESSURE']='0'; print(a,m.RECEIVE_PRESSURE_ENABLED)"
  for value,expected in [('0','False False'),('1','True True'),('bad',None),('',None)]:
   r=subprocess.run([sys.executable,'-B','-c',code],env=dict(os.environ,R116_RECEIVE_PRESSURE=value,PYTHONPATH=str(HERE)),capture_output=True,text=True)
   if expected is None:self.assertNotEqual(r.returncode,0);self.assertIn('must be 0 or 1',r.stderr)
   else:self.assertEqual(r.returncode,0);self.assertEqual(r.stdout.strip(),expected)


 def test_real_environment_instance_and_exit_binding(self):
  # Fresh imports and Admission constructor use actual process selectors;
  # no patching of module constants or instance selector fields in the child.
  code="""
import hashlib,json,os
from pathlib import Path
from types import SimpleNamespace as NS
import r88_async_admission as m
from r88_async_admission_tests import MinNode,FakeWorker,message
n,w=MinNode(),FakeWorker()
n._r86_fail_closed=lambda:False
w.result_readable=lambda:bool(w.results)
a=m.Admission(n,w)
n._r88_admission=a
a.start(message(16),{})
for i in range(32):a.receive('attempt',NS(data='negative'),lambda msg:n.order.append(msg.data))
w.finish()
a.receive('attempt',NS(data='incoming'),lambda msg:n.order.append(msg.data))
print(json.dumps(dict(module_file=m.__file__,module_sha256=hashlib.sha256(Path(m.__file__).read_bytes()).hexdigest(),
 module_enabled=m.enabled(),module_R116=m.RECEIVE_PRESSURE_ENABLED,module_R110=m.POLL_YIELD_ENABLED,
 instance_R116=a.receive_pressure_enabled,instance_R110=a.poll_yield_enabled,instance_R94=a.scan_ready_poll,
 counts_before_close=dict(a.counts),queue_count_before_close=len(a.events),queue_bytes_before_close=a.queue_bytes,
 order=n.order,receipt=a.close(),worker_fixture=True,ROS=False)))
"""
  bindings={}
  for r116,r110,r94 in [('0','1','1'),('1','1','1'),('1','0','1'),('1','1','0')]:
   env=dict(os.environ,R88_ASYNC_PHYSICAL='1',R116_RECEIVE_PRESSURE=r116,R110_POLL_YIELD=r110,
     R94_SCAN_READY_POLL=r94,PYTHONPATH=str(HERE))
   child=subprocess.run([sys.executable,'-B','-c',code],env=env,capture_output=True,text=True,check=True)
   value=json.loads(child.stdout);bindings[f'R116={r116},R110={r110},R94={r94}']=value
   self.assertTrue(value['module_enabled'])
   self.assertEqual(value['module_R116'],r116=='1');self.assertEqual(value['instance_R116'],r116=='1')
   self.assertEqual(value['module_R110'],r110=='1');self.assertEqual(value['instance_R110'],r110=='1')
   self.assertEqual(value['instance_R94'],r94=='1')
   active=r116==r110==r94=='1'
   if r116=='1':
    receipt=value['receipt']['r116_receive_pressure']
    self.assertEqual(receipt['receive_pressure_after_result_calls'],int(active))
    self.assertEqual(receipt['consumed'],int(active))
    self.assertEqual(receipt['still_rejected'],0)
   else:self.assertNotIn('r116_receive_pressure',value['receipt'])
   if active:
    self.assertEqual(value['queue_count_before_close'],32)
    self.assertEqual(value['counts_before_close']['consumed'],1)
    self.assertEqual(value['receipt']['r110_poll_yield']['result_yields'],0)
   elif r110=='0':self.assertEqual(value['counts_before_close']['consumed'],3)
   else:self.assertEqual(value['receipt']['fault'],'r88_scan_event_queue_bound')
  OUTCOMES['actual_environment_bindings']=bindings

if __name__=='__main__':
 suite=unittest.defaultTestLoader.loadTestsFromTestCase(ReceivePressureChecks)
 result=unittest.TextTestRunner(verbosity=2).run(suite)
 output=dict(passed=result.wasSuccessful(),tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),envelope_outcomes=OUTCOMES,
  fixture_limitations=FIXTURE['limitations'],production_selector='R116_RECEIVE_PRESSURE',benchmark=False,ROS=False,simulation=False)
 output['baseline']=BASELINE
 output['fixture']=str(FIXTURE_PATH)
 if os.environ.get('R116_CHECK_OUTPUT'):
  Path(os.environ['R116_CHECK_OUTPUT']).write_text(json.dumps(output,indent=2)+'\n')
 else:print(json.dumps(output,indent=2))
 raise SystemExit(0 if result.wasSuccessful()else 1)
