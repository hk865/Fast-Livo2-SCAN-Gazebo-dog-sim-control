"""Finite no-Node contracts: actual runtime methods/trigger and verifier projection.

AST extraction retains the production statements, with explicit writer/Node state
fixtures. It is not a complete ROS callback or physical obstacle qualification.
"""
import ast,copy,hashlib,json,subprocess,types,unittest,tempfile
from pathlib import Path
from types import SimpleNamespace as NS
import numpy as np
import r117_dynamic_speed_lead as helper
import r85_dynamic_obstacle_runtime as runtime
import r85_dynamic_obstacle_verifier as verifier
HERE=Path(__file__).resolve().parent;A=HERE.parents[2]
P2=json.loads((A/'speed_clock/PROFILE_DRAFT_02.json').read_text());P5=json.loads((A/'speed_clock/PROFILE_DRAFT_05.json').read_text())
ROUTE=json.loads(Path(P2['r85_task']['typed_route_source_file']).read_text());POINTS=np.array(ROUTE['body_points'],float)
ARCS=np.r_[0,np.cumsum(np.linalg.norm(np.diff(POINTS,axis=0),axis=1))];ENCOUNTER=float(ARCS[180])
OLD={}
for name in ('r85_dynamic_obstacle_runtime','r85_dynamic_obstacle_verifier'):
 m=types.ModuleType('old_'+name);m.__file__=str(HERE/(name+'.py'))
 text=subprocess.check_output(['git','show','4bac533b:candidate/'+name+'.py'],cwd=HERE,text=True)
 exec(compile(text,m.__file__,'exec'),m.__dict__);OLD[name]=(m,text)

def class_ast(text):
 build=next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef)and n.name=='build_node')
 return next(n for n in build.body if isinstance(n,ast.ClassDef))
def method(text,name):return next(n for n in class_ast(text).body if isinstance(n,ast.FunctionDef)and n.name==name)
def position(remaining):
 arc=ENCOUNTER-remaining;i=int(np.searchsorted(ARCS,arc)-1);return POINTS[i]+(POINTS[i+1]-POINTS[i])*(arc-ARCS[i])/(ARCS[i+1]-ARCS[i])
def node(module,text,profile,root):
 records=[];setting=helper.configuration(profile)
 n=NS(speed_lead=setting,lead_window=helper.lead_window(setting),completed_history_enabled=False,
  pre_last={k:1 for k in ('actual_pose','mission_state','entity_observation','service_ack')},scope_start_sequence=1,
  binding={'fixture':'same-owner'},reference_sha='a'*64,encounter_arc=ENCOUNTER,crossing=POINTS[180]+[0,0,.3],
  task_sha='b'*64,typed_path=Path(profile['r85_task']['typed_route_source_file']),task={'source_typed_route_sha256':profile['r85_task']['typed_route_source_sha256']},
  source_sha='c'*64,registration_sha='d'*64,started=None,reference_record={'fixture':'reference'},points=POINTS,
  park_ack=True,pending=None,failure=None,protocol_cancelled=False,state={'fixture':'fresh'},latest_observation={'fixture':'parked'})
 n.writer=NS(atomic=lambda path,value:records.append(('contract',copy.deepcopy(value))))
 n.record=lambda kind,value:records.append((kind,copy.deepcopy(value)))
 n.cancel_business_protocol=lambda reason,details:False
 env=dict(module.__dict__,run=root,task_path=root/'r85_task_contract.json')
 exec(compile(ast.Module(body=[copy.deepcopy(method(text,'freeze_contract'))],type_ignores=[]),str(HERE/'r85_dynamic_obstacle_runtime.py'),'exec'),env)
 n.freeze_contract=types.MethodType(env['freeze_contract'],n)
 return n,records
NEW_TEXT=(HERE/'r85_dynamic_obstacle_runtime.py').read_text()

def trigger(n,remaining):
 n.pose={'position':position(remaining).tolist(),'stamp_ns':50_000_000_000}
 block=next(x for x in method(NEW_TEXT,'tick').body if isinstance(x,ast.If)and ast.unparse(x.test).startswith('fresh and self.park_ack'))
 env=dict(runtime.__dict__,self=n,fresh=True,parked=True,ns=50_000_000_000)
 exec(compile(ast.Module(body=[copy.deepcopy(block)],type_ignores=[]),'<actual runtime trigger>','exec'),env)

def projection(remaining,contract,profile):
 text=(HERE/'r85_dynamic_obstacle_verifier.py').read_text();fn=next(n for n in ast.parse(text).body if isinstance(n,ast.FunctionDef)and n.name=='validate_r85_dynamic_evidence')
 body=next(n for n in fn.body if isinstance(n,ast.Try)).body
 start=next(i for i,n in enumerate(body)if isinstance(n,ast.Assign)and any(isinstance(t,ast.Name)and t.id=='obstacle_arc'for t in n.targets))
 env=dict(verifier.__dict__,points=POINTS,trigger_position=position(remaining),lead_window=helper.validate_contract(contract,profile))
 exec(compile(ast.Module(body=copy.deepcopy(body[start:start+4]),type_ignores=[]),'<actual verifier projection>','exec'),env)
 return env['remaining']

class DynamicSpeedChecks(unittest.TestCase):
 def test_legacy_and_typed_point2_contract_bytes_and_motion_identical(self):
  legacy=copy.deepcopy(P2);legacy.pop('r109_speed_tier');legacy['cascade']['r85_student_lower_loop_ack'].pop('r109_speed_tier')
  self.assertIsNone(helper.configuration(legacy));self.assertIsNone(helper.configuration(P2))
  with tempfile.TemporaryDirectory()as d:
   root=Path(d);old,text=OLD['r85_dynamic_obstacle_runtime'];a,ar=node(old,text,P2,root);b,br=node(runtime,NEW_TEXT,P2,root)
   a.freeze_contract();b.freeze_contract();self.assertEqual(json.dumps(ar,allow_nan=False),json.dumps(br,allow_nan=False))
  for t in (0.,1.,8.399,8.4,20.,28.4,36.8,40.):self.assertEqual(old.target_at(t),runtime.target_at(t))
 def test_strict_typed_half_and_unsupported_configs(self):
  setting=helper.configuration(P5);self.assertEqual(helper.lead_window(setting),(8.,9.));self.assertEqual(setting['nominal_cruise_lead_s'],[16.,18.])
  self.assertGreater(8/.6,3.2/.3);self.assertGreater(8/.6,8.4)
  for change in ('missing','speed1','ACK','cap','horizon','bool'):
   p=copy.deepcopy(P5)
   if change=='missing':p.pop('r109_speed_tier')
   elif change=='speed1':p['r109_speed_tier']['cruise_mps']=1.
   elif change=='ACK':p['cascade']['r85_student_lower_loop_ack']['r109_speed_tier']['cruise_mps']=.2
   elif change=='cap':p['r109_speed_tier']['student_longitudinal_limit_mps']=.3
   elif change=='horizon':p['continuous_route_contract']['local_planning_horizon_m']=1.625
   else:p['r109_speed_tier']['simulation_only']=1
   with self.assertRaises(ValueError,msg=change):helper.configuration(p)
 def test_actual_runtime_contract_trigger_missed_and_verifier_projection(self):
  with tempfile.TemporaryDirectory()as d:
   for profile,inside,outside in ((P2,3.4,3.19),(P5,8.5,7.99)):
    n,records=node(runtime,NEW_TEXT,profile,Path(d));trigger(n,inside)
    self.assertEqual(n.started,50_000_000_000);contract=next(v for k,v in records if k=='contract')
    self.assertAlmostEqual(projection(inside,contract,profile),inside)
    with self.assertRaises(ValueError):projection(outside,contract,profile)
    other,_=node(runtime,NEW_TEXT,profile,Path(d));trigger(other,outside)
    self.assertEqual(other.failure,'encounter_trigger_window_missed_no_chase_or_late_forced_crossing')
    self.assertIsNone(other.started)
   n,records=node(runtime,NEW_TEXT,P5,Path(d));trigger(n,3.4);self.assertIsNone(n.started);self.assertIsNotNone(n.failure)
 def test_contract_cannot_self_authorize_speed_or_window(self):
  with tempfile.TemporaryDirectory()as d:
   n,r=node(runtime,NEW_TEXT,P5,Path(d));n.freeze_contract();c=r[0][1]
   for profile in (None,P2):
    with self.assertRaises(ValueError):helper.validate_contract(c,profile)
   for case in ('missing','window','budget','type'):
    bad=copy.deepcopy(c)
    if case=='missing':bad.pop(helper.KEY)
    if case=='window':bad['trigger']['lead_window_m']=[3.2,3.6]
    if case=='budget':bad[helper.KEY]['nominal_cruise_lead_s']=[6.4,7.2]
    if case=='type':bad[helper.KEY]['physical_stopping_bound_claimed']=0
    result=verifier.validate_r85_dynamic_evidence({},bad,profile=P5)
    self.assertEqual(result['state'],'failed');self.assertIn('dynamic_',result['failure_reason'])
 def test_actual_verify_result_scope_helper_and_profile_fail_closed(self):
  with tempfile.TemporaryDirectory()as d:
   root=Path(d);vp=Path(verifier.__file__).resolve();hp=Path(helper.__file__).resolve();rp=Path(runtime.__file__).resolve()
   refs={str(p):hashlib.sha256(p.read_bytes()).hexdigest()for p in (vp,hp,rp)}
   c={'binding':{'run_id':root.name,'request_id':'q'},'runtime_source_file':str(rp),'runtime_source_sha256':refs[str(rp)]}
   (root/'r85_dynamic_task_contract.json').write_text(json.dumps(c))
   scope={'profile':copy.deepcopy(P5),'references':dict(refs)}
   good=verifier.verify_result(root,'q',P5,scope)
   self.assertEqual(good['state'],'waiting');self.assertIn('r85_dynamic_obstacle_state.json',good['failure_reason'])
   bad=copy.deepcopy(scope);bad['profile']=P2
   self.assertIn('profile differs',verifier.verify_result(root,'q',P5,bad)['failure_reason'])
   bad=copy.deepcopy(scope);bad['references'].pop(str(hp))
   self.assertIn('helper is not frozen',verifier.verify_result(root,'q',P5,bad)['failure_reason'])
   old,_=OLD['r85_dynamic_obstacle_verifier'];scope={'profile':P2,'references':refs}
   self.assertEqual(old.verify_result(root,'q',P2,scope),verifier.verify_result(root,'q',P2,scope))
 def test_actual_startup_receipt_fragment_and_scope_inclusion(self):
  init=method(NEW_TEXT,'__init__');block=next(n for n in init.body if isinstance(n,ast.If)and ast.unparse(n.test)=='self.speed_lead is not None'and any('r117_dynamic_speed_lead_loaded.json' in ast.unparse(x)for x in n.body))
  with tempfile.TemporaryDirectory()as d:
   root=Path(d);hp=Path(helper.__file__).resolve();scope={'references':{str(hp):runtime.sha(hp)}}
   n,records=node(runtime,NEW_TEXT,P5,root)
   exec(compile(ast.Module(body=[copy.deepcopy(block)],type_ignores=[]),'<actual startup receipt>','exec'),dict(runtime.__dict__,self=n,run=root,scope=scope))
   receipt=records[0][1];self.assertEqual(receipt['helper'],dict(path=str(hp),sha256=runtime.sha(hp)));self.assertEqual(receipt['lead_window_m'],[8.,9.]);self.assertFalse(receipt['actual_trigger_observed'])
   n,records=node(runtime,NEW_TEXT,P2,root)
   exec(compile(ast.Module(body=[copy.deepcopy(block)],type_ignores=[]),'<actual startup receipt>','exec'),dict(runtime.__dict__,self=n,run=root,scope=scope));self.assertEqual(records,[])
  scope_ast=ast.parse((HERE/'pid_scope.py').read_text());fn=next(n for n in scope_ast.body if isinstance(n,ast.FunctionDef)and n.name=='runtime_files')
  self.assertIn("sorted(HERE.glob('*.py'))",ast.unparse(fn));self.assertIn(hp,sorted(HERE.glob('*.py')))

if __name__=='__main__':
 result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(DynamicSpeedChecks))
 output=dict(passed=result.wasSuccessful(),tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),
  actual_methods=['runtime.freeze_contract','runtime.tick exact trigger block','runtime.__init__ exact startup-receipt block','verifier.validate_r85_dynamic_evidence','verifier.verify_result','verifier actual remaining/path projection statements'],
  limitations=['Runtime methods use explicit no-Node state/writer fixtures and AST slices, not full ROS callbacks','Verifier full physical evidence chain is not fabricated or claimed PASS','Typed lead preserves nominal timing only; .95m actual braking/encounter remains physical validation'],ROS=False,simulation=False,benchmark=False)
 (A/'speed_dynamic/CHECKS.json').write_text(json.dumps(output,indent=2)+'\n');raise SystemExit(0 if result.wasSuccessful()else 1)
