"""Pure original-window + exact staged method negatives. No nodes or physics."""
import ast
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as S
import unittest
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT))
from navigation.goal_regions import parse_goal,definitions_sha256
from dynamic_region_contract import evaluate_regions
GOALS=tuple(parse_goal(dict(goal_id='dynamic:'+str(i),center=g,arrival=dict(type='sphere',radius_m=.30,
    dwell_sim_s=.4,control_band=dict(type='sphere',radius_m=.22)),timeout_sim_s=90)) for i,g in enumerate([[0,2,0],[2,2,0]]))
HASH=definitions_sha256(GOALS);REQUEST='actual-two-unit-fixture'

def fixture():
    poses=[dict(stamp_ns=0,p=[0.,0.,0.],q=[0,0,0,1])];receipts=[];statuses=[]
    for i,g in enumerate(GOALS):
        start=(i+1)*1000000000;end=start+400000000
        for j in range(5):poses.append(dict(stamp_ns=start+j*100000000,p=list(g.center),q=[0,0,0,1]))
        receipts.append(dict(request_id=REQUEST,goal_id=g.goal_id,waypoint_index=i,goals_definition_sha256=HASH,
            start_stamp_ns=start,stamp_ns=end,dwell_ns=400000000,raw_position=list(g.center),center_error_m=0.,
            region_inside=True,control_region_inside=True,protected=False,reason='arrived',max_observation_gap_ns=200000000,
            arrival_definition=g.definition()['arrival'],control_arrival_definition=g.control_arrival_definition()))
        statuses.append(dict(sim=end/1e9,received_stamp_ns=end,status=dict(request_id=REQUEST,state='succeeded' if i==1 else 'running',
            waypoint_index=i+1,total=2,goals_definition_sha256=HASH,goals_definitions=[g.definition() for g in GOALS],region_arrivals=copy.deepcopy(receipts))))
    return dict(statuses=statuses,poses=poses,truth=copy.deepcopy(poses),origin=0,end=2400000000)
def evaluate(f):return evaluate_regions(GOALS,REQUEST,f['statuses'],f['poses'],f['truth'],f['origin'],f['end'],expected_count=2)

class Pure(unittest.TestCase):
    def test_same_declared_two_centers_and_original_bands(self):
        self.assertEqual([list(g.center) for g in GOALS],[[0,2,0],[2,2,0]])
        self.assertTrue(all(g.radius==.30 and g.control_goal().radius==.22 and g.dwell_sim_s==.4 and g.timeout_sim_s==90 for g in GOALS))
        self.assertTrue(evaluate(fixture())['passed'])
    def test_raw_outer_only_cannot_complete_inner_window(self):
        f=fixture();f['poses'][3]['p'][0]+=.23
        self.assertFalse(evaluate(f)['passed'])
    def test_gt_over_point_three_never_repaired_by_later_window(self):
        f=fixture();f['truth'][3]['p'][0]+=.300001
        self.assertFalse(evaluate(f)['passed'])
    def test_protected_native_window_cannot_count(self):
        f=fixture();f['statuses'][0]['status']['region_arrivals'][0]['protected']=True
        self.assertFalse(evaluate(f)['passed'])
    def test_dwell_hole_over_point_two_fails(self):
        f=fixture();f['poses']=[p for p in f['poses'] if p['stamp_ns'] not in [1100000000,1200000000]]
        self.assertFalse(evaluate(f)['passed'])
    def test_real_truth_tail_missing_fails(self):
        f=fixture();f['truth']=f['truth'][:-1];self.assertFalse(evaluate(f)['passed'])
    def test_truth_interior_gap_over_point_onefive_fails(self):
        f=fixture();f['truth']=[p for p in f['truth'] if p['stamp_ns']!=1200000000]
        self.assertFalse(evaluate(f)['passed'])
    def test_fake_completed_index_without_original_receipts_fails(self):
        f=fixture();f['statuses'][-1]['status']['region_arrivals']=f['statuses'][-1]['status']['region_arrivals'][:1]
        self.assertFalse(evaluate(f)['passed'])
    def test_different_request_hash_replay_fails(self):
        f=fixture();f['statuses'][0]['status']['goals_definition_sha256']='different'
        self.assertFalse(evaluate(f)['passed'])

class Base:
    def create_subscription(self,*a,**kw):return a
    def on_nav(self,msg):
        self.nav=json.loads(msg.data);self.statuses.append(dict(sim=self.now(),status=self.nav))
        if self.nav.get('state')=='succeeded':self.complete=self.now()

tree=ast.parse((HERE/'probe_dynamic_drift.py').read_text())
klass=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='DynamicProbe')
names={'on_bridge','on_nav','create_subscription'}
klass=ast.ClassDef(name='ExactMethods',bases=[ast.Name(id='Base',ctx=ast.Load())],keywords=[],
    body=[copy.deepcopy(n) for n in klass.body if isinstance(n,ast.FunctionDef) and n.name in names],decorator_list=[])
scope=dict(Base=Base,json=json,np=np,Bool=lambda **k:S(**k),QoSProfile=lambda **k:S(**k),ReliabilityPolicy=S(BEST_EFFORT='best_effort'))
exec(compile(ast.fix_missing_locations(ast.Module(body=[klass],type_ignores=[])),str(HERE/'probe_dynamic_drift.py'),'exec'),scope)
Methods=scope['ExactMethods']
def probe():
    p=Methods();p.origin=None;p.expected_feedback_enabled=False;p.feedback_health_established=False;p.feedback_health_mode_errors=[]
    p.failure=None;p.bridge_failure_seen=False;p.bridge_events=[];p.statuses=[];p.terminal_sim=None;p.terminal_stamp_ns=None;p.complete=None
    p.now=lambda:3.;p.get_clock=lambda:S(now=lambda:S(nanoseconds=3000000000));p.stops=[]
    p.stop_pub=S(publish=lambda m:p.stops.append(m.data));p.event=lambda *a:None
    p.request_id=REQUEST;p.goal_hash=HASH;p.declared_goals=GOALS;p.last_confirmed_index=0;p.nav_confirmations=[]
    return p
def msg(v):return S(data=json.dumps(v))

class Actual(unittest.TestCase):
    def test_actual_truth_qos_and_single_modern_topic(self):
        p=probe();q=p.create_subscription(object,'/demo/ground_truth',None,object());self.assertEqual(q[3].depth,2000)
        self.assertEqual(p.create_subscription(object,'/demo/test/joint_stop_safety',None,10)[1],'/demo/control/joint_stop_safety')
    def test_null_feedback_waits_then_required_disabled_bool(self):
        p=probe();p.on_bridge(msg(dict(state='hold',body_feedback=None)));self.assertIsNone(p.failure)
        p.on_bridge(msg(dict(state='ready',body_feedback_required=True,body_feedback=dict(enabled=False))))
        self.assertTrue(p.feedback_health_established)
    def test_mode_mismatch_or_loss_immediate_stop(self):
        for feedback in [dict(enabled=True),dict(enabled=0)]:
            p=probe();p.on_bridge(msg(dict(state='ready',body_feedback_required=True,body_feedback=feedback)))
            self.assertIsNotNone(p.failure);self.assertEqual(p.stops,[True])
        p=probe();p.feedback_health_established=True;p.on_bridge(msg(dict(state='ready',body_feedback=None)))
        self.assertIsNotNone(p.failure);self.assertEqual(p.stops,[True])
    def test_aggregate_first_failed_preserved_and_stopped(self):
        p=probe();p.on_bridge(msg(dict(state='failed',reason='actual_first_failure',body_feedback=None)))
        self.assertIn('actual_first_failure',p.failure);self.assertEqual(p.stops,[True]);self.assertEqual(p.terminal_stamp_ns,3000000000)
        p.on_bridge(msg(dict(state='failed',reason='later_failure',body_feedback=None)))
        self.assertIn('actual_first_failure',p.failure)
    def test_actual_missing_declared_hash_stops(self):
        p=probe();n=fixture()['statuses'][0]['status'];n['goals_definition_sha256']='wrong';p.on_nav(msg(n))
        self.assertIsNotNone(p.failure);self.assertEqual(p.stops,[True])
    def test_actual_fake_index_two_without_native_receipts_stops(self):
        p=probe();n=fixture()['statuses'][-1]['status'];n['region_arrivals']=[];p.on_nav(msg(n))
        self.assertIsNotNone(p.failure);self.assertEqual(p.stops,[True])
    def test_actual_matching_native_success_records_two_indices(self):
        p=probe();p.on_nav(msg(fixture()['statuses'][-1]['status']))
        self.assertIsNone(p.failure);self.assertEqual(len(p.nav_confirmations),2);self.assertEqual(p.terminal_stamp_ns,3000000000)
    def test_request_method_no_truth_or_twist_publication(self):
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='DynamicProbe')
        tick=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='tick')
        self.assertFalse(any(isinstance(n,ast.Attribute) and n.attr=='truth' for n in ast.walk(tick)))
        self.assertFalse(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='create_publisher'
            and n.args and ast.unparse(n.args[0])=='Twist' for n in ast.walk(cls)))
        text=ast.unparse(tick);self.assertIn('schema_version=2',text);self.assertNotIn('waypoints=self.goals',text)

if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__]);r=unittest.TextTestRunner(verbosity=2).run(suite)
    value=dict(passed=r.wasSuccessful(),tests=r.testsRun,failures=len(r.failures),errors=len(r.errors),
        scope='selected two sphere windows + actual driver methods, no nodes/physics',
        sha256={n:hashlib.sha256((HERE/n).read_bytes()).hexdigest() for n in ['probe_dynamic_drift.py','dynamic_region_contract.py','held_command_contract.py','test_dynamic_contract.py']})
    (HERE/'dynamic_contract_result.json').write_text(json.dumps(value,indent=2)+'\n')
    raise SystemExit(not r.wasSuccessful())
