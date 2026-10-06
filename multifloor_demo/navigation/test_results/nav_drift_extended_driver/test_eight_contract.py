"""Pure acceptance and exact real-driver AST method counterexamples; no ROS."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import sys
import types
import unittest
import numpy as np

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
sys.path.insert(0,str(ROOT))
from mission.route_regions import transformed_route_goals
from navigation.goal_regions import definitions_sha256
from eight_region_contract import evaluate_regions

SCENARIO=json.loads((ROOT/'simulation/scenario.json').read_text())
GOALS=transformed_route_goals(SCENARIO,'exploration',[0,0,0],{'yaw_camera_init_from_world':0})[:8]
HASH=definitions_sha256(GOALS);REQUEST='first8-unit-only'
SOURCE=(HERE/'probe_first_eight_regions.py').read_text()
TREE=ast.parse(SOURCE)

def fixture():
    poses=[dict(stamp_ns=0,p=[0.,0.,0.],q=[0,0,0,1])]
    receipts=[];statuses=[]
    for i,g in enumerate(GOALS):
        start=(i+1)*1_000_000_000;end=start+400_000_000
        for j in range(5):poses.append(dict(stamp_ns=start+j*100_000_000,p=list(g.center),q=[0,0,0,1]))
        receipts.append(dict(request_id=REQUEST,goal_id=g.goal_id,waypoint_index=i,goals_definition_sha256=HASH,
             start_stamp_ns=start,stamp_ns=end,dwell_ns=400_000_000,raw_position=list(g.center),center_error_m=0.,
             region_inside=True,control_region_inside=True,protected=False,reason='arrived',max_observation_gap_ns=200_000_000,
             arrival_definition=g.definition()['arrival'],control_arrival_definition=g.control_arrival_definition()))
        statuses.append(dict(received_stamp_ns=end,status=dict(request_id=REQUEST,state='succeeded' if i==7 else 'running',
            waypoint_index=i+1,total=8,goals_definition_sha256=HASH,goals_definitions=[x.definition() for x in GOALS],region_arrivals=copy.deepcopy(receipts))))
    # Evaluate against deliberately synthetic, identical truth samples. This
    # fixture only exercises evaluation contracts and claims no robot outcome.
    truth=copy.deepcopy(poses)
    return dict(statuses=statuses,poses=poses,truth=truth,origin=0,end=8_400_000_000)

def evaluate(f):return evaluate_regions(GOALS,REQUEST,f['statuses'],f['poses'],f['truth'],f['origin'],f['end'])

class EvaluationTests(unittest.TestCase):
    def test_eight_contract_rejects_wrong_expected_count(self):
        f=fixture()
        with self.assertRaises(ValueError):evaluate_regions(GOALS,REQUEST,f['statuses'],f['poses'],f['truth'],f['origin'],f['end'],expected_count=4)
    def test_original_eighth_slope_normal_outside_not_repaired(self):
        f=fixture();normal=np.asarray(GOALS[7].axes[2])
        f['truth'][-2]['p']=(np.asarray(f['truth'][-2]['p'])+.11*normal).tolist()
        self.assertFalse(evaluate(f)['regions'][7]['passed'])
    def test_nominal_exact_original_eight_windows(self):
        r=evaluate(fixture());self.assertTrue(r['passed']);self.assertEqual(len(r['regions']),8)
    def test_one_original_window_gt_outside_not_rescued_later(self):
        f=fixture();f['truth'][3]['p'][0]+=.36
        self.assertFalse(evaluate(f)['passed'])
    def test_outer_only_raw_cannot_arrive(self):
        f=fixture();f['poses'][3]['p'][0]+=.26
        self.assertFalse(evaluate(f)['regions'][0]['passed'])
    def test_wrong_floor(self):
        f=fixture();f['truth'][3]['p'][2]+=1.2
        self.assertFalse(evaluate(f)['passed'])
    def test_gt_missing_active_tail_explicitly_fails(self):
        f=fixture();f['truth']=f['truth'][:-1];r=evaluate(f)
        self.assertFalse(r['passed']);self.assertIn(f['end'],r['coverage']['unpaired_native_stamps'])
    def test_gt_bracket_more_than150ms_fails(self):
        f=fixture();f['truth']=[x for x in f['truth'] if x['stamp_ns']!=1_200_000_000]
        self.assertFalse(evaluate(f)['passed'])
    def test_raw_dwell_hole_more_than200ms_fails(self):
        f=fixture();f['poses']=[x for x in f['poses'] if x['stamp_ns'] not in [1_100_000_000,1_200_000_000]]
        self.assertFalse(evaluate(f)['passed'])
    def test_receipt_requires_protected_boolean_false(self):
        for bad in [True,0,None]:
            f=fixture()
            for s in f['statuses']:s['status']['region_arrivals'][0]['protected']=bad
            self.assertFalse(evaluate(f)['passed'])
    def test_same_request_wrong_definition_hash_fails(self):
        f=fixture();f['statuses'][0]['status']['goals_definition_sha256']='forged'
        self.assertFalse(evaluate(f)['passed'])
    def test_mutated_earlier_receipt_is_not_repaired(self):
        f=fixture();f['statuses'][0]['status']['region_arrivals'][0]['dwell_ns']=1
        self.assertFalse(evaluate(f)['checks']['native_receipts_immutable'])
    def test_receipt_endpoint_must_be_actual_raw_pose(self):
        f=fixture()
        for s in f['statuses']:s['status']['region_arrivals'][0]['raw_position'][0]+=.01
        self.assertFalse(evaluate(f)['passed'])
    def test_no_native_integer_stamp(self):
        f=fixture();f['poses'][2].pop('stamp_ns');self.assertFalse(evaluate(f)['passed'])
    def test_zero_invalid_quaternion_fails(self):
        f=fixture();f['truth'][0]['q']=[0,0,0,0];self.assertFalse(evaluate(f)['passed'])
    def test_missing_real_eighth_complete_callback(self):
        f=fixture();f['statuses'][-1]['status']['state']='running';self.assertFalse(evaluate(f)['passed'])

class Bool:
    def __init__(self,data):self.data=data
class Pub:
    def __init__(self):self.messages=[]
    def publish(self,msg):self.messages.append(msg)
class Qos:
    def __init__(self,**kw):self.__dict__.update(kw)
class Base:
    def __init__(self):
        self.failure=None;self.terminal_stamp_ns=None;self.origin=None
        self.feedback_health_established=False;self.feedback_health_mode_errors=[];self.bridge_events=[]
        self.expected_feedback_enabled=False;self.stop_pub=Pub();self.events=[];self.statuses=[]
        self.request_id=REQUEST;self.goals=GOALS;self.goal_hash=HASH;self.complete=None
        self.eighth_segment_entry_stamp_ns=None;self.eighth_segment_previous_receipt_stamp_ns=None
        self.calls=[]
    def now(self):return 9.
    def now_ns(self):return 9_000_000_000
    def event(self,*x):self.events.append(x)
    def create_subscription(self,*args,**kw):self.calls.append((args,kw));return args

methods={'create_subscription','abort','on_bridge','on_nav'}
cls=next(x for x in TREE.body if isinstance(x,ast.ClassDef) and x.name=='FirstEightRegionProbe')
unit_cls=ast.ClassDef(name='ActualMethods',bases=[ast.Name(id='Base',ctx=ast.Load())],keywords=[],
    body=[copy.deepcopy(x) for x in cls.body if isinstance(x,ast.FunctionDef) and x.name in methods],decorator_list=[])
unit_module=ast.fix_missing_locations(ast.Module(body=[unit_cls],type_ignores=[]))
space={'Base':Base,'json':json,'Bool':Bool,'QoSProfile':Qos,'ReliabilityPolicy':types.SimpleNamespace(BEST_EFFORT='BEST_EFFORT')}
exec(compile(unit_module,str(HERE/'probe_first_eight_regions.py'),'exec'),space)
ActualMethods=space['ActualMethods']
def message(data):return types.SimpleNamespace(data=json.dumps(data))

class ActualMethodTests(unittest.TestCase):
    def test_real_method_truth_buffer_and_modern_topic(self):
        p=ActualMethods();ordinary=object()
        a=p.create_subscription(object,'/demo/ground_truth',None,ordinary)
        self.assertEqual(a[3].depth,2000);self.assertEqual(a[3].reliability,'BEST_EFFORT')
        b=p.create_subscription(object,'/demo/test/joint_stop_safety',None,10)
        self.assertEqual(b[1],'/demo/control/joint_stop_safety')
        self.assertIs(p.create_subscription(object,'/demo/slam/body_odom',None,ordinary)[3],ordinary)
    def test_initial_null_feedback_waits_then_exact_mode_establishes(self):
        p=ActualMethods();p.on_bridge(message({'state':'hold','body_feedback':None}))
        self.assertIsNone(p.failure);self.assertFalse(p.feedback_health_established)
        p.on_bridge(message({'state':'ready','body_feedback_required':True,'body_feedback':{'enabled':False}}))
        self.assertTrue(p.feedback_health_established);self.assertIsNone(p.failure)
    def test_wrong_mode_and_lost_established_mode_immediately_stop(self):
        p=ActualMethods();p.on_bridge(message({'state':'ready','body_feedback_required':True,'body_feedback':{'enabled':True}}))
        self.assertIsNotNone(p.failure);self.assertTrue(p.stop_pub.messages[-1].data)
        p=ActualMethods();p.on_bridge(message({'state':'ready','body_feedback_required':True,'body_feedback':{'enabled':False}}))
        p.on_bridge(message({'state':'ready','body_feedback':None}))
        self.assertIsNotNone(p.failure);self.assertTrue(p.stop_pub.messages[-1].data)
    def test_aggregate_failure_before_origin_stops(self):
        p=ActualMethods();p.on_bridge(message({'state':'failed','reason':'real sensor stale','body_feedback':None}))
        self.assertIsNotNone(p.failure);self.assertEqual(p.terminal_stamp_ns,9_000_000_000)
        self.assertTrue(p.stop_pub.messages[-1].data)
    def test_failed_actual_nav_immediately_stops(self):
        p=ActualMethods();s=fixture()['statuses'][0]['status'];s['state']='failed';s['message']='tilt'
        p.on_nav(message(s));self.assertIsNotNone(p.failure);self.assertTrue(p.stop_pub.messages[-1].data)
    def test_no_fake_complete_index_without_eight_receipts(self):
        p=ActualMethods();s=fixture()['statuses'][-1]['status'];s['region_arrivals']=s['region_arrivals'][:7]
        p.on_nav(message(s));self.assertIsNone(p.complete);self.assertIsNotNone(p.failure)
    def test_real_eighth_receipt_sets_terminal_and_stops(self):
        p=ActualMethods();p.on_nav(message(fixture()['statuses'][6]['status']))
        self.assertEqual(p.eighth_segment_previous_receipt_stamp_ns,7_400_000_000)
        p.on_nav(message(fixture()['statuses'][-1]['status']))
        self.assertEqual(p.complete,9.);self.assertEqual(p.terminal_stamp_ns,9_000_000_000)
        self.assertTrue(p.stop_pub.messages[-1].data)
    def test_control_request_never_reads_truth_and_uses_full_canonical_transform(self):
        tick=next(x for x in cls.body if isinstance(x,ast.FunctionDef) and x.name=='tick')
        self.assertFalse(any(isinstance(x,ast.Attribute) and x.attr=='truth' for x in ast.walk(tick)))
        self.assertTrue(any(isinstance(x,ast.Call) and isinstance(x.func,ast.Name) and x.func.id=='transformed_route_goals' for x in ast.walk(tick)))
        publishers=[x for x in ast.walk(cls) if isinstance(x,ast.Call) and isinstance(x.func,ast.Attribute) and x.func.attr=='create_publisher']
        self.assertFalse(any(x.args and isinstance(x.args[0],ast.Name) and x.args[0].id=='Twist' for x in publishers))

if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromModule(sys.modules[__name__])
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    receipt=dict(scope='pure evaluation + unmodified actual driver method AST, no ROS nodes or physics',
        tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful(),
        files={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in ['probe_first_eight_regions.py','eight_region_contract.py','test_eight_contract.py']})
    (HERE/'contract_result.json').write_text(json.dumps(receipt,indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
