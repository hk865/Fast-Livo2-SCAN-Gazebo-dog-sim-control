#!/usr/bin/env python3
"""Execute actual staged result/health methods without ROS or sensor publishers."""
import ast,hashlib,json,tempfile,types,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

HERE=Path(__file__).resolve().parent
SOURCE=HERE/'probe_first_eight.py'
tree=ast.parse(SOURCE.read_text())
nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='bounded_truth'
       or isinstance(n,ast.ClassDef) and n.name=='FirstEightProbe']

class Parent:
    def on_bridge(self,msg):self.bridge=json.loads(msg.data)
    def create_subscription(self,msg_type,topic,callback,qos_profile,*args,**kwargs):
        return qos_profile

OFFSETS=[[0,-1,0],[1,-1,0],[1,0,0],[0,0,0],[0,2,0],[2,2,0],[5,2,.3],[8,2,.6]]
scope=dict(SlopeProbe=Parent,np=np,Rotation=Rotation,Slerp=Slerp,json=json,math=__import__('math'),
    hashlib=hashlib,OFFSETS=OFFSETS,__doc__='Actual first8 method fixture, no ROS',
    QoSProfile=lambda **kw:types.SimpleNamespace(**kw),
    ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='BEST_EFFORT'),
    Bool=lambda **kw:types.SimpleNamespace(**kw),audit_held_commands=lambda *a:{} )
exec(compile(ast.Module(body=nodes,type_ignores=[]),str(SOURCE),'exec'),scope)
Probe=scope['FirstEightProbe']

def point(t):return dict(stamp=t,p=[0.,0.,0.],q=[0.,0.,0.,1.])
def healthy(enabled=True):return dict(state='ready',body_feedback_required=True,
    body_feedback=dict(state='active' if enabled else 'disabled',enabled=enabled,failed=False))

def bare():
    p=object.__new__(Probe);p.bridge_events=[];p.feedback_health_mode_errors=[]
    p.feedback_health_established=False;p.expected_feedback_enabled=True
    p.origin=None;p.failure=None;p.bridge={};p.now=lambda:10.0
    p.stops=[];p.stop_pub=types.SimpleNamespace(publish=lambda m:p.stops.append(m.data));p.event=lambda *a:None
    return p

class Contracts(unittest.TestCase):
    def test_truth_subscription_bounded_queue_2000(self):
        p=bare();original=types.SimpleNamespace(depth=5)
        qos=p.create_subscription(object,'/demo/ground_truth',None,original)
        self.assertEqual(qos.depth,2000);self.assertEqual(qos.reliability,'BEST_EFFORT')
    def test_control_and_raw_IMU_subscription_unchanged(self):
        p=bare();original=types.SimpleNamespace(depth=1)
        for topic in ['/livox/imu','/demo/control/safety','/demo/navigation/status']:
            self.assertIs(p.create_subscription(object,topic,None,original),original)
    def method_result(self,poses,truth,complete=10.2):
        with tempfile.TemporaryDirectory() as directory:
            p=bare();p.output=Path(directory)/'result.json'
            (p.output.parent/'trajectory_payloads.jsonl').write_text('')
            p.curve_file=types.SimpleNamespace(flush=lambda:None)
            p.origin=[0,0,0];p.origin_stamp=10.;p.complete=complete;p.request_id='actual'
            p.run_id='actual';p.goals=OFFSETS;p.poses=poses;p.truth=truth
            p.statuses=[dict(sim=9.99,status=dict(request_id='actual',waypoint_index=0,state='running')),
                dict(sim=complete,status=dict(request_id='actual',waypoint_index=8,state='succeeded'))]
            p.nav_confirmations=[];p.obstacle_states=[];p.command_evidence=[];p.raw=[]
            p.baseline_holds=0;p.contacts=[];p.contact_publisher_seen=True
            p.command_publisher_names=['demo_navigation'];p.alignment={}
            p.foot_contacts={leg:dict(nonempty=1,violations=[]) for leg in ('lf','rf','lh','rh')}
            p.bridge_events=[dict(sim=10.,safety=healthy())]
            return p.result()
    def test_missing_active_truth_tail_fails(self):
        r=self.method_result([point(10),point(10.1)],[point(10)])
        self.assertFalse(r['checks']['independent_truth_full_pose_coverage'])
        self.assertEqual(r['unpaired_truth_pose_rows'],1)
    def test_cleanup_pose_tail_is_outside_actual_task(self):
        r=self.method_result([point(10),point(10.1),point(10.4)],[point(10),point(10.1)])
        self.assertTrue(r['checks']['independent_truth_full_pose_coverage'])
        self.assertEqual(r['independent_truth_evaluation_interval'],[10.,10.2])
    def test_oversize_interior_truth_gap_fails(self):
        r=self.method_result([point(10),point(10.1)],[point(10),point(10.3)])
        self.assertFalse(r['checks']['independent_truth_full_pose_coverage'])
    def test_bounded_interior_truth_bracket_passes(self):
        r=self.method_result([point(10),point(10.05)],[point(10),point(10.1)])
        self.assertTrue(r['checks']['independent_truth_full_pose_coverage'])
    def test_precalibration_none_waits(self):
        p=bare();p.on_bridge(types.SimpleNamespace(data=json.dumps(dict(state='hold',body_feedback=None))))
        self.assertIsNone(p.failure);self.assertFalse(p.feedback_health_established);self.assertFalse(p.stops)
    def test_mode_established_then_missing_fails(self):
        p=bare();p.on_bridge(types.SimpleNamespace(data=json.dumps(healthy())))
        self.assertTrue(p.feedback_health_established)
        p.on_bridge(types.SimpleNamespace(data=json.dumps(dict(state='ready',body_feedback=None))))
        self.assertIsNotNone(p.failure);self.assertEqual(p.stops,[True])
    def test_candidate_disabled_bypass_fails(self):
        p=bare();p.on_bridge(types.SimpleNamespace(data=json.dumps(healthy(False))))
        self.assertIsNotNone(p.failure);self.assertEqual(p.stops,[True])
    def test_baseline_disabled_is_valid(self):
        p=bare();p.expected_feedback_enabled=False;p.on_bridge(types.SimpleNamespace(data=json.dumps(healthy(False))))
        self.assertTrue(p.feedback_health_established);self.assertIsNone(p.failure)
    def test_enabled_integer_is_not_valid_boolean(self):
        p=bare();s=healthy();s['body_feedback']['enabled']=1
        p.on_bridge(types.SimpleNamespace(data=json.dumps(s)));self.assertIsNotNone(p.failure)
    def test_required_flag_missing_fails(self):
        p=bare();s=healthy();s.pop('body_feedback_required')
        p.on_bridge(types.SimpleNamespace(data=json.dumps(s)));self.assertIsNotNone(p.failure)
    def test_aggregate_failure_stops_before_calibration(self):
        p=bare();s=healthy();s.update(state='failed',reason='actual_feedback_sensor_stale')
        p.on_bridge(types.SimpleNamespace(data=json.dumps(s)))
        self.assertEqual(p.stops,[True]);self.assertIn('actual_feedback_sensor_stale',p.failure)

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Contracts))
    (HERE/'acceptance_contract_result.json').write_text(json.dumps(dict(passed=result.wasSuccessful(),
        tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),scope=__doc__,
        actual_staged_driver_SHA256=hashlib.sha256(SOURCE.read_bytes()).hexdigest()),indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
