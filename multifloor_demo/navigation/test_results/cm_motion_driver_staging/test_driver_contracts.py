#!/usr/bin/env python3
"""Actual staged methods via AST, no ROS initialization or physics claim."""
import ast
import hashlib
import json
import math
from pathlib import Path
import tempfile
import types
import unittest
import numpy as np
from scipy.spatial.transform import Rotation,Slerp

HERE=Path(__file__).resolve().parent


class Parent:
    def create_subscription(self,msg_type,topic,callback,qos,*args,**kwargs):return (topic,qos)


def load(source,name,offsets):
    tree=ast.parse(source.read_text())
    nodes=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='bounded_truth'
           or isinstance(n,ast.ClassDef) and n.name==name]
    scope=dict(SlopeProbe=Parent,OFFSETS=offsets,np=np,Rotation=Rotation,Slerp=Slerp,
        math=math,json=json,hashlib=hashlib,__doc__=__doc__,
        QoSProfile=lambda **kw:types.SimpleNamespace(**kw),ReliabilityPolicy=types.SimpleNamespace(BEST_EFFORT='best_effort'),
        Bool=lambda **kw:types.SimpleNamespace(**kw),audit_held_commands=lambda *args:dict(passed=True))
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(source),'exec'),scope)
    return scope[name]


SOURCES=[(HERE/'probe_slope_motion.py','SlopeStrengthProbe',[[3.,0.,.3]]),
         (HERE/'probe_dynamic_motion.py','DynamicProbe',[[0.,2.,0.],[2.,2.,0.]])]
CLASSES=[(load(path,name,offsets),offsets) for path,name,offsets in SOURCES]


def point(t):return dict(stamp=t,p=[0.,0.,0.],q=[0.,0.,0.,1.])


def bare(cls):
    p=object.__new__(cls)
    p.failure=None;p.origin=None;p.origin_stamp=None;p.terminal_sim=None;p.complete=None
    p.bridge={};p.bridge_events=[];p.adapter_events=[];p.bridge_failure_seen=False
    p.stops=[];p.stop_pub=types.SimpleNamespace(publish=lambda m:p.stops.append(m.data))
    p.now=lambda:10.2;p.event=lambda *a:None
    return p


class ActualMethodContracts(unittest.TestCase):
    def result(self,cls,offsets,poses,truth):
        with tempfile.TemporaryDirectory() as folder:
            p=bare(cls);p.output=Path(folder)/'result.json'
            (p.output.parent/'trajectory_payloads.jsonl').write_text('')
            p.curve_file=types.SimpleNamespace(flush=lambda:None)
            p.origin=[0,0,0];p.origin_stamp=10.;p.complete=10.2;p.terminal_sim=10.2
            p.request_id='actual';p.run_id='actual';p.goals=offsets
            p.poses=poses;p.truth=truth;p.nav_confirmations=[]
            p.statuses=[dict(sim=9.99,status=dict(request_id='actual',waypoint_index=0,state='running')),
                dict(sim=10.2,status=dict(request_id='actual',waypoint_index=len(offsets),state='succeeded'))]
            p.obstacle_states=[];p.command_evidence=[];p.raw=[];p.baseline_holds=0
            p.contacts=[];p.contact_publisher_seen=True;p.command_publisher_names=['demo_navigation'];p.alignment={}
            p.foot_contacts={leg:dict(nonempty=1,violations=[]) for leg in ('lf','rf','lh','rh')}
            p.adapter_events=[dict(sim=10.,status=dict(test_only=False,failed=False))]
            return p.result()

    def test_actual_subscription_truth_depth_and_modern_single_topic(self):
        for cls,_ in CLASSES:
            p=bare(cls);old=types.SimpleNamespace(depth=5)
            topic,qos=p.create_subscription(object,'/demo/ground_truth',None,old)
            self.assertEqual(topic,'/demo/ground_truth');self.assertEqual(qos.depth,2000)
            self.assertEqual(qos.reliability,'best_effort')
            topic,qos=p.create_subscription(object,'/demo/test/joint_stop_safety',None,old)
            self.assertEqual(topic,'/demo/control/joint_stop_safety');self.assertIs(qos,old)

    def test_other_input_qos_unchanged(self):
        for cls,_ in CLASSES:
            p=bare(cls);q=object()
            for topic in ('/livox/imu','/demo/slam/body_odom','/demo/control/safety'):
                mapped,actual=p.create_subscription(object,topic,None,q)
                self.assertEqual(mapped,topic);self.assertIs(actual,q)

    def test_failed_bridge_before_origin_immediately_requests_stop(self):
        for cls,_ in CLASSES:
            p=bare(cls);p.on_bridge(types.SimpleNamespace(data=json.dumps(dict(state='failed',reason='observed_sensor_stale'))))
            self.assertEqual(p.stops,[True]);self.assertIsNone(p.origin)
            self.assertIn('observed_sensor_stale',p.failure);self.assertTrue(p.bridge_failure_seen)
            self.assertEqual(p.terminal_sim,10.2)
            p.tick() # No readiness or goal publication after latched failure.

    def test_healthy_startup_hold_does_not_invent_failure_or_origin(self):
        for cls,_ in CLASSES:
            p=bare(cls);p.on_bridge(types.SimpleNamespace(data=json.dumps(dict(state='hold',reason='imu_initial_stability'))))
            self.assertIsNone(p.failure);self.assertFalse(p.stops);self.assertIsNone(p.origin)

    def test_modern_adapter_failed_immediate_stop(self):
        for cls,_ in CLASSES:
            p=bare(cls);p.on_adapter(types.SimpleNamespace(data=json.dumps(dict(failed=True,test_only=False))))
            self.assertEqual(p.stops,[True]);self.assertIn('modern joint-reference',p.failure)
            self.assertEqual(p.terminal_sim,10.2)

    def test_missing_active_truth_tail_fails_real_result_method(self):
        for cls,offsets in CLASSES:
            r=self.result(cls,offsets,[point(10),point(10.1)],[point(10)])
            self.assertFalse(r['checks']['independent_truth_full_pose_coverage'])
            self.assertEqual(r['unpaired_truth_pose_rows'],1)

    def test_large_interior_truth_gap_fails_real_result_method(self):
        for cls,offsets in CLASSES:
            r=self.result(cls,offsets,[point(10),point(10.1)],[point(10),point(10.3)])
            self.assertFalse(r['checks']['independent_truth_full_pose_coverage'])
            self.assertEqual(r['unpaired_truth_pose_rows'],1)

    def test_bounded_truth_bracket_coverage_is_accepted(self):
        for cls,offsets in CLASSES:
            r=self.result(cls,offsets,[point(10),point(10.05)],[point(10),point(10.1)])
            self.assertTrue(r['checks']['independent_truth_full_pose_coverage'])

    def test_cleanup_pose_after_actual_terminal_is_only_excluded_tail(self):
        for cls,offsets in CLASSES:
            r=self.result(cls,offsets,[point(10),point(10.1),point(10.4)],[point(10),point(10.1)])
            self.assertTrue(r['checks']['independent_truth_full_pose_coverage'])
            self.assertEqual(r['independent_truth_evaluation_interval'],[10.,10.2])

    def test_no_strength_override_or_Twist_publisher_in_drivers(self):
        for source,_,_ in SOURCES:
            text=source.read_text();tree=ast.parse(text)
            self.assertNotIn('controller_strength_wrapper',text)
            self.assertNotIn('MAX_WALK_YAW_RATE',text)
            for node in ast.walk(tree):
                if isinstance(node,ast.Call) and isinstance(node.func,ast.Attribute) and node.func.attr=='create_publisher' and node.args:
                    self.assertNotEqual(ast.unparse(node.args[0]),'Twist')


if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ActualMethodContracts))
    (HERE/'contract_result.json').write_text(json.dumps(dict(scope=__doc__,passed=result.wasSuccessful(),tests=result.testsRun,
        failures=len(result.failures),errors=len(result.errors),driver_sha256={path.name:hashlib.sha256(path.read_bytes()).hexdigest() for path,_,_ in SOURCES}),indent=2)+'\n')
    raise SystemExit(not result.wasSuccessful())
