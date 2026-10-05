#!/usr/bin/env python3
"""Actual controller AST with excluded shared inner candidate, no ROS/physics."""
import copy,hashlib,importlib.util,json,math,os,sys,unittest
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
HERE=Path(__file__).parent

def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);mod=importlib.util.module_from_spec(spec)
    sys.modules[name]=mod;spec.loader.exec_module(mod);return mod
NAV=HERE.parents[1]
PRODUCTION=os.environ.get('DEMO_TEST_PRODUCTION_INNER')=='1'
CORE_PATH=(NAV if PRODUCTION else HERE)/'goal_regions.py'
CONTROLLER_PATH=(NAV if PRODUCTION else HERE)/'controller.py'
core=load('goal_regions',CORE_PATH)
os.environ['DEMO_TEST_CONTROLLER_SOURCE']=str(CONTROLLER_PATH)
old=load('actual_method_fixture',HERE.parent/'goal_region_staging/test_controller_methods.py')

def setup():
    n=old.ControlCallbacks();n.state='stopped'
    r=old.request()
    r['request_id']='inner-contract'
    r['goals'][0]['arrival']['control_band']=dict(radius_m=.25,height_half_span_m=.1)
    n.on_request(SimpleNamespace(data=json.dumps(r)))
    n.pose=n.region_raw_pose=n.tracking_pose=np.array([.3,0.,0.]);n.rotation=Rotation.from_euler('z',math.pi).as_matrix()
    n.segment_start=np.array([1.,0.,0.]);n.segment_started_ros=10.
    n.samples=np.array([[1.,0.,0.],[0.,0.,0.]])
    n.cloud=np.array([[5.,5.,-.5]])
    n.heading_gate.phase='drive';n.obstacle_hold=False
    return n,r

class InnerMethods(unittest.TestCase):
    def test_outer_inside_does_not_stop_real_checked_path_execution(self):
        n,r=setup();n.tick_at(10.)
        self.assertEqual(n.state,'running');self.assertEqual(n.waypoint_index,0)
        self.assertTrue(n.region_arrival_evidence['region_inside']);self.assertFalse(n.region_arrival_evidence['control_region_inside'])
        self.assertIsNone(n.region_arrival.since);self.assertGreater(n.zeros[-1][0],.015)
        self.assertEqual(n.region_arrival_evidence['control_arrival_definition']['radius_m'],.25)
    def test_inner_receipt_has_both_geometries_and_full_native_dwell(self):
        n,r=setup();n.pose=n.region_raw_pose=n.tracking_pose=np.array([.24,0.,0.])
        for t in (10.,10.1,10.2,10.3,10.4):n.tick_at(t)
        self.assertEqual(n.state,'succeeded');self.assertEqual(n.waypoint_index,1)
        e=n.region_arrivals[0];self.assertTrue(e['region_inside']);self.assertTrue(e['control_region_inside'])
        self.assertEqual(e['stamp_ns']-e['start_stamp_ns'],400000000)
        self.assertEqual(e['arrival_definition']['radius_m'],.35)
        self.assertEqual(e['arrival_definition']['control_band'],e['control_arrival_definition'])
        self.assertEqual(e['goals_definition_sha256'],core.definitions_sha256(n.goals))
        self.assertTrue(all(c==[0.,0.,0.] for c in n.zeros))
    def test_changing_only_control_band_same_request_ID_is_rejected(self):
        n,r=setup();h=n.goals_definition_sha256;samples=n.samples
        r['goals'][0]['arrival']['control_band']['radius_m']=.20
        n.on_request(SimpleNamespace(data=json.dumps(r)))
        self.assertEqual(n.goals_definition_sha256,h);self.assertIs(n.samples,samples);self.assertEqual(n.state,'running')
        self.assertIsNotNone(n.last_rejected)

if __name__=='__main__':
    suite=unittest.TestSuite([unittest.defaultTestLoader.loadTestsFromTestCase(k) for k in (old.MethodsTests,old.HeldRegionControlTests,InnerMethods)])
    r=unittest.TextTestRunner(verbosity=2).run(suite)
    (HERE/('production_controller_result.json' if PRODUCTION else 'controller_result.json')).write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,scope=('Actual production controller methods AST and production shared inner geometry; no ROS/physics.' if PRODUCTION else __doc__),production=PRODUCTION,source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (CORE_PATH,CONTROLLER_PATH)},failures=[(str(t),s) for t,s in r.errors+r.failures]),indent=2)+'\n')
    raise SystemExit(not r.wasSuccessful())
