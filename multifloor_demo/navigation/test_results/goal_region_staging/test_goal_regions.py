#!/usr/bin/env python3
"""Design-kernel tests only; no ROS, physics, arrival success or control claim."""
import copy,hashlib,json,math,unittest,os,sys
from dataclasses import FrozenInstanceError
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
if os.environ.get('DEMO_TEST_PRODUCTION_GOAL_CORE')=='1':
    sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from goal_regions import *

def disc():
    return dict(goal_id='floor-1',center=[0.,2.,0.],arrival=dict(type='disc_prism',radius_m=.35,height_half_span_m=.1,dwell_sim_s=.4),timeout_sim_s=90,route_surface_id='floor_1')

def ramp():
    t=np.array([1.,0.,.1]);t/=np.linalg.norm(t);l=np.array([0.,1.,0.]);n=np.cross(t,l)
    return dict(goal_id='ramp-8',center=[8.,2.,.6],arrival=dict(type='oriented_box',axes=[t.tolist(),l.tolist(),n.tolist()],half_extents_m=[.35,.30,.10],dwell_sim_s=.4),timeout_sim_s=90,route_surface_id='ramp_12')

class KernelTests(unittest.TestCase):
    def test_legacy_ball_is_still_open_and_three_dimensional(self):
        rid,gs=parse_request(dict(request_id='old',frame_id='camera_init',waypoints=[[0,0,0]]))
        self.assertTrue(gs[0].legacy);self.assertTrue(contains(gs[0],[0,0,.219999]))
        self.assertFalse(contains(gs[0],[0,0,.22]));self.assertFalse(contains(gs[0],[.17,.17,0]))
    def test_flat_region_wrong_floor_rejected_even_exact_XY(self):
        g=parse_goal(disc());self.assertTrue(contains(g,[.2,2,.05]))
        for z in (1.2,2.4,-1.2):self.assertFalse(contains(g,[0,2,z]))
    def test_flat_region_closed_boundary_and_real_outside(self):
        g=parse_goal(disc());self.assertTrue(contains(g,[.35,2,.1]))
        self.assertFalse(contains(g,[.350001,2,0]));self.assertFalse(contains(g,[0,2,.100001]))
    def test_ramp_longitudinal_cross_and_normal_limits_independent(self):
        g=parse_goal(ramp());axes=np.array(g.axes);center=np.array(g.center)
        self.assertTrue(contains(g,center+axes.T@np.array([.35,.30,.10])))
        for local in ([.351,0,0],[0,.301,0],[0,0,.101]):self.assertFalse(contains(g,center+axes.T@local))
    def test_ramp_acceptance_follows_actual_prespecified_plane(self):
        g=parse_goal(ramp());self.assertTrue(contains(g,[8.30,2,.63]))
        self.assertFalse(contains(g,[8.30,2,.8]));self.assertFalse(contains(g,[8,2,1.2]))
    def test_transform_center_and_axes_together(self):
        g=parse_goal(ramp());tf=relative_world_transform([10,-3,2],math.pi/2);rot=tf.goal(g)
        self.assertTrue(np.allclose(rot.center,[8,5,2.6]))
        for p in ([8.1,2.05,.66],[8.6,2,.6],[8,2,1.2]):self.assertEqual(contains(g,p),contains(rot,tf.point(p)))
    def test_transform_preserves_region_definition_and_goal_identity(self):
        g=parse_goal(ramp());h=relative_world_transform([1,2,3],.7).goal(g)
        self.assertEqual(g.goal_id,h.goal_id);self.assertEqual(g.half_extents,h.half_extents)
        self.assertEqual(g.timeout_sim_s,h.timeout_sim_s);self.assertEqual(g.route_surface_id,h.route_surface_id)
    def test_one_initial_SE3_truth_uses_same_region(self):
        rg=Rotation.from_euler('xyz',[.02,-.04,.8]);rs=Rotation.from_euler('xyz',[-.01,.03,-.2])
        tf=initial_truth_alignment([3,4,5],rs.as_quat(),[1,2,3],rg.as_quat())
        self.assertTrue(np.allclose(tf.point([1,2,3]),[3,4,5]))
        g=parse_goal(disc());self.assertEqual(contains(g,[0,2,0]),contains(tf.goal(g),tf.point([0,2,0])))
        outside=[0,2,1.2];self.assertFalse(contains(tf.goal(g),tf.point(outside)))
    def test_schema_conflicts_duplicate_goal_and_missing_version(self):
        base=dict(request_id='new',frame_id='camera_init',schema_version=2,goals=[disc()])
        for changed in (dict(base,waypoints=[[0,0,0]]),dict(base,goals=[disc(),disc()]),dict(base,schema_version=1),dict(base,frame_id='world')):
            with self.assertRaises(ValueError):parse_request(changed)
    def test_nonfinite_and_boolean_geometry_rejected(self):
        for value in (float('nan'),float('inf'),True):
            d=disc();d['center'][0]=value
            with self.assertRaises(ValueError):parse_goal(d)
        for point in ([0,2,float('nan')],[0,2,float('inf')]):
            with self.assertRaises(ValueError):contains(parse_goal(disc()),point)
    def test_extreme_finite_input_does_not_create_infinite_roundoff_acceptance(self):
        g=parse_goal(disc());self.assertFalse(contains(g,[1e200,2,0]))
        d=disc();d['center'][0]=1e200
        with self.assertRaises(ValueError):parse_goal(d)
    def test_wrong_basis_dimension_scale_handedness_and_height_rejected(self):
        for axes in (None,[[1,0,0],[0,1,0]],[[2,0,0],[0,1,0],[0,0,1]],[[1,0,0],[0,1,0],[0,0,-1]]):
            d=disc();d['arrival']['axes']=axes
            with self.assertRaises(ValueError):parse_goal(d)
        d=disc();del d['arrival']['height_half_span_m']
        with self.assertRaises(ValueError):parse_goal(d)
    def test_geometry_cannot_be_mutated_after_freeze(self):
        raw=disc();g=parse_goal(raw);raw['center'][0]=99
        self.assertEqual(g.center[0],0.)
        with self.assertRaises(FrozenInstanceError):g.radius=3
        with self.assertRaises(TypeError):g.center[0]=3
    def test_definition_hash_changes_on_any_new_acceptance_bounds(self):
        g=parse_goal(disc());d=disc();d['arrival']['radius_m']=.4;h=parse_goal(d)
        self.assertNotEqual(definitions_sha256([g]),definitions_sha256([h]))
        self.assertEqual(definitions_sha256([g]),definitions_sha256([parse_goal(g.definition())]))
    def test_dwell_only_new_actual_stamps_and_exact_integer_boundary(self):
        window=ArrivalWindow(parse_goal(disc()));p=[0,2,0]
        for ns in (1000000000,1100000000,1200000000,1300000000):self.assertFalse(window.observe(p,ns))
        for _ in range(100):self.assertFalse(window.observe(p,1300000000))
        self.assertTrue(window.observe(p,1400000000))
    def test_hold_stale_and_outside_restart_full_dwell(self):
        for event in ('protected','stale','outside'):
            w=ArrivalWindow(parse_goal(disc()));p=[0,2,0]
            for ns in (0,100000000,200000000,300000000):self.assertFalse(w.observe(p,ns))
            if event=='protected':self.assertFalse(w.observe(p,350000000,protected=True))
            elif event=='stale':self.assertFalse(w.observe(p,350000000,fresh=False))
            else:self.assertFalse(w.observe([1,2,0],350000000))
            for ns in (400000000,500000000,600000000,700000000):self.assertFalse(w.observe(p,ns))
            self.assertTrue(w.observe(p,800000000))
    def test_observation_gap_and_backward_time_do_not_bridge_dwell(self):
        w=ArrivalWindow(parse_goal(disc()));p=[0,2,0]
        for ns in (0,100000000,200000000,300000000):w.observe(p,ns)
        self.assertFalse(w.observe(p,700000000));self.assertEqual(w.since,700000000)
        self.assertFalse(w.observe(p,600000000));self.assertEqual(w.reason,'out_of_order')
        self.assertFalse(w.observe(p,800000000))
    def test_bad_pose_during_dwell_does_not_keep_prior_evidence(self):
        w=ArrivalWindow(parse_goal(disc()));p=[0,2,0]
        for ns in (0,100000000,200000000,300000000):w.observe(p,ns)
        with self.assertRaises(ValueError):w.observe([float('nan'),2,0],350000000)
        self.assertFalse(w.observe(p,400000000));self.assertEqual(w.since,400000000)

if __name__=='__main__':
    tests=unittest.defaultTestLoader.loadTestsFromTestCase(KernelTests);result=unittest.TextTestRunner(verbosity=2).run(tests)
    here=Path(__file__).resolve().parent
    receipt=dict(scope=__doc__,passed=result.wasSuccessful(),tests=result.testsRun,
                 failures=[(str(t),s) for t,s in result.failures+result.errors],
                 sha256={name:hashlib.sha256((here/name).read_bytes()).hexdigest() for name in ('goal_regions.py','test_goal_regions.py')})
    if os.environ.get('DEMO_TEST_PRODUCTION_GOAL_CORE')=='1':
        receipt['tested_production_core_sha256']=hashlib.sha256((here.parents[1]/'goal_regions.py').read_bytes()).hexdigest()
    (here/('production_kernel_result.json' if 'tested_production_core_sha256' in receipt else 'result.json')).write_text(json.dumps(receipt,indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
