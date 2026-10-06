#!/usr/bin/env python3
import copy,hashlib,json,math,os,sys,unittest
from pathlib import Path
from dataclasses import replace
import numpy as np
PRODUCTION=os.environ.get('DEMO_TEST_PRODUCTION_INNER')=='1'
if PRODUCTION:sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import goal_regions as goal_module
from goal_regions import (Goal,parse_goal,definitions_sha256,contains,contains_control,
                          ArrivalWindow,relative_world_transform,RigidTransform)
HERE=Path(__file__).parent;ROOT=HERE.parents[2]

def goal(kind='disc_prism',band=True):
    a=dict(type=kind,dwell_sim_s=.4)
    if kind=='sphere':a.update(radius_m=.22)
    elif kind=='disc_prism':a.update(radius_m=.35,height_half_span_m=.1)
    else:a.update(half_extents_m=[.35,.30,.10])
    if band:
        a['control_band']={'sphere':dict(radius_m=.17),'disc_prism':dict(radius_m=.25,height_half_span_m=.1),
                           'oriented_box':dict(half_extents_m=[.25,.20,.07])}[kind]
    return dict(goal_id='target',center=[0,0,0],arrival=a,timeout_sim_s=90)

class InnerTests(unittest.TestCase):
    def test_disc_outer_boundary_cannot_stop_or_start_dwell(self):
        g=parse_goal(goal());p=[.30,0,0]
        self.assertTrue(contains(g,p));self.assertFalse(contains_control(g,p))
        w=ArrivalWindow(g)
        for t in range(0,1000000001,100000000):self.assertFalse(w.observe(p,t));self.assertIsNone(w.since)
    def test_full_inner_dwell_is_native_continuous_and_outer_also_inside(self):
        g=parse_goal(goal());w=ArrivalWindow(g)
        for t in (0,100000000,200000000,300000000):self.assertFalse(w.observe([.24,0,0],t))
        self.assertTrue(w.observe([.24,0,0],400000000));self.assertTrue(contains(g,[.24,0,0]))
        w.observe([.3,0,0],500000000);self.assertIsNone(w.since)
    def test_wrong_floor_still_rejected_by_control_and_outer(self):
        g=parse_goal(goal())
        self.assertFalse(contains(g,[0,0,1.2]));self.assertFalse(contains_control(g,[0,0,1.2]))
    def test_ramp_inner_along_lateral_normal_are_all_smaller(self):
        raw=goal('oriented_box');theta=.1
        axes=[[math.cos(theta),0,math.sin(theta)],[0,1,0],[-math.sin(theta),0,math.cos(theta)]]
        raw['arrival']['axes']=axes;g=parse_goal(raw);A=np.array(axes)
        for local in ([.30,0,0],[0,.25,0],[0,0,.08]):
            p=A.T@local;self.assertTrue(contains(g,p));self.assertFalse(contains_control(g,p))
        p=A.T@[.24,.19,.06];self.assertTrue(contains_control(g,p))
    def test_original_origin_outer_remains_point22_with_control17(self):
        g=parse_goal(goal('sphere'))
        self.assertTrue(contains(g,[.20,0,0]));self.assertFalse(contains_control(g,[.20,0,0]))
        self.assertFalse(contains_control(g,[.17,0,0]));self.assertTrue(contains_control(g,[.169,0,0]))
        self.assertFalse(contains(g,[.22,0,0]))
    def test_control_center_axes_dwell_cannot_be_independently_changed(self):
        for b in (dict(center=[.1,0,0]),dict(type='sphere'),dict(axes=[[0,1,0],[-1,0,0],[0,0,1]]),dict(dwell_sim_s=.1)):
            raw=goal();raw['arrival']['control_band'].update(b)
            with self.assertRaises(ValueError):parse_goal(raw)
    def test_control_band_must_be_positive_finite_and_subset(self):
        for v in (.36,0,-.1,float('nan'),True):
            raw=goal();raw['arrival']['control_band']['radius_m']=v
            with self.assertRaises(ValueError):parse_goal(raw)
        raw=goal('oriented_box');raw['arrival']['control_band']['half_extents_m']=[.25,.2,.11]
        with self.assertRaises(ValueError):parse_goal(raw)
    def test_missing_dimensions_and_extra_properties_are_rejected(self):
        for band in (None,{},dict(radius_m=.25),dict(radius_m=.25,height_half_span_m=.1,control_inset=.1)):
            raw=goal();raw['arrival']['control_band']=band
            with self.assertRaises(ValueError):parse_goal(raw)
    def test_new_goal_hash_includes_control_geometry_and_stays_immutable(self):
        raw=goal();g=parse_goal(raw);h=definitions_sha256([g]);raw['arrival']['control_band']['radius_m']=.20
        self.assertEqual(definitions_sha256([g]),h);self.assertNotEqual(definitions_sha256([parse_goal(raw)]),h)
        d=g.definition();self.assertEqual(d['arrival']['control_band']['radius_m'],.25)
        self.assertEqual(d['arrival']['control_band']['dwell_sim_s'],.4)
    def test_world_to_camera_conversion_transforms_both_geometries_once(self):
        g=parse_goal(goal('oriented_box'));T=relative_world_transform([2,3,1],.7);new=T.goal(g)
        self.assertEqual(new.control_goal().center,new.center);self.assertEqual(new.control_goal().axes,new.axes)
        self.assertTrue(contains_control(new,T.point([.24,.19,.06])))
        self.assertFalse(contains_control(new,T.point([.3,0,0])))
        self.assertEqual(parse_goal(new.definition()).definition(),new.definition())
    def test_legacy_waypoint_cannot_opt_in_a_control_band(self):
        with self.assertRaises(ValueError):Goal('legacy',(0,0,0),'sphere',radius=.22,legacy=True,control_extents=(.17,))
    def test_old_formal_v2_definition_hash_is_exactly_unchanged(self):
        run=ROOT/'runs/20261001_203152_88725b'
        if not (run/'mission.json').exists():self.skipTest('immutable live/run evidence unavailable')
        m=json.loads((run/'mission.json').read_text());definitions=m['navigation']['goals_definitions']
        parsed=[parse_goal(d) for d in definitions]
        self.assertEqual([g.definition() for g in parsed],definitions)
        self.assertEqual(definitions_sha256(parsed),m['navigation']['goals_definition_sha256'])
    def test_old_outer_only_windows_remain_outer_only(self):
        g=parse_goal(goal(band=False));self.assertIs(g.control_goal(),g)
        self.assertTrue(contains_control(g,[.34,0,0]));self.assertNotIn('control_band',g.definition()['arrival'])
    def test_hold_duplicates_gap_reset_inner_dwell(self):
        g=parse_goal(goal());w=ArrivalWindow(g);self.assertFalse(w.observe([.2,0,0],0))
        self.assertFalse(w.observe([.2,0,0],0));self.assertFalse(w.observe([.2,0,0],100000000,protected=True));self.assertIsNone(w.since)
        self.assertFalse(w.observe([.2,0,0],500000000));self.assertEqual(w.since,500000000)

if __name__=='__main__':
    r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(InnerTests))
    (HERE/('production_result.json' if PRODUCTION else 'result.json')).write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,scope=('Actual production shared geometry pure regression; no ROS/physics.' if PRODUCTION else 'Excluded shared geometry candidate pure regression; no ROS/physics.'),production=PRODUCTION,source_sha256=hashlib.sha256(Path(goal_module.__file__).read_bytes()).hexdigest(),failures=[(str(t),s) for t,s in r.errors+r.failures]),indent=2)+'\n')
    raise SystemExit(not r.wasSuccessful())
