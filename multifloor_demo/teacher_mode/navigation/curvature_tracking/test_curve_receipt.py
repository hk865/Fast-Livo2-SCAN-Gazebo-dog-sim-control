"""Offline refusal/geometry tests, not actual plant or curve results."""
import copy
import importlib.util
from pathlib import Path
import tempfile
import unittest

import numpy as np

from curve_receipt import (COMMON_SCHEMA,COMMON_CHECKS,common_pass,native_coverage,
                          geometric_trace,ordered_traversal,rolling_forward,turning_segments,evaluate,
                          continuous_modes,require_digest,sha)
from core import StaticPath
from profiles import circle_profile,s_curve_profile


def geometry_fixture(kind='circle'):
    p=circle_profile(1.) if kind=='circle' else s_curve_profile(4.,.4)
    path=StaticPath(p['path']);s=np.linspace(0,path.length,math_count(path.length))
    position=path.position(s);g=geometric_trace(path,position)
    a={'t':np.arange(len(s))*.005,'position':position,'unwrapped_yaw':g['reference_heading'].copy()}
    return path,a,g


def math_count(length):return int(length/.015)+1


class RefusalAndContinuity(unittest.TestCase):
    def common(self):
        return {'schema':COMMON_SCHEMA,'status':'passed','navigation_ground_truth_used':True,
                'SLAM_navigation_verified':False,'real_robot_verified':False,
                'checks':{k:{'status':'passed'} for k in (*COMMON_CHECKS,'strict_terrain_support_applicability')}}

    def test_failed_or_missing_common_gate_never_promoted(self):
        self.assertTrue(common_pass(self.common()))
        for mode in ['failed','unverified','missing']:
            c=self.common()
            if mode=='missing':c['checks'].pop(COMMON_CHECKS[0])
            else:c['checks'][COMMON_CHECKS[0]]['status']=mode
            self.assertFalse(common_pass(c))
        c=self.common();c['status']='failed';self.assertFalse(common_pass(c))

    def test_native_missing_iteration_or_clock_coverage(self):
        a={'t':np.array([.005,.010,.015]),'force_t':np.array([.010,.015,.020]),
           'dt':np.full(3,.005),'iteration':np.array([1,2,3])}
        self.assertTrue(native_coverage(a))
        b=copy.deepcopy(a);b['iteration'][1]=4;self.assertFalse(native_coverage(b))
        b=copy.deepcopy(a);b['dt'][1]=.01;self.assertFalse(native_coverage(b))

    def test_startpoint_or_shortcut_does_not_complete_lap(self):
        path=StaticPath(circle_profile(1.)['path'])
        for position in [np.tile(path.position(0.),(100,1)),
                         np.tile(path.final_xyz,(100,1)),
                         path.position(np.linspace(0,path.entry,100))]:
            g=geometric_trace(path,position)
            a={'t':np.arange(len(position))*.005,'position':position}
            passed,_=ordered_traversal(path,a,g)
            self.assertFalse(passed)

    def test_complete_ordered_signed_circle_and_S_geometry(self):
        for kind in ['circle','s_curve']:
            path,a,g=geometry_fixture(kind)
            self.assertTrue(ordered_traversal(path,a,g)[0])
            self.assertTrue(turning_segments(path,a,g)[0])
            a['unwrapped_yaw']=-a['unwrapped_yaw']
            self.assertFalse(turning_segments(path,a,g)[0])

    def test_all_windows_no_best_window_and_stop_in_middle(self):
        t=np.arange(601)*.005;v=np.full(601,.3);eligible=np.ones(601,dtype=bool)
        r=rolling_forward(t,v,eligible);self.assertGreater(r['complete_windows'],300)
        self.assertAlmostEqual(r['minimum_mean_mps'],.3)
        v[200:401]=0.;r=rolling_forward(t,v,eligible)
        self.assertLess(r['minimum_mean_mps'],.7*.3)
        eligible[:]=False;r=rolling_forward(t,v,eligible)
        self.assertEqual(r['complete_windows'],0);self.assertIsNone(r['minimum_mean_mps'])

    def test_stop_turn_and_missing_curved_samples_cannot_pass(self):
        valid=np.ones(5,dtype=bool);curved=np.ones(5,dtype=bool)
        mode=np.array(['drive']*5)
        self.assertTrue(continuous_modes(mode,curved,valid)[0])
        for forbidden in ['pre_turn','turn','settle']:
            mode=np.array(['drive',forbidden,'drive','drive','drive'])
            self.assertFalse(continuous_modes(mode,curved,valid)[0])
        self.assertFalse(continuous_modes(np.array(['drive']*5),~curved,valid)[0])

    def test_source_hash_mismatch_is_rejected(self):
        with tempfile.TemporaryDirectory(prefix='curve_source_sha_negative_') as name:
            path=Path(name)/'frozen';path.write_text('original')
            expected=sha(path);self.assertEqual(require_digest(path,expected),expected)
            path.write_text('modified')
            with self.assertRaises(ValueError):require_digest(path,expected)

    def test_no_samples_and_missing_sources_cannot_pass(self):
        with tempfile.TemporaryDirectory(prefix='curve_receipt_refusal_test_') as name:
            result=evaluate(Path(name),write=False)
            self.assertNotEqual(result['status'],'passed')
            self.assertFalse(result['continuous_curve_verified'])
            self.assertFalse((Path(name)/'summary_curve_independent.json').exists())

    def test_append_only_refuses_existing_receipt_even_missing_inputs(self):
        with tempfile.TemporaryDirectory(prefix='curve_receipt_append_test_') as name:
            p=Path(name);receipt=p/'summary_curve_independent.json';receipt.write_text('PRESERVE')
            with self.assertRaises(FileExistsError):evaluate(p)
            self.assertEqual(receipt.read_text(),'PRESERVE')


if __name__=='__main__':unittest.main(verbosity=2)
