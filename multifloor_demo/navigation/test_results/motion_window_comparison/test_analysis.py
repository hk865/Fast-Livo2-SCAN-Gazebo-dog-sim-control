#!/usr/bin/env python3
"""Meaningful passive-analysis boundaries, no ROS/physical pass claim."""
import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from compare_windows import Observer, WINDOWS, NS, episodes, truth_pose, opposite, transition_causes


class AnalysisBoundaries(unittest.TestCase):
    def fixture(self, width, velocity=lambda t:.1, yaw=lambda t:-.04*t, command=lambda t:[.1,0,-.04], context=lambda t:('run',0,1)):
        o=Observer(window_ns=width,min_span_ns=width-50_000_000)
        for n in range(0,1_600_000_001,20_000_000):
            t=n/NS
            o.observe_applied_command(n,command(t),'walk',context(t))
            if n%100_000_000==0:
                o.observe_pose(n,[velocity(t)*t,0,0],Rotation.from_euler('z',yaw(t)).as_quat(),n)
        return o

    def test_constant_velocity_all_windows_have_declared_latency(self):
        for w in WINDOWS:
            s=self.fixture(w).snapshot(1_600_000_000,1_600_000_000)
            self.assertTrue(s['valid'])
            self.assertEqual(s['dt_ns'],w)
            self.assertAlmostEqual(s['heading_rate_world'],-.04,places=10)
            self.assertEqual(s['effective_measurement_age_ns'],w//2)
            self.assertFalse(s['control_output_generated']);self.assertFalse(s['feedback_eligible'])

    def test_real_motion_transition_stays_invalid_all_windows(self):
        for w in WINDOWS:
            o=self.fixture(w,command=lambda t:[.1,0,-.04] if t<1.5 else [0,0,-.12])
            s=o.snapshot(1_600_000_000,1_600_000_000)
            self.assertEqual(s['invalid_reason'],'execution_transition')
            self.assertIn('actual_motion_mode_change',transition_causes(o,s['window_start_ns'],s['stamp_ns']))

    def test_only_trajectory_transition_is_separately_reported(self):
        o=self.fixture(800_000_000,context=lambda t:('run',0,1 if t<1.5 else 2))
        s=o.snapshot(1_600_000_000,1_600_000_000)
        self.assertEqual(s['invalid_reason'],'execution_transition')
        self.assertEqual(transition_causes(o,s['window_start_ns'],s['stamp_ns']),['trajectory_identity_only_change'])

    def test_nominal_periodic_sway_not_called_long_term_reverse_gain(self):
        # yaw has zero-mean .6s physical gait sway plus a correctly signed drift.
        # Longer windows reduce oscillation without changing actual drift sign.
        values={}
        for w in (600_000_000,1_200_000_000):
            s=self.fixture(w,yaw=lambda t:-.04*t+.025*np.sin(2*np.pi*t/.6)).snapshot(1_600_000_000,1_600_000_000)
            values[w]=s['heading_rate_world']
        self.assertLess(abs(values[1_200_000_000]+.04),abs(values[600_000_000]+.04))

    def test_persistent_wrong_sign_is_visible_in_long_window(self):
        for w in WINDOWS:
            s=self.fixture(w,yaw=lambda t:.08*t).snapshot(1_600_000_000,1_600_000_000)
            self.assertTrue(s['valid']);self.assertTrue(opposite(s['actual_command_average'][2],s['heading_rate_world']))

    def test_episode_does_not_bridge_invalid_or_context_or_sign(self):
        base=dict(stage='exploring',context=['r',0,1],command_sign=-1)
        a=[dict(base,stamp_ns=i*100_000_000,flag=True) for i in range(8)]
        self.assertEqual(sum(x['at_least_one_nominal_gait_cycle'] for x in episodes(a,'flag')),1)
        a[4]['flag']=False
        self.assertEqual(max(x['duration_ns'] for x in episodes(a,'flag')),300_000_000)
        a[4]['flag']=True;a[4]['context']=['r',0,2]
        self.assertEqual(max(x['duration_ns'] for x in episodes(a,'flag')),300_000_000)
        a[4]['context']=['r',0,1];a[4]['command_sign']=1
        self.assertEqual(max(x['duration_ns'] for x in episodes(a,'flag')),300_000_000)

    def test_GT_missing_tail_or_wide_bracket_never_extrapolated(self):
        vs=[dict(p=[0,0,0],q=[0,0,0,1]),dict(p=[1,0,0],q=[0,0,0,1])]
        self.assertIsNone(truth_pose(200_000_001,np.array([0,200_000_000]),vs,np.eye(3),np.zeros(3)))
        self.assertIsNone(truth_pose(100_000_000,np.array([0,200_000_000]),vs,np.eye(3),np.zeros(3)))
        self.assertIsNotNone(truth_pose(50_000_000,np.array([0,100_000_000]),vs,np.eye(3),np.zeros(3)))


if __name__=='__main__':
    unittest.main(verbosity=2)
