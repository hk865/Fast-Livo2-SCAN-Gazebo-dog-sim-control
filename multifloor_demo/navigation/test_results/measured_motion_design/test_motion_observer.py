#!/usr/bin/env python3
import hashlib,json,math,unittest
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from motion_observer import MeasuredMotionObserver as O
NS=1_000_000_000

def make(p=lambda t:[.1*t,0,0],q=lambda t:Rotation.identity().as_quat(),
         c=lambda t:[.1,0,0],mode=lambda t:None,state=lambda t:'driving',context=lambda t:'path1'):
    o=O()
    for i in range(21):
        t=i*.02;ns=round(t*NS)
        o.observe_applied_command(ns,c(t),state(t),context(t),mode(t))
        if i%5==0:o.observe_pose(ns,p(t),q(t),ns)
    return o

class ObserverTests(unittest.TestCase):
    def test_current_body_axes_not_world_or_no_slip(self):
        q=lambda t:Rotation.from_euler('z',math.pi/2).as_quat()
        a=make(p=lambda t:[0,.1*t,0],q=q).snapshot(400000000,400000000)
        self.assertTrue(a['valid']);np.testing.assert_allclose(a['window_body_twist']['linear'],[.1,0,0],atol=1e-12)
        b=make(q=q).snapshot(400000000,400000000)
        np.testing.assert_allclose(b['window_body_twist']['linear'],[0,-.1,0],atol=1e-12)
        self.assertAlmostEqual(b['response_error_body'][1],-.1)
    def test_one_ns_real_boundary_jitter_uses_measured_bracket_not_extrapolation(self):
        o=O()
        for t in range(0,420000000,20000000):o.observe_applied_command(t,[.1,0,0])
        for t in (0,100000001,200000000,300000000,400000001):
            o.observe_pose(t,[.1*t/NS,0,0],[0,0,0,1],t)
        a=o.snapshot(400000001,400000001);self.assertTrue(a['valid'])
        self.assertEqual(a['dt_ns'],400000000);self.assertEqual(a['window_start_ns'],1)
        self.assertTrue(190000000<a['effective_measurement_age_ns']<210000000)
        self.assertEqual(a['measured_support_start_stamp_ns'],0)
        self.assertTrue(a['window_start_interpolated'])
        self.assertAlmostEqual(a['window_body_twist']['linear'][0],.1)
    def test_heading_wrap_and_body_rate(self):
        o=make(q=lambda t:Rotation.from_euler('z',math.radians(179)+.2*t).as_quat(),c=lambda t:[.1,0,.2])
        a=o.snapshot(400000000,400000000);self.assertTrue(a['valid'])
        self.assertAlmostEqual(a['heading_rate_world'],.2);self.assertAlmostEqual(a['window_body_twist']['angular'][2],.2)
    def test_slope_heading_rate_is_separate_from_body_z(self):
        o=make(q=lambda t:(Rotation.from_euler('z',.1*t)*Rotation.from_euler('y',.2)).as_quat())
        a=o.snapshot(400000000,400000000);self.assertTrue(a['valid'])
        self.assertAlmostEqual(a['heading_rate_world'],.1)
        self.assertAlmostEqual(a['window_body_twist']['angular'][2],.1*math.cos(.2))
        self.assertNotAlmostEqual(a['heading_rate_world'],a['window_body_twist']['angular'][2],places=5)
    def test_duplicate_cannot_refresh_native_pose_or_wall_freshness(self):
        o=make();self.assertFalse(o.observe_pose(400000000,[5,0,0],[0,0,0,1],700000000))
        self.assertEqual(o.snapshot(700000000,700000000)['invalid_reason'],'stale_pose')
        np.testing.assert_allclose(o.poses[-1]['p'],[.04,0,0])
    def test_clock_reset_clears_window_and_applied_command_history(self):
        o=make();self.assertFalse(o.observe_pose(1,[0,0,0],[0,0,0,1],1))
        self.assertFalse(o.poses);self.assertFalse(o.commands);self.assertFalse(o.snapshot(1,1)['valid'])
    def test_first_samples_are_not_a_velocity(self):
        o=O();o.observe_pose(0,[0,0,0],[0,0,0,1],0)
        self.assertEqual(o.snapshot(0,0)['invalid_reason'],'insufficient_pose_span')
    def test_native_time_types_and_frames_are_strict(self):
        for t in (0.,True,-1):
            with self.assertRaises(ValueError):O().observe_pose(t,[0,0,0],[0,0,0,1],0)
        for p,q,frame in (([float('nan'),0,0],[0,0,0,1],'camera_init'),([0,0,0],[0,0,0,0],'camera_init'),([0,0,0],[0,0,0,1],'world')):
            with self.assertRaises(ValueError):O().observe_pose(0,p,q,0,frame_id=frame)
    def test_future_and_native_gap_reject_available_fit(self):
        o=make();self.assertEqual(o.snapshot(399999999,400000000)['invalid_reason'],'future_or_clock_reset')
        o.poses[2]['t']=180000000;o.poses[1]['t']=10000000
        self.assertEqual(o.snapshot(400000000,400000000)['invalid_reason'],'pose_gap')
    def test_missing_or_stale_applied_command_cannot_imply_execution(self):
        o=make();o.commands.clear();self.assertFalse(o.snapshot(400000000,400000000)['valid'])
        o=make();o.commands.pop();o.commands.pop();o.commands.pop();o.commands.pop();o.commands.pop();o.commands.pop()
        self.assertEqual(o.snapshot(400000000,400000000)['invalid_reason'],'command_stale')
    def test_mode_adapter_ack_and_trajectory_transitions_do_not_mix(self):
        for kwargs in (dict(mode=lambda t:'walk' if t<.3 else 'turn'),dict(state=lambda t:'driving' if t<.3 else 'returning'),dict(context=lambda t:'path1' if t<.3 else 'path2')):
            self.assertEqual(make(**kwargs).snapshot(400000000,400000000)['invalid_reason'],'execution_transition')
    def test_same_stamp_nonzero_cannot_overwrite_zero_edge(self):
        o=make();self.assertFalse(o.observe_applied_command(400000000,[0,0,0]))
        self.assertFalse(o.commands);self.assertFalse(o.snapshot(400000000,400000000)['valid'])
    def test_jump_fit_retains_measured_values_but_disallows_feedback(self):
        o=make(p=lambda t:[.1*t+(.2 if abs(t-.2)<1e-6 else 0),0,0])
        a=o.snapshot(400000000,400000000);self.assertTrue(a['available']);self.assertFalse(a['valid'])
        self.assertEqual(a['invalid_reason'],'fit_residual_high');self.assertGreater(a['fit_residuals']['position_rms_m'],.05)
        self.assertFalse(a['control_output_generated'])
    def test_signed_response_does_not_assume_applied_yaw_sign(self):
        o=make(q=lambda t:Rotation.from_euler('z',.1*t).as_quat(),c=lambda t:[.1,0,-.04])
        a=o.snapshot(400000000,400000000);self.assertTrue(a['valid'])
        self.assertLess(a['signed_body_response'][2],0);self.assertAlmostEqual(a['response_error_body'][2],.14)
    def test_pure_turn_retreat_and_zero_are_measured_not_cancelled(self):
        o=make(p=lambda t:[-.04*t,0,0],c=lambda t:[0,0,.12])
        a=o.snapshot(400000000,400000000);self.assertTrue(a['valid']);self.assertEqual(a['execution_mode'],'turn')
        self.assertAlmostEqual(a['window_body_twist']['linear'][0],-.04);self.assertEqual(a['actual_command_average'][0],0)
        self.assertFalse(a['control_output_generated']);self.assertFalse(a['ground_truth_consumed'])
    def test_reverse_command_is_explicit_and_failed_state_is_never_feedback_eligible(self):
        a=make(p=lambda t:[-.1*t,0,0],c=lambda t:[-.1,0,0]).snapshot(400000000,400000000)
        self.assertTrue(a['valid']);self.assertEqual(a['execution_mode'],'reverse')
        self.assertFalse(a['observed_motion_ready']);self.assertFalse(a['feedback_eligible'])
        a=make(state=lambda t:'failed').snapshot(400000000,400000000)
        self.assertTrue(a['available']);self.assertFalse(a['observed_motion_ready'])
        self.assertFalse(a['feedback_eligible']);self.assertFalse(a['control_output_generated'])
    def test_buffers_bounded_and_high_rate_needs_adequate_window(self):
        o=O()
        for i in range(2000):
            o.observe_applied_command(i,[.1,0,0]);o.observe_pose(i,[0,0,0],[0,0,0,1],i)
        self.assertEqual(len(o.commands),1024);self.assertEqual(len(o.poses),64)
        self.assertEqual(o.snapshot(1999,1999)['invalid_reason'],'insufficient_pose_span')

if __name__=='__main__':
    r=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ObserverTests))
    Path(__file__).with_name('observer_unit_result.json').write_text(json.dumps(dict(passed=r.wasSuccessful(),tests=r.testsRun,scope='Excluded observer only, no command or physical test.',source_sha256=hashlib.sha256(Path(__file__).with_name('motion_observer.py').read_bytes()).hexdigest(),failures=[(str(t),s) for t,s in r.errors+r.failures]),indent=2)+'\n')
    raise SystemExit(not r.wasSuccessful())
