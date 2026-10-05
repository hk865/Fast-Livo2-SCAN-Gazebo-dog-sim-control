"""Pure negative/positive receipt tests. No simulator, policy or process use."""
import copy
import importlib.util
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.dont_write_bytecode = True
spec = importlib.util.spec_from_file_location('active_hold_receipt_tested', Path(__file__).with_name('active_hold_receipt.py'))
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)


class ReceiptTests(unittest.TestCase):
    def window(self, yaw=None, velocity=None):
        t = np.arange(1201)*.005+10.
        p = np.zeros((len(t), 3)); p[:, 0] = .005*np.sin(t)
        y = np.zeros(len(t)) if yaw is None else yaw
        v = np.zeros((len(t), 3)) if velocity is None else velocity
        return t, p, y, v, np.zeros((len(t), 3)), np.zeros(len(t))

    def test_first_window_pass(self):
        result = m.fixed_first_window(*self.window(), 10.)
        self.assertTrue(result['complete']); self.assertEqual(result['window_s'], [10., 15.])
        self.assertEqual(result['native_rows'], 1001)

    def test_cannot_move_start_to_quiet_window(self):
        values = self.window(); values[2][40] = .11
        self.assertGreater(m.fixed_first_window(*values, 10.)['yaw_drift_rad'], .1)
        # It would pass a later window; the public interface always uses first declaration.
        self.assertLess(m.fixed_first_window(*values, 10.5)['yaw_drift_rad'], .1)

    def test_missing_first_sample_does_not_pick_next(self):
        with self.assertRaises(ValueError): m.fixed_first_window(*tuple(x[1:] for x in self.window()), 10.)

    def test_native_gap_is_not_complete(self):
        values = tuple(np.delete(x, 20, axis=0) for x in self.window())
        self.assertFalse(m.fixed_first_window(*values, 10.)['complete'])

    def test_truncated_window_is_not_complete(self):
        self.assertFalse(m.fixed_first_window(*tuple(x[:500] for x in self.window()), 10.)['complete'])

    def test_real_nonzero_velocity_is_not_ignored(self):
        values = self.window(); values[3][500, 0] = .081
        self.assertGreater(m.fixed_first_window(*values, 10.)['peak_origin_planar_speed_mps'], .08)

    def test_unwrapped_yaw_is_not_modulo_hidden(self):
        values = self.window(); values[2][:] = np.linspace(0, 2*math.pi, len(values[2]))
        self.assertGreater(m.fixed_first_window(*values, 10.)['yaw_drift_rad'], 5.)

    def common(self):
        x = dict(schema='independent_truth_teacher_pid_benchmark/v2', status='failed',
                 navigation_ground_truth_used=True, SLAM_navigation_verified=False,
                 real_robot_verified=False, checks={k: {'status': 'passed'} for k in m.COMMON_NONPARK})
        x['checks'][m.OLD_PARKING] = {'status': 'failed'}
        return x

    def test_old_failed_zero_parking_stays_failed_but_new_gate_can_qualify(self):
        old = self.common(); before = copy.deepcopy(old)
        self.assertTrue(m.common_nonparking_gate(old, copy.deepcopy(old)))
        self.assertEqual(old, before); self.assertEqual(old['status'], 'failed')

    def test_actual_drive_failure_is_never_excluded(self):
        old = self.common(); old['checks']['drive_heading']['status'] = 'failed'
        self.assertFalse(m.common_nonparking_gate(old, self.common()))

    def test_replay_safety_failure_blocks(self):
        replay = self.common(); replay['checks']['as_recorded_physical_safety']['status'] = 'failed'
        self.assertFalse(m.common_nonparking_gate(self.common(), replay))

    def test_missing_gate_blocks(self):
        old = self.common(); del old['checks']['stable_real_COM_speed']
        self.assertFalse(m.common_nonparking_gate(old, self.common()))

    def test_unverified_never_scores_or_passes(self):
        self.assertEqual(m.status_from_checks({'x': {'status':'unverified'}}), 'unverified')
        self.assertEqual(m.status_from_checks({'x': {'status':'failed'}, 'y': {'status':'unverified'}}), 'failed')

    def test_original_native_feedback_join_rejects_future_or_missing(self):
        t = np.asarray([1., 1.005, 1.01])
        self.assertEqual(m.native_index(t, 1.005), 1)
        for bad in (.995, 1.001, 1.02):
            with self.assertRaises(ValueError): m.native_index(t, bad)

    def test_frozen_reference_paths_are_not_identity_by_digest_only(self):
        self.assertEqual(m.archive_key('/freeze/navigation/curvature_tracking_v2/reference_v1_core.py'), 'references/v1_core.py')
        self.assertEqual(m.archive_key('/freeze/simulation/build/libteacher_actuator.so'), 'native/libteacher_actuator.so')
        self.assertEqual(m.archive_key('/freeze/policy/worker.py'), 'policy/worker.py')
        self.assertEqual(m.archive_key('/freeze/navigation/curvature_tracking_v2/worker.py'), 'truth/worker.py')

    def pi_fixture(self):
        # Independent closed-form stationary measurement and nonzero fixed
        # endpoint correction; no Controller call is used to produce expected PI.
        dt = .04; control = []; integral = 0.; last_pi = None
        velocity = .18*(.02-.003); target = np.asarray([velocity, 0., 0.])
        for frame in range(11):
            fresh = frame % 2 == 0; actual_dt = 0. if frame == 0 else dt
            if fresh:
                integral += actual_dt*velocity
                last_pi = dict(error=target.tolist(), P=(.6*target).tolist(),
                               I=[.5*integral, 0., 0.], filtered_actual_body=[0., 0., 0.],
                               blocked_axis=[False]*3, blocked_slew=[False]*3)
            raw = target+np.asarray(last_pi['P'])+np.asarray(last_pi['I'])
            control.append(dict(mode='capture', controller_updated=fresh,
                feedback_time_s=1.+(frame if fresh else frame-1)*.02,
                header_dt_s=actual_dt, active_integral_dt_s=actual_dt,
                entry_integral_reset=frame == 0, velocity_PI=copy.deepcopy(last_pi),
                v_reference_body=target.tolist(), raw_command_body=raw.tolist(), command_body=raw.tolist(),
                reference_velocity_world=target.tolist(), parking_reference_yaw_rate_radps=0.))
        t = np.arange(41)*.005+1.
        a = dict(t=t, position=np.tile([-.02, 0., .32], (len(t), 1)),
                 quaternion_wxyz=np.tile([1., 0., 0., 0.], (len(t), 1)),
                 body_COM_velocity=np.zeros((len(t), 3)), body_omega=np.zeros((len(t), 3)),
                 origin_world_velocity=np.zeros((len(t), 3)), Euler_yaw_rate=np.zeros(len(t)),
                 unwrapped_yaw=np.zeros(len(t)))
        ap = dict(target=dict(position_world_xyz=[0., 0., .32], heading0_rad=0.),
                  command_limits_body=[.06, .04, .1], position_kp=.18, position_kd=.2,
                  yaw_kp=.65, yaw_kd=.18, position_deadband_m=.003, yaw_deadband_rad=.005)
        class Rotations:
            @staticmethod
            def rotations(q): return np.tile(np.eye(3), (len(q), 1, 1))
        gains = dict(velocity_kp_x=.6, velocity_kp_y=.4, rate_kp=.3,
                     velocity_ki_x=.5, velocity_ki_y=.3, rate_ki=.2, COM_offset=[0., 0., 0.])
        return control, a, ap, Rotations(), gains

    def test_causal_nonzero_weak_hold_PI_reconstruction(self):
        result = m.hold_pi_replay(*self.pi_fixture())
        self.assertTrue(result['passed']); self.assertLess(result['maximum_absolute_replay_error'], 1e-12)

    def test_held_row_integral_forgery_rejects(self):
        c, a, ap, helper, gains = self.pi_fixture(); c[3]['velocity_PI']['I'][0] += .001
        self.assertFalse(m.hold_pi_replay(c, a, ap, helper, gains)['passed'])

    def test_wrong_actual_native_measurement_rejects_PI(self):
        c, a, ap, helper, gains = self.pi_fixture(); a['body_COM_velocity'][16, 0] = .1
        self.assertFalse(m.hold_pi_replay(c, a, ap, helper, gains)['passed'])

    def capture_fixture(self):
        t = np.arange(161)*.005+1.
        a = dict(t=t, position=np.zeros((len(t), 3)), unwrapped_yaw=np.zeros(len(t)),
                 origin_body_velocity=np.zeros((len(t), 3)), Euler_yaw_rate=np.zeros(len(t)))
        class PathFixture:
            final_xyz = np.zeros(3); heading0 = 0.
        ap = dict(capture_xy_entry_m=.025, capture_xy_hold_m=.03,
                  capture_yaw_entry_rad=.035, capture_yaw_hold_rad=.045)
        control = []
        for i in range(17):
            stamp = 1.+.04*i
            mode = 'capture' if i < 15 else 'active_hold'
            control.append(dict(controller_updated=True, mode=mode, feedback_time_s=stamp,
                parking_capture_state_time_s=1., parking_capture_elapsed_s=0.,
                parking_hold_declared_state_time_s=None if i < 15 else 1.6,
                parking_hold_declared_elapsed_s=None if i < 15 else .6,
                entry_integral_reset=i == 0, capture_measurement_eligible=True,
                capture_fresh_dwell_start_state_time_s=1.))
        return control, a, PathFixture(), ap

    def test_fresh_capture_dwell_can_declare(self):
        result, _ = m.capture_evidence(*self.capture_fixture())
        self.assertAlmostEqual(result['first_hold_state_time_s'], 1.6)

    def test_hold_duplicate_does_not_extend_capture(self):
        c, a, p, ap = self.capture_fixture(); c[15]['feedback_time_s'] = c[14]['feedback_time_s']
        c[15]['parking_hold_declared_state_time_s'] = c[14]['feedback_time_s']
        with self.assertRaises(ValueError): m.capture_evidence(c, a, p, ap)

    def test_capture_reset_must_be_exactly_once(self):
        c, a, p, ap = self.capture_fixture(); c[3]['entry_integral_reset'] = True
        with self.assertRaises(ValueError): m.capture_evidence(c, a, p, ap)

    def test_early_capture_declaration_rejects(self):
        c, a, p, ap = self.capture_fixture(); c[14]['mode'] = 'active_hold'
        c[14]['parking_hold_declared_state_time_s'] = c[14]['feedback_time_s']
        c[14]['parking_hold_declared_elapsed_s'] = .56
        with self.assertRaises(ValueError): m.capture_evidence(c, a, p, ap)

    def test_false_native_eligibility_rejects(self):
        c, a, p, ap = self.capture_fixture(); a['position'][8, 0] = .04
        with self.assertRaises(ValueError): m.capture_evidence(c, a, p, ap)

    def test_source_missing_is_unverified_not_crash_or_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            result = m.evaluate(directory, write=False)
        self.assertEqual(result['status'], 'unverified'); self.assertIsNone(result['score'])

    def test_append_only_refuses_existing_receipt_before_parsing(self):
        with tempfile.TemporaryDirectory() as directory:
            (Path(directory)/m.OUTPUT).write_text('old immutable evidence')
            with self.assertRaises(FileExistsError): m.evaluate(directory)
            self.assertEqual((Path(directory)/m.OUTPUT).read_text(), 'old immutable evidence')


if __name__ == '__main__': unittest.main()
