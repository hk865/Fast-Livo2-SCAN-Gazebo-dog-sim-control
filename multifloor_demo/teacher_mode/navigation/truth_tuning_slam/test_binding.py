#!/usr/bin/env python3
"""Pure causal binding/TTL negative checks; creates no ROS or simulator."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.dont_write_bytecode = True
HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import adapter

spec = importlib.util.spec_from_file_location('readonly_truth_controller_for_binding_checks', HERE.parent/'truth_tuning/core.py')
core = importlib.util.module_from_spec(spec); spec.loader.exec_module(core)


class BindingTests(unittest.TestCase):
    def binding(self, hz=10, offset=(.1, 0, 0)):
        c = {'maximum_gyro_pair_gap_s': .02, 'imu_frame': 'imu_link',
             'fixed_route': {'points_xyz': [[0, 0, 0], [6, 0, 0]]}}
        p = {'feedback_hz': hz, 'route_world_xyz': [[0, 0, 0], [6, 0, 0]],
             'design': 'cascade_pi', 'command_limits': [.4, .2, .3], 'desired_speed': .2,
             'base_com_offset': list(offset)}
        receipt = {'config_sha256': 'a'*64, 'controller_sha256': 'b'*64,
                   'profile_sha256': 'c'*64, 'route_pending_actual_SLAM_anchor': True}
        return adapter.FreshBinding(c, core, p, np.eye(3), np.eye(3), receipt)
    def sample(self, b, t, wall, *, position=(2, 3, 0), yaw=0, gyro=(0, 0, 0), velocity=(0, 0, 0), gyro_t=None):
        ns = round(t*1e9); b.clock(ns, wall)
        gns = ns if gyro_t is None else round(gyro_t*1e9)
        b.imu({'stamp_ns': gns, 'frame_id': 'imu_link', 'received_monotonic_wall': wall,
               'callback_ros_clock_ns': ns, 'quaternion_xyzw': [0, 0, 0, 1],
               'orientation_available': True, 'angular_velocity_sensor': list(gyro)})
        b.odom({'stamp_ns': ns, 'frame_id': 'camera_init', 'child_frame_id': 'demo_slam_body',
                'received_monotonic_wall': wall, 'callback_ros_clock_ns': ns,
                'quaternion_xyzw': [0, 0, np.sin(yaw/2), np.cos(yaw/2)],
                'position': list(position), 'origin_linear_velocity_body': list(velocity),
                'twist_covariance_diagonal': [.04]*6})
        return b.tick(wall)
    def active(self, b):
        self.sample(b, 1, 100)
        for i in range(1, 41):
            out, row = self.sample(b, 1+i*.1, 100+i*.1)
        return out, row
    def test_real_ten_hz_not_two_hz_or_fifty_cached(self):
        b = self.binding(); self.active(b)
        self.assertEqual(b.update_count, 40)
        n = b.update_count
        for i in range(4):
            b.clock(5000000000+i*10000000, 104+i*.01)
            b.tick(104+i*.01)
        self.assertEqual(b.update_count, n)
        self.assertTrue(np.allclose(np.diff(b.update_stamps), 100000000))
    def test_declared_25_profile_is_only_ten_fresh_sources(self):
        b = self.binding(25); self.active(b)
        self.assertEqual(b.update_count, 40)
        self.assertTrue(np.allclose(np.diff(b.update_stamps), 100000000))
    def test_anchor_only_actual_SLAM_pose_yaw_and_stays_fixed(self):
        b = self.binding(); self.sample(b, 1, 100, yaw=.4)
        self.sample(b, 1.1, 100.1, position=(2, 3, 0), yaw=.4)
        expected = np.array([2+6*np.cos(.4), 3+6*np.sin(.4), 0])
        self.assertTrue(np.allclose(b.controller.route[1], expected))
        saved = b.controller.route.copy()
        self.sample(b, 1.2, 100.2, position=(8, 9, 0), yaw=.9)
        self.assertTrue(np.array_equal(saved, b.controller.route))
        self.assertEqual(b.route_anchor['original_stamp_ns'], 1100000000)
    def test_COM_offset_and_original_stamp_phase(self):
        b = self.binding(); self.active(b)
        out, row = self.sample(b, 5.1, 104.1, gyro=(0, 0, .2), velocity=(.15, 0, 0))
        r = row['core_original_row']
        self.assertTrue(np.allclose(r['measured_COM_velocity_body'], [.15, .02, 0]))
        self.assertTrue(np.allclose(r['measured_origin_velocity_world'], [.15, 0, 0]))
        self.assertAlmostEqual(r['feedback_time_s'], 5.1, places=10)
    def test_gyro_only_IMU_preserves_SLAM_tilt_protection(self):
        b = self.binding(); self.active(b)
        b.clock(5100000000, 104.1)
        self.assertTrue(b.imu({'stamp_ns': 5100000000, 'frame_id': 'imu_link',
            'received_monotonic_wall': 104.1, 'callback_ros_clock_ns': 5100000000,
            'quaternion_xyzw': [0, 0, 0, 0], 'orientation_available': False,
            'orientation_covariance': [-1]+[0]*8, 'angular_velocity_sensor': [0, 0, .2]}))
        self.assertIsNone(b.gyros[-1]['tilt_rad'])
        pose={**b.pose, 'stamp_ns': 5100000000, 'received_monotonic_wall': 104.1,
            'callback_ros_clock_ns': 5100000000}
        self.assertTrue(b.odom(pose)); out, _=b.tick(104.1)
        self.assertTrue(out['healthy'])
        pose.update(stamp_ns=5200000000,received_monotonic_wall=104.2,
            callback_ros_clock_ns=5200000000, quaternion_xyzw=[np.sin(.8/2),0,0,np.cos(.8/2)])
        b.clock(5200000000,104.2)
        self.assertTrue(b.imu({**b.gyros[-1], 'stamp_ns':5200000000,
            'received_monotonic_wall':104.2,'callback_ros_clock_ns':5200000000}))
        self.assertTrue(b.odom(pose))
        out,_=b.tick(104.2)
        self.assertFalse(out['healthy']);self.assertEqual(b.fault,'actual_attitude_limit_latched')
    def test_duplicate_does_not_refresh_receive_wall(self):
        b = self.binding(); self.active(b); old = b.pose['received_monotonic_wall']
        repeated = dict(b.pose, received_monotonic_wall=104.2, callback_ros_clock_ns=5200000000)
        self.assertFalse(b.odom(repeated)); self.assertEqual(b.pose['received_monotonic_wall'], old)
        b.clock(5350000000, 104.35); out, _ = b.tick(104.35)
        self.assertFalse(out['healthy']); self.assertEqual(out['command'], [0, 0, 0])
    def test_future_gyro_not_used_to_fill_missing_causal_pair(self):
        b = self.binding(); self.sample(b, 1, 100)
        out, _ = self.sample(b, 1.1, 100.1, gyro_t=1.105)
        self.assertFalse(out['healthy']); self.assertIn('causal_IMU', out['reason'])
    def test_clock_pause_protects_even_if_clock_heartbeat_repeats(self):
        b = self.binding(); self.active(b)
        b.clock(5000000000, 104.4); out, _ = b.tick(104.4)
        self.assertFalse(out['healthy']); self.assertEqual(out['command'], [0, 0, 0])
        self.assertIn('paused', out['reason'])
    def test_backward_clock_latches(self):
        b = self.binding(); self.active(b)
        b.clock(4999000000, 104.01); self.assertEqual(b.fault, 'clock_reset')
        b.clock(5100000000, 104.1); out, _ = b.tick(104.1)
        self.assertFalse(out['healthy'])
    def test_reader_checks_source_age_even_with_new_heartbeat(self):
        b = self.binding(); out, _ = self.active(b)
        out.update(clock_ns=5400000000, monotonic_wall=104.4)
        with tempfile.TemporaryDirectory(dir=HERE) as d:
            path = Path(d)/'command.json'; path.write_text(adapter.canonical(out))
            cmd, bad, reason = adapter.read_command(path, 5400000000, 'a'*64, [.4, .2, .3], wall=104.4)
            self.assertTrue(bad); self.assertEqual(cmd.tolist(), [0, 0, 0]); self.assertIn('source TTL', reason)
    def test_atomic_publish_during_read_uses_completion_without_relaxing_TTL(self):
        b=self.binding();out,_=self.active(b)
        with tempfile.TemporaryDirectory(dir=HERE) as d:
            path=Path(d)/'command.json';path.write_text(adapter.canonical(out));seq={}
            with patch.object(adapter.time,'monotonic',side_effect=[103.999,104.002]):
                _,bad,_=adapter.read_command(path,5000000000,'a'*64,[.4,.2,.3],seq)
            self.assertFalse(bad);self.assertEqual(seq['read_attempt']['read_monotonic_wall'],104.002)
            self.assertEqual(seq['read_attempt']['read_begin_monotonic_wall'],103.999)
            with patch.object(adapter.time,'monotonic',side_effect=[104.299,104.301]):
                cmd,bad,reason=adapter.read_command(path,5000000000,'a'*64,[.4,.2,.3],seq)
            self.assertTrue(bad);self.assertEqual(cmd.tolist(),[0,0,0]);self.assertIn('TTL',reason)
    def test_reader_rejects_changed_same_sequence_and_old_scan_label(self):
        b = self.binding(); out, _ = self.active(b)
        with tempfile.TemporaryDirectory(dir=HERE) as d:
            path = Path(d)/'command.json'; path.write_text(adapter.canonical(out)); seq = {}
            _, bad, _ = adapter.read_command(path, 5000000000, 'a'*64, [.4, .2, .3], seq, wall=104)
            self.assertFalse(bad)
            out['command'][0] += .001; path.write_text(adapter.canonical(out))
            _, bad, reason = adapter.read_command(path, 5000000000, 'a'*64, [.4, .2, .3], seq, wall=104)
            self.assertTrue(bad); self.assertIn('sequence', reason)
            out['source'] = 'scan_slam'; path.write_text(adapter.canonical(out))
            _, bad, reason = adapter.read_command(path, 5000000000, 'a'*64, [.4, .2, .3], wall=104)
            self.assertTrue(bad); self.assertIn('schema', reason)
    def test_cached_source_cannot_advance_goal_dwell(self):
        b = self.binding(); self.active(b)
        for i in range(2):
            out, row = self.sample(b, 5.1+i*.1, 104.1+i*.1, position=(8, 3, 0))
        self.assertIsNone(b.controller.completed_t)
        for i in range(10):
            b.clock(5210000000+i*1000000, 104.21+i*.001)
            b.tick(104.21+i*.001)
        self.assertIsNone(b.controller.completed_t)
    def test_no_valid_twist_no_movement(self):
        b = self.binding(); self.sample(b, 1, 100)
        row = dict(b.pose, stamp_ns=1100000000, received_monotonic_wall=100.1,
                   callback_ros_clock_ns=1100000000, twist_covariance_diagonal=[1e6]*6)
        with self.assertRaises(ValueError):
            b.odom(row)
    def test_completion_preserves_original_integer_source_stamp(self):
        b = self.binding(); self.active(b)
        declared = None
        for i in range(1, 15):
            out, _ = self.sample(b, 5+i*.1, 104+i*.1, position=(8, 3, 0))
            if b.controller.completed_t is not None and declared is None:
                declared = b.pose['stamp_ns']
        self.assertIsNotNone(declared)
        self.assertEqual(out['controller_completed_pose_stamp_ns'], declared)
    def test_rejected_read_attempt_is_not_last_accepted_envelope(self):
        b = self.binding(); out, _ = self.active(b)
        with tempfile.TemporaryDirectory(dir=HERE) as d:
            path = Path(d)/'command.json'; path.write_text(adapter.canonical(out)); seq = {}
            _, bad, _ = adapter.read_command(path, 5000000000, 'a'*64, [.4, .2, .3], seq, wall=104)
            self.assertFalse(bad); old_sequence = out['sequence']
            out.update(sequence=old_sequence+1, healthy=False)
            raw = adapter.canonical(out)+'\n'; path.write_text(raw)
            _, bad, _ = adapter.read_command(path, 5000000000, 'a'*64, [.4, .2, .3], seq, wall=104)
            self.assertTrue(bad)
            self.assertEqual(seq['envelope']['sequence'], old_sequence)
            self.assertEqual(seq['read_attempt']['decoded_envelope']['sequence'], old_sequence+1)
            self.assertEqual(seq['read_attempt']['raw_utf8'], raw)
            self.assertEqual(seq['read_attempt']['status'], 'rejected')


if __name__ == '__main__':
    unittest.main(verbosity=2)
