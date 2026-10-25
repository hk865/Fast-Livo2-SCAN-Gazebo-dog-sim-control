"""Focused clock-boundary tests; no ROS, simulator, model or historical writes.

The production clock module is exercised with the real unchanged cascade core.
Minimal host methods represent the original stop/tilt/archive boundaries, so
these tests establish safety bookkeeping, not actual Gazebo motion success.
"""
import copy
import ast
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
from cascade_core import Controller
from clock_hold import ControlClockHold


class Evidence:
    def __init__(self):
        self.error = None
        self.rows = []
    def append(self, path, row):
        self.rows.append((path.name, copy.deepcopy(row)))


class Publisher:
    def __init__(self): self.messages = []
    def publish(self, message): self.messages.append(message)


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0., y=0., z=0.)
        self.angular = SimpleNamespace(x=0., y=0., z=0.)


def actual_zero_publication():
    # Execute the exact saved production method, with only ROS message types
    # replaced. This verifies its no-row exact-zero branch preserves the actual
    # source file, reset bookkeeping and freeze publication; it is not a copy.
    tree = ast.parse((Path(__file__).parent/'controller.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name=='PIDNavigation')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name=='publish_command')
    module = ast.Module(body=[method], type_ignores=[])
    scope = {'np': np, 'json': json}
    exec(compile(ast.fix_missing_locations(module), str(Path(__file__).parent/'controller.py'), 'exec'), scope)
    return scope['publish_command']


class Host:
    def __init__(self, root):
        self.run = root
        self.state = 'running'
        self.profile = {'pose_cloud_timeout_s': .3}
        self.gate = ControlClockHold()
        self.cascade = Controller({'external_heading_gate': True}, [4., 0., .32], 0., 'goal')
        self.cascade.set_path([[0., 0., .32], [4., 0., .32]], 'scan', 0, 0)
        self.pose_stamp = 1_000_000_000
        self.cloud_stamp = 1_000_000_000
        self.pose_updated = self.cloud_updated = 11.
        self.feedback = self.pose(self.pose_stamp, 11_000_000_000)
        self.paired_imu = self.imu(self.pose_stamp, 11_000_000_000)
        self.raw_imu = SimpleNamespace(fresh=lambda now, clock: self.raw_fresh)
        self.raw_fresh = True
        self.guard_cloud_receipt = self.cloud(self.cloud_stamp, 11.)
        self.last_actual_guard_clock_ns = None
        self.obstacle_clear_since = None
        self.arrival_since = None
        self.region_arrival = SimpleNamespace(since=None)
        self.pid_records = self.guard_sequence = 0
        self.pid_row = self.pending_guard_row = None
        self.pid_guard_needs_evaluation = False
        self.command = [.3, 0., 0.]
        self.request_id = 'test'
        self.waypoint_index = 0
        self.stale_since = None
        self.evidence = Evidence()
        self.severe = False
        self.tilt_calls = self.publish_calls = 0
        self.pid = SimpleNamespace(resets=0)
        def reset(reason): self.pid.resets += 1
        self.pid.reset = reset
        self.cmd_pub = Publisher()
        self.freeze_pub = Publisher()
        self.counts = {'commands': 0}
        self.obstacle_hold = self.alignment_hold = False
        self.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.clock_ns))
        self.clock_ns = 1_070_000_000
        self.message = ''
        self.reset_reasons = []
        self.source_file = root / 'cascade_command_source.json'
        self.source_file.write_text('{"original_receipt": 11000000000}\n')

    @staticmethod
    def pose(stamp, wall):
        return dict(stamp_ns=stamp, received_wall_ns=wall, frame_id='camera_init',
                    position_world_xyz=[0., 0., .32], quaternion_wxyz=[1., 0., 0., 0.],
                    origin_velocity_body=[0., 0., 0.])
    @staticmethod
    def imu(stamp, wall):
        return dict(stamp_ns=stamp-10_000_000, received_wall_ns=wall-10_000_000,
                    frame_id='body', angular_velocity_body=[0., 0., 0.])
    @staticmethod
    def cloud(stamp, wall, digest='cloud1'):
        return dict(stamp_ns=stamp, received_monotonic_wall=wall,
                    filtered_xyz_float64_sha256=digest, nearest_slam_pose_stamp_ns=stamp)
    def apply_tilt_guard(self, now, clock):
        self.tilt_calls += 1
        if self.severe:
            self.state = 'failed'
            self.message = 'Original physical/bridge severe protection'
        return self.severe
    def reset_region_arrival(self, reason):
        self.region_arrival.since = None
        self.reset_reasons.append(reason)
    def publish_command(self):
        actual_zero_publication()(self)
        self.publish_calls += 1
    def ready_cascade(self):
        for stamp in (800_000_000, 900_000_000, 1_000_000_000):
            wall = 10_000_000_000 + stamp
            ack = dict(stamp_ns=stamp-5_000_000, received_wall_ns=wall-5_000_000,
                       sequence=stamp//1_000_000, requested_command_body=[0., 0., 0.],
                       applied_command_body=[0., 0., 0.])
            self.cascade.update(self.pose(stamp, wall), self.imu(stamp, wall), ack,
                                stamp+5_000_000, wall)


class ClockBoundaryChecks(unittest.TestCase):
    def setUp(self):
        self.messages = patch.dict('sys.modules', {
            'geometry_msgs.msg': SimpleNamespace(Twist=Twist),
            'std_msgs.msg': SimpleNamespace(Bool=lambda data: SimpleNamespace(data=data))})
        self.messages.start()
        self.temp = tempfile.TemporaryDirectory()
        self.n = Host(Path(self.temp.name))
        self.n.ready_cascade()
        self.original_source = self.n.source_file.read_bytes()
        self.clock = 1_070_000_000
        self.wall = 11_070_000_000
        self.assertFalse(self.n.gate.before_control(self.n, self.clock, self.wall))
        self.n.last_actual_guard_clock_ns = self.clock
    def tearDown(self):
        self.temp.cleanup()
        self.messages.stop()

    def hold(self, wall=None, clock=None):
        return self.n.gate.before_control(self.n, self.clock if clock is None else clock,
                                         self.wall+50_000_000 if wall is None else wall)

    def test_same_clock_new_cloud_stops_without_source_refresh_or_math(self):
        before = self.n.gate.snapshot(self.n)
        self.n.cloud_stamp += 30_000_000
        self.n.cloud_updated += .1
        self.n.guard_cloud_receipt = self.n.cloud(self.n.cloud_stamp, self.n.cloud_updated, 'cloud2')
        self.assertTrue(self.hold())
        self.assertEqual(self.n.command, [0., 0., 0.])
        self.assertEqual(before, self.n.gate.snapshot(self.n))
        self.assertEqual(self.n.state, 'running')
        self.assertEqual(self.n.source_file.read_bytes(), self.original_source)
        self.assertTrue(self.n.freeze_pub.messages[-1].data)
        self.assertEqual(self.n.counts['commands'], 1)
        row = self.n.evidence.rows[-1][1]
        self.assertFalse(row['controller_math_called'])
        self.assertFalse(row['native_geometry_called'])
        self.assertTrue(row['native_guard_required_before_nonzero_resume'])
        self.assertEqual(row['source_cloud_stamp_ns'], 1_030_000_000)

    def test_resume_requires_guard_even_without_new_SLAM_math(self):
        self.assertTrue(self.hold())
        self.assertFalse(self.hold(clock=self.clock+5_000_000, wall=self.wall+100_000_000))
        self.assertTrue(self.n.gate.guard_required)
        # Production select ORs this flag even for duplicate_SLAM_header.
        ack = dict(stamp_ns=995_000_000, received_wall_ns=10_995_000_000,
                   sequence=1000, requested_command_body=[0., 0., 0.], applied_command_body=[0., 0., 0.])
        _, row = self.n.cascade.update(self.n.feedback, self.n.paired_imu, ack,
                                       self.clock+5_000_000, self.wall+100_000_000)
        self.assertFalse(row['controller_updated'])
        self.assertTrue(bool(row['controller_updated']) or self.n.gate.guard_required)
        self.n.gate.guard_completed()
        self.assertFalse(self.n.gate.guard_required)

    def test_last_guard_clock_covers_clock_advance_inside_prior_callback(self):
        self.n.gate.last_clock_ns = self.clock-5_000_000
        self.assertTrue(self.hold())
        self.assertEqual(self.n.command, [0., 0., 0.])
        self.assertEqual(self.n.state, 'running')

    def test_300ms_wall_stall_resets_PI_clear_and_arrival_dwell(self):
        self.n.obstacle_clear_since = .5
        self.n.region_arrival.since = 900_000_000
        self.n.cascade.capture_dwell = 900_000_000
        self.assertTrue(self.hold(wall=self.wall+300_000_000))
        row = self.n.evidence.rows[-1][1]
        self.assertEqual(row['decision'], 'stalled')
        self.assertTrue(row['protected_or_stale_reset'])
        np.testing.assert_array_equal(self.n.cascade.velocity_integral, [0., 0., 0.])
        self.assertIsNone(self.n.obstacle_clear_since)
        self.assertIsNone(self.n.region_arrival.since)
        self.assertIsNone(self.n.cascade.capture_dwell)
        self.assertIsNone(self.n.cascade.failure_latched)
        self.assertEqual(self.n.state, 'running')

    def test_original_continuous_stale_8s_failure_is_retained(self):
        self.assertTrue(self.hold(wall=self.wall+300_000_000))
        self.assertTrue(self.hold(wall=self.wall+8_310_000_000))
        self.assertEqual(self.n.state, 'failed')
        self.assertEqual(self.n.command, [0., 0., 0.])
        self.assertIn('8秒', self.n.message)

    def test_actual_clock_backward_latches_failure(self):
        self.assertTrue(self.hold(clock=self.clock-5_000_000))
        self.assertEqual(self.n.state, 'failed')
        self.assertEqual(self.n.cascade.failure_latched, 'clock_backwards')

    def test_same_header_changed_actual_feedback_not_benign_duplicate(self):
        self.n.feedback['position_world_xyz'][0] = .1
        self.assertTrue(self.hold())
        self.assertEqual(self.n.state, 'failed')
        self.assertIn('same_header_changed_payload_or_receipt', self.n.message)
        self.assertEqual(self.n.cascade.failure_latched, 'actual_source_identity_invalid')

    def test_same_header_changed_cloud_not_benign_duplicate(self):
        self.n.guard_cloud_receipt['filtered_xyz_float64_sha256'] = 'altered'
        self.assertTrue(self.hold())
        self.assertEqual(self.n.state, 'failed')

    def test_short_duplicate_executes_original_severe_guard(self):
        self.n.severe = True
        self.assertTrue(self.hold())
        self.assertEqual(self.n.tilt_calls, 1)
        self.assertEqual(self.n.state, 'failed')
        self.assertEqual(self.n.command, [0., 0., 0.])

    def test_evidence_error_is_fatal_during_duplicate(self):
        self.n.evidence.error = 'archive failed'
        self.assertTrue(self.hold())
        self.assertEqual(self.n.state, 'failed')
        self.assertEqual(self.n.command, [0., 0., 0.])

    def test_original_source_wall_TTL_protects_short_clock_duplicate(self):
        self.n.pose_updated = 10.7
        self.assertTrue(self.hold())
        self.assertTrue(self.n.cascade.protected)
        self.assertIsNone(self.n.cascade.failure_latched)
        self.assertEqual(self.n.command, [0., 0., 0.])

    def test_original_source_sim_TTL_protects_short_clock_duplicate(self):
        # Accepted sources were already old before this test clock identity.
        self.n.cloud_stamp = 700_000_000
        self.n.guard_cloud_receipt = self.n.cloud(self.n.cloud_stamp, 11.)
        self.n.gate.sources.pop('registered_cloud')
        self.assertTrue(self.hold())
        self.assertTrue(self.n.cascade.protected)
        self.assertEqual(self.n.command, [0., 0., 0.])

    def test_short_parking_duplicate_cannot_advance_any_dwell(self):
        self.n.state = 'succeeded'
        self.n.cascade.capture_dwell = 900_000_000
        self.n.cascade.hold_stamp = 950_000_000
        self.n.cascade.turn_dwell = 910_000_000
        self.n.obstacle_clear_since = .7
        self.n.arrival_since = .8
        self.n.region_arrival.since = 900_000_000
        before = self.n.gate.snapshot(self.n)
        self.assertTrue(self.hold())
        self.assertEqual(before, self.n.gate.snapshot(self.n))
        self.assertEqual(self.n.state, 'succeeded')

    def test_fatal_unflushed_row_cannot_refresh_cascade_receipt(self):
        self.n.pid_row = {'unflushed': True}
        with self.assertRaisesRegex(RuntimeError, 'unflushed'):
            self.hold()
        self.assertEqual(self.n.source_file.read_bytes(), self.original_source)


if __name__ == '__main__':
    unittest.main()
