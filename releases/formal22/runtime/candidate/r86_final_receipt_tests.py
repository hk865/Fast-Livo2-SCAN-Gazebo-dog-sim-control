"""T1: final_bundle keeps the original path receipt without a deep copy.

No Node, ROS runtime or physical state: the production builders under test are
controller._final_path_receipt_snapshot, PIDNavigation.final_bundle_reference,
the R96 probe hook and PIDNavigation.control_parking's binding lines.
"""
import copy
import gc
import hashlib
import time
import tracemalloc
import unittest
from types import SimpleNamespace as NS

import numpy as np

import controller
import corridor
import path_commit

POINTS = 26000


def receipt(n=POINTS, trajectory_id=80, request_id='request1', waypoint_index=9,
            stamp_ns=100_000_000_000, layer_id='layer-1'):
    points = [[i * .01, (i % 7) * .001, 0.] for i in range(n)]
    array = np.asarray(points, dtype='<f8')
    return dict(
        points_xyz=points,
        source_sample_indices=list(range(n)),
        source_samples_count=n,
        source_samples_float64_sha256='a' * 64,
        points_float64_sha256=hashlib.sha256(array.tobytes()).hexdigest(),
        path_id=request_id + ':' + str(waypoint_index) + ':1:' + str(trajectory_id),
        path_sha256='b' * 64, layer_id=layer_id, frame_id='camera_init',
        stamp_ns=stamp_ns, received_wall_ns=1_000_000_123,
        array_file='trajectory_' + str(trajectory_id) + '.npz',
        request_id=request_id, waypoint_index=waypoint_index,
        trajectory_id=trajectory_id,
        metadata=dict(trajectory=dict(traj_id=trajectory_id), reference_stamp=[100, 0]))


def node(r, waypoint_index=9, trajectory_id=80, reference_stamp=(100, 0)):
    return NS(path_receipt=r, waypoint_index=waypoint_index,
              segment_start=np.array([0., 0., .1]), active_trajectory_id=trajectory_id,
              trajectory_archive_reference='trajectory_' + str(trajectory_id) + '.npz',
              active_reference_stamp=reference_stamp)


class FakeCascade:
    """Only the original Cascade.set_path record shape used by path_commit."""

    def __init__(self):
        self.capture_stamp = None
        self.path_id = None
        self.path_sha = None
        self.path_record = {}

    def set_path(self, points_xyz, path_id, stamp_ns, received_wall_ns,
                 frame_id='camera_init', position=None):
        self.path_id = path_id
        self.path_record = dict(points_xyz=np.asarray(points_xyz, float).tolist(),
                                path_id=path_id, stamp_ns=int(stamp_ns),
                                received_wall_ns=int(received_wall_ns),
                                frame_id=frame_id)
        self.path_sha = 'e' * 64


def build(r, **kwargs):
    return controller.PIDNavigation.final_bundle_reference(node(r, **kwargs))


class FinalReceiptTests(unittest.TestCase):
    def test_receipt_keys_values_and_identity_preserved(self):
        source = receipt()
        bundle = build(source)
        copied = bundle['path_receipt']
        self.assertEqual(list(copied), list(source))
        self.assertEqual(copied, source)
        self.assertEqual(copied['points_xyz'], source['points_xyz'])
        self.assertEqual(len(copied['points_xyz']), POINTS)
        # Original receipt/stamp/hash identity is never refreshed or re-derived.
        for key in ('path_id', 'path_sha256', 'stamp_ns', 'received_wall_ns', 'frame_id',
                    'points_float64_sha256', 'request_id', 'waypoint_index',
                    'trajectory_id', 'array_file'):
            self.assertEqual(copied[key], source[key], key)
        self.assertEqual(copied['metadata'], source['metadata'])
        self.assertEqual(source['stamp_ns'], 100_000_000_000)

    def test_bundle_fields_unchanged(self):
        source = receipt()
        n = node(source, waypoint_index=9, trajectory_id=80, reference_stamp=(100, 0))
        bundle = controller.PIDNavigation.final_bundle_reference(n)
        self.assertEqual(sorted(bundle), ['array', 'index', 'path_receipt', 'reference_stamp',
                                          'start', 'trajectory_id'])
        self.assertEqual(bundle['index'], 9)
        self.assertEqual(bundle['trajectory_id'], 80)
        self.assertEqual(bundle['array'], 'trajectory_80.npz')
        self.assertEqual(bundle['reference_stamp'], (100, 0))
        self.assertTrue(np.array_equal(bundle['start'], n.segment_start))
        self.assertIsNot(bundle['start'], n.segment_start)

    def test_isolation_on_replacement_and_documented_nested_alias(self):
        source = receipt(64)
        old_points, old_metadata = source['points_xyz'], source['metadata']
        n = node(source)
        bundle = controller.PIDNavigation.final_bundle_reference(n)
        copied = bundle['path_receipt']
        # Top-level isolation: a later receipt replacement/binding cannot rewrite
        # the already built bundle.
        self.assertIsNot(copied, source)
        replacement = receipt(64, trajectory_id=81)
        n.path_receipt = replacement
        self.assertEqual(bundle['path_receipt']['trajectory_id'], 80)
        self.assertEqual(bundle['path_receipt'], {**source})
        copied['stamp_ns'] = 7
        self.assertNotEqual(source['stamp_ns'], 7)
        # Documented alias: the large nested containers are borrowed, never
        # copied. No control/consumer owner mutates them in place.
        self.assertIs(copied['points_xyz'], old_points)
        self.assertIs(copied['metadata'], old_metadata)
        copied['points_xyz'][0][0] = 42.
        self.assertEqual(source['points_xyz'][0][0], 42.)
        source['points_xyz'][0][0] = 0.

    def test_control_parking_binds_receipt(self):
        source = receipt(32, waypoint_index=7, trajectory_id=80)
        bundle = build(source, waypoint_index=9)
        now = time.monotonic()
        calls = []
        cascade = NS(goal_yaw=0., begun=False,
                     begin_ordinary_terminal=lambda *a: setattr(cascade, 'begun', True))
        parking = NS(final_bundle=bundle, waypoint_index=9,
                     active_reference_stamp=None, active_trajectory_id=None,
                     trajectory_archive_reference=None, path_receipt=None, segment_start=None,
                     waypoints=[np.array([1., 0., 0.]) for _ in range(11)],
                     goals=[NS(timeout_sim_s=90.)] * 11, profile={},
                     pose=np.zeros(3), command=[0., 0., 0.], pending_guard_row=None,
                     pose_updated=now - 1., cloud_updated=now - 1., pose_timeout=.3,
                     cloud_timeout=.3, paired_imu=dict(stamp_ns=1), obstacle_hold=False,
                     tilt_hold=False, state='succeeded', cascade=cascade,
                     evidence=NS(error=None),
                     get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)),
                     apply_tilt_guard=lambda now, ros: False,
                     select_pid_velocity=lambda *a, **k: calls.append(('pid', k.get('mode_override'))),
                     publish_command=lambda *a, **k: calls.append(('publish', a)))
        controller.PIDNavigation.control_parking(parking, now)
        self.assertIs(parking.path_receipt, bundle['path_receipt'])
        self.assertIs(parking.segment_start, bundle['start'])
        self.assertEqual(parking.active_reference_stamp, (100, 0))
        self.assertEqual(parking.active_trajectory_id, 80)
        self.assertEqual(parking.trajectory_archive_reference, 'trajectory_80.npz')
        self.assertEqual(parking.path_receipt['path_id'], source['path_id'])
        self.assertEqual(parking.path_receipt['path_sha256'], source['path_sha256'])
        self.assertEqual(parking.path_receipt['stamp_ns'], source['stamp_ns'])
        self.assertEqual(parking.waypoint_index, 9)
        self.assertIn(('publish', ()), calls)
        self.assertEqual(calls[0], ('pid', 'hold'))

    def test_corridor_and_path_commit_read_points_after_binding(self):
        source = receipt(200)
        bundle = build(source)
        prefix = corridor.request_prefix(bundle['path_receipt'], .25)
        self.assertTrue(prefix)
        self.assertEqual(len(prefix[0]), 3)
        fake = FakeCascade()
        n = node(source)
        n.cascade = fake
        n.cascade_key = (source['request_id'], source['waypoint_index'])
        n.request_id, n.waypoint_index = source['request_id'], source['waypoint_index']
        n.pose = np.zeros(3)
        prepared, row = path_commit.staged_cascade_binding(n, bundle['path_receipt'])
        self.assertIsNotNone(prepared)
        self.assertEqual(prepared.path_id, source['path_id'])
        self.assertEqual(prepared.path_record['points_xyz'], source['points_xyz'])
        self.assertEqual(row['path_sha256'], 'e' * 64)
        self.assertEqual(row['points_float64_sha256'], source['points_float64_sha256'])
        self.assertEqual(row['trajectory_id'], 80)

    def test_no_large_copy_no_gen2_twenty_six_thousand_points(self):
        source = receipt(POINTS)
        n = node(source)
        gc.collect()
        tracemalloc.start()
        base = tracemalloc.get_traced_memory()[0]
        tracemalloc.reset_peak()
        gc2 = gc.get_stats()[2]['collections']
        started = time.monotonic()
        bundle = controller.PIDNavigation.final_bundle_reference(n)
        snapshot_wall = time.monotonic() - started
        current, peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        snapshot_peak = peak - base
        self.assertIs(bundle['path_receipt']['points_xyz'], source['points_xyz'])
        self.assertEqual(gc.get_stats()[2]['collections'], gc2,
                         'snapshot build must not add a gen2 collection')
        self.assertLess(snapshot_peak, 256 * 1024, snapshot_peak)
        # Paired contrast: the removed deep copy allocates the whole 26k graph
        # (and is the R97 allocation point that triggered gen2).
        gc.collect()
        tracemalloc.start()
        base = tracemalloc.get_traced_memory()[0]
        tracemalloc.reset_peak()
        deep = copy.deepcopy(source)
        _current, deep_peak = tracemalloc.get_traced_memory()
        tracemalloc.stop()
        deep_peak -= base
        self.assertIsNot(deep['points_xyz'], source['points_xyz'])
        self.assertGreater(deep_peak, 2 * 1024 * 1024, deep_peak)
        self.assertGreater(deep_peak, 8 * snapshot_peak)
        self.assertLess(snapshot_wall, .005, snapshot_wall)

    def test_probe_hook_times_snapshot_without_copying(self):
        source = receipt(1024)

        class Probe:
            def __init__(self):
                self.stages, self.closed = [], False
                self.rows, self.gc_rows, self.errors = [], [], 0
                self.slow_post, self.exception_post = [], []

            def begin(self, stage, node=None):
                self.stages.append(stage)
                return (1, None, None, 1, stage, None, time.monotonic_ns(),
                        time.thread_time_ns())

            def end(self, token, error=None):
                self.rows.append(token)

        probe = Probe()
        saved = controller._r96_cpu.PROBE
        controller._r96_cpu.PROBE = probe
        try:
            bundle = controller.PIDNavigation.final_bundle_reference(node(source))
        finally:
            controller._r96_cpu.PROBE = saved
        self.assertEqual(probe.stages, ['control.final_bundle_receipt_snapshot'])
        self.assertIs(bundle['path_receipt']['points_xyz'], source['points_xyz'])
        self.assertIs(bundle['path_receipt']['metadata'], source['metadata'])

    def test_none_receipt_stays_none(self):
        bundle = controller.PIDNavigation.final_bundle_reference(node(None))
        self.assertIsNone(bundle['path_receipt'])
        self.assertIsNone(controller._final_path_receipt_snapshot(None))


if __name__ == '__main__':
    unittest.main()
