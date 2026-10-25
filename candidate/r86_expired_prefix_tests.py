"""T2 (R86 S1): retire only the clearly expired FIFO head before business.

No ROS node, simulator or actuator: real CDR/hash mailbox, the real
KnownSceneInputMixin commit path and the real PIDNavigation->TeacherNavigation
callbacks are used with in-memory fixtures only.
"""
import json
import random
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np

from known_scene_inputs import KnownSceneInputMixin
from point_provenance import verify_corrected, message_sha
from r86_prepared_feedback import (ros_mailbox, expired_prefix_enabled,
    expired_prefix_length, expired_prefix_timeout_ns, EXPIRED_SCHEMA, SCHEMA)
from r86_prepared_feedback_tests import packet, feed, Base
from ordinary_parking import OrdinaryParking
import controller

EXPIRED = dict(enabled=True, schema=EXPIRED_SCHEMA)


class S1Base(Base):
    def __init__(self, prepared=True, expired=True):
        super().__init__(prepared)
        if expired:
            self.profile['r86_expired_prefix'] = dict(EXPIRED)


class S1Node(KnownSceneInputMixin, S1Base):
    pass


def node(clock_ns=1_400_000_000, expired=True):
    n = S1Node(expired=expired)
    n.clock.nanoseconds = clock_ns
    # Production Teacher sets this from pose_cloud_timeout_s; the bare Base
    # fixture does not declare it.
    n.pose_timeout = .3
    return n


def bad_frame_packet(ns, commit):
    data = packet(ns, commit)
    # create_cloud_xyz32 keeps the same Header object, so the frame change is
    # visible on both messages and both recorded hashes must follow it.
    data['body'].header.frame_id = 'not_camera_init'
    data['metadata']['corrected_body_message_sha256'] = message_sha(data['body'])
    data['metadata']['corrected_cloud_message_sha256'] = message_sha(data['cloud'])
    return data


def expired_rows(node):
    return [row for row in node.rows if row.get('schema') == 'r86_expired_prefix/v1']


def parking_node(ready):
    n = node(1_400_000_000)
    ordinary = OrdinaryParking()
    ordinary.begin(1_000_000_000, 10_000_000_000)
    ordinary.dwell = 1_000_000_000
    if ready:
        ordinary.ready_stamp = 1_000_000_000
        ordinary.ready_clock = 1_000_000_000
        ordinary.origin = np.zeros(3)
        ordinary.origin_yaw = 0.
    # Enough state for the real cascade_core.Controller._protect latch logic.
    core = NS(ordinary_parking=ordinary, failure_latched=None, protected=False,
              protection_reason=None, command=np.zeros(3),
              velocity_integral=np.zeros(3), hold_integral=np.zeros(3),
              filtered=None, inner_filtered=None, last_math_stamp=None,
              last_math_wall=None, last_reference_speed=None,
              last_curvature_yaw_reference=0., turn_phase=None, turn_dwell=None,
              capture_dwell=None, recovery=[], hold_stamp=None,
              parking_interrupted=False, diagnostic_observe_only=None)
    from cascade_core import Controller
    def _protect(reason, fatal=False):
        return Controller._protect(core, reason, fatal)
    core._protect = _protect
    n.cascade = core
    return n, ordinary


class PredicateTests(unittest.TestCase):
    def test_predicate_is_monotone_and_the_expired_set_is_a_head_prefix(self):
        rng = random.Random(20261023)
        timeout = expired_prefix_timeout_ns(.3)
        for _ in range(200):
            stamps = sorted(rng.randrange(0, 3_000_000_000) for _ in range(rng.randrange(0, 9)))
            clock = rng.randrange(0, 3_000_000_000)
            box = ros_mailbox(verify_corrected)
            # Build an immutable batch through the real join/verify path.
            for index, stamp in enumerate(stamps):
                feed(box, packet(stamp, index + 1))
            batch = box.take_batch()
            self.assertEqual([b.source_ns for b in batch], stamps)
            expected = sum(1 for stamp in stamps if clock - stamp >= timeout)
            if expected and any(stamp > clock for stamp in stamps[:expected]):
                self.fail('a future stamp was classified expired')
            self.assertEqual(expired_prefix_length(batch, clock, timeout), expected)
            remaining = expired_prefix_length(batch, clock, timeout)
            if remaining:
                self.assertGreaterEqual(clock - batch[remaining - 1].source_ns, timeout)
            if remaining < len(batch):
                self.assertLess(clock - batch[remaining].source_ns, timeout)
            box.release_batch(batch)

    def test_future_head_is_never_expired(self):
        box = ros_mailbox(verify_corrected)
        feed(box, packet(2_000_000_000, 1))
        feed(box, packet(1_050_000_000, 2))
        batch = box.take_batch()
        self.assertEqual(expired_prefix_length(batch, 1_400_000_000,
                                              expired_prefix_timeout_ns(.3)), 0)
        box.release_batch(batch)

    def test_exact_300ms_boundary_and_strict_option(self):
        self.assertEqual(expired_prefix_timeout_ns(.3), 300_000_000.0)
        box = ros_mailbox(verify_corrected)
        feed(box, packet(1_100_000_000, 1))
        batch = box.take_batch()
        self.assertEqual(expired_prefix_length(batch, 1_399_999_999,
                                              expired_prefix_timeout_ns(.3)), 0)
        self.assertEqual(expired_prefix_length(batch, 1_400_000_000,
                                              expired_prefix_timeout_ns(.3)), 1)
        box.release_batch(batch)
        self.assertFalse(expired_prefix_enabled({}))
        self.assertFalse(expired_prefix_enabled({'r86_expired_prefix': dict(enabled=False,
                                                                          schema=EXPIRED_SCHEMA)}))
        for bad in ({'enabled': True}, {'enabled': 'yes', 'schema': EXPIRED_SCHEMA},
                    {'enabled': True, 'schema': 'wrong'}, EXPIRED and
                    {'enabled': True, 'schema': EXPIRED_SCHEMA, 'extra': 1}):
            with self.assertRaises(ValueError):
                expired_prefix_enabled({'r86_expired_prefix': bad})


class CommitTests(unittest.TestCase):
    def test_expired_then_two_fresh_one_retired_two_dispatched_no_fault(self):
        n = node()
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        feed(n._r86_prepared_mailbox, packet(1_200_000_000, 2))
        feed(n._r86_prepared_mailbox, packet(1_300_000_000, 3))
        n.tick()
        self.assertIsNone(n._r86_prepared_mailbox.fault_reason())
        self.assertEqual([p[0] for p in n.observed], [1_200_000_000, 1_300_000_000])
        self.assertEqual(n.pose_stamp, 1_300_000_000)
        self.assertEqual(n._r86_prepared_mailbox.accounting()['retired_expired'], 1)
        self.assertTrue(n._r86_prepared_mailbox.accounting()['closes'])
        self.assertEqual(n.state, 'running')
        rows = expired_rows(n)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['first_retired_source_ns'], 700_000_000)
        self.assertEqual(rows[0]['retired_count'], 1)

    def test_single_expired_batch_retires_and_later_batch_processes(self):
        n = node()
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        n.tick()
        self.assertIsNone(n._r86_prepared_mailbox.fault_reason())
        self.assertEqual(n.observed, [])
        self.assertEqual(n.pose_stamp, -1)
        self.assertEqual(n._r86_prepared_mailbox.accounting()['retired_expired'], 1)
        feed(n._r86_prepared_mailbox, packet(1_390_000_000, 2))
        n.tick()
        self.assertIsNone(n._r86_prepared_mailbox.fault_reason())
        self.assertEqual([p[0] for p in n.observed], [1_390_000_000])
        self.assertEqual(n.pose_stamp, 1_390_000_000)

    def test_two_expired_release_bytes_once_and_accounting_closes(self):
        n = node()
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        feed(n._r86_prepared_mailbox, packet(800_000_000, 2))
        n.tick()
        mailbox = n._r86_prepared_mailbox
        after = mailbox.accounting()
        self.assertEqual(after['retired_expired'], 2)
        self.assertEqual(after['retired_bytes'] > 0, True)
        self.assertEqual(after['inflight'], 0)
        self.assertTrue(after['closes'])
        self.assertEqual(mailbox.snapshot()['retained_bytes'], 0)
        self.assertEqual(mailbox.counts['retired_bytes'],
                         mailbox.counts['retired_bytes'])  # single accounting owner
        self.assertIsNone(mailbox.fault_reason())

    def test_future_head_is_not_retired_and_the_fresh_behind_is_not_overtaken(self):
        # R99 wait stays OFF (user decision): the original future path still
        # faults. S1 must neither retire the future head nor dispatch past it.
        n = node(1_400_000_000)
        feed(n._r86_prepared_mailbox, packet(2_000_000_000, 1))
        feed(n._r86_prepared_mailbox, packet(1_390_000_000, 2))
        n.tick()
        self.assertEqual(n._r86_prepared_mailbox.accounting()['retired_expired'], 0)
        self.assertEqual(n.observed, [])
        self.assertIsNotNone(n._r86_prepared_mailbox.fault_reason())
        self.assertIn('body_not_accepted_at_commit', n._r86_prepared_mailbox.fault_reason())
        self.assertFalse(expired_rows(n))
        # Once the same head is no longer future it commits normally, and the
        # suffix is not left half-accepted.
        n2 = node(2_100_000_000)
        feed(n2._r86_prepared_mailbox, packet(2_000_000_000, 1))
        feed(n2._r86_prepared_mailbox, packet(2_050_000_000, 2))
        n2.tick()
        self.assertIsNone(n2._r86_prepared_mailbox.fault_reason())
        self.assertEqual([p[0] for p in n2.observed], [2_000_000_000, 2_050_000_000])

    def test_verify_rejected_is_not_retired(self):
        n = node()
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1, bad_hash=True))
        n.tick()
        mailbox = n._r86_prepared_mailbox
        self.assertEqual(mailbox.accounting()['retired_expired'], 0)
        self.assertEqual(mailbox.counts['rejected'], 1)
        self.assertIsNone(mailbox.fault_reason())
        self.assertEqual(n.observed, [])

    def test_default_off_keeps_the_original_fatal(self):
        n = node(expired=False)
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        n.tick()
        self.assertIn('body_not_accepted_at_commit', n._r86_prepared_mailbox.fault_reason())
        self.assertEqual(n._r86_prepared_mailbox.accounting()['retired_expired'], 0)


class HardeningTests(unittest.TestCase):
    """Checks added after an independent review of the retirement predicate."""

    def test_intake_monotonicity_makes_a_fresh_head_expired_tail_unrepresentable(self):
        # R86 ready is strictly increasing in source_ns (receive drops
        # source_ns <= last_source as a duplicate), so a fresh head followed by
        # an older expired member cannot exist. The prefix rule is therefore
        # exact, not merely sufficient, for every real batch.
        n = node()
        feed(n._r86_prepared_mailbox, packet(1_390_000_000, 1))
        feed(n._r86_prepared_mailbox, packet(1_000_000_000, 2))
        batch = n._r86_prepared_mailbox.take_batch()
        self.assertEqual([bundle.source_ns for bundle in batch], [1_390_000_000])
        self.assertEqual(n._r86_prepared_mailbox.counts['duplicates'], 3)
        n._r86_prepared_mailbox.release_batch(batch)

    def test_future_and_within_tolerance_deltas_are_never_retired(self):
        for source_ns, clock_ns in ((1_400_000_050, 1_400_000_000),
                                    (1_400_000_001, 1_400_000_000),
                                    (1_400_000_000, 1_400_000_000),
                                    (1_399_999_999, 1_400_000_000)):
            box = ros_mailbox(verify_corrected)
            feed(box, packet(source_ns, 1))
            batch = box.take_batch()
            self.assertEqual(expired_prefix_length(batch, clock_ns,
                                                  expired_prefix_timeout_ns(.3)), 0)
            box.release_batch(batch)

    def test_full_batch_retirement_advances_nothing(self):
        n = node()
        n.known_scene_commit_token = ('a' * 32, 1)
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        feed(n._r86_prepared_mailbox, packet(800_000_000, 2))
        n.tick()
        mailbox = n._r86_prepared_mailbox
        self.assertEqual(mailbox.counts['retired_expired'], 2)
        self.assertEqual(n.observed, [])
        self.assertEqual(n.pose_stamp, -1)
        self.assertEqual(n.pose_updated, -float('inf'))
        self.assertEqual(n.known_scene_commit_token, ('a' * 32, 1))
        self.assertTrue(mailbox.accounting()['closes'])
        self.assertEqual(mailbox.snapshot()['retained_bytes'], 0)
        rows = expired_rows(n)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]['batch_count'], 2)
        self.assertEqual(rows[0]['remaining_count'], 0)

    def test_later_member_fault_keeps_retired_evidence_and_one_release(self):
        n = node()
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        feed(n._r86_prepared_mailbox, packet(800_000_000, 2))
        feed(n._r86_prepared_mailbox, packet(1_300_000_000, 3))
        n.reject_body = True  # the fresh third member is rejected by business
        n.tick()
        mailbox = n._r86_prepared_mailbox
        self.assertEqual(mailbox.counts['retired_expired'], 2)
        self.assertEqual(len(expired_rows(n)), 1)
        self.assertIn('body_not_accepted_at_commit', mailbox.fault_reason())
        self.assertTrue(mailbox.accounting()['closes'])
        self.assertEqual(mailbox.snapshot()['retained_bytes'], 0)

    def test_configuration_error_releases_the_batch_once(self):
        n = node()
        n.pose_timeout = None  # misconfiguration: exact timeout required
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        with self.assertRaises(ValueError):
            n.tick()
        mailbox = n._r86_prepared_mailbox
        self.assertEqual(mailbox.snapshot()['retained_bytes'], 0)
        self.assertEqual(mailbox.accounting()['inflight'], 0)
        self.assertEqual(mailbox.accounting()['retired_expired'], 0)

    def test_duplicate_source_stamp_never_forms_two_batch_members(self):
        n = node()
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        feed(n._r86_prepared_mailbox, packet(700_000_000, 2), arrival=2_000_000)
        batch = n._r86_prepared_mailbox.take_batch()
        self.assertEqual(len(batch), 1)
        self.assertEqual(n._r86_prepared_mailbox.counts['duplicates'], 3)
        n._r86_prepared_mailbox.release_batch(batch)
        self.assertEqual(n._r86_prepared_mailbox.snapshot()['retained_bytes'], 0)


class ParkingTests(unittest.TestCase):
    def test_capture_retire_invalidates_positive_dwell(self):
        n, ordinary = parking_node(False)
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        n.tick()
        self.assertIsNone(ordinary.dwell)
        self.assertEqual(ordinary.stage, 'capture')
        self.assertIsNone(ordinary.ready_stamp)
        self.assertIsNone(ordinary.failure)
        rows = expired_rows(n)
        self.assertEqual(rows[0]['parking_action'], 'capture_dwell_invalidated')

    def test_fresh_frame_after_retirement_restarts_dwell_from_zero(self):
        n, ordinary = parking_node(False)
        self.assertEqual(ordinary.dwell, 1_000_000_000)
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        n.tick()
        self.assertIsNone(ordinary.dwell)
        # The next real control update starts a new dwell at the fresh source
        # stamp. A 700 ms stale gap can never qualify as 600 ms of continuity.
        ordinary.update(1_390_000_000, 1_400_000_000, np.zeros(3), 0.,
                        np.zeros(3), 0., 0., 0., 0.)
        self.assertEqual(ordinary.dwell, 1_390_000_000)
        self.assertLess(1_390_000_000 - ordinary.dwell, 600_000_000)
        self.assertIsNone(ordinary.ready_stamp)

    def test_zero_hold_retire_uses_the_original_no_retry_guard(self):
        n, ordinary = parking_node(True)
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        n.tick()
        self.assertEqual(ordinary.ready_stamp, 1_000_000_000)
        self.assertEqual(ordinary.stage, 'failed')
        self.assertEqual(ordinary.failure,
                         'ordinary_zero_hold_interrupted: expired_prefix_retired')
        self.assertEqual(n.cascade.failure_latched,
                         'ordinary_zero_hold_interrupted: expired_prefix_retired')
        # A later fresh packet cannot wash the failure away or grant a dwell.
        feed(n._r86_prepared_mailbox, packet(1_390_000_000, 2))
        n.tick()
        self.assertEqual(ordinary.failure,
                         'ordinary_zero_hold_interrupted: expired_prefix_retired')
        self.assertEqual(ordinary.stage, 'failed')
        self.assertEqual(ordinary.dwell, 1_000_000_000)
        rows = expired_rows(n)
        self.assertEqual(rows[0]['parking_action'], 'zero_hold_invalidated')


class ReceiptTests(unittest.TestCase):
    def test_receipt_token_and_pose_do_not_advance_on_retirement(self):
        n = node()
        n.pose_stamp = 500_000_000
        n.known_scene_commit_token = ('a' * 32, 1)
        before_updated = n.pose_updated
        feed(n._r86_prepared_mailbox, packet(700_000_000, 1))
        n.tick()
        self.assertEqual(n.pose_stamp, 500_000_000)
        self.assertEqual(n.known_scene_commit_token, ('a' * 32, 1))
        self.assertEqual(n.pose_updated, before_updated)
        row = expired_rows(n)[0]
        self.assertEqual(row['pose_stamp_advanced'], False)
        self.assertEqual(row['known_scene_commit_token_advanced'], False)
        self.assertEqual(row['source_receipts_refreshed'], False)
        # The next fresh commit still advances the token high-water exactly once.
        feed(n._r86_prepared_mailbox, packet(1_390_000_000, 2))
        n.tick()
        self.assertIsNone(n._r86_prepared_mailbox.fault_reason())
        self.assertEqual(n.known_scene_commit_token, ('a' * 32, 2))
        self.assertEqual(n.pose_stamp, 1_390_000_000)


class ProductionChainTests(unittest.TestCase):
    class Fixture(KnownSceneInputMixin, controller.PIDNavigation):
        def __init__(self, expired=True):
            self.profile = {'r86_prepared_feedback': dict(enabled=True, schema=SCHEMA)}
            if expired:
                self.profile['r86_expired_prefix'] = dict(EXPIRED)
            self.run = Path('/offline-r86-s1-fixture')
            self.rows = []
            self.scan_poses = []
            self.scan_clouds = []
            self.archived = []
            self.heading = []
            self.evidence = NS(append=lambda path, row: self.rows.append((path.name, row)))
            self.clock = NS(nanoseconds=1_440_000_000)
            self._r86_prepared_mailbox = ros_mailbox(verify_corrected)
            self._r86_committing_bundle = None
            self._r86_scope = None
            self._r86_scope_since_ns = 0
            self._r86_expired_prefix = expired
            self._r99_policy = None
            self._r99_waiting = False
            self._r99_timer = None
            self._r99_timer_active = False
            self._r99_zero_published = False
            self.known_scene_inputs = None
            self.known_scene_callbacks = {'body': self.on_odom, 'cloud': self.on_cloud}
            self.pose_stamp = self.cloud_stamp = -1
            self.pose_updated = self.cloud_updated = -float('inf')
            self.pose_timeout = self.cloud_timeout = .3
            self.pose_history = __import__('collections').deque(maxlen=128)
            self.imu_history = __import__('collections').deque(maxlen=128)
            self.gyro_body = self.gyro_stamp = self.gyro_wall = None
            self.imu_body_rotation = np.eye(3)
            self.imu_reference_rotation = np.eye(3)
            from control_core import TrackingPositionFilter, RawImuTilt
            from imu_causal_buffer import CausalImuBuffer
            self.raw_imu = RawImuTilt(np.eye(3))
            self.imu_source_buffer = CausalImuBuffer(self, RawImuTilt)
            self.heading_gate = NS(quiet_target_enabled=False, handoff_active=False,
                                   controlled_braking=False,
                                   observe=lambda *a, **k: self.heading.append((a, k)),
                                   evidence=lambda *a: {})
            self.tracking_filter = TrackingPositionFilter()
            self.yaw_rate = 0.
            from collections import Counter
            self.counts = Counter()
            self.scan_odom_pub = NS(publish=self.scan_poses.append)
            self.scan_cloud_pub = NS(publish=self.scan_clouds.append)
            self.command = [0., 0., 0.]
            self.request_id = 'fixture'
            self.waypoint_index = 0
            self.feedback = self.paired_imu = self.cascade = None
            self.cloud_callback_records = 0
            self.pending_cloud_callback_row = None
            self.cloud_input_context = None
            self.state = 'running'
            self.message = ''
            self._r86_update_scope()

        def get_clock(self):
            return NS(now=lambda: self.clock)

        def archive_cloud(self, msg):
            self.archived.append(self.stamp(msg))

    def test_expired_bad_frame_id_is_retired_but_fresh_bad_frame_id_stays_fatal(self):
        # The real on_odom rejects a non camera_init frame and never advances
        # pose_stamp, so a dispatched bad frame is the original fatal. Retirement
        # is the documented masking bound: an already expired frame is dropped
        # before any business callback and therefore cannot fault.
        n = self.Fixture()
        feed(n._r86_prepared_mailbox, bad_frame_packet(1_040_000_000, 1))
        n._r86_commit_feedback()
        self.assertIsNone(n._r86_prepared_mailbox.fault_reason())
        self.assertEqual(n._r86_prepared_mailbox.accounting()['retired_expired'], 1)
        self.assertEqual(n.pose_stamp, -1)
        feed(n._r86_prepared_mailbox, bad_frame_packet(1_400_000_000, 2))
        n._r86_commit_feedback()
        self.assertIn('body_not_accepted_at_commit', n._r86_prepared_mailbox.fault_reason())
        self.assertEqual(n.pose_stamp, -1)

    def test_real_chain_one_retire_one_fresh_commit(self):
        n = self.Fixture()
        feed(n._r86_prepared_mailbox, packet(1_040_000_000, 1))
        n._r86_commit_feedback()
        self.assertIsNone(n._r86_prepared_mailbox.fault_reason())
        self.assertEqual(n.pose_stamp, -1)
        self.assertEqual(len(n.scan_poses), 0)
        self.assertEqual(n._r86_prepared_mailbox.accounting()['retired_expired'], 1)
        feed(n._r86_prepared_mailbox, packet(1_400_000_000, 2))
        n._r86_commit_feedback()
        self.assertIsNone(n._r86_prepared_mailbox.fault_reason())
        self.assertEqual(n.pose_stamp, 1_400_000_000)
        self.assertEqual(n.cloud_stamp, 1_400_000_000)
        self.assertEqual(len(n.scan_poses), 1)
        self.assertEqual(n.feedback['stamp_ns'], 1_400_000_000)
        self.assertTrue(n._r86_prepared_mailbox.accounting()['closes'])


if __name__ == '__main__':
    unittest.main()
