#!/usr/bin/env python3
"""Independent, ROS-free current-reference/payload ordering review fixtures.

These are synthetic protocol fixtures, not a replay of missing run11 B-splines.
They cannot establish collision freedom or complete mission success.
"""
import hashlib
import itertools
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

NAV = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAV))
from trajectory_contract import TrajectoryAssociation, payload


def message(identity=1, start=101):
    return NS(order=3, traj_id=identity, start_time=NS(sec=start, nanosec=123),
              pos_pts=[NS(x=.03*i, y=.4*i, z=.001*i) for i in range(4)],
              knots=[0.]*4+[5.]*4)


def metadata(msg, stamp=(100, 0), goal=(0., 2., 0.)):
    return dict(schema=1, reference_stamp=list(stamp), body_goal=list(goal),
                adjusted_body_goal=[.2, 1.8, 0.], trajectory=payload(msg))


class IndependentAssociationReview(unittest.TestCase):
    def setUp(self):
        self.c = TrajectoryAssociation(4)
        self.c.request([100, 0], [0., 2., 0.])

    def complete(self, msg, meta=None, reverse=False):
        meta = metadata(msg) if meta is None else meta
        calls = [(self.c.add_spline, msg), (self.c.add_metadata, meta)]
        if reverse:
            calls.reverse()
        result = None
        for fn, value in calls:
            result = fn(value) or result
        return result

    def test_both_arrival_orders_preserve_original_object(self):
        for reverse in (False, True):
            self.setUp()
            msg = message()
            self.assertIs(self.complete(msg, reverse=reverse)[0], msg)

    def test_all_two_trajectory_interleavings_keep_latest(self):
        # Includes delays/replay beyond normal ordered single-writer delivery.
        for order in itertools.permutations(['s1', 'm1', 's2', 'm2']):
            with self.subTest(order=order):
                self.setUp()
                messages = {i: message(i, 101+i) for i in (1, 2)}
                accepted = []
                for event in order:
                    msg = messages[int(event[1])]
                    result = (self.c.add_spline(msg) if event[0] == 's'
                              else self.c.add_metadata(metadata(msg)))
                    if result:
                        accepted.append(result[0].traj_id)
                self.assertEqual(accepted[-1], 2)
                self.assertEqual(accepted, sorted(set(accepted)))

    def test_same_reference_old_pair_cannot_reactivate(self):
        self.assertIsNotNone(self.complete(message(1)))
        self.assertIsNotNone(self.complete(message(2)))
        self.assertIsNone(self.complete(message(1)))

    def test_new_reference_may_restart_identity_without_old_cache(self):
        self.complete(message(99))
        self.c.add_spline(message(100))
        self.c.request([102, 0], [1., 0., 0.])
        msg = message(1)
        self.assertIsNotNone(self.complete(msg, metadata(msg, [102, 0], [1., 0., 0.])))
        self.assertEqual(len(self.c.splines), 0)

    def test_same_goal_new_reference_rejects_previous_freeze_shifted_time(self):
        old = message(4, 900)
        self.c.request([101, 0], [0., 2., 0.])
        self.assertIsNone(self.complete(old))
        self.assertIn('reference', self.c.last_rejected)

    def test_new_goal_before_first_reference_rejects_both_old_messages(self):
        self.c.reset()
        self.assertIsNone(self.complete(message()))
        self.assertIsNone(self.complete(message(), reverse=True))

    def test_endpoint_adjustment_never_replaces_original_goal(self):
        msg = message()
        meta = metadata(msg)
        self.assertIsNotNone(self.complete(msg, meta))
        self.setUp()
        meta['body_goal'] = meta['adjusted_body_goal']
        self.assertIsNone(self.complete(msg, meta))

    def test_full_payload_all_fields_must_match(self):
        changes = {
            'order': lambda p: p.update(order=2),
            'id': lambda p: p.update(traj_id=2),
            'start': lambda p: p['start_time'].__setitem__(1, 124),
            'points': lambda p: p['pos_pts'][2].__setitem__(0, 500.),
            'knots': lambda p: p['knots'].__setitem__(-1, 6.),
        }
        for label, change in changes.items():
            with self.subTest(field=label):
                self.setUp()
                msg = message()
                meta = metadata(msg)
                change(meta['trajectory'])
                self.assertIsNone(self.complete(msg, meta))

    def test_failed_reference_no_metadata_cannot_accept(self):
        self.c.request([102, 0], [0., 2., 0.])
        self.assertIsNone(self.c.add_spline(message(10)))
        self.assertIsNone(self.c.add_metadata(metadata(message(10))))

    def test_both_caches_are_bounded(self):
        for i in range(30):
            self.c.add_spline(message(i))
            self.c.add_metadata(metadata(message(100+i)))
        self.assertEqual(len(self.c.splines), 4)
        self.assertEqual(len(self.c.metadata), 4)
        self.c.reset()
        self.assertFalse(self.c.splines or self.c.metadata)

    def test_nonobject_json_rejected_without_uncaught_attribute_error(self):
        for value in (None, [], 123, 'metadata'):
            with self.subTest(value=value):
                # Controller already catches ValueError/TypeError/KeyError.
                try:
                    result = self.c.add_metadata(value)
                except (ValueError, TypeError, KeyError):
                    continue
                self.assertIsNone(result)

    def test_provenance_does_not_invent_geometry_acceptance(self):
        msg = message()
        # Protocol validates identity only. Controller must still check shape,
        # start distance and nondegenerate progress; LiDAR checks stay separate.
        for p in msg.pos_pts:
            p.x, p.y, p.z = 0., 0., 0.
        self.assertIsNotNone(self.complete(msg))


if __name__ == '__main__':
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(IndependentAssociationReview)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    evidence = {
        'kind': 'independent synthetic protocol regression; no ROS or Gazebo',
        'source_sha256': {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in
                          (NAV/'trajectory_contract.py', Path(__file__))},
        'tests_run': result.testsRun, 'passed': result.wasSuccessful(),
        'failures': [{'test': str(t), 'traceback': trace} for t, trace in result.failures],
        'errors': [{'test': str(t), 'traceback': trace} for t, trace in result.errors],
        'not_claimed': ['exact run11 trajectory replay', 'collision freedom', 'full mission pass'],
    }
    out = NAV/'test_results'/'review_trajectory_association.json'
    out.write_text(json.dumps(evidence, indent=2)+'\n')
    print(out)
    raise SystemExit(0 if result.wasSuccessful() else 1)
