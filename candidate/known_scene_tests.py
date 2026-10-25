"""Targeted safety/geometry tests, including both retained R7 sensor frames.

Conditional confirmations use explicitly artificial *provenance only* to test
the gate's logic; they do not upgrade the historical R7 evidence or navigation.
"""
import copy
import time
import unittest
import numpy as np
from known_scene_matcher import KnownSceneMatcher, CorrectionState, checked_transform
from known_scene_adapter import ExactSourceJoin, MatcherWorker, validate_runtime_witness
from replay_r7 import load_reference, load_capture


class KnownSceneTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.reference = load_reference()
        cls.jobs = [load_capture(label)[0] for label in ('F2_before_drop', 'F2_after_drop')]
        cls.results = []
        matcher = KnownSceneMatcher(cls.reference)
        for job in cls.jobs:
            cls.results.append(matcher.match(job['source_ns'], job['body_points'],
                                             job['T_odom_body'], provenance=job['provenance']))

    def test_real_frames_find_upper_face_but_reject_missing_witness(self):
        for result in self.results:
            self.assertEqual(result['reason'], 'timing_or_stage_evidence_missing')
            self.assertFalse(result['accepted'])
            self.assertFalse(result['control_allowed'])
            self.assertAlmostEqual(result['T_world_body'][2][3], 1.50, delta=.03)
            self.assertLess(result['best']['rms_capped_m'], .06)

    def test_real_bottom_alias_rejected_by_ray_not_just_small_residual(self):
        for result in self.results:
            bottom = [c for c in result['candidates'] if 1.25 < c['T_world_body'][2][3] < 1.36]
            self.assertTrue(bottom)
            best = min(bottom, key=lambda c:c['rms_capped_m'])
            self.assertLess(best['rms_capped_m'], .06)
            self.assertGreater(best['rays']['behind_fraction'], .45)
            self.assertIn('ray_occlusion', best['rejections'])

    def test_real_wrong_floor_rejected(self):
        for result in self.results:
            wrong_floor = [c for c in result['candidates'] if c['T_world_body'][2][3] < .5]
            self.assertTrue(wrong_floor)
            self.assertTrue(all(not c['quality_passed'] for c in wrong_floor))

    def test_actual_accepted_requires_independent_stamp(self):
        # Inject the already-computed pose into the fitting function to test
        # confirmation logic without repeating expensive real sensor fitting.
        matcher = KnownSceneMatcher(self.reference)
        targets = [checked_transform(r['T_world_body']) for r in self.results]
        provenance = dict(point_time_verified=True, post_lio_association_verified=True,
                          source_hashes_verified=True, no_robot_truth_pose=True,
                          testing_only_artificial_provenance=True)
        for index, job in enumerate(self.jobs):
            matcher._fit = lambda cloud, initial, deadline, t=targets[index]: (t.copy(), True, 1)
            result = matcher.match(job['source_ns'], job['body_points'], job['T_odom_body'],
                                   provenance=provenance)
            self.assertEqual(result['accepted'], index == 1)
            if index == 0:
                same = matcher.match(job['source_ns'], job['body_points'], job['T_odom_body'],
                                     provenance=provenance)
                self.assertFalse(same['accepted'])
                self.assertEqual(same['reason'], 'non_increasing_confirmation_stamp')

    def test_witness_boolean_without_bindings_rejected(self):
        witness = dict(schema='go2_known_scene_runtime_witness/v1', run_id='fake',
                       producer_model='gz_gpu_lidar_instantaneous_header_snapshot',
                       actual_loaded_libraries_verified=True, header_snapshot_time_verified=True,
                       post_lio_single_owner_publication_verified=True, no_robot_truth_pose=True)
        status = validate_runtime_witness(witness, 'fake')
        self.assertFalse(status['valid'])
        self.assertIn('missing_loaded_producer_roles', status['reasons'])
        self.assertIn('missing_stage_sources', status['reasons'])

    def test_correction_rate_generation_expiry_and_common_version(self):
        state = CorrectionState()
        c = np.eye(4)
        c[2, 3] = .30
        self.assertFalse(state.propose(dict(accepted=False)))
        self.assertTrue(state.propose(dict(accepted=True, evidence_verified=True,
                                           source_ns=1_000_000_000, C_world_odom=c.tolist())))
        state.tick(1_000_000_000)
        snapshot = state.tick(1_100_000_000)
        self.assertEqual(snapshot['generation'], 1)
        self.assertAlmostEqual(snapshot['C_world_odom'][2][3], .004)
        pose, cloud, velocity = state.apply(snapshot, np.eye(4), np.zeros((2, 3)), [1, 0, 0])
        self.assertAlmostEqual(pose[2, 3], cloud[0, 2])
        np.testing.assert_equal(velocity, [1, 0, 0])
        expired = state.tick(5_000_000_000)
        self.assertTrue(expired['correction_stale'])
        np.testing.assert_equal(expired['C_world_odom'], snapshot['C_world_odom'])
        self.assertFalse(expired['frontend_velocity_or_bias_repaired'])

    def test_reference_hash_and_transform_rejected(self):
        with self.assertRaises(ValueError):
            checked_transform(np.ones((4, 4)))
        from known_scene_matcher import SceneReference
        from replay_r7 import STAGE, WORLD_SHA
        with self.assertRaises(ValueError):
            SceneReference.load(STAGE / 'REFERENCE_GEOMETRY.json', 'wrong', WORLD_SHA)

    def test_public_world_origin_cancelled_for_navigation_pose_and_cloud(self):
        baseline = np.eye(4)
        baseline[2, 3] = .3
        nav_from_world = np.linalg.inv(baseline)
        state = CorrectionState(initial_c=baseline)
        snapshot = state.tick(1_000_000_000)
        raw = np.eye(4)
        raw[2, 3] = 1.2
        cloud_raw = np.array([[0, 0, 1.2], [1, 0, 1.2]])
        pose, cloud, _ = state.apply(snapshot, raw, cloud_raw,
                                    T_navigation_world=nav_from_world)
        self.assertAlmostEqual(pose[2, 3], 1.2)
        np.testing.assert_allclose(cloud, cloud_raw, atol=1e-15)
        for result in self.results:
            c = checked_transform(result['C_world_odom'])
            job = self.jobs[self.results.index(result)]
            converted = nav_from_world @ c @ job['T_odom_body']
            self.assertAlmostEqual(converted[2, 3], 1.2, delta=.03)

    def test_join_and_match_are_deferred_and_parent_generation_is_bound(self):
        real_job = self.jobs[0]
        class SlowFixtureJoin:
            def pop_latest(self, clock_ns):
                time.sleep(.05)
                return real_job
        worker = MatcherWorker(KnownSceneMatcher(self.reference))
        try:
            started = time.monotonic()
            self.assertTrue(worker.submit_join(SlowFixtureJoin(), real_job['source_ns'],
                                               np.eye(4), parent_generation=12))
            self.assertLess(time.monotonic() - started, .02)
            deadline = time.monotonic() + 3
            result = None
            while result is None and time.monotonic() < deadline:
                started = time.monotonic()
                result = worker.poll()
                self.assertLess(time.monotonic() - started, .02)
                if result is None:
                    time.sleep(.01)
            self.assertIsNotNone(result)
            self.assertEqual(result['parent_generation'], 12)
            self.assertEqual(result['source_ns'], real_job['source_ns'])
            self.assertFalse(result['accepted'])
        finally:
            self.assertTrue(worker.close())


if __name__ == '__main__':
    unittest.main(verbosity=2)
