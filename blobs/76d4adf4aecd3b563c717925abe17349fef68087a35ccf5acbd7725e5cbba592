"""Synthetic CPU contract tests only; no real map/matcher/localization pass.

python3 -B submap_shadow_tests.py --output <new result directory>
The optional receipt is <=1 MiB, binds code/documentation hashes, and refuses
to overwrite prior evidence. Tests create tiny temporary synthetic map files.
"""
from __future__ import annotations

import argparse
import copy
from dataclasses import replace
import hashlib
import io
import json
import math
from pathlib import Path
import resource
import tempfile
import time
import unittest

import numpy as np

from submap_shadow import (ContractError, FrozenMap, MAP_ORDER, MAX_MAP_BYTES,
                           MatchCandidate, MatchQuery, Overlap, ShadowPolicy, SubmapShadow,
                           probe_existing_sensor_sources)


SHA = hashlib.sha256(b"synthetic calibration fixture; not real sensor evidence").hexdigest()
MATCHER_SHA = hashlib.sha256(b"synthetic matcher fixture; not implemented matcher").hexdigest()


def pose(x=0., y=0., z=0., yaw=0., roll=0.):
    cz, sz, cx, sx = math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll)
    out = np.eye(4)
    out[:3, :3] = np.array([[cz, -sz, 0.], [sz, cz, 0.], [0., 0., 1.]]) @ np.array(
        [[1., 0., 0.], [0., cx, -sx], [0., sx, cx]])
    out[:3, 3] = (x, y, z)
    return out


def manifest(map_id, blob):
    return {"schema": "frozen_sensor_submap/v1", "map_id": map_id, "frame_id": "submap/" + map_id,
            "revision": 1, "content_sha256": hashlib.sha256(blob).hexdigest(),
            "source_kind": "independent_sensor_capture", "source_capture_id": "SYNTHETIC-UNIT-FIXTURE",
            "source_session_id": "synthetic-previous-capture", "source_start_ns": 1,
            "source_end_ns": 100, "source_time_domain": "synthetic-source-clock",
            "frozen_at_utc": "2026-10-07T09:00:00Z", "calibration_sha256": SHA,
            "preprocessing_sha256": SHA, "validation_evidence_sha256": SHA,
            "independence_evidence_sha256": SHA, "gravity_evidence_sha256": SHA,
            "gravity_map": [0., 0., -9.81], "navigation_truth_used": False,
            "geometry_reviewed": True, "independence_reviewed": True}


def evidence():
    return {"schema": "independent_match_evidence/v1", "matcher_kind": "independent_frozen_map_registration",
            "converged": True, "search_complete": True, "independent_association_reviewed": True,
            "search_domain_sha256": SHA, "association_evidence_sha256": SHA,
            "tested_map_ids": list(MAP_ORDER), "distinct_hypotheses": 2,
            "best_score_m": .01, "second_score_m": .03, "rms_m": .01, "p95_m": .025,
            "inliers": 250, "occupied_cells": 16, "normal_bins": 4, "support_span_m": [2., 2., 1.],
            "inlier_fraction": .9, "overlap_fraction": .8,
            "information_model": "normalized_geometry_jacobian_gram/v1",
            "parameter_scales_m_rad": [1., 1.], "normalized_information": np.eye(6).tolist()}


class ShadowTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="v33-submap-synthetic-")
        self.addCleanup(self.temp.cleanup)
        self.maps, self.files, self.manifests = [], {}, {}
        for name in MAP_ORDER:
            blob = ("synthetic bytes, NOT a point map: " + name).encode()
            p = Path(self.temp.name) / (name + ".fixture")
            p.write_bytes(blob)
            self.files[name], self.manifests[name] = p, manifest(name, blob)
            self.maps.append(FrozenMap.load(p, self.manifests[name]))
        self.edges = [Overlap(a.map_id, b.map_id, pose(3., .2, 1.2, yaw=.1),
                      [-100., -100., -100.], [100., 100., 100.], a.content_sha256,
                      b.content_sha256, SHA, False, True) for a, b in zip(self.maps[:-1], self.maps[1:])]
        self.core = self.new_core()
        self.clock = 1_000_000_000

    def new_core(self, **kwargs):
        args = dict(session_id="synthetic-runtime-session", odom_revision=0,
                    matcher_session_id="synthetic-matcher-session", matcher_revision=0,
                    matcher_source_sha256=MATCHER_SHA, calibration_sha256=SHA,
                    odom_frame="camera_init", body_frame="demo_slam_body", maps=self.maps, overlaps=self.edges)
        args.update(kwargs)
        return SubmapShadow(**args)

    def query(self, ns=None, transform=None, gravity=None, **kwargs):
        ns = self.clock if ns is None else ns
        args = dict(session_id="synthetic-runtime-session", odom_revision=0, source_ns=ns,
                    received_wall_ns=ns, odom_frame="camera_init", body_frame="demo_slam_body",
                    odom_from_body=pose(.2, .4, .7, yaw=.2) if transform is None else transform,
                    query_payload_sha256=hashlib.sha256(str(ns).encode()).hexdigest(), calibration_sha256=SHA,
                    gravity_odom=[0., 0., -9.81] if gravity is None else gravity, gravity_evidence_sha256=SHA)
        args.update(kwargs)
        return MatchQuery(**args)

    def candidate(self, query, map_id="F1", correction=None, **kwargs):
        m = next(m for m in self.maps if m.map_id == map_id)
        correction = np.eye(4) if correction is None else correction
        args = dict(session_id=query.session_id, odom_revision=query.odom_revision,
                    matcher_session_id="synthetic-matcher-session", matcher_revision=0,
                    matcher_source_sha256=MATCHER_SHA, source_ns=query.source_ns,
                    received_wall_ns=query.received_wall_ns, map_id=map_id, map_revision=m.revision,
                    map_frame=m.frame_id, body_frame=query.body_frame, map_sha256=m.content_sha256,
                    query_payload_sha256=query.query_payload_sha256, calibration_sha256=SHA,
                    map_from_body=np.linalg.inv(self.core.canonical[map_id]) @ correction @ query.odom_from_body,
                    evidence=evidence())
        args.update(kwargs)
        return MatchCandidate(**args)

    def offer(self, q=None, c=None, **kwargs):
        q = q or self.query()
        c = c or self.candidate(q)
        args = dict(now_source_ns=q.source_ns, now_wall_ns=q.received_wall_ns)
        args.update(kwargs)
        return self.core.offer(q, c, **args)

    def ready(self, target="F1", correction=None):
        for i in range(6):
            self.clock += 100_000_000
            q = self.query()
            if target != self.core.active_shadow_map:
                self.offer(q, self.candidate(q, self.core.active_shadow_map, correction))
            result = self.offer(q, self.candidate(q, target, correction))
        self.assertEqual(result["state"], "PROPOSAL_READY", result)
        return result

    def select(self, result):
        return self.core.select_shadow_reference(result["proposal"]["generation"],
            now_source_ns=self.clock, now_wall_ns=self.clock)

    def test_five_map_sequence_has_only_shadow_transitions(self):
        for target in MAP_ORDER:
            result = self.ready(target)
            self.assertEqual(result["proposal"]["map_id"], target)
            self.assertFalse(result["control_allowed"])
            self.assertFalse(result["proposal"]["control_allowed"])
            self.assertFalse(result["proposal"]["runtime_enabled"])
            self.select(result)
            self.assertEqual(self.core.active_shadow_map, target)
        self.assertEqual(self.core.status()["state"], "NO_PROPOSAL")

    def test_correction_algebra_uses_same_source_pose_and_nonidentity_map_frames(self):
        correction = pose(.08, -.04, .20, yaw=.1)
        result = self.ready("F1", correction)
        self.select(result)
        result = self.ready("ramp12_transition", correction)
        p = result["proposal"]
        expected = np.linalg.inv(self.core.canonical["ramp12_transition"]) @ correction
        np.testing.assert_allclose(p["map_from_odom"], expected, atol=1e-14)
        np.testing.assert_allclose(p["canonical_from_odom"], correction, atol=1e-14)
        self.assertEqual(p["velocity_action"], "none_never_differentiate_correction")
        self.assertNotIn("linear_velocity", p)
        self.assertFalse(p["covariance_valid"])

    def test_physical_ramp_rise_is_not_normalized_to_floor_label(self):
        for i in range(6):
            self.clock += 100_000_000
            q = self.query(transform=pose(i * .2, 0., i * .1))
            result = self.offer(q, self.candidate(q))
        self.assertEqual(result["state"], "PROPOSAL_READY")
        np.testing.assert_allclose(result["proposal"]["map_from_odom"], np.eye(4), atol=1e-14)

    def test_one_frame_and_instant_repeated_evidence_cannot_release(self):
        r = self.offer()
        self.assertEqual(r["state"], "COLLECTING")
        r = self.offer()
        self.assertEqual(r["reason"], "duplicate_or_out_of_order_source")
        self.assertIsNone(r["proposal"])

    def test_source_duration_without_wall_duration_is_insufficient(self):
        for i in range(6):
            self.clock += 100_000_000
            q = self.query(received_wall_ns=1_000_000_000 + i)
            r = self.offer(q)
        self.assertEqual(r["state"], "COLLECTING")

    def test_wall_duration_without_source_duration_is_insufficient(self):
        for i in range(6):
            self.clock += 100_000_000
            q = self.query(ns=1_000_000_000 + i, received_wall_ns=self.clock)
            r = self.offer(q)
        self.assertEqual(r["state"], "COLLECTING")

    def test_gap_resets_the_continuous_window(self):
        r = self.ready()
        self.clock += 200_000_000
        r = self.offer()
        self.assertEqual(r["state"], "COLLECTING")
        self.assertEqual(r["buffer_counts"]["F1"], 1)

    def test_backward_input_receipt_rejects_even_without_clock_rollback(self):
        self.offer()
        self.clock += 100_000_000
        q = self.query(received_wall_ns=self.clock - 100_000_001)
        r = self.offer(q, now_wall_ns=self.clock)
        self.assertEqual(r["reason"], "wall_receipt_order_violation")

    def test_pose_drift_hidden_by_low_residual_is_rejected(self):
        self.offer()
        self.clock += 100_000_000
        q = self.query()
        r = self.offer(q, self.candidate(q, correction=pose(z=.041)))
        self.assertEqual(r["reason"], "multiframe_transform_inconsistent")

    def test_oscillating_correction_cannot_pass_pairwise_neighbor_test(self):
        for z in (-.03, 0., .03):
            self.clock += 100_000_000
            q = self.query()
            r = self.offer(q, self.candidate(q, correction=pose(z=z)))
        self.assertEqual(r["reason"], "multiframe_transform_inconsistent")

    def test_adjacent_map_requires_same_source_active_match(self):
        self.select(self.ready())
        self.clock += 100_000_000
        q = self.query()
        r = self.offer(q, self.candidate(q, "ramp12_transition"))
        self.assertEqual(r["reason"], "awaiting_same_source_active_overlap_match")

    def test_adjacent_match_cannot_pair_1ns_neighbor(self):
        self.select(self.ready())
        self.clock += 100_000_000
        q = self.query()
        self.offer(q)
        q2 = self.query(ns=q.source_ns + 1, received_wall_ns=q.received_wall_ns + 1)
        r = self.offer(q2, self.candidate(q2, "ramp12_transition"))
        self.assertEqual(r["reason"], "awaiting_same_source_active_overlap_match")

    def test_adjacent_maps_must_match_overlap_transform(self):
        self.select(self.ready())
        self.clock += 100_000_000
        q = self.query()
        self.offer(q)
        r = self.offer(q, self.candidate(q, "ramp12_transition", correction=pose(z=.051)))
        self.assertEqual(r["reason"], "overlap_transform_inconsistent")

    def test_geometric_overlap_volume_is_required(self):
        self.select(self.ready())
        self.clock += 100_000_000
        q = self.query(transform=pose(x=101.))
        self.offer(q)
        r = self.offer(q, self.candidate(q, "ramp12_transition"))
        self.assertEqual(r["reason"], "candidate_outside_reviewed_overlap_volume")

    def test_cannot_jump_from_F1_to_F2_by_region_label(self):
        q = self.query()
        r = self.offer(q, self.candidate(q, "F2"))
        self.assertEqual(r["reason"], "nonadjacent_map_requires_overlap_chain")

    def test_reverse_overlap_transition_is_also_supported(self):
        self.select(self.ready())
        self.select(self.ready("ramp12_transition"))
        self.select(self.ready("F2"))
        r = self.ready("ramp12_transition")
        self.assertTrue(r["proposal"]["same_source_overlap_verified"])

    def test_small_covariance_cannot_substitute_for_geometry(self):
        q = self.query()
        e = evidence(); e["normalized_information"] = np.diag([1., 1., 0., 1., 1., 1.]).tolist()
        r = self.offer(q, self.candidate(q, evidence=e))
        self.assertEqual(r["reason"], "unobservable_coupled_geometry")

    def test_coupled_null_direction_is_rejected_with_nonzero_z_diagonal(self):
        q = self.query()
        e = evidence()
        gram = np.eye(6); gram[2, 3] = gram[3, 2] = 1.
        e["normalized_information"] = gram.tolist()
        r = self.offer(q, self.candidate(q, evidence=e))
        self.assertEqual(r["reason"], "unobservable_coupled_geometry")

    def test_lookalike_floor_ambiguity_is_rejected(self):
        q = self.query()
        e = evidence(); e["second_score_m"] = .011
        r = self.offer(q, self.candidate(q, evidence=e))
        self.assertEqual(r["reason"], "ambiguous_matching_hypotheses")

    def test_all_five_maps_must_be_considered_for_uniqueness(self):
        q = self.query()
        e = evidence(); e["tested_map_ids"] = ["F1"]
        r = self.offer(q, self.candidate(q, evidence=e))
        self.assertEqual(r["reason"], "lookalike_maps_not_all_tested")

    def test_every_evidence_field_is_required_not_unknown_to_pass(self):
        for key in evidence():
            with self.subTest(missing=key):
                core = self.new_core()
                q = self.query(); e = evidence(); del e[key]
                r = core.offer(q, self.candidate(q, evidence=e), now_source_ns=q.source_ns, now_wall_ns=q.received_wall_ns)
                self.assertEqual(r["reason"], "missing_or_unknown_matching_evidence_fields")

    def test_spatial_distribution_cannot_be_replaced_by_point_count(self):
        q = self.query()
        e = evidence(); e["inliers"] = 100000; e["support_span_m"] = [10., .01, .01]
        r = self.offer(q, self.candidate(q, evidence=e))
        self.assertEqual(r["reason"], "insufficient_three_dimensional_support")

    def test_gravity_rejects_tilt_correction(self):
        q = self.query()
        r = self.offer(q, self.candidate(q, correction=pose(roll=math.radians(3.1))))
        self.assertEqual(r["reason"], "gravity_attitude_inconsistent")

    def test_residual_tail_gate_is_separate_from_rms(self):
        q = self.query()
        e = evidence(); e["p95_m"] = .081
        r = self.offer(q, self.candidate(q, evidence=e))
        self.assertEqual(r["reason"], "residual_distribution_exceeds_shadow_policy")

    def test_query_payload_and_source_must_be_identical(self):
        for updates in ({"source_ns": self.clock + 1}, {"query_payload_sha256": SHA}):
            with self.subTest(updates=updates):
                self.core = self.new_core(); q = self.query()
                r = self.offer(q, self.candidate(q, **updates))
                self.assertEqual(r["reason"], "candidate_query_not_same_source_and_payload")

    def test_wrong_map_hash_or_revision_or_frame_rejected(self):
        for updates in ({"map_sha256": SHA}, {"map_revision": 2}, {"map_frame": "world"}):
            with self.subTest(updates=updates):
                self.core = self.new_core(); q = self.query()
                r = self.offer(q, self.candidate(q, **updates))
                self.assertEqual(r["reason"], "candidate_not_bound_to_verified_frozen_map")

    def test_session_revision_matcher_change_latches_fault(self):
        for updates in ({"session_id": "other"}, {"odom_revision": 1}, {"matcher_revision": 1},
                        {"matcher_session_id": "other"}, {"matcher_source_sha256": SHA}):
            with self.subTest(updates=updates):
                self.core = self.new_core(); q = self.query()
                r = self.offer(q, self.candidate(q, **updates))
                self.assertEqual(r["state"], "FAULT")
                self.clock += 1
                self.assertEqual(self.offer()["state"], "FAULT")

    def test_boolean_revision_is_not_integer_evidence(self):
        q = self.query()
        r = self.offer(q, self.candidate(q, odom_revision=False))
        self.assertEqual(r["state"], "REJECTED")

    def test_calibration_or_child_frame_mismatch(self):
        for updates, reason in (({"calibration_sha256": MATCHER_SHA}, "current_sensor_calibration_mismatch"),
                                ({"body_frame": "lidar"}, "odom_or_body_frame_mismatch")):
            self.core = self.new_core(); q = self.query()
            self.assertEqual(self.offer(q, self.candidate(q, **updates))["reason"], reason)

    def test_truth_and_online_frontend_sources_are_rejected(self):
        q = self.query()
        r = self.offer(q, self.candidate(q, navigation_truth_used=True))
        self.assertEqual(r["reason"], "truth_or_unknown_candidate_schema")
        with self.assertRaises(ContractError):
            self.query(geometry_source="gazebo_truth")
        with self.assertRaises(ContractError):
            self.query(gravity_source="frontend_small_P")
        for source in ("gazebo_truth", "current_frontend_map"):
            m = dict(self.manifests["F1"], source_kind=source)
            with self.assertRaises(ContractError):
                FrozenMap.load(self.files["F1"], m)

    def test_source_and_wall_TTL_boundaries(self):
        q = self.query()
        r = self.offer(q, now_source_ns=q.source_ns + 300_000_000, now_wall_ns=q.received_wall_ns + 300_000_000)
        self.assertEqual(r["state"], "COLLECTING")
        for kwargs in ({"now_source_ns": q.source_ns + 300_000_001}, {"now_wall_ns": q.received_wall_ns + 300_000_001},
                       {"now_source_ns": q.source_ns - 1}, {"now_wall_ns": q.received_wall_ns - 1}):
            self.core = self.new_core()
            self.assertEqual(self.offer(q, **kwargs)["reason"], "stale_or_future_source_or_wall_receipt")

    def test_ready_proposal_expires_without_new_matching(self):
        r = self.ready(); generation = r["proposal"]["generation"]
        r = self.core.tick(now_source_ns=self.clock + 300_000_001, now_wall_ns=self.clock + 300_000_001)
        self.assertEqual(r["state"], "EXPIRED")
        self.assertIsNone(r["proposal"])
        with self.assertRaises(ContractError):
            self.core.select_shadow_reference(generation, now_source_ns=self.clock + 300_000_001,
                                               now_wall_ns=self.clock + 300_000_001)

    def test_clock_rollback_latches_fault(self):
        self.offer()
        r = self.offer(now_wall_ns=self.clock - 1)
        self.assertEqual(r["state"], "FAULT")
        self.clock += 1
        self.assertEqual(self.offer()["state"], "FAULT")

    def test_invalid_current_match_clears_pending_adjacent_history(self):
        self.select(self.ready())
        for i in range(3):
            self.clock += 100_000_000
            q = self.query(); self.offer(q)
            self.offer(q, self.candidate(q, "ramp12_transition"))
        self.assertEqual(self.core.status()["buffer_counts"]["ramp12_transition"], 3)
        self.clock += 100_000_000
        q = self.query(); e = evidence(); e["converged"] = False
        r = self.offer(q, self.candidate(q, evidence=e))
        self.assertEqual(r["buffer_counts"]["ramp12_transition"], 0)

    def test_rejected_target_clears_window_and_recovers_only_with_fresh_evidence(self):
        self.ready()
        self.clock += 100_000_000
        q = self.query(); e = evidence(); e["converged"] = False
        r = self.offer(q, self.candidate(q, evidence=e))
        self.assertIsNone(r["proposal"])
        self.assertEqual(r["buffer_counts"]["F1"], 0)
        r = self.ready()
        self.assertEqual(r["state"], "PROPOSAL_READY")

    def test_selection_requires_exact_fresh_generation(self):
        r = self.ready()
        with self.assertRaises(ContractError):
            self.core.select_shadow_reference(r["proposal"]["generation"] - 1,
                                               now_source_ns=self.clock, now_wall_ns=self.clock)
        self.select(r)
        with self.assertRaises(ContractError):
            self.select(r)

    def test_returned_proposal_cannot_mutate_pending_selection(self):
        r = self.ready(); generation = r["proposal"]["generation"]
        r["proposal"]["map_id"] = "F3"
        r["proposal"]["map_from_odom"][2][3] = 99.
        self.core.select_shadow_reference(generation, now_source_ns=self.clock, now_wall_ns=self.clock)
        self.assertEqual(self.core.active_shadow_map, "F1")
        np.testing.assert_allclose(self.core.last_selected["canonical_from_odom"], np.eye(4), atol=1e-14)

    def test_buffers_and_output_stay_bounded_without_point_clouds(self):
        for i in range(200):
            self.clock += 20_000_000
            self.offer()
        r = self.core.status()
        self.assertEqual(r["buffer_counts"]["F1"], 32)
        self.assertLess(len(json.dumps(r)), 16 * 1024)
        self.assertLess(r["verified_map_bytes"], 1024)
        q = self.query(ns=self.clock + 1, received_wall_ns=self.clock + 1)
        e = evidence(); e["point_cloud"] = [[1., 2., 3.]]
        self.assertEqual(self.offer(q, self.candidate(q, evidence=e))["reason"], "missing_or_unknown_matching_evidence_fields")

    def test_map_hash_source_time_calibration_and_freeze_time_are_required(self):
        changes = ({"content_sha256": SHA}, {"source_end_ns": 0}, {"source_start_ns": 101},
                   {"calibration_sha256": ""}, {"frozen_at_utc": "2026-10-07T09:00:00"},
                   {"geometry_reviewed": False}, {"independence_reviewed": None}, {"gravity_map": [0., 0., 0.]})
        for updates in changes:
            with self.subTest(updates=updates), self.assertRaises(ContractError):
                FrozenMap.load(self.files["F1"], dict(self.manifests["F1"], **updates))

    def test_frozen_map_mutation_invalidates_hash_verification(self):
        p = self.files["F1"]; p.write_bytes(p.read_bytes() + b"changed")
        with self.assertRaises(ContractError):
            FrozenMap.load(p, self.manifests["F1"])

    def test_map_budget_checked_before_full_read(self):
        with self.assertRaises(ContractError):
            FrozenMap.load(self.files["F1"], self.manifests["F1"], maximum_bytes=1)
        with self.assertRaises(ContractError):
            FrozenMap.load(self.files["F1"], self.manifests["F1"], maximum_bytes=MAX_MAP_BYTES + 1)

    def test_copy_or_manual_map_cannot_bypass_file_hash_check(self):
        forged = replace(self.maps[0], content_sha256=SHA)
        with self.assertRaises(ContractError):
            self.new_core(maps=[forged, *self.maps[1:]])

    def test_maps_captured_in_current_session_are_not_independent(self):
        m = dict(self.manifests["F1"], source_session_id="synthetic-runtime-session")
        current = FrozenMap.load(self.files["F1"], m)
        with self.assertRaises(ContractError):
            self.new_core(maps=[current, *self.maps[1:]])

    def test_overlap_binding_and_geometry_review_are_mandatory(self):
        with self.assertRaises(ContractError):
            self.new_core(overlaps=[replace(self.edges[0], left_map_sha256=SHA), *self.edges[1:]])
        with self.assertRaises(ContractError):
            replace(self.edges[0], independently_reviewed=False)
        with self.assertRaises(ContractError):
            self.new_core(overlaps=self.edges[:-1])

    def test_policy_cannot_remove_bounded_memory_or_temporal_checks(self):
        for updates in ({"maximum_frames": 65}, {"minimum_frames": 1}, {"ttl_ns": False},
                        {"minimum_unique_score_gap_fraction": 0.}, {"maximum_gap_ns": 400_000_000}):
            with self.subTest(updates=updates), self.assertRaises(ContractError):
                ShadowPolicy(**updates)

    def test_non_SE3_and_nonfinite_sensor_data_rejected_at_interface(self):
        bad = np.eye(4); bad[0, 0] = 2.
        with self.assertRaises(ContractError):
            self.query(transform=bad)
        with self.assertRaises(ContractError):
            self.query(gravity=[float("nan"), 0., -9.81])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--existing-source-probe", action="store_true",
                        help="Read only the five hash-bound historical small inputs; no ROS or truth")
    args = parser.parse_args()
    stream = io.StringIO()
    start = time.monotonic()
    result = unittest.TextTestRunner(stream=stream, verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(ShadowTests))
    report = {"schema": "submap_shadow_synthetic_cpu_test_receipt/v1", "passed": result.wasSuccessful(),
              "tests_run": result.testsRun, "failures": len(result.failures), "errors": len(result.errors),
              "duration_seconds": time.monotonic() - start, "maximum_rss_kib": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
              "synthetic_only": True, "actual_map_accuracy_verified": False, "actual_localization_verified": False,
              "matcher_implemented": False, "runtime_enabled": False, "control_allowed": False,
              "ros_started": False, "gpu_used": False, "point_clouds_collected": False,
              "source_hashes": {}, "output": stream.getvalue()}
    for name in ("submap_shadow.py", "submap_shadow_tests.py", "SUBMAP_SHADOW.md"):
        file = Path(__file__).resolve().with_name(name)
        if file.exists():
            report["source_hashes"][name] = hashlib.sha256(file.read_bytes()).hexdigest()
    if args.existing_source_probe:
        root = Path(__file__).resolve().parents[2]
        run = root / "runs/20261007_152015_closed_loop_cascade_clock_hold_V32_surface_baseline_r1_a21e"
        maprun = root.parent / "camera_mode/runs/20261002_194830_d16898"
        query = root / "test_results/map_surface_v32_20261007/navigation_failure/preserved_native_guards" / run.name
        paths = {"pcd": maprun / "colored_map.pcd", "map_metadata": maprun / "map_metadata.json",
                 "query": query / "005861_197080000000.npy", "archive": run / "navigation_cloud_xyz.jsonl",
                 "pose": run / "mission46_actual_slam_poses.jsonl"}
        hashes = {"pcd": "cb907d100020518b1b0727d0f5653cfb17a2d515e94971e5cc59d5ddd8a879c4",
                  "map_metadata": "c5591f573c0ee617d6a442c53b4fbf26d36a539cee33b59cdc5eb523bdb2aabc",
                  "query": "45bf7da7736abae8d8c1f7527ed669841b346964e3e82d1e593385280d9e8f83",
                  "archive": "e83fc124ca7add7037e182707a74d9f3be236d0c42a67b85a2d7def950bf5b46",
                  "pose": "c0bfd0bbf5661b0318ead1325d69758a4c1216b2588c39b954135a2d7855b7c4"}
        actual = probe_existing_sensor_sources(paths, hashes, source_ns=197080000000,
                                               seed_z_offsets=(-1., -.5, 0., .5, 1.))
        assert not any(actual[k] for k in ("control_allowed", "runtime_enabled", "search_complete", "shadow_candidate_produced"))
        assert actual["pcd_points"] == 235235 and actual["online_point_count_not_used"] == 252326
        assert actual["sample_local_roundtrip_max_m"] < 1e-12
        checks = []
        for name, altered_hashes, stamp, prior in (
                ("one_ns_pairing_rejected", hashes, 197080000001, None),
                ("wrong_query_hash_rejected", dict(hashes, query="0"*64), 197080000000, None),
                ("prior_information_rejected", hashes, 197080000000, np.eye(6)*1e12)):
            try:
                probe_existing_sensor_sources(paths, altered_hashes, source_ns=stamp,
                                              seed_z_offsets=(-.5, .5), prior_information=prior)
            except ContractError:
                checks.append(name)
            else:
                raise AssertionError(name)
        actual["source_contract_checks"] = checks
        report["existing_source_probe"] = actual
        report["synthetic_only"] = False
        # The 46 unit cases remain synthetic; this separate probe adds no
        # localization, wrong-floor, covariance, timing or control PASS.
        report["synthetic_unit_cases"] = result.testsRun
        report["synthetic_unit_duration_seconds"] = report["duration_seconds"]
        report["duration_seconds"] = time.monotonic() - start
        report["maximum_rss_kib"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    encoded = (json.dumps(report, ensure_ascii=False, indent=2) + "\n").encode()
    if len(encoded) > 1024 * 1024:
        raise RuntimeError("receipt exceeds 1 MiB budget")
    print(stream.getvalue(), end="")
    print(json.dumps({k: v for k, v in report.items() if k not in ("source_hashes", "output")}, ensure_ascii=False))
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        with (args.output / "TEST_RECEIPT.json").open("xb") as output:
            output.write(encoded)
    raise SystemExit(0 if result.wasSuccessful() else 1)


if __name__ == "__main__":
    main()
