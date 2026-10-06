#!/usr/bin/env python3
"""Mission acceptance rejects stale or geometrically false region claims."""
import copy
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mission.route_regions import add_route_regions, validate_route_regions
from mission.state_machine import Mission
from navigation.goal_regions import contains, parse_goal


class MissionRegionTests(unittest.TestCase):
    def setUp(self):
        self.scenario = add_route_regions({
            'exploration': [[0, -1, 0], [5, 2, .3]],
            'return_origin': [[0, 0, 0]], 'navigation_f1_f3': [[0, 7, 2.4]],
            'initial_stable_s': 1., 'waypoint_reference': 'relative_world_axes'})
        self.m = Mission()
        self.m.start('region-test', self.scenario)
        self.m.heading_alignment = {'yaw_camera_init_from_world': -math.pi/2}
        self.m.tick(1, True, [1, 2, .1], 100)
        self.actions = self.m.tick(2.1, True, [1, 2, .1], 100)

    def status(self):
        receipts = []
        for i, raw in enumerate(self.m.current_goals):
            receipts.append(dict(request_id=self.m.current_request, goal_id=raw['goal_id'],
                goals_definition_sha256=self.m.current_goals_sha256,
                stamp_ns=(i+1)*1_000_000_000, start_stamp_ns=(i+1)*1_000_000_000-400_000_000,
                dwell_ns=400_000_000, raw_position=list(raw['center']), region_inside=True,
                protected=False, arrival_definition=raw['arrival'], reason='arrived',
                control_region_inside=True,
                control_arrival_definition=parse_goal(raw).control_arrival_definition(),
                max_observation_gap_ns=200_000_000))
        return dict(request_id=self.m.current_request, state='succeeded',
            goals_definition_sha256=self.m.current_goals_sha256, region_arrivals=receipts,
            goals_definitions=copy.deepcopy(self.m.current_goals),
            waypoint_index=len(receipts), total=len(receipts), dynamic_obstacle_verified=True)

    def complete(self, status=None, pose=None):
        return self.m.navigation_result(status or self.status(), pose or self.m.goal, 10.)

    def test_request_rotates_region_axes_and_centers_together(self):
        action = self.actions[0]
        self.assertEqual(action['schema_version'], 2)
        self.assertNotIn('waypoints', action)
        g = parse_goal(action['goals'][1])
        self.assertTrue(contains(g, [3, -3, .4]))
        self.assertTrue(contains(g, [3, -3.3, .43]))
        self.assertFalse(contains(g, [3.31, -3, .4]))
        self.assertFalse(contains(g, [3, -3, .6]))

    def test_complete_requires_all_ordered_measured_receipts(self):
        self.complete()
        self.assertEqual(self.m.stage, 'returning')

    def test_reject_modified_geometry_hash(self):
        status = self.status();status['goals_definition_sha256'] = 'altered'
        self.complete(status)
        self.assertEqual(self.m.stage, 'failed')

    def test_reject_protected_receipt(self):
        status = self.status();status['region_arrivals'][0]['protected'] = True
        self.complete(status)
        self.assertEqual(self.m.stage, 'failed')

    def test_reject_short_dwell(self):
        status = self.status();status['region_arrivals'][0]['start_stamp_ns'] += 1
        status['region_arrivals'][0]['dwell_ns'] -= 1
        self.complete(status)
        self.assertEqual(self.m.stage, 'failed')

    def test_reject_reordered_or_omitted_receipts(self):
        for mode in ('reversed', 'omitted'):
            with self.subTest(mode=mode):
                status = self.status()
                status['region_arrivals'] = list(reversed(status['region_arrivals'])) if mode == 'reversed' else status['region_arrivals'][:-1]
                self.assertFalse(self.m.region_completion_valid(status, self.m.goal))

    def test_reject_receipt_raw_pose_outside(self):
        status = self.status();status['region_arrivals'][0]['raw_position'][2] += 1.2
        self.complete(status)
        self.assertEqual(self.m.stage, 'failed')

    def test_reject_final_wrong_floor(self):
        self.complete(pose=[*self.m.goal[:2], self.m.goal[2]+1.2])
        self.assertEqual(self.m.stage, 'failed')

    def test_return_origin_preserves_precision(self):
        self.complete()
        self.assertEqual(self.m.goal_region['arrival']['radius_m'], .22)
        pose = [self.m.origin[0]+.25, *self.m.origin[1:]]
        self.complete(pose=pose)
        self.assertEqual(self.m.stage, 'failed')

    def test_configuration_center_mismatch_rejected_before_start(self):
        bad = copy.deepcopy(self.scenario)
        bad['route_goals']['exploration'][0]['center'][0] += .1
        with self.assertRaises(ValueError):validate_route_regions(bad)
        with self.assertRaises(ValueError):Mission().start('bad', bad)

    def test_configuration_requires_every_stage(self):
        bad = copy.deepcopy(self.scenario);bad['route_goals'].pop('return_origin')
        with self.assertRaises(ValueError):validate_route_regions(bad)

    def test_profile_cannot_hide_larger_arrival_volume(self):
        bad = copy.deepcopy(self.scenario)
        bad['route_goals']['exploration'][0]['arrival']['radius_m'] = 10.
        with self.assertRaises(ValueError):validate_route_regions(bad)

    def test_configuration_cannot_relax_dwell_or_timeout(self):
        for field in ('dwell', 'timeout'):
            with self.subTest(field=field):
                bad = copy.deepcopy(self.scenario)
                raw = bad['route_goals']['exploration'][0]
                if field == 'dwell':raw['arrival']['dwell_sim_s'] = .1
                else:raw['timeout_sim_s'] = 900.
                with self.assertRaises(ValueError):validate_route_regions(bad)

    def test_outer_inside_is_insufficient_for_control_arrival(self):
        status = self.status()
        g = parse_goal(self.m.current_goals[0])
        status['region_arrivals'][0]['raw_position'] = [g.center[0]+.30, *g.center[1:]]
        self.assertTrue(contains(g, status['region_arrivals'][0]['raw_position']))
        self.assertFalse(self.m.region_completion_valid(status, self.m.goal))

    def test_control_claim_requires_matching_declared_band(self):
        for field, value in [('control_region_inside', False), ('control_arrival_definition', {})]:
            with self.subTest(field=field):
                status = self.status();status['region_arrivals'][0][field] = value
                self.assertFalse(self.m.region_completion_valid(status, self.m.goal))

    def test_v1_profile_and_receipts_still_supported(self):
        old = add_route_regions(self.scenario, profile='three_platform_body_arrival_v1')
        validate_route_regions(old)
        self.m.scenario = old
        self.m.request_route('exploration', 0.)
        status = self.status()
        for receipt in status['region_arrivals']:
            receipt.pop('control_region_inside');receipt.pop('control_arrival_definition')
        self.assertTrue(self.m.region_completion_valid(status, self.m.goal))

    def test_control_band_cannot_be_silently_expanded(self):
        bad = copy.deepcopy(self.scenario)
        bad['route_goals']['exploration'][0]['arrival']['control_band']['radius_m'] = .34
        with self.assertRaises(ValueError):validate_route_regions(bad)


if __name__ == '__main__':
    unittest.main(verbosity=2)
