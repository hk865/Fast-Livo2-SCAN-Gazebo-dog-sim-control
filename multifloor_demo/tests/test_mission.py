#!/usr/bin/env python3
import sys
from pathlib import Path
import unittest
import math
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from mission.state_machine import Mission, ObstacleEvidence


class ObstacleEvidenceTests(unittest.TestCase):
    def test_resume_after_zero_commands_while_replanning(self):
        evidence = ObstacleEvidence()
        obstacle = {'active': True, 'visible_phase': 'blocking', 'gazebo_updates': 10}
        evidence.observe(True, 0., obstacle)
        evidence.observe(False, 0., obstacle)
        evidence.observe(False, 0., obstacle)
        self.assertFalse(evidence.verified)
        evidence.observe(False, .08, obstacle)
        self.assertTrue(evidence.verified)
        self.assertEqual(evidence.hold_events, 1)

    def test_static_hold_is_not_dynamic_obstacle_evidence(self):
        evidence = ObstacleEvidence()
        evidence.observe(True, 0., {'active': False, 'visible_phase': 'parked', 'gazebo_updates': 100})
        evidence.observe(False, .08, {})
        self.assertFalse(evidence.verified)


class MissionTests(unittest.TestCase):
    def test_world_route_waits_for_heading_and_rotates_offsets(self):
        m = Mission()
        config = dict(self.config, waypoint_reference='relative_world_axes')
        m.start('calibrated', config)
        m.tick(0, True, [1, 2, .1], 1000)
        self.assertEqual(m.tick(4, True, [1, 2, .1], 1000), [])
        self.assertEqual(m.stage, 'waiting_sensors')
        m.heading_alignment = {'yaw_camera_init_from_world': -math.pi/2}
        m.tick(5, True, [1, 2, .1], 1000)
        actions = m.tick(9, True, [1, 2, .1], 1000)
        point = actions[0]['waypoints'][0]
        for actual, expected in zip(point, [1, 0, 1.1]):
            self.assertAlmostEqual(actual, expected)

    def setUp(self):
        self.m = Mission()
        self.config = {'exploration': [[2, 0, 1]], 'return_origin': [[0, 0, 0]],
                       'navigation_f1_f3': [[3, 0, 2]], 'initial_stable_s': 1.}
        self.m.start('run-a', self.config)
        self.m.tick(1, True, [0, 0, 0], 100)
        self.m.tick(2.1, True, [0, 0, 0], 100)

    def done(self, pose, t):
        return self.m.navigation_result({'request_id': self.m.current_request, 'state': 'succeeded', 'dynamic_obstacle_verified': True}, pose, t)

    def test_full_sequence_requires_return_and_save(self):
        self.assertEqual(self.m.stage, 'exploring')
        self.done([2, 0, 1], 10)
        self.assertEqual(self.m.stage, 'returning')
        self.assertEqual(self.done([0, 0, 0], 20), [{'kind': 'save_map'}])
        self.assertEqual(self.m.stage, 'saving_map')
        events = self.m.map_saved(True, 1000, 21)
        self.assertEqual(self.m.stage, 'navigating')
        self.assertTrue(events[0]['enabled'])
        self.done([3, 0, 2], 30)
        self.assertEqual(self.m.stage, 'completed')

    def test_stale_completion_cannot_advance(self):
        self.m.navigation_result({'request_id': 'old-run', 'state': 'succeeded'}, [2, 0, 1], 5)
        self.assertEqual(self.m.stage, 'exploring')

    def test_false_navigation_success_fails(self):
        self.done([0, 0, 0], 8)
        self.assertEqual(self.m.stage, 'failed')

    def test_failed_map_does_not_start_cross_floor(self):
        self.done([2, 0, 1], 10)
        self.done([0, 0, 0], 20)
        self.m.map_saved(False, 0, 21, 'no RGB points')
        self.assertEqual(self.m.stage, 'failed')

    def test_sensor_dropout_aborts(self):
        self.m.tick(5, False, [0, 0, 0], 100)
        actions = self.m.tick(9, False, [0, 0, 0], 100)
        self.assertEqual(self.m.stage, 'failed')
        self.assertEqual(actions, [{'kind': 'stop_navigation'}])

    def test_cancel_prevents_late_success(self):
        old = self.m.current_request
        self.m.stop(4)
        self.m.navigation_result({'request_id': old, 'state': 'succeeded'}, [2, 0, 1], 5)
        self.assertEqual(self.m.stage, 'stopped')

    def test_cross_floor_needs_dynamic_response_evidence(self):
        self.done([2, 0, 1], 10)
        self.done([0, 0, 0], 20)
        self.m.map_saved(True, 1000, 21)
        self.m.navigation_result({'request_id': self.m.current_request, 'state': 'succeeded'}, [3, 0, 2], 30)
        self.assertEqual(self.m.stage, 'failed')


if __name__ == '__main__':
    unittest.main(verbosity=2)
