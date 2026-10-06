"""Finite V25 full46 source/launch tests; not physical mission acceptance."""
from pathlib import Path
import ast
import copy
import hashlib
import json
import tempfile
import unittest
from unittest.mock import patch

import corridor_preflight
import full46_launch_contract as contract
import geometry_archive
import mission46_profile
import pid_scope
import run
import scan_workspace
import slam_workspace
from mission.route_regions import transformed_route_goals

HERE = Path(__file__).resolve().parent
OLD = HERE.parent/'corridor_tracking_v22_curvature_archive'


class Full46LaunchChecks(unittest.TestCase):
    def setUp(self):
        self.profile = json.loads((HERE/'profiles'/contract.PROFILE_NAME).read_text())

    def test_exact_full_baseline_with_only_declared_candidate_and_curvature_changes(self):
        self.assertEqual(self.profile, contract.expected_profile(HERE))
        contract.validate_full46(self.profile, HERE)
        base = json.loads(contract.baseline_profile_path(HERE).read_text())
        for key in ('original_scenario', 'spawn', 'duration_s', 'teacher_transition', 'control_clock_contract',
                'registered_route_fence', 'warmup_history', 'raw_storage_budget', 'mission46_runtime_interfaces'):
            self.assertEqual(self.profile[key], base[key], key)

    def test_all_original_46_regions_remain_exact_under_three_heading_registrations(self):
        base = json.loads(contract.baseline_profile_path(HERE).read_text())
        for origin, yaw in (([0., 0., .3], 0.), ([.01, -.02, .31], .4), ([2., -1., .3], -.8)):
            heading = dict(yaw_camera_init_from_world=yaw)
            total = 0
            for phase, count in (('exploration', 18), ('return_origin', 14), ('navigation_f1_f3', 14)):
                got = [g.definition() for g in transformed_route_goals(self.profile['original_scenario'], phase, origin, heading)]
                expected = [g.definition() for g in transformed_route_goals(base['original_scenario'], phase, origin, heading)]
                self.assertEqual(got, expected); self.assertEqual(len(got), count)
                self.assertTrue(all(g['arrival']['dwell_sim_s'] == .4 and g['timeout_sim_s'] == 90. for g in got))
                total += len(got)
            self.assertEqual(total, 46)

    def test_full_contract_rejects_prefix9_and_protection_or_geometry_mutations(self):
        prefix = json.loads((OLD/'profiles/curvature_original46_prefix9_on.json').read_text())
        with self.assertRaises(ValueError): contract.validate_full46(prefix, HERE)
        for field, value in (('duration_s', 600), ('mission46_required', False), ('expected_region_count', 9),
                ('pose_cloud_timeout_s', .6), ('requires_current_run_rgb_save', False),
                ('requires_original_dynamic_obstacle', False), ('requires_bidirectional_layer_switches', False),
                ('mission46_final_hold_sim_s', 0), ('navigation_ground_truth_used', True)):
            changed = copy.deepcopy(self.profile); changed[field] = value
            with self.subTest(field=field), self.assertRaises(ValueError): contract.validate_full46(changed, HERE)
        changed = copy.deepcopy(self.profile); changed['registered_route_fence']['horizontal_radius_m'] = .6
        with self.assertRaises(ValueError): contract.validate_full46(changed, HERE)
        changed = copy.deepcopy(self.profile); changed['original_scenario']['route_goals']['exploration'][9]['timeout_sim_s'] = 120.
        with self.assertRaises(ValueError): contract.validate_full46(changed, HERE)

    def test_controller_curvature_math_and_runtime_safety_sources_identical_to_V22(self):
        for name in ('controller.py', 'cascade_core.py', 'spatial_reference.py', 'worker.py', 'bridge.py',
                'clock_hold.py', 'publication_ledger.py', 'pipeline_lifecycle.py', 'mission46.py',
                'mission46_runtime.py', 'mission46_runtime_evidence.py', 'mission46_obstacle_runtime.py',
                'mission46_terrain.py', 'mission46_guard.py', 'route_fence.py', 'geometry_archive.py',
                'route.py', 'prefix_contract.py', 'mission46_profile.py'):
            self.assertEqual((HERE/name).read_bytes(), (OLD/name).read_bytes(), name)
        on = json.loads((OLD/'profiles/curvature_original46_prefix9_on.json').read_text())
        self.assertEqual(self.profile['cascade'], on['cascade'])

    def test_protected_baseline_SCAN_and_V19_SLAM_are_explicitly_inherited(self):
        workspace, _, _ = scan_workspace.scan_contract(self.profile)
        self.assertEqual(workspace, (HERE.parents[2]/'navigation/ros2_ws').resolve())
        self.assertEqual(slam_workspace.SLAM_WORKSPACE.resolve(), (HERE.parent/'pipeline_v19/slam_ws').resolve())
        slam_workspace.verify_slam_workspace()
        self.assertFalse((HERE/'slam_ws').exists()); self.assertFalse((HERE/'scan_ws').exists())

    def test_five_geometry_sources_still_mandatory_runtime_inputs(self):
        with patch('corridor_preflight.evidence_files', return_value=[]):
            actual = {p.resolve() for p in pid_scope.runtime_files(self.profile)}
        self.assertTrue({p.resolve() for p in geometry_archive.geometry_helper_files(HERE)}.issubset(actual))

    def test_run_uses_full_validator_not_unconditional_prefix_validation(self):
        source = ast.unparse(ast.parse((HERE/'run.py').read_text()))
        self.assertIn('validate_full46(p, HERE)', source)
        self.assertNotIn('validate_prefix(p)', source)
        self.assertIn('13 if p.get', source)
        source = (HERE/'stack.launch.py').read_text()
        self.assertIn("mission46_runtime.py' if profile.get('mission46_required')", source)
        self.assertIn("here/'mission46_obstacle_runtime.py'", source)

    def test_sources_receipts_RGB_dynamic_terrain_and_hold_required(self):
        for name in ('requires_original_origin_initialization_receipt', 'requires_current_run_rgb_save',
                'requires_original_dynamic_obstacle', 'requires_bidirectional_layer_switches'):
            self.assertIs(self.profile[name], True)
        self.assertEqual(self.profile['mission46_required_source_files'],
            [str(p.resolve()) for p in mission46_profile.required_source_files()])
        self.assertEqual(self.profile['mission46_final_hold_sim_s'], 5.)
        self.assertEqual(self.profile['pose_cloud_timeout_s'], .3)
        self.assertEqual(self.profile['registered_route_fence']['horizontal_radius_m'], .45)

    def test_gate_cannot_reuse_V22_prefix_authorization(self):
        with tempfile.TemporaryDirectory() as t:
            here = Path(t)
            (here/'TERRAIN_EVENT_V25_PREFLIGHT.json').write_bytes((OLD/'CURVATURE_V22_PREFLIGHT.json').read_bytes())
            with self.assertRaises(RuntimeError): corridor_preflight.verify_preflight(here, self.profile)

    def test_gate_has_exact_full_profile_allowlist_and_no_navigation_outcome_claim(self):
        source = (HERE/'corridor_preflight.py').read_text()
        self.assertIn("allowed != ['curvature_original46_on.json']", source)
        self.assertIn('profile != expected', source)
        self.assertIn("d.get('actual_navigation_verified') is not False", source)
        self.assertIn('validate_full46(profile,here)', source)
        self.assertIn('*required_source_files()', source)
        self.assertIn('terrain_event_v25_full46_launch_review/v1', source)

    def test_same_physical_loaded_binary_checks_run_before_Gazebo(self):
        tree = ast.parse((HERE/'run.py').read_text())
        execute = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'execute')
        text = ast.unparse(execute)
        self.assertLess(text.index('verify_loaded_slam('), text.index("start('gazebo'"))
        self.assertLess(text.index('verify_loaded_scan('), text.index("start('gazebo'"))


if __name__ == '__main__': unittest.main()
