#!/usr/bin/env python3
"""Display-integrity fixtures only; no actual navigation certification or ROS."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

HERE = Path(__file__).resolve().parent
TEACHER = HERE.parents[2]
SERVE = TEACHER / 'scripts/serve.py'
spec = importlib.util.spec_from_file_location('teacher_viewer_receipt_fixture', SERVE)
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)
CRITERION_BYTES = (TEACHER / 'test_results/lidar_density_rate_20261005/evaluation/clock_hold_v9/PROSPECTIVE_CRITERIA.json').read_bytes()


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(value, indent=2) + '\n')


class ReceiptIntegrity(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='go2_viewer_receipt_fixture_')
        self.root = Path(self.temp.name)
        self.old_root = viewer.ROOT
        viewer.ROOT = self.root
        self.run = self.root / 'runs/synthetic_display_integrity_only'
        self.run.mkdir(parents=True)
        authority = self.root / 'test_results/lidar_density_rate_20261005/evaluation/clock_hold_v9/PROSPECTIVE_CRITERIA.json'
        authority.parent.mkdir(parents=True)
        authority.write_bytes(CRITERION_BYTES)
        self.criterion = json.loads(CRITERION_BYTES)
        original = self.root / 'navigation/lidar_sampling_v9/PROSPECTIVE_CLOCK_HOLD_CRITERIA.json'
        original.parent.mkdir(parents=True)
        original.write_bytes(CRITERION_BYTES)
        self.snapshot = self.run / 'sources/navigation/lidar_sampling_v9/PROSPECTIVE_CLOCK_HOLD_CRITERIA.json'
        self.snapshot.parent.mkdir(parents=True)
        self.snapshot.write_bytes(CRITERION_BYTES)
        self.snapshots = {str(original): {'snapshot': str(self.snapshot), 'sha256': digest(self.snapshot)}}
        profile = {'control_clock_contract': self.criterion['frozen_control_clock_contract']}
        write(self.run / 'navigation_scope.json', {
            'schema': 'teacher_closed_loop_navigation_scope/v1', 'run_dir': str(self.run),
            'navigation_ground_truth_used': False, 'profile': profile,
            'references': {str(original): digest(original), str(self.snapshot): digest(self.snapshot)}})
        write(self.run / 'source_manifest.json', {str(self.snapshot): digest(self.snapshot)})
        write(self.run / 'navigation_source_snapshots.json', self.snapshots)
        write(self.run / 'navigation_profile.json', profile)
        write(self.run / 'navigation_request.json', {'goals': [{'goal_id': 'goal_' + str(i)} for i in range(32)]})
        write(self.run / 'runtime_manifest.json', {'synthetic_display_fixture_only': True})
        write(self.run / 'slam_loaded_binary.json', {'synthetic_display_fixture_only': True})
        checks = {name: {'status': 'passed', 'passed': True} for name in viewer.CLOCK_HOLD_REQUIRED_COMMON_CHECKS}
        checks['execution_phase_status_and_cleanup_boundary']['execution_states'] = ['running', 'succeeded']
        checks['all_original_SLAM_3D_region_arrivals'].update(actual_last_state='succeeded', arrivals=[
            {'goal_id': 'goal_' + str(i), 'status': 'passed', 'passed': True} for i in range(32)])
        names = ['navigation_scope.json', 'source_manifest.json', 'navigation_source_snapshots.json',
                 'navigation_profile.json', 'navigation_request.json']
        self.common = {'schema': 'independent_actual_SLAM_SCAN_cascade_navigation/v1',
            'run': str(self.run), 'status': 'passed', 'checks': checks,
            'scope': {'simulation_only': True, 'navigation_ground_truth_used': False},
            'criteria': self.criterion['unchanged_common_numerical_criteria'],
            'evaluator_sha256': self.criterion['unchanged_common_evaluator_sha256'],
            'verified_input_source_sha256': {str(self.run / name): digest(self.run / name) for name in names}}
        self.write_common()
        self.receipt = self.new_receipt()
        self.write_receipt()

    def tearDown(self):
        viewer.ROOT = self.old_root
        self.temp.cleanup()

    def write_common(self):
        write(self.run / 'summary_closed_loop_cascade_independent.json', self.common)

    def new_receipt(self):
        raw = copy.deepcopy(self.common)
        raw.update(schema=viewer.CLOCK_HOLD_RECEIPT_SCHEMA,
            criteria_sha256=viewer.CLOCK_HOLD_CRITERIA_SHA256,
            source_bindings_verified=True, unchanged_common_checks_verified=True,
            raw_common_receipt_sha256=digest(self.run / 'summary_closed_loop_cascade_independent.json'),
            raw_common_status=self.common['status'], explicit_replaced_common_checks=[viewer.CLOCK_HOLD_REPLACED_CHECK],
            criteria_binding_source=str(self.snapshot), source_bindings={name: digest(self.run / name) for name in (
                'navigation_scope.json', 'source_manifest.json', 'navigation_source_snapshots.json',
                'navigation_profile.json', 'runtime_manifest.json', 'slam_loaded_binary.json')})
        raw['checks'].update({name: {'status': 'passed', 'passed': True} for name in viewer.CLOCK_HOLD_ADDITIONAL_CHECKS})
        raw['checks']['prospective_clock_hold_criteria_and_archived_sources'].update(
            criterion_snapshot=str(self.snapshot), criteria_sha256=viewer.CLOCK_HOLD_CRITERIA_SHA256,
            common_evaluator_sha256=self.criterion['unchanged_common_evaluator_sha256'],
            source_mismatches=[], verified_archived_sources=copy.deepcopy(self.snapshots))
        return raw

    def write_receipt(self):
        write(self.run / 'summary_closed_loop_clock_hold_independent.json', self.receipt)

    def selected(self):
        return viewer.closed_loop_receipt_view(self.run)

    def rejected(self):
        self.write_receipt()
        result = self.selected()
        self.assertEqual(result['status'], 'unverified')
        self.assertFalse(result['clock_hold']['valid_for_selected_run'])
        self.assertTrue(result['clock_hold']['validation_errors'])
        self.assertIn('selected_run_receipt_integrity', result['selected_validation_summary']['checks'])
        return result

    def test_complete_consistent_synthetic_receipt_selects_new_and_keeps_common(self):
        result = self.selected()
        self.assertEqual(result['status'], 'passed')
        self.assertEqual(result['selection_kind'], 'clock_hold_independent')
        self.assertEqual(result['counts']['total'], 25)
        self.assertEqual(result['common']['raw'], self.common)
        self.assertEqual(result['clock_hold']['validation_errors'], [])

    def test_foreign_run_receipt_rejected(self):
        self.receipt['run'] = str(self.root / 'runs/foreign')
        self.rejected()

    def test_missing_additional_check_cannot_claim_pass(self):
        self.receipt['checks'].pop('actual_publication_slew_with_hold_chronology')
        self.rejected()

    def test_nonpassing_additional_check_cannot_claim_pass(self):
        self.receipt['checks']['actual_clock_hold_nonzero_resume_requires_native_geometry'] = {'status': 'failed', 'passed': False}
        self.rejected()

    def test_source_binding_false_cannot_claim_pass(self):
        self.receipt['source_bindings_verified'] = False
        self.rejected()

    def test_source_binding_digest_rejected(self):
        self.receipt['source_bindings']['slam_loaded_binary.json'] = '0' * 64
        self.rejected()

    def test_actual_source_file_changed_rejected(self):
        write(self.run / 'runtime_manifest.json', {'changed': True})
        self.rejected()

    def test_malformed_source_profile_is_rejected_without_display_exception(self):
        scope = json.loads((self.run / 'navigation_scope.json').read_text())
        scope['profile'] = 'invalid-source-contract'
        write(self.run / 'navigation_scope.json', scope)
        self.rejected()

    def test_criterion_hash_or_foreign_snapshot_rejected(self):
        self.receipt['criteria_sha256'] = '0' * 64
        self.rejected()
        self.receipt['criteria_sha256'] = viewer.CLOCK_HOLD_CRITERIA_SHA256
        self.receipt['criteria_binding_source'] = str(self.root / 'foreign_criterion.json')
        self.rejected()

    def test_original_common_digest_rejected(self):
        self.receipt['raw_common_receipt_sha256'] = '0' * 64
        self.rejected()

    def test_unchanged_common_payload_and_criteria_required(self):
        self.receipt['checks']['native_physical_safety']['reason'] = 'changed'
        self.rejected()
        self.receipt = self.new_receipt()
        self.receipt['criteria']['source_ttl_sim_s'] = 1.0
        self.rejected()

    def test_prefix_21_of_32_cannot_claim_full_pass_even_in_both_receipts(self):
        self.common['checks']['all_original_SLAM_3D_region_arrivals']['arrivals'] = self.common['checks']['all_original_SLAM_3D_region_arrivals']['arrivals'][:21]
        self.write_common()
        self.receipt = self.new_receipt()
        result = self.rejected()
        self.assertTrue(any('partial region prefix' in x for x in result['clock_hold']['validation_errors']))

    def test_failed_phase_cannot_be_green_even_if_result_flags_forged(self):
        self.common['checks']['execution_phase_status_and_cleanup_boundary']['execution_states'] = ['running', 'failed']
        self.write_common()
        self.receipt = self.new_receipt()
        self.rejected()

    def test_authentic_failed_partial_route_remains_failed_and_parking_unverified(self):
        self.common['status'] = 'failed'
        phase = self.common['checks']['execution_phase_status_and_cleanup_boundary']
        phase.update(status='failed', passed=False, execution_states=['running', 'failed'])
        region = self.common['checks']['all_original_SLAM_3D_region_arrivals']
        region.update(status='failed', passed=False, actual_last_state='failed')
        for row in region['arrivals'][21:]:
            row.update(status='failed', passed=False)
        self.common['checks']['new_first_declared_active_hold_fixed_5s'] = {'status': 'unverified', 'passed': None}
        self.write_common()
        self.receipt = self.new_receipt()
        self.write_receipt()
        result = self.selected()
        self.assertEqual(result['status'], 'failed')
        self.assertTrue(result['clock_hold']['valid_for_selected_run'])
        self.assertEqual(result['selected_validation_summary']['checks']['new_first_declared_active_hold_fixed_5s']['status'], 'unverified')
        self.assertEqual(result['counts']['failed'], 2)

    def test_pending_or_incomplete_new_receipt_cannot_fall_back_to_old_pass(self):
        path = self.run / 'summary_closed_loop_clock_hold_independent.json'
        path.unlink()
        result = self.selected()
        self.assertEqual(result['status'], 'unverified')
        self.assertEqual(result['selection_kind'], 'clock_hold_pending')
        self.assertEqual(result['common']['status'], 'passed')
        path.write_text('{"schema":')
        self.assertEqual(self.selected()['status'], 'unverified')

    def test_historical_no_clock_contract_keeps_old_selection(self):
        (self.run / 'summary_closed_loop_clock_hold_independent.json').unlink()
        write(self.run / 'navigation_scope.json', {'schema': 'teacher_closed_loop_navigation_scope/v1', 'profile': {}})
        self.assertEqual(self.selected()['selection_kind'], 'cascade_independent')
        self.assertEqual(self.selected()['status'], 'passed')


if __name__ == '__main__':
    unittest.main(verbosity=2)
