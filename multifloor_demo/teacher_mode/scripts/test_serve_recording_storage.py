"""Read-only external recording boundaries and bounded V20 display regressions."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('teacher_dashboard_storage_test', HERE / 'serve.py')
serve = importlib.util.module_from_spec(spec)
spec.loader.exec_module(serve)


class ExternalRecordingTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.root = self.base / 'teacher'
        self.runs = self.root / 'runs'
        self.runs.mkdir(parents=True)
        self.storage = self.base / 'pipeline'
        self.storage.mkdir(mode=0o700)
        self.name = '20261006_130405_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r1_39ea'
        self.run = self.storage / self.name
        self.run.mkdir()
        self.alias = self.runs / self.name
        self.alias.symlink_to(self.run, target_is_directory=True)
        self.patch_root = mock.patch.object(serve, 'ROOT', self.root)
        self.patch_storage = mock.patch.object(serve, 'PIPELINE_RUN_STORAGE', self.storage)
        self.patch_root.start()
        self.patch_storage.start()

    def tearDown(self):
        self.patch_storage.stop()
        self.patch_root.stop()
        self.temp.cleanup()

    def test_explicit_storage_alias_and_direct_read_are_accepted(self):
        for candidate in (self.alias, self.run, self.alias / 'state.json', self.run / 'state.json'):
            self.assertEqual(serve.recording_path(candidate, self.runs), candidate.resolve())
            self.assertTrue(serve.recording_read_allowed(candidate))
        (self.runs / 'latest').symlink_to(self.run, target_is_directory=True)
        self.assertEqual(serve.Dashboard(self.runs).resolve('latest'), self.run)

    def test_allowlist_is_one_exact_production_root(self):
        # Patching the new constant does not turn its parent into a permitted root.
        sibling = self.base / 'arbitrary' / self.name
        sibling.mkdir(parents=True)
        self.alias.unlink()
        self.alias.symlink_to(sibling, target_is_directory=True)
        self.assertIsNone(serve.recording_path(self.alias, self.runs))
        self.assertIsNone(serve.Dashboard(self.runs).resolve(self.name))

    def test_alias_required_and_alternate_name_rejected(self):
        alternate = self.runs / 'another_name'
        alternate.symlink_to(self.run, target_is_directory=True)
        self.assertIsNone(serve.recording_path(alternate, self.runs))
        self.alias.unlink()
        self.assertIsNone(serve.recording_path(self.run, self.runs))

    def test_root_must_have_exact_mode_and_owner(self):
        for mode in (0o755, 0o750, 0o770):
            self.storage.chmod(mode)
            self.assertIsNone(serve.recording_path(self.alias, self.runs))
        self.storage.chmod(0o700)
        with mock.patch.object(serve.os, 'getuid', return_value=os.getuid()+1):
            self.assertIsNone(serve.recording_path(self.alias, self.runs))

    def test_symlinked_root_and_child_are_rejected(self):
        moved = self.base / 'moved'
        self.storage.rename(moved)
        self.storage.symlink_to(moved, target_is_directory=True)
        self.assertIsNone(serve.recording_path(self.alias, self.runs))
        self.storage.unlink()
        moved.rename(self.storage)
        child_target = self.storage / (self.name + 'other')
        self.run.rename(child_target)
        self.run.symlink_to(child_target, target_is_directory=True)
        self.assertIsNone(serve.recording_path(self.alias, self.runs))

    def test_unknown_run_name_and_nested_escape_are_rejected(self):
        invalid = self.storage / 'not_a_run'
        invalid.mkdir()
        (self.runs / invalid.name).symlink_to(invalid, target_is_directory=True)
        self.assertIsNone(serve.recording_path(invalid, self.runs))
        foreign = self.base / 'secret.json'
        foreign.write_text('{}\n')
        (self.run / 'state.json').symlink_to(foreign)
        self.assertFalse(serve.recording_read_allowed(self.run / 'state.json'))
        self.assertFalse(serve.recording_read_allowed(self.alias / 'state.json'))
        self.assertEqual(serve.read_json(self.alias / 'state.json'), {})

    def test_historical_in_tree_recordings_remain_accepted(self):
        old = self.runs / 'historic_test'
        old.mkdir()
        self.assertEqual(serve.recording_path(old, self.runs), old)

    def shadow_record(self):
        (self.run / 'navigation_scope.json').write_text(json.dumps(dict(profile=dict(
            controller_selector='corridor_tracking_v20'))))
        return dict(schema='teacher_corridor_certificate/v1', status='blocked',
            reason='unknown_body_sweep', shadow_only=True, control_authority=False,
            navigation_ground_truth_used=False, binding=dict(path_id=self.name+':request:1'))

    def test_shadow_bounded_tail_ignores_partial_record_and_never_certifies(self):
        row = self.shadow_record()
        target = self.run / 'corridor_certificates.jsonl'
        target.write_text(('x'*(256 << 10))+'\n'+json.dumps(row)+'\n{"schema":')
        result = serve.corridor_shadow_view(self.run)
        self.assertTrue(result['record_valid_for_selected_run'])
        self.assertEqual(result['raw'], row)
        self.assertLessEqual(result['display_tail_bytes'], 256 << 10)
        self.assertFalse(result['independent_acceptance'])
        self.assertFalse(result['control_authority'])

    def test_shadow_rejects_foreign_truth_or_authoritative_record(self):
        row = self.shadow_record()
        for changes in (dict(binding=dict(path_id='foreign:1')),
                        dict(navigation_ground_truth_used=True),
                        dict(control_authority=True), dict(shadow_only=False)):
            with self.subTest(changes=changes):
                (self.run / 'corridor_certificates.jsonl').write_text(json.dumps({**row, **changes})+'\n')
                self.assertFalse(serve.corridor_shadow_view(self.run)['record_valid_for_selected_run'])

    def test_shadow_does_not_appear_on_v19_or_without_v20_profile(self):
        self.assertEqual(serve.corridor_shadow_view(self.run), {})
        (self.run / 'navigation_scope.json').write_text(json.dumps(dict(profile=dict(
            controller_selector='pipeline_v19'))))
        self.assertEqual(serve.corridor_shadow_view(self.run), {})


if __name__ == '__main__':
    unittest.main()
