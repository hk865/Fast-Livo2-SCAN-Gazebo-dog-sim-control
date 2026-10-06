#!/usr/bin/env python3
"""Synthetic root/alias access tests; no HTTP server or actual run writes."""
from pathlib import Path
import importlib.util,json,os,tempfile,unittest
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('teacher_viewer_storage_test',HERE.parents[2]/'scripts/serve.py')
viewer=importlib.util.module_from_spec(spec);spec.loader.exec_module(viewer)

class StorageIntegrity(unittest.TestCase):
 def setUp(self):
  self.temp=tempfile.TemporaryDirectory(prefix='go2_viewer_storage_fixture_')
  self.base=Path(self.temp.name);self.old_root=viewer.ROOT;self.old_external=viewer.EXTERNAL_RUN_STORAGE
  viewer.ROOT=self.base/'teacher';self.runs=viewer.ROOT/'runs';self.runs.mkdir(parents=True)
  self.storage=self.base/'private_external';self.storage.mkdir(mode=0o700);viewer.EXTERNAL_RUN_STORAGE=self.storage
  self.name='20261005_210000_closed_loop_cascade_storage_display_fixture_1234'
  self.actual=self.storage/self.name;self.actual.mkdir()
  self.alias=self.runs/self.name;self.alias.symlink_to(self.actual,target_is_directory=True)
  (self.actual/'state.json').write_text(json.dumps({'run':str(self.actual),'fixture_only':True}))
  self.dashboard=viewer.Dashboard(self.runs)
 def tearDown(self):
  viewer.ROOT=self.old_root;viewer.EXTERNAL_RUN_STORAGE=self.old_external;self.temp.cleanup()
 def test_valid_private_root_exact_alias_resolves_real_path(self):
  self.assertEqual(self.dashboard.resolve(self.name),self.actual)
  self.assertEqual(viewer.read_json(self.alias/'state.json')['run'],str(self.actual))
  self.assertIn(self.name,[x['id'] for x in self.dashboard.directories()])
 def test_valid_latest_requires_the_matching_named_alias(self):
  (self.runs/'latest').symlink_to(self.actual,target_is_directory=True)
  self.assertEqual(self.dashboard.resolve('latest'),self.actual)
  self.alias.unlink()
  self.assertIsNone(self.dashboard.inside(self.runs/'latest'))
 def test_non_private_root_rejected(self):
  self.storage.chmod(0o755)
  self.assertIsNone(self.dashboard.resolve(self.name));self.assertEqual(viewer.read_json(self.alias/'state.json'),{})
 def test_symlink_storage_root_rejected(self):
  symlink=self.base/'external_root_symlink';symlink.symlink_to(self.storage,target_is_directory=True)
  viewer.EXTERNAL_RUN_STORAGE=symlink
  self.assertIsNone(self.dashboard.resolve(self.name))
 def test_wrong_basename_alias_rejected_even_if_valid_alias_exists(self):
  wrong=self.runs/'foreign_alias';wrong.symlink_to(self.actual,target_is_directory=True)
  self.assertIsNone(self.dashboard.resolve(wrong.name))
 def test_missing_matching_project_alias_rejected(self):
  self.alias.unlink();self.assertIsNone(self.dashboard.inside(self.actual/'state.json'))
 def test_arbitrary_external_root_rejected(self):
  other=self.base/'arbitrary_other_root';other.mkdir();run=other/self.name;run.mkdir()
  self.alias.unlink();self.alias.symlink_to(run,target_is_directory=True)
  self.assertIsNone(self.dashboard.resolve(self.name))
 def test_non_run_file_directly_under_external_root_rejected(self):
  file=self.storage/'private-note.json';file.write_text('{"fixture":"private"}')
  self.assertIsNone(self.dashboard.inside(file));self.assertEqual(viewer.read_json(file),{})
 def test_escape_symlink_media_json_and_telemetry_rejected(self):
  outside=self.base/'outside.json';outside.write_text('{"fixture":"outside"}')
  for name in ('frame.jpg','navigation_request.json','telemetry.jsonl'):
   (self.actual/name).symlink_to(outside)
  self.assertIsNone(self.dashboard.inside(self.actual/'frame.jpg'))
  self.assertEqual(viewer.read_json(self.actual/'navigation_request.json'),{})
  self.assertEqual(self.dashboard.telemetry.read(self.actual/'telemetry.jsonl')['records'],0)
 def test_malformed_run_prefix_rejected(self):
  other=self.storage/'not_a_run';other.mkdir();alias=self.runs/'not_a_run';alias.symlink_to(other,target_is_directory=True)
  self.assertIsNone(self.dashboard.resolve('not_a_run'))

if __name__=='__main__':unittest.main(verbosity=2)
