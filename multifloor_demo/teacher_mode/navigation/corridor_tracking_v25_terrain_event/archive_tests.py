"""Finite archive and direct dependency checks; no ROS or Gazebo."""
from pathlib import Path
import copy
import hashlib
import json
import tempfile
import unittest
from unittest.mock import patch

import corridor_preflight
import geometry_archive
import pid_scope
import scan_workspace
import slam_workspace

HERE = Path(__file__).resolve().parent
OLD = HERE.parent/'corridor_tracking_v21_curvature'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class ArchiveChecks(unittest.TestCase):
    def test_five_geometry_helpers_are_explicit_runtime_dependencies(self):
        profile = json.loads((HERE/'profiles/curvature_original46_prefix9_on.json').read_text())
        helpers = geometry_archive.geometry_helper_files(HERE)
        self.assertEqual(len(helpers), 5)
        self.assertEqual({p.name for p in helpers}, {'route.py', 'prefix_contract.py',
            'mission46_profile.py', 'goal_regions.py', 'route_regions.py'})
        # The gate is deliberately not required to exist yet in this finite check.
        with patch('corridor_preflight.evidence_files', return_value=[]):
            files = {p.resolve() for p in pid_scope.runtime_files(profile)}
        self.assertTrue({p.resolve() for p in helpers}.issubset(files))
        self.assertFalse((HERE/'scan_ws').exists())
        self.assertFalse((HERE/'slam_ws').exists())

    def test_control_and_original_region_code_are_byte_identical_to_V21(self):
        for name in ('controller.py', 'cascade_core.py', 'spatial_reference.py',
                'route.py', 'prefix_contract.py', 'mission46_profile.py', 'worker.py',
                'clock_hold.py', 'publication_ledger.py', 'pipeline_lifecycle.py'):
            self.assertEqual((HERE/name).read_bytes(), (OLD/name).read_bytes(), name)

    def test_profile_changes_are_only_candidate_metadata(self):
        for path in (HERE/'profiles').glob('*.json'):
            new = json.loads(path.read_text())
            old = json.loads((OLD/'profiles'/path.name).read_text())
            for key in ('controller_selector', 'profile_version', 'prospective_change',
                        'mission46_required_source_files'):
                new.pop(key, None); old.pop(key, None)
            self.assertEqual(new, old, path.name)

    def archive_fixture(self, run):
        root = HERE.parents[1]
        refs = {}; snapshots = {}; manifest = {}
        for original in geometry_archive.geometry_helper_files(HERE):
            original = original.resolve(); digest = sha(original)
            rel = (original.relative_to(root) if original.is_relative_to(root)
                else Path('external')/(digest[:16]+'_'+original.name))
            dest = run/'sources'/rel; dest.parent.mkdir(parents=True, exist_ok=True)
            dest.write_bytes(original.read_bytes())
            refs[str(original)] = digest; refs[str(dest)] = digest
            snapshots[str(original)] = dict(snapshot=str(dest), sha256=digest)
            manifest[str(rel)] = digest; manifest[str(original)] = digest
        return refs, snapshots, manifest

    def test_direct_archive_positive_has_all_five_original_bytes(self):
        with tempfile.TemporaryDirectory() as t:
            run = Path(t); args = self.archive_fixture(run)
            rows = geometry_archive.verify_geometry_archive(run, *args, HERE)
            self.assertEqual(len(rows), 5)

    def test_each_missing_snapshot_is_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            run = Path(t); refs, snapshots, manifest = self.archive_fixture(run)
            for source in snapshots:
                mutated = dict(snapshots); mutated.pop(source)
                with self.subTest(source=source), self.assertRaises(RuntimeError):
                    geometry_archive.verify_geometry_archive(run, refs, mutated, manifest, HERE)

    def test_changed_snapshot_bytes_are_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            run = Path(t); args = self.archive_fixture(run)
            row = next(v for k, v in args[1].items() if k.endswith('/mission/route_regions.py'))
            Path(row['snapshot']).write_bytes(b'changed fixture bytes\n')
            with self.assertRaises(RuntimeError):
                geometry_archive.verify_geometry_archive(run, *args, HERE)

    def test_missing_absolute_alias_or_relative_manifest_key_is_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            run = Path(t); refs, snapshots, manifest = self.archive_fixture(run)
            source = next(k for k in snapshots if k.endswith('/mission/route_regions.py'))
            relative = str(Path(snapshots[source]['snapshot']).relative_to(run/'sources'))
            for key in (source, relative):
                changed = dict(manifest); changed.pop(key)
                with self.subTest(key=key), self.assertRaises(RuntimeError):
                    geometry_archive.verify_geometry_archive(run, refs, snapshots, changed, HERE)

    def test_foreign_or_noncanonical_snapshot_is_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            run = Path(t); refs, snapshots, manifest = self.archive_fixture(run)
            source = next(k for k in snapshots if k.endswith('/mission/route_regions.py'))
            for target in (run/'sources/other.py', HERE/'route.py'):
                changed = copy.deepcopy(snapshots); changed[source]['snapshot'] = str(target)
                with self.subTest(target=str(target)), self.assertRaises(RuntimeError):
                    geometry_archive.verify_geometry_archive(run, refs, changed, manifest, HERE)

    def gate_fixture(self, here):
        profile = json.loads((HERE/'profiles/curvature_original46_prefix9_on.json').read_text())
        (here/'profiles').mkdir(); (here/'profiles/curvature_original46_prefix9_on.json').write_text(json.dumps(profile))
        (here/'CURVATURE_V22_CONTRACT.json').write_text('{}')
        for name in ('review.json', 'archive_review.json', 'finite.json'):
            (here/name).write_text('{}')
        files = [here/'profiles/curvature_original46_prefix9_on.json', here/'CURVATURE_V22_CONTRACT.json',
                 here/'review.json', here/'archive_review.json', here/'finite.json',
                 *geometry_archive.geometry_helper_files(HERE), *slam_workspace.evidence_files(),
                 *scan_workspace.evidence_files(profile)]
        binding = {str(p.resolve()): sha(p) for p in files}
        row = lambda name: dict(path=str(here/name), sha256=sha(here/name))
        d = dict(schema='curvature_v22_archive_preflight/v1', status='PASS_LIMITED_NEW_EXPERIMENT',
            candidate_root=str(here), allowed=True, actual_navigation_verified=False, historical_pass_inherited=False,
            checks={k: True for k in corridor_preflight.REQUIRED}, allowed_profiles=['curvature_original46_prefix9_on.json'],
            control_review=row('review.json'), archive_review=row('archive_review.json'),
            finite_receipts=[row('finite.json')], verified_inputs_sha256=binding)
        return profile, d

    def test_each_missing_direct_helper_gate_binding_is_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            here = Path(t).resolve(); profile, gate = self.gate_fixture(here)
            for helper in geometry_archive.geometry_helper_files(HERE):
                changed = copy.deepcopy(gate); changed['verified_inputs_sha256'].pop(str(helper.resolve()))
                (here/'CURVATURE_V22_PREFLIGHT.json').write_text(json.dumps(changed))
                with patch('corridor_preflight.geometry_helper_files', return_value=geometry_archive.geometry_helper_files(HERE)):
                    with self.subTest(helper=str(helper)), self.assertRaisesRegex(RuntimeError, 'mandatory candidate'):
                        corridor_preflight.verify_preflight(here, profile)

    def test_wrong_direct_geometry_hash_is_rejected(self):
        with tempfile.TemporaryDirectory() as t:
            here = Path(t).resolve(); profile, gate = self.gate_fixture(here)
            helper = next(p for p in geometry_archive.geometry_helper_files(HERE) if p.name == 'route_regions.py')
            gate['verified_inputs_sha256'][str(helper.resolve())] = '0'*64
            (here/'CURVATURE_V22_PREFLIGHT.json').write_text(json.dumps(gate))
            with patch('corridor_preflight.geometry_helper_files', return_value=geometry_archive.geometry_helper_files(HERE)):
                with self.assertRaisesRegex(RuntimeError, 'reviewed source or receipt changed'):
                    corridor_preflight.verify_preflight(here, profile)


if __name__ == '__main__':
    unittest.main()
