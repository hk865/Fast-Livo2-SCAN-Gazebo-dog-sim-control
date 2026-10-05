#!/usr/bin/env python3
"""Synthetic UI-integrity fixtures only. No ROS, actual navigation, or serving."""
import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
TEACHER = HERE.parents[2]
spec = importlib.util.spec_from_file_location('teacher_old_receipt_integrity', HERE.parent / 'viewer_audit/test_viewer_receipts.py')
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)
viewer = old.viewer
write, digest = old.write, old.digest
V2_BYTES = (TEACHER / 'test_results/lidar_density_rate_20261005/evaluation/publication_ledger_v11/PROSPECTIVE_CRITERIA.json').read_bytes()


class PublicationLedgerIntegrity(old.ReceiptIntegrity):
    def setUp(self):
        self.is_v2_fixture = False
        super().setUp()
        criterion = json.loads(V2_BYTES)
        authority = self.root / viewer.PROSPECTIVE_RECEIPT_CONTRACTS[viewer.PUBLICATION_LEDGER_RECEIPT_SCHEMA]['authority']
        authority.parent.mkdir(parents=True)
        authority.write_bytes(V2_BYTES)
        original = self.root / viewer.PROSPECTIVE_RECEIPT_CONTRACTS[viewer.PUBLICATION_LEDGER_RECEIPT_SCHEMA]['candidates'][0]
        original.parent.mkdir(parents=True)
        original.write_bytes(V2_BYTES)
        self.snapshot = self.run / 'sources/navigation/lidar_sampling_v11/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json'
        self.snapshot.parent.mkdir(parents=True)
        self.snapshot.write_bytes(V2_BYTES)
        self.snapshots = {str(original): {'snapshot': str(self.snapshot), 'sha256': digest(self.snapshot)}}
        for index, (source, expected) in enumerate(criterion['pre_test_source_sha256'].items()):
            snapshot = self.run / ('sources/pinned_source_' + str(index) + '_' + Path(source).name)
            snapshot.write_bytes(Path(source).read_bytes())
            self.assertEqual(digest(snapshot), expected)
            archived_source = source if Path(source).name == 'evaluate_closed_loop.py' else str(original.parent / Path(source).name)
            self.snapshots[archived_source] = {'snapshot': str(snapshot), 'sha256': expected}
        self.criterion = criterion
        scope = json.loads((self.run / 'navigation_scope.json').read_text())
        scope['references'] = {name: row['sha256'] for name, row in self.snapshots.items()}
        scope['references'].update({row['snapshot']: row['sha256'] for row in self.snapshots.values()})
        write(self.run / 'navigation_scope.json', scope)
        write(self.run / 'source_manifest.json', {row['snapshot']: row['sha256'] for row in self.snapshots.values()})
        write(self.run / 'navigation_source_snapshots.json', self.snapshots)
        self.common['verified_input_source_sha256'] = {name: digest(name) for name in self.common['verified_input_source_sha256']}
        self.write_common()
        self.is_v2_fixture = True
        self.receipt = self.new_receipt()
        self.write_receipt()

    def new_receipt(self):
        raw = super().new_receipt()
        if not self.is_v2_fixture:
            return raw
        raw.update(schema=viewer.PUBLICATION_LEDGER_RECEIPT_SCHEMA,
                   criteria_sha256=viewer.PUBLICATION_LEDGER_CRITERIA_SHA256,
                   parent_prospective_criteria_sha256=viewer.CLOCK_HOLD_CRITERIA_SHA256)
        raw['checks']['prospective_clock_hold_criteria_and_archived_sources']['criteria_sha256'] = viewer.PUBLICATION_LEDGER_CRITERIA_SHA256
        raw['checks']['actual_control_publication_ledger_complete_and_original'] = {'status': 'passed', 'passed': True, 'synthetic_display_fixture_only': True}
        return raw

    def test_complete_consistent_synthetic_receipt_selects_new_and_keeps_common(self):
        result = self.selected()
        self.assertEqual(result['status'], 'passed')
        self.assertTrue(result['publication_ledger_contract'])
        self.assertEqual(result['counts']['total'], 26)
        self.assertEqual(result['common']['raw'], self.common)
        self.assertEqual(result['clock_hold']['validation_errors'], [])

    def test_missing_seventh_ledger_gate_rejected(self):
        self.receipt['checks'].pop('actual_control_publication_ledger_complete_and_original')
        self.rejected()

    def test_nonpassing_seventh_ledger_gate_cannot_claim_pass(self):
        self.receipt['checks']['actual_control_publication_ledger_complete_and_original'] = {'status': 'unverified', 'passed': None}
        self.rejected()

    def test_parent_contract_digest_rejected(self):
        self.receipt['parent_prospective_criteria_sha256'] = '0' * 64
        self.rejected()

    def test_unknown_schema_rejected(self):
        self.receipt['schema'] = 'independent_future_navigation/v99'
        self.rejected()

    def test_actual_pinned_producer_snapshot_change_rejected(self):
        name = Path(next(iter(self.criterion['pre_test_source_sha256']))).name
        key = next(key for key in self.snapshots if Path(key).name == name)
        path = Path(self.snapshots[key]['snapshot'])
        path.write_bytes(path.read_bytes() + b'\n')
        self.rejected()

    def test_wrong_candidate_source_rejected_even_if_all_bindings_agree(self):
        key = next(name for name in self.snapshots if Path(name).name == 'PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json')
        other = str(self.root / 'navigation/foreign/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json')
        self.snapshots[other] = self.snapshots.pop(key)
        scope = json.loads((self.run / 'navigation_scope.json').read_text())
        scope['references'][other] = scope['references'].pop(key)
        write(self.run / 'navigation_scope.json', scope)
        write(self.run / 'navigation_source_snapshots.json', self.snapshots)
        self.common['verified_input_source_sha256'] = {name: digest(name) for name in self.common['verified_input_source_sha256']}
        self.write_common()
        self.receipt = self.new_receipt()
        self.rejected()

    def test_fixed_V12_candidate_with_identical_pinned_bytes_is_allowed(self):
        remapped = {}
        for source, row in self.snapshots.items():
            remapped[source.replace('/lidar_sampling_v11/', '/lidar_sampling_v12/')] = row
        self.snapshots = remapped
        scope = json.loads((self.run / 'navigation_scope.json').read_text())
        scope['references'] = {source: row['sha256'] for source, row in remapped.items()}
        scope['references'].update({row['snapshot']: row['sha256'] for row in remapped.values()})
        write(self.run / 'navigation_scope.json', scope)
        write(self.run / 'navigation_source_snapshots.json', remapped)
        self.common['verified_input_source_sha256'] = {name: digest(name) for name in self.common['verified_input_source_sha256']}
        self.write_common()
        self.receipt = self.new_receipt()
        self.write_receipt()
        self.assertEqual(self.selected()['status'], 'passed')

    def test_duplicate_producer_basename_not_accepted(self):
        source = next(key for key in self.snapshots if Path(key).name == 'controller.py')
        other = str(self.root / 'navigation/foreign/controller.py')
        self.snapshots[other] = copy.deepcopy(self.snapshots[source])
        scope = json.loads((self.run / 'navigation_scope.json').read_text())
        scope['references'][other] = self.snapshots[other]['sha256']
        write(self.run / 'navigation_scope.json', scope)
        write(self.run / 'navigation_source_snapshots.json', self.snapshots)
        self.common['verified_input_source_sha256'] = {name: digest(name) for name in self.common['verified_input_source_sha256']}
        self.write_common()
        self.receipt = self.new_receipt()
        self.rejected()

    def test_cached_digest_invalidates_when_content_same_size_mtime_retained(self):
        path = self.run / 'cache_fixture'
        path.write_bytes(b'abcd')
        original_stat = path.stat()
        first = viewer.source_file_sha256(path)
        self.assertEqual(viewer.source_file_sha256(path), first)
        path.write_bytes(b'efgh')
        os.utime(path, ns=(original_stat.st_atime_ns, original_stat.st_mtime_ns))
        self.assertNotEqual(viewer.source_file_sha256(path), first)

    def test_cached_digest_invalidates_when_path_inode_replaced(self):
        path = self.run / 'cache_fixture'
        path.write_bytes(b'abcd')
        first = viewer.source_file_sha256(path)
        replacement = self.run / 'cache_fixture_replacement'
        replacement.write_bytes(b'efgh')
        os.replace(replacement, path)
        self.assertNotEqual(viewer.source_file_sha256(path), first)


if __name__ == '__main__':
    unittest.main(verbosity=2)
