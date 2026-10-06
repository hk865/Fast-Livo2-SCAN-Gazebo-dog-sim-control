#!/usr/bin/env python3
"""Three additive V14 viewer boundaries; synthetic receipts only."""
from pathlib import Path
import importlib.util,json,unittest
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('teacher_v11_fixture_base',HERE.parent/'viewer_audit_v11/test_viewer_receipts_v2.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
viewer=old.viewer

class V14CandidateIntegrity(unittest.TestCase):
 def setUp(self):
  self.fixture=old.PublicationLedgerIntegrity(methodName='test_complete_consistent_synthetic_receipt_selects_new_and_keeps_common')
  self.fixture.setUp()
 def tearDown(self):self.fixture.tearDown()
 def remap(self,target):
  f=self.fixture
  f.snapshots={source.replace('/lidar_sampling_v11/','/'+target+'/'):row for source,row in f.snapshots.items()}
  scope=json.loads((f.run/'navigation_scope.json').read_text())
  scope['references']={source:row['sha256'] for source,row in f.snapshots.items()}
  scope['references'].update({row['snapshot']:row['sha256'] for row in f.snapshots.values()})
  if target=='lidar_sampling_v14':scope['profile']['cpu_affinity_enabled']=True
  old.write(f.run/'navigation_scope.json',scope)
  old.write(f.run/'navigation_source_snapshots.json',f.snapshots)
  old.write(f.run/'source_manifest.json',{row['snapshot']:row['sha256'] for row in f.snapshots.values()})
  f.common['verified_input_source_sha256']={name:old.digest(name) for name in f.common['verified_input_source_sha256']}
  f.write_common();f.receipt=f.new_receipt();f.write_receipt()
 def test_fixed_V14_same_frozen_seven_sources_is_allowed(self):
  self.remap('lidar_sampling_v14');r=self.fixture.selected()
  self.assertEqual(r['status'],'passed');self.assertTrue(r['publication_ledger_contract'])
  self.assertEqual(r['clock_hold']['validation_errors'],[])
  self.assertEqual(r['execution_variant'],'V14 固定绑核实验')
  (self.fixture.run/'summary_closed_loop_clock_hold_independent.json').unlink()
  pending=self.fixture.selected();self.assertEqual(pending['status'],'unverified')
  self.assertIn('实际验收尚未完成',pending['display_note'])
 def test_foreign_V13_candidate_is_rejected_even_with_same_bytes(self):
  self.remap('lidar_sampling_v13');r=self.fixture.selected()
  self.assertEqual(r['status'],'unverified')
  self.assertTrue(any('another candidate' in x for x in r['clock_hold']['validation_errors']))
 def test_V14_changed_producer_cannot_use_same_frozen_criterion(self):
  self.remap('lidar_sampling_v14');f=self.fixture
  source=next(k for k in f.snapshots if Path(k).name=='controller.py');row=f.snapshots[source]
  snapshot=Path(row['snapshot']);snapshot.write_bytes(snapshot.read_bytes()+b'\n# synthetic changed producer\n')
  row['sha256']=old.digest(snapshot)
  scope=json.loads((f.run/'navigation_scope.json').read_text());scope['references'][source]=row['sha256'];scope['references'][row['snapshot']]=row['sha256']
  old.write(f.run/'navigation_scope.json',scope);old.write(f.run/'navigation_source_snapshots.json',f.snapshots)
  old.write(f.run/'source_manifest.json',{x['snapshot']:x['sha256'] for x in f.snapshots.values()})
  f.common['verified_input_source_sha256']={name:old.digest(name) for name in f.common['verified_input_source_sha256']}
  f.write_common();f.receipt=f.new_receipt();f.write_receipt();r=f.selected()
  self.assertEqual(r['status'],'unverified')
  self.assertTrue(any('producer snapshot differs' in x for x in r['clock_hold']['validation_errors']))

if __name__=='__main__':unittest.main(verbosity=2)
