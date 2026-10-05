#!/usr/bin/env python3
"""Read-only receipt boundary tests using isolated copies and a byte-hash oracle.

The oracle contains the actual evaluator-bound digests; mutations never touch
any actual run file. Real streaming hashes and cache are verified separately.
"""
import copy,hashlib,importlib.util,json,unittest
from pathlib import Path
from unittest.mock import patch
HERE=Path(__file__).resolve().parent
TEACHER=HERE.parents[2]
spec=importlib.util.spec_from_file_location('teacher_metadata_viewer',TEACHER/'scripts/serve.py')
viewer=importlib.util.module_from_spec(spec);spec.loader.exec_module(viewer)
RUN=Path('/var/tmp/go2_teacher_simulation_20261005/20261005_211806_closed_loop_cascade_clock_hold_lidar64_30hz_rgb30_lockfree4_full_r1_fbcb')
META='summary_closed_loop_terrain_metadata_join_independent_v2.json'
COMMON='summary_closed_loop_cascade_independent.json';V2='summary_closed_loop_clock_hold_independent.json';RAMP='summary_closed_loop_ramp_independent.json'
class MetadataReceiptIntegrity(unittest.TestCase):
 @classmethod
 def setUpClass(cls):
  names=[META,COMMON,V2,RAMP,'summary_closed_loop_terrain_metadata_join_independent.json','ramp_acceptance_contract.json','navigation_scope.json','navigation_source_snapshots.json','navigation_request.json']
  cls.base={str(RUN/n):json.loads((RUN/n).read_text())for n in names}
  cls.digests={str(RUN/n):hashlib.sha256((RUN/n).read_bytes()).hexdigest()for n in names}
  for m in cls.base[str(RUN/META)]['source_and_check_closure']['all_three_ancestor_binding_maps_rehashed'].values():cls.digests.update(m)
  cls.digests.update(cls.base[str(RUN/META)]['verified_input_source_sha256'])
 def setUp(self):
  self.data=copy.deepcopy(self.base);self.hashes=copy.deepcopy(self.digests)
  self.original_read=viewer.read_json;self.original_sha=viewer.source_file_sha256
  def read(path):return copy.deepcopy(self.data[str(Path(path).resolve())])if str(Path(path).resolve())in self.data else self.original_read(path)
  def sha(path):return self.hashes.get(str(Path(path).resolve()))or self.original_sha(path)
  self.patches=[patch.object(viewer,'read_json',read),patch.object(viewer,'source_file_sha256',sha)]
  for p in self.patches:p.start()
 def tearDown(self):
  for p in reversed(self.patches):p.stop()
 def raw(self,name=META):return self.data[str(RUN/name)]
 def selected(self):return viewer.closed_loop_receipt_view(RUN)
 def rejected(self):
  result=self.selected();r=result['terrain_metadata'];self.assertEqual(r['status'],'unverified');self.assertFalse(r['valid_for_selected_run']);self.assertTrue(r['validation_errors']);self.assertNotEqual(result['status'],'passed')
 def test_complete_metadata_only_pass_keeps_original_three_unverified(self):
  r=self.selected();self.assertEqual(r['terrain_metadata']['status'],'passed');self.assertEqual(r['terrain_metadata']['counts']['total'],5)
  self.assertEqual(r['status'],'unverified');self.assertEqual(r['selected_receipt_filename'],V2)
  for key,name in [('common',COMMON),('clock_hold',V2),('ramp',RAMP)]:
   self.assertEqual(r[key]['status'],'unverified');self.assertEqual(r[key]['raw'],self.base[str(RUN/name)])
 def test_foreign_run_rejected(self):self.raw()['run']=str(RUN.parent/'20261005_211806_closed_loop_cascade_foreign_0000');self.rejected()
 def test_unknown_schema_rejected(self):self.raw()['schema']='independent_future/v3';self.rejected()
 def test_unknown_reader_rejected(self):self.raw()['evaluator_sha256']='0'*64;self.rejected()
 def test_reader_source_byte_change_rejected(self):
  self.hashes[str((TEACHER/'test_results/lidar_density_rate_20261005/evaluation/audit_terrain_metadata_v2.py').resolve())]='0'*64;self.rejected()
 def test_missing_metadata_gate_rejected(self):self.raw()['checks'].pop('all_original_terrain_switch_ray_and_native_conditions');self.rejected()
 def test_nonpassing_metadata_gate_rejected(self):self.raw()['checks']['all_original_terrain_switch_ray_and_native_conditions'].update(status='failed',passed=False);self.rejected()
 def test_bad_ancestor_sha_rejected(self):self.raw()['original_receipts'][COMMON]['sha256']='0'*64;self.rejected()
 def test_missing_ancestor_rejected(self):self.data[str(RUN/RAMP)]={};self.rejected()
 def test_missing_publication_ledger_gate_rejected(self):self.raw(V2)['checks'].pop('actual_control_publication_ledger_complete_and_original');self.rejected()
 def test_partial_region_prefix_cannot_be_passed(self):
  for name in [COMMON,V2]:self.raw(name)['checks']['all_original_SLAM_3D_region_arrivals']['arrivals']=self.raw(name)['checks']['all_original_SLAM_3D_region_arrivals']['arrivals'][:25]
  self.raw(V2)['raw_common_checks']=copy.deepcopy(self.raw(COMMON)['checks']);self.rejected()
 def test_failed_original_ramp_gate_rejected(self):
  self.raw(RAMP)['checks']['leg_1_ramp_23_all_twelve_metre_bins_actual_support'].update(status='failed',passed=False);self.raw(RAMP)['status']='failed';self.rejected()
 def test_pilot_incomplete_ramp_scope_rejected(self):
  self.raw('ramp_acceptance_contract.json')['segments']=self.raw('ramp_acceptance_contract.json')['segments'][:1]
  self.raw(RAMP)['prospective_contract']=copy.deepcopy(self.raw('ramp_acceptance_contract.json'));self.rejected()
 def test_changed_original_ledger_byte_binding_rejected(self):self.hashes[str(RUN/'navigation_command_publications.jsonl')]='0'*64;self.rejected()
 def test_missing_closed_ancestor_byte_map_rejected(self):self.raw()['source_and_check_closure']['all_three_ancestor_binding_maps_rehashed'].pop(V2+':clock_hold_verified_input_source_sha256');self.rejected()
 def test_missing_independent_input_binding_rejected(self):self.raw()['verified_input_source_sha256'].pop(str(RUN/'actuator.jsonl'));self.rejected()
 def test_altered_archived_status_producer_rejected(self):
  key=next(k for k in self.raw()['verified_input_source_sha256']if Path(k).name=='teacher_wrapper.py');self.hashes[key]='0'*64;self.rejected()
 def test_third_removed_field_rejected(self):self.raw()['checks']['exact_unique_full_payload_and_original_causal_join']['removed_archival_fields'].append('command');self.rejected()
 def test_unknown_candidate_rejected(self):
  self.raw('navigation_source_snapshots.json')[str(TEACHER/'navigation/lidar_sampling_v13/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json')]=self.raw('navigation_source_snapshots.json').pop(str(TEACHER/'navigation/lidar_sampling_v12/PROSPECTIVE_PUBLICATION_LEDGER_CRITERIA.json'));self.rejected()
 def test_missing_metadata_receipt_does_not_upgrade_main(self):
  self.data[str(RUN/META)]={};r=self.selected();self.assertEqual(r['terrain_metadata'],{});self.assertEqual(r['status'],'unverified')
if __name__=='__main__':unittest.main(verbosity=2)
