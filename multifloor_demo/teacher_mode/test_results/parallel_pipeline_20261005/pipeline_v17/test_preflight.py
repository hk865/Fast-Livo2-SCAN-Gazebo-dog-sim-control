import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

HERE=Path(__file__).resolve().parents[3]/'navigation/ingress_pipeline_v17'
spec=importlib.util.spec_from_file_location('v17_guard_test',HERE/'pipeline_preflight.py')
guard=importlib.util.module_from_spec(spec);spec.loader.exec_module(guard)

class Guard(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory(prefix='v17_guard_')
        self.directory=Path(self.temp.name)
        self.receipt=json.loads((HERE/'INGRESS_PIPELINE_PREFLIGHT.json').read_text())
        self.profile=json.loads((HERE/'profiles/l64_r30_c30_ingress_smoke60.json').read_text())
        contract=self.directory/'INGRESS_PIPELINE_CONTRACT.json'
        contract.write_bytes((HERE/'INGRESS_PIPELINE_CONTRACT.json').read_bytes())
        self.receipt['sha256'][str(contract.resolve())]=hashlib.sha256(contract.read_bytes()).hexdigest()
        guard.HERE=self.directory
    def tearDown(self):
        guard.HERE=HERE;self.temp.cleanup()
    def run_guard(self):
        (self.directory/'INGRESS_PIPELINE_PREFLIGHT.json').write_text(json.dumps(self.receipt))
        return guard.verify_pipeline_preflight(self.profile)
    def test_positive_current_byte_bound_limited_scope(self):self.run_guard()
    def test_inherited_pass_is_not_new_proof(self):
        self.receipt['status']='PASS'
        with self.assertRaises(RuntimeError):self.run_guard()
    def test_missing_pure_check(self):
        self.receipt['checks'].pop('production_packet_semantics')
        with self.assertRaises(RuntimeError):self.run_guard()
    def test_pipeline_off_rejected(self):
        self.profile['ingress_pipeline']['enabled']=False
        with self.assertRaises(RuntimeError):self.run_guard()
    def test_unvalidated_full_duration_rejected(self):
        self.profile['duration_s']=600
        with self.assertRaises(RuntimeError):self.run_guard()
    def test_changed_bound_source_rejected_without_mutating_source(self):
        self.receipt['sha256'][str(HERE/'pipeline_preflight.py')]='0'*64
        with self.assertRaises(RuntimeError):self.run_guard()

if __name__=='__main__':unittest.main(verbosity=2)
