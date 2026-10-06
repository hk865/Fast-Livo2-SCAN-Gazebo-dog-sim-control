#!/usr/bin/env python3
"""Finite temporary-file archive and failed-runtime closure tests; no actual run data mutation."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
SPEC = importlib.util.spec_from_file_location('archive27bd', Path(__file__).with_name('lossless_v24_27bd_clouds.py'))
mod = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(mod)

class Archive27bdTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.root = Path(self.tmp.name)
        self.run = self.root/'fake_27bd'; self.run.mkdir(); self.cloud=self.run/'navigation_cloud_arrays'; self.cloud.mkdir()
        self.dest=self.root/'archive'; self.dest.mkdir(mode=0o700)
        self.audit=self.root/'audit.json'; self.audit.write_text('{"finite_fixture":true}\n')
        self.patches=[patch.object(mod,'RUN',self.run),patch.object(mod,'CLOUD',self.cloud)]
        for p in self.patches:p.start()
        self.owners=[dict(role=role,pid=999999910+i,pgid=999999910+i,start_ticks=1) for i,role in enumerate(sorted(mod.REQUIRED_ROLES))]
        self.started={x['role']:dict(x) for x in self.owners}
        self.children=[dict(pid=999999930+i,action='finite'+str(i),clean_exit=True)for i in range(13)]
        self.mapping=dict(pid=self.children[0]['pid'],pgid=999999911,start_ticks=1)
        q=dict(schema='staged_input_pipeline_v19/v1',mode='staged',normal_completed=True,context_valid_at_drain=True,
               failure='',uncommitted_packets=[],accepted=5,delivered=5,committed=5,
               **{k:0 for k in ['pending','inflight','ready','bytes','canceled','rejected_capacity','closed_rejections']})
        self.stop=dict(run=str(self.run),normal_completed=True,error=None,gate_errors=[],summary=q,saved_identity=self.mapping)
        self.runtime=dict(run=str(self.run),runtime_status='failed',error=mod.INTERRUPT_ERROR,
            all_owned_and_children_clean=False,owned_processes=self.owners,pipeline_normal_completed=True,
            cleanup={x['role']:dict(started=True,remaining_owned_group_members=[])for x in self.owners},
            launch_child_exit_evidence=dict(expected_children=13,started_children=13,children=self.children,died=[],all_expected_children_clean=True))
        self.loaded=dict(run=str(self.run),verified=True,mapping_identity=self.mapping)
        self.result=dict(run=str(self.run),status='failed',runtime_error=mod.INTERRUPT_ERROR)
        self.write_docs()
        self.payloads={'000001_10.npy':b'\x93NUMPY'+bytes(range(256))*100,'000001_10.bin':b'\0\xffpointcloud'*1000}
        for n,b in self.payloads.items():(self.cloud/n).write_bytes(b)
        self.quiet=contextlib.redirect_stdout(io.StringIO());self.quiet.__enter__()
    def write_docs(self):
        def w(name,obj):(self.run/name).write_text(json.dumps(obj))
        w('pipeline_normal_stop.json',self.stop)
        self.runtime['pipeline_normal_stop_sha256']=mod.file_hash(self.run/'pipeline_normal_stop.json')
        w('runtime_manifest.json',self.runtime)
        self.result['runtime_manifest_sha256']=mod.file_hash(self.run/'runtime_manifest.json')
        w('run_result.json',self.result);w('owned_processes_started.json',self.started);w('slam_loaded_binary.json',self.loaded)
    def tearDown(self):
        self.quiet.__exit__(None,None,None)
        for p in reversed(self.patches):p.stop()
        self.tmp.cleanup()
    def test_roundtrip_exact_bytes_and_explicit_failed_manifest(self):
        mod.archive(self.dest,self.audit)
        m=json.loads((self.dest/'MANIFEST.json').read_text());self.assertTrue(m['worker_result_missing'])
        self.assertFalse(m['runtime_clean_pass']);self.assertFalse(m['navigation_pass'])
        self.assertEqual(m['targeted_audit_receipt_sha256'],mod.file_hash(self.audit))
        mod.prune(self.dest);self.assertEqual(list(self.cloud.iterdir()),[])
        mod.restore(self.dest)
        for n,b in self.payloads.items():self.assertEqual((self.cloud/n).read_bytes(),b)
        self.assertFalse((self.run/'worker_result.json').exists())
    def test_preflight_does_not_create_archive_or_worker(self):
        with patch('sys.argv',['tool','preflight','--archive-root',str(self.root/'not_created')]):mod.main()
        self.assertFalse((self.root/'not_created').exists());self.assertFalse((self.run/'worker_result.json').exists())
    def test_wrong_interrupt_error_rejected(self):
        self.runtime['error']='TimeoutError: region';self.write_docs()
        with self.assertRaises(RuntimeError):mod.closed_run()
    def test_false_pass_runtime_rejected(self):
        self.runtime['all_owned_and_children_clean']=True;self.write_docs()
        with self.assertRaises(RuntimeError):mod.closed_run()
    def test_worker_result_created_rejected(self):
        (self.run/'worker_result.json').write_text('{"completed":true}')
        with self.assertRaises(RuntimeError):mod.closed_run()
    def test_live_saved_owner_rejected(self):
        ticks=int(Path(f'/proc/{os.getpid()}/stat').read_text().rsplit(')',1)[1].split()[19])
        role=self.owners[0]['role'];self.owners[0].update(pid=os.getpid(),start_ticks=ticks);self.started[role].update(self.owners[0]);self.write_docs()
        with self.assertRaises(RuntimeError):mod.closed_run()
    def test_live_launch_child_pid_rejected(self):
        self.children[-1]['pid']=os.getpid();self.write_docs()
        with self.assertRaises(RuntimeError):mod.closed_run()
    def test_missing_child_exit_rejected(self):
        self.children[-1]['clean_exit']=False;self.write_docs()
        with self.assertRaises(RuntimeError):mod.closed_run()
    def test_missing_role_rejected(self):
        self.owners.pop();self.write_docs()
        with self.assertRaises(RuntimeError):mod.closed_run()
    def test_owner_identity_rebinding_rejected(self):
        self.started[self.owners[0]['role']]['start_ticks']=7;self.write_docs()
        with self.assertRaises(RuntimeError):mod.closed_run()
    def test_pipeline_closed_rejection_rejected(self):
        self.stop['summary']['closed_rejections']=1;self.write_docs()
        with self.assertRaises(RuntimeError):mod.closed_run()
    def test_hash_binding_failure_rejected(self):
        self.write_docs();self.result['runtime_manifest_sha256']='0'*64
        (self.run/'run_result.json').write_text(json.dumps(self.result))
        with self.assertRaises(RuntimeError):mod.closed_run()
    def test_symlink_source_rejected(self):
        (self.cloud/'000001_10.npy').unlink();(self.cloud/'000001_10.npy').symlink_to(self.audit)
        with self.assertRaises(RuntimeError):mod.archive(self.dest,self.audit)
        self.assertTrue((self.cloud/'000001_10.bin').exists())
    def test_identity_change_blocks_prune(self):
        mod.archive(self.dest,self.audit);p=self.cloud/'000001_10.npy';b=p.read_bytes();p.unlink();p.write_bytes(b)
        with self.assertRaises(RuntimeError):mod.prune(self.dest)
        self.assertEqual(len(list(self.cloud.iterdir())),2)
    def test_corrupt_archive_blocks_prune(self):
        mod.archive(self.dest,self.audit);p=self.dest/'v24_27bd_cloud_arrays.tar.zst'
        with p.open('r+b')as f:f.seek(5);f.write(b'INVALID')
        with self.assertRaises(RuntimeError):mod.prune(self.dest)
        self.assertEqual(len(list(self.cloud.iterdir())),2)
    def test_audit_flag_still_required(self):
        with patch('sys.argv',['tool','archive','--archive-root',str(self.dest),'--audit-receipt',str(self.audit)]):
            with self.assertRaises(RuntimeError):mod.main()
        self.assertFalse((self.dest/'MANIFEST.json').exists())
    def test_audit_changed_blocks_prune(self):
        mod.archive(self.dest,self.audit);self.audit.write_text('{"different":true}')
        with self.assertRaises(RuntimeError):mod.prune(self.dest)
        self.assertEqual(len(list(self.cloud.iterdir())),2)

if __name__=='__main__':unittest.main()
