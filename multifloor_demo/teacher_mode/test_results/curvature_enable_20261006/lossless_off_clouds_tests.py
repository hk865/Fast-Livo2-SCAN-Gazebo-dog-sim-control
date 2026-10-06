#!/usr/bin/env python3
"""Tiny temporary fixtures only; never accesses the actual OFF cloud directory."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

SPEC = importlib.util.spec_from_file_location('archive_tool', Path(__file__).with_name('lossless_off_clouds.py'))
mod = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mod)


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.run = self.root / 'fake_off_run'; self.run.mkdir()
        self.cloud = self.run / 'navigation_cloud_arrays'; self.cloud.mkdir()
        self.dest = self.root / 'archive'; self.dest.mkdir(mode=0o700)
        self.audit = self.root / 'audit.json'; self.audit.write_text('{"test_fixture":true}\n')
        self.patches = [patch.object(mod, 'RUN', self.run), patch.object(mod, 'CLOUD', self.cloud)]
        for p in self.patches: p.start()
        summary = dict(normal_completed=True, context_valid_at_drain=True, failure='',
                       uncommitted_packets=[], accepted=5, delivered=5, committed=5,
                       **{k:0 for k in ['pending','inflight','ready','bytes','canceled','rejected_capacity','closed_rejections']})
        docs = {
            'run_result.json':dict(run=str(self.run), runtime_error=None, status='runtime_completed_navigation_unverified'),
            'worker_result.json':dict(completed=True, fault=None),
            'runtime_manifest.json':dict(run=str(self.run), owned_processes=[]),
            'pipeline_normal_stop.json':dict(run=str(self.run), normal_completed=True, error=None, gate_errors=[],
                       summary=summary, saved_identity=dict(pid=999999999, start_ticks=1)),
        }
        for n,d in docs.items(): (self.run/n).write_text(json.dumps(d))
        self.payloads = {'000001_10.npy':b'\x93NUMPY'+bytes(range(256))*100,
                         '000001_10.bin':b'\0\xffpoint cloud\x01'*1000}
        for n,b in self.payloads.items(): (self.cloud/n).write_bytes(b)
        self.quiet = contextlib.redirect_stdout(io.StringIO()); self.quiet.__enter__()

    def tearDown(self):
        self.quiet.__exit__(None,None,None)
        for p in reversed(self.patches): p.stop()
        self.tmp.cleanup()

    def archive(self): mod.archive(self.dest,self.audit)

    def test_roundtrip_all_bytes_and_independent_prune(self):
        self.archive()
        for n,b in self.payloads.items(): self.assertEqual((self.cloud/n).read_bytes(),b)
        m = json.loads((self.dest/'MANIFEST.json').read_text())
        self.assertEqual(len(m['entries']),2)
        self.assertEqual(m['gross_original_bytes'],sum(map(len,self.payloads.values())))
        mod.prune(self.dest)
        self.assertEqual(list(self.cloud.iterdir()),[])
        mod.restore(self.dest)
        for n,b in self.payloads.items(): self.assertEqual((self.cloud/n).read_bytes(),b)

    def test_symlink_source_rejected_without_original_delete(self):
        (self.cloud/'000001_10.npy').unlink()
        (self.cloud/'000001_10.npy').symlink_to(self.audit)
        with self.assertRaises(RuntimeError): self.archive()
        self.assertTrue((self.cloud/'000001_10.bin').exists())

    def test_rename_identity_change_blocks_prune(self):
        self.archive()
        p=self.cloud/'000001_10.npy'; data=p.read_bytes(); p.unlink(); p.write_bytes(data)
        with self.assertRaises(RuntimeError): mod.prune(self.dest)
        self.assertEqual(len(list(self.cloud.iterdir())),2)

    def test_corrupt_compressed_bytes_block_prune(self):
        self.archive()
        p=self.dest/'off97a7_cloud_arrays.tar.zst'
        with p.open('r+b') as f: f.seek(5); f.write(b'INVALID')
        with self.assertRaises(RuntimeError): mod.prune(self.dest)
        self.assertEqual(len(list(self.cloud.iterdir())),2)

    def test_clean_stop_required(self):
        p=self.run/'pipeline_normal_stop.json'; d=json.loads(p.read_text())
        d['summary']['closed_rejections']=1; p.write_text(json.dumps(d))
        with self.assertRaises(RuntimeError): self.archive()
        self.assertFalse((self.dest/'off97a7_cloud_arrays.tar.zst').exists())

    def test_live_saved_identity_rejected(self):
        fields=Path(f'/proc/{os.getpid()}/stat').read_text().rsplit(')',1)[1].split()
        p=self.run/'pipeline_normal_stop.json'; d=json.loads(p.read_text())
        d['saved_identity']=dict(pid=os.getpid(),start_ticks=int(fields[19])); p.write_text(json.dumps(d))
        with self.assertRaises(RuntimeError): self.archive()

    def test_restore_partial_prune_and_no_overwrite(self):
        self.archive()
        (self.cloud/'000001_10.npy').unlink()
        mod.restore(self.dest)
        for n,b in self.payloads.items(): self.assertEqual((self.cloud/n).read_bytes(),b)
        (self.cloud/'000001_10.bin').write_bytes(b'other data')
        with self.assertRaises(RuntimeError): mod.restore(self.dest)
        self.assertEqual((self.cloud/'000001_10.bin').read_bytes(),b'other data')

    def test_operator_audit_flag_required(self):
        with patch('sys.argv',['tool','archive','--archive-root',str(self.dest),'--audit-receipt',str(self.audit)]):
            with self.assertRaises(RuntimeError): mod.main()
        self.assertFalse((self.dest/'MANIFEST.json').exists())


if __name__=='__main__': unittest.main()
