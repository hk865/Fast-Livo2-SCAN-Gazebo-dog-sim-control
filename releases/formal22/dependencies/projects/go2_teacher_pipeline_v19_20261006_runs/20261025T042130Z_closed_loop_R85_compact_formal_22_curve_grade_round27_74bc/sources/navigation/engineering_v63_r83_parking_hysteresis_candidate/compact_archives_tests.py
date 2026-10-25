"""No ROS, model, simulation, socket or GPU: reversible archives and live-writer isolation."""
from pathlib import Path
import copy
import io
import json
import sys
import tempfile
import subprocess
import threading
import time
import unittest

sys.path.append('/opt/experiment/workspace/multifloor_demo/teacher_mode/navigation')
from compact_archives import (Codec, decode, context, compact_path, configured,
    make_evidence_writer, make_command_writer, FORMAT, STATIC_PATHS, open_history)
from runtime_io import LatestCommandWriter, EvidenceWriter

PROFILE = {'engineering_recording': {'compact_histories': dict(enabled=True, codec=FORMAT,
    streams=list(STATIC_PATHS), original_JSON_values_preserved=True, live_inputs_unchanged=True)}}


class ArchiveTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix='v33_archive_test_'); self.run = Path(self.temp.name)
        (self.run/'input_profile.json').write_text(json.dumps(PROFILE))
        (self.run/'source_manifest.json').write_text('{"source":"CPU test only"}')
    def tearDown(self): self.temp.cleanup()
    def stream(self, rows, name='navigation_command_history.jsonl'):
        codec = Codec(name, context(self.run)); raw = ''.join(codec.encode(r) for r in rows)+codec.footer()
        return raw
    def test_all_original_values_and_type_changes_roundtrip(self):
        rows = [dict(sequence=i, acceptance={'profile': {'Unicode':'层', 'value':v}},
            command=[.2, -0.0, v], dynamic={'$compact_ref':'literal original data'})
            for i,v in enumerate([1, 1.0, True, -0.0, 0.0, None])]
        raw = self.stream(rows); report = {}
        result = list(decode(io.StringIO(raw), expected_sources=context(self.run), integrity=report))
        self.assertEqual([json.dumps(r) for r in result], [json.dumps(r) for r in rows])
        self.assertTrue(report['archive_integrity_complete']); self.assertFalse(report['formal_acceptance_allowed'])
    def test_original_objects_untouched_and_context_once(self):
        row = dict(acceptance={'profile':{'large':'A'*4096}}, command=[.1,0,0]); before=copy.deepcopy(row)
        raw=self.stream([row]*10); self.assertEqual(row,before)
        self.assertEqual(sum(json.loads(l)['kind']=='dictionary' for l in raw.splitlines()),1)
        self.assertEqual(len(list(decode(io.StringIO(raw)))),10)
    def test_strict_source_identity_and_frame_corruption(self):
        raw=self.stream([dict(acceptance={'a':1},command=[.1,0,0])])
        with self.assertRaises(ValueError):list(decode(io.StringIO(raw),expected_sources={'wrong':'source'}))
        lines=[json.loads(l) for l in raw.splitlines()];next(r for r in lines if r['kind']=='frame')['data']['command'][0]=.9
        with self.assertRaises(ValueError):list(decode([json.dumps(r)+'\n' for r in lines]))
    def test_missing_reference_and_reordered_frame_refused(self):
        raw=self.stream([dict(acceptance={'a':1},sequence=i) for i in range(2)])
        rows=[json.loads(l) for l in raw.splitlines()];frames=[r for r in rows if r['kind']=='frame']
        frames[0]['references'][0]['sha256']='f'*64
        with self.assertRaises((KeyError,ValueError)):list(decode([json.dumps(r)+'\n' for r in rows]))
        raw=self.stream([dict(acceptance={'a':1},sequence=i) for i in range(2)])
        rows=[json.loads(l) for l in raw.splitlines()];frames=[r for r in rows if r['kind']=='frame'];frames[0]['index']=2
        with self.assertRaises(ValueError):list(decode([json.dumps(r)+'\n' for r in rows]))
    def test_truncated_tail_explicit_prefix_only(self):
        row=dict(acceptance={'p':1},sequence=1);raw=self.stream([row]);raw=raw[:raw.rfind('{"kind": "footer"')]+'{"kind":"fra'
        with self.assertRaises((ValueError,json.JSONDecodeError)):list(decode(io.StringIO(raw)))
        report={};self.assertEqual(list(decode(io.StringIO(raw),require_complete=False,integrity=report)),[row])
        self.assertTrue(report['truncated_tail_record']);self.assertFalse(report['archive_integrity_complete'])
    def test_malformed_middle_never_silently_skipped(self):
        raw=self.stream([dict(acceptance={'p':1})]);raw=raw.replace('\n','\n{bad}\n',1)
        with self.assertRaises(json.JSONDecodeError):list(decode(io.StringIO(raw),require_complete=False))
    def test_data_after_footer_never_marked_complete(self):
        raw=self.stream([dict(acceptance={'p':1})])+'{"kind":"footer"}\n';report={}
        with self.assertRaises(ValueError):list(decode(io.StringIO(raw),integrity=report))
        self.assertFalse(report['archive_integrity_complete'])
    def test_restore_CLI_refuses_input_alias_and_existing_receipt(self):
        source=self.run/'navigation_command_history.compact.jsonl';raw=self.stream([dict(acceptance={'p':1})]);source.write_text(raw)
        tool=Path(__file__).with_name('restore_compact_history.py')
        result=subprocess.run([sys.executable,'-B',str(tool),str(source),str(self.run/'restored.jsonl'),'--receipt',str(source)],capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0);self.assertEqual(source.read_text(),raw)
        receipt=self.run/'existing.json';receipt.write_text('unique evidence')
        result=subprocess.run([sys.executable,'-B',str(tool),str(source),str(self.run/'restored.jsonl'),'--receipt',str(receipt)],capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0);self.assertEqual(receipt.read_text(),'unique evidence')
    def test_restore_CLI_prefix_and_output_limit_receipts(self):
        source=self.run/'navigation_command_history.compact.jsonl';raw=self.stream([dict(acceptance={'p':1},command=[.2,0,0])]);source.write_text(raw)
        tool=Path(__file__).with_name('restore_compact_history.py');receipt=self.run/'receipt.json'
        result=subprocess.run([sys.executable,'-B',str(tool),str(source),str(self.run/'restored.jsonl'),'--receipt',str(receipt),'--max-output-bytes','1'],capture_output=True,text=True)
        self.assertNotEqual(result.returncode,0);self.assertFalse(json.loads(receipt.read_text())['restore_completed'])
        source.write_text(raw[:raw.rfind('{"kind": "footer"')]+'{"kind":"fra')
        receipt=self.run/'prefix_receipt.json';output=self.run/'prefix_restored.jsonl'
        result=subprocess.run([sys.executable,'-B',str(tool),str(source),str(output),'--receipt',str(receipt),'--allow-prefix'],capture_output=True,text=True)
        self.assertEqual(result.returncode,0);d=json.loads(receipt.read_text());self.assertTrue(d['truncated_tail_record'])
        self.assertFalse(d['archive_integrity_complete']);self.assertFalse(d['formal_acceptance_allowed'])
    def test_evidence_latest_and_other_ledgers_identical(self):
        writer=make_evidence_writer(self.run,PROFILE);data=dict(command=[0,0,0],acceptance={'p':1})
        writer.atomic(self.run/'latest.json',data);writer.append(self.run/'navigation_guard_history.jsonl',data)
        writer.append(self.run/'navigation_status.jsonl',data);writer.close()
        expected=json.dumps(data,allow_nan=False,ensure_ascii=False)+'\n'
        self.assertEqual((self.run/'latest.json').read_text(),expected)
        self.assertEqual((self.run/'navigation_guard_history.jsonl').read_text(),expected)
        self.assertFalse((self.run/'navigation_status.jsonl').exists())
        self.assertEqual(list(open_history(self.run/'navigation_status.jsonl')),[data])
    def test_command_native_loop_inherited_and_zero_preserved(self):
        class LiveWriter:
            def __init__(self):self.records=[];self.written=threading.Event()
            def write(self,value):self.records.append(copy.deepcopy(value));self.written.set()
        live=LiveWriter();transport=make_command_writer(live,self.run/'navigation_command_history.jsonl',{'profile':PROFILE})
        self.assertIs(transport._run.__func__,LatestCommandWriter._run)
        move=dict(sequence=1,monotonic_wall=time.monotonic(),command=[.2,0,0],acceptance={'profile':PROFILE})
        stop=dict(sequence=2,monotonic_wall=time.monotonic(),command=[0,0,0],acceptance={'profile':PROFILE})
        transport.submit(move);self.assertTrue(live.written.wait(1));transport.close(stop)
        restored=list(open_history(self.run/'navigation_command_history.jsonl'))
        self.assertEqual([r['command'] for r in restored],[r['command'] for r in live.records])
        self.assertEqual(restored[-1]['command'],[0,0,0]);self.assertEqual(move['command'],[.2,0,0])
        self.assertIn('transport_write_duration_wall_s',restored[-1])
    def test_disabled_factory_and_failure_remain_explicit(self):
        writer=make_evidence_writer(self.run,{});self.assertIs(type(writer),EvidenceWriter);writer.close()
        with self.assertRaises(ValueError):configured({'engineering_recording':{'compact_histories':{'enabled':1}}})
        writer=make_evidence_writer(self.run,PROFILE);writer.error='injected original writer failure'
        with self.assertRaises(RuntimeError):writer.append(self.run/'navigation_status.jsonl',{})
        with self.assertRaises(RuntimeError):writer.close()
        self.assertFalse(compact_path(self.run/'navigation_status.jsonl').exists())


if __name__=='__main__':unittest.main()
