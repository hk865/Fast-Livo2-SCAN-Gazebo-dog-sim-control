#!/usr/bin/env python3
"""Pure evidence safety/integrity tests; does not load Teacher, ROS or simulator."""
import copy
import hashlib
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

if not __package__:sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
from experiments.recording.bounded_binary import BoundedBinaryWriter, read_records
from experiments.recording.worker_logging import RecordingContext
from experiments.recording.raw_capture import selected_windows, DEFAULT_WINDOWS

NATIVE_FIELDS=('world_sim_time','state_physics_world_time','body_lin_vel','body_ang_vel','rpy',
    'qd','applied_torque','q_target','fault','actor_inferred_this_frame','body_clearance','contacts','command','command_expired')

def evidence(sequence=42):
    envelope=dict(schema_version=1,sequence=sequence,mode='scan_slam',source='scan_slam',
        sim_time=209.5,monotonic_wall=12345.,healthy=True,command=[.1,0.,.2],
        valid_for_wall_s=.3,acceptance=dict(sha256='f'*64,large_frozen_contract='x'*40000),
        controller_source=dict(source_pose_stamp_ns=209480000000,paired_imu_stamp_ns=209475000000,
            control_stamp_ns=209500000000,command_after_slew=[.1,0.,.2],navigation_ground_truth_used=False))
    raw=json.dumps(envelope,sort_keys=True,separators=(',',':'))
    return dict(sim_time=209.5,physics_clock_ns=209500000000,command=[.1,0.,.2],rejected=False,
        reason='Fresh actual source',navigation_ground_truth_used=False,actual_original_envelope=envelope,
        actual_read_attempt=dict(read_clock_ns=209500000000,read_monotonic_wall=12345.,
            raw_utf8=raw,raw_bytes_sha256=hashlib.sha256(raw.encode()).hexdigest(),decoded_envelope=envelope,
            status='accepted',reason='Fresh actual source',producer_sim_age_s=.02,producer_wall_age_s=.01))

class Tests(unittest.TestCase):
    def test_native_telemetry_untouched_exact_command_bytes_deduplicated(self):
        with tempfile.TemporaryDirectory() as td:
            r=RecordingContext(td);e=evidence();before=copy.deepcopy(e)
            row={k:[k] for k in NATIVE_FIELDS};row['closed_loop_read_evidence']=e
            original=copy.deepcopy(row);small=r.project_telemetry(row)
            again=r.project_telemetry(small)
            for k in NATIVE_FIELDS:self.assertEqual(small[k],row[k])
            self.assertEqual(row,original);self.assertEqual(e,before)
            self.assertEqual(again,small)
            projected=small['closed_loop_read_evidence']
            self.assertEqual(projected['actual_read_attempt']['producer_sim_age_s'],.02)
            self.assertEqual(projected['command_envelope_projection']['controller_source'],e['actual_original_envelope']['controller_source'])
            self.assertEqual(projected['command_envelope_projection']['acceptance']['sha256'],'f'*64)
            self.assertLess(len(json.dumps(small)),len(json.dumps(row))//10)
            r.project_evidence(e);result=r.close()
            records=list(read_records(Path(td)/'bounded_worker_evidence/original_commands.bin'))
            self.assertEqual(len(records),1)
            self.assertEqual(records[0][1],e['actual_read_attempt']['raw_utf8'].encode())
            self.assertTrue(result['original_commands']['capture_complete'])

    def test_sequence_or_raw_hash_changes_record_actual_read(self):
        with tempfile.TemporaryDirectory() as td:
            r=RecordingContext(td);r.project_evidence(evidence(1));r.project_evidence(evidence(2));r.close()
            records=list(read_records(Path(td)/'bounded_worker_evidence/original_commands.bin'))
            self.assertEqual([x[0]['command_sequence'] for x in records],[1,2])

    def test_height_projection_has_explicit_window_and_no_mutation(self):
        with tempfile.TemporaryDirectory() as td:
            r=RecordingContext(td);row=dict(sim_time=11.,height_scan_raw=[1]*187,height_scan_hit_z=[2]*187,
                height_scan_valid_mask=[True]*187,height_scan_hit_models=['floor_2']*187,height_scan_valid_count=187)
            before=copy.deepcopy(row)
            self.assertIn('height_scan_raw',r.project_height(row))
            outside=r.project_height({**row,'sim_time':100.})
            self.assertNotIn('height_scan_raw',outside)
            self.assertFalse(outside['detailed_height_source_recorded'])
            self.assertEqual(row,before);r.close()

    def test_only_optional_height_path_redirected(self):
        with tempfile.TemporaryDirectory() as td:
            r=RecordingContext(td);legacy=SimpleNamespace(Path=Path);r.install_height_path(legacy)
            run=legacy.Path(td);sink=(run/'height_scan_sources.jsonl').open('w',buffering=1)
            sink.write('{"sim_time":11.0}\n');sink.close()
            with (run/'state.json').open('w') as f:f.write('real synchronous state')
            self.assertEqual((Path(td)/'state.json').read_text(),'real synchronous state')
            self.assertFalse((Path(td)/'height_scan_sources.jsonl').exists())
            result=r.close();self.assertEqual(result['height_sources']['written'],1)

    def test_hard_capacity_reports_failed_capture_without_blocking(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'small.bin';w=BoundedBinaryWriter(p,maximum_file_bytes=10000,maximum_queue_bytes=12000)
            self.assertFalse(w.submit({'test':'too large'},b'x'*20000));s=w.close()
            self.assertEqual(s['dropped'],1);self.assertFalse(s['capture_complete'])
            self.assertLessEqual(p.stat().st_size,10000)

    def test_tamper_and_truncation_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'record.bin';w=BoundedBinaryWriter(p,maximum_file_bytes=100000)
            self.assertTrue(w.submit({'original':'sensor'},b'original bytes'));w.close()
            data=p.read_bytes();p.write_bytes(data[:-1]+bytes([data[-1]^1]))
            with self.assertRaises(ValueError):list(read_records(p))
            p.write_bytes(data[:-1])
            with self.assertRaises(ValueError):list(read_records(p))

    def test_source_time_windows_are_bounded(self):
        self.assertEqual(selected_windows(3000000000,DEFAULT_WINDOWS),[0])
        self.assertEqual(selected_windows(4500000000,DEFAULT_WINDOWS),[0])
        self.assertEqual(selected_windows(100000000000,DEFAULT_WINDOWS),[])
        self.assertEqual(selected_windows(215000000000,DEFAULT_WINDOWS),[2])
        self.assertEqual(selected_windows(215000000001,DEFAULT_WINDOWS),[])

if __name__=='__main__':
    suite=unittest.defaultTestLoader.loadTestsFromTestCase(Tests)
    result=unittest.TextTestRunner(verbosity=2).run(suite)
    here=Path(__file__).resolve().parent
    receipt=dict(schema='evidence_runtime_v32_pure_tests/v1',tests_run=result.testsRun,
        failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful(),
        physics_or_model_or_ros_started=False,
        validated=['Exact native telemetry fields preserved','Original command raw bytes/hash retained once per change',
            'No command/evidence object mutation','Optional path interception excludes synchronous state/telemetry',
            'Hard byte limit and loss fail capture','Tamper/truncation detection','Source-time windows'],
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in here.glob('*.py')})
    (here/'PURE_TEST_RECEIPT.json').write_text(json.dumps(receipt,indent=2)+'\n')
    raise SystemExit(0 if result.wasSuccessful() else 1)
