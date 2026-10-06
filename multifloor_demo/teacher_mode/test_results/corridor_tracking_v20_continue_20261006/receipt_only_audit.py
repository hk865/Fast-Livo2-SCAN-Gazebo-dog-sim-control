#!/usr/bin/env python3
"""Bounded receipt-only corridor statistics. Never opens voxel/cloud payloads.

No pure corridor function is imported or replayed. Snapshot payload hashes are
cross-checked as receipt declarations only, not independently recomputed here.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import struct

R1 = Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_130405_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r1_39ea')
SOURCE_KEYS = ['navigation/corridor_tracking_v20/'+name for name in (
    'corridor.py', 'corridor_runtime.py', 'controller.py', 'cascade_core.py',
    'spatial_reference.py', 'scan_workspace.py', 'slam_workspace.py',
    'scan_ws/src/plan_env/src/corridor_snapshot.cpp',
    'scan_ws/src/plan_env/src/grid_map.cpp',
    'scan_ws/src/plan_env/include/plan_env/grid_map.h')]

def sha(raw):
    return hashlib.sha256(raw).hexdigest()

def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()

def distribution(values):
    a = sorted(values)
    if not a:
        return dict(count=0)
    return dict(count=len(a), min=a[0], median=statistics.median(a),
        p95=a[min(len(a)-1, round((len(a)-1)*.95))], max=a[-1], mean=statistics.fmean(a))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    os.sched_setaffinity(0, {max(os.sched_getaffinity(0))})
    os.nice(15)
    run = args.run.resolve()
    if args.output.exists() or args.output.resolve().is_relative_to(run):
        raise ValueError('Refuse overwrite or output inside immutable run')
    reads = {}
    def raw_read(path, limit=12_000_000):
        if path.stat().st_size > limit:
            raise ValueError('Receipt/source file exceeds byte bound: '+str(path))
        data = path.read_bytes()
        reads[str(path)] = dict(bytes=len(data), sha256=sha(data))
        return data
    def read_json(path):
        return json.loads(raw_read(path))
    def read_rows(name):
        raw = raw_read(run/name)
        lines = raw.splitlines()
        if len(lines)>10000 or any(len(line)>200000 for line in lines):
            raise ValueError('Receipt line/count bound exceeded')
        return [json.loads(line) for line in lines if line.strip()]
    manifest = read_json(run/'runtime_manifest.json')
    source_manifest_raw = raw_read(run/'source_manifest.json')
    source_manifest = json.loads(source_manifest_raw)
    old_sources = read_json(R1/'source_manifest.json')
    errors = []
    def check(ok, reason, sequence=None):
        if not ok:
            errors.append(dict(reason=reason, sequence=sequence))
    check(manifest['source_manifest_sha256']==sha(source_manifest_raw), 'runtime_source_manifest_hash')
    sources = []
    for key in SOURCE_KEYS:
        actual = sha(raw_read(run/'sources'/key))
        matches = actual==source_manifest.get(key)
        unchanged = actual==old_sources.get(key)
        check(matches, 'archived_source_hash:'+key)
        check(unchanged, 'source_changed_from_r1:'+key)
        sources.append(dict(key=key, sha256=actual, archived_binding_verified=matches, unchanged_from_r1=unchanged))
    loaded = {}
    for label in ('scan','slam'):
        name=label+'_loaded_binary.json'
        data=raw_read(run/name)
        receipt=json.loads(data)
        check(sha(data)==manifest[label+'_loaded_binary_receipt_sha256'],label+'_loaded_receipt_hash')
        check(receipt.get('verified') is True,label+'_loaded_not_verified')
        if label=='scan':
            expected=manifest['scan_binary_contract']['sha256']
            check(receipt['actual_executable_sha256']==expected,'scan_loaded_expected_sha')
            loaded[label]=dict(actual_executable_sha256=receipt['actual_executable_sha256'],expected_sha256=expected,
                verified=receipt['verified'], witness_before_physics=receipt.get('before_physics'))
        else:
            expected=manifest['slam_binary_contract']
            check(receipt['actual_executable_sha256']==expected['executable']['sha256'],'slam_loaded_expected_executable_sha')
            check(receipt['actual_core_sha256']==expected['core']['sha256'],'slam_loaded_expected_core_sha')
            loaded[label]=dict(actual_executable_sha256=receipt['actual_executable_sha256'],
                actual_core_sha256=receipt['actual_core_sha256'],verified=receipt['verified'],
                witness_before_physics=receipt.get('before_physics'))
    inputs=read_rows('corridor_inputs.jsonl')
    certificates=read_rows('corridor_certificates.jsonl')
    worker=read_rows('corridor_worker_receipt.jsonl')
    rejections=read_rows('corridor_input_rejections.jsonl')
    counters=defaultdict(Counter)
    metrics=defaultdict(list)
    sequence_seen=set()
    for cert in certificates:
        obs=cert['observation'];seq=obs['sequence']
        check(seq not in sequence_seen,'duplicate_observation_sequence',seq);sequence_seen.add(seq)
        if not 1<=seq<=len(inputs):
            check(False,'missing_input',seq);continue
        entry=inputs[seq-1];path=entry['path'];state=entry['actual_state'];limits=entry['limits']
        now=entry['compute_request_clock_ns'];received=cert['recorded_clock_ns']
        check(cert.get('control_authority') is False and cert.get('shadow_only') is True,'nonshadow_certificate',seq)
        check(entry.get('control_authority') is False,'input_authority',seq)
        check(obs.get('snapshot_sha256')==entry['snapshot_sha256'],'declared_snapshot_hash_binding',seq)
        check(obs.get('requested_clock_ns')==now,'request_clock_binding',seq)
        check(obs.get('source_pose_stamp_ns')==state['stamp_ns'],'pose_source_binding',seq)
        for name in ('path_id','path_sha256','frame_id','layer_id'):
            check(obs.get(name)==path.get(name),'observation_'+name,seq)
            if 'binding' in cert:
                check(cert['binding'].get(name)==path.get(name),'certificate_'+name,seq)
        if 'binding'in cert:
            packed=b''.join(struct.pack('<ddd',*point)for point in path['points_xyz'])
            check(cert['binding'].get('points_float64_sha256')==sha(packed),'path_point_bytes_binding',seq)
            check(cert['binding'].get('map_revision')==obs.get('map_revision'),'map_revision_binding',seq)
            check(cert['binding'].get('evaluated_at_ns')==now,'evaluated_clock_binding',seq)
        support=cert.get('support_diagnostic',{})
        if support:
            check(support.get('map_snapshot_sha256')==cert['binding'].get('map_snapshot_sha256'),'support_declared_map_hash_binding',seq)
        counters['status'][cert.get('status')]+=1
        counters['reason'][cert.get('reason')]+=1
        counters['interval_failure_counts'].update(cert.get('failure_counts',{}))
        counters['width_rejection_counts'].update(cert.get('width_rejection_counts',{}))
        counters['support_failure_counts'].update(support.get('failure_counts',{}))
        switched=cert.get('active_path_id_at_receipt')!=obs.get('path_id')
        counters['active_path_at_receipt']['obsolete'if switched else'current']+=1
        start=obs['compute_started_wall_ns'];finish=obs['compute_finished_wall_ns']
        compute=(finish-start)/1e6
        queue=(start-obs['snapshot_received_wall_ns'])/1e6
        metrics['compute_wall_ms'].append(compute);metrics['received_to_compute_started_wall_ms'].append(queue)
        counters['compute_wall_over_300ms'][str(compute>300)]+=1
        metrics['source_clock_advance_during_async_ms'].append((received-now)/1e6)
        for label,stamp,deadline in (
                ('pose',state.get('stamp_ns'),limits.get('state_max_age_ns',300000000)),
                ('cloud',support.get('source_cloud_stamp_ns'),limits.get('map_max_age_ns',300000000))):
            if not isinstance(stamp,int):
                counters[label+'_source_at_result_receipt']['not_in_receipts']+=1
                counters[label+'_missing_reason'][cert.get('reason')]+=1
                continue
            submit_age=now-stamp;receipt_age=received-stamp
            metrics[label+'_source_age_at_submit_ms'].append(submit_age/1e6)
            metrics[label+'_source_age_at_result_receipt_ms'].append(receipt_age/1e6)
            classification='future'if receipt_age<0 else'expired'if receipt_age>deadline else'within_original_deadline'
            counters[label+'_source_at_result_receipt'][classification]+=1
        if 'valid_until_ns'in cert:
            counters['certificate_valid_until_at_result_receipt']['expired'if received>cert['valid_until_ns']else'within_deadline']+=1
    check(sequence_seen==set(range(1,len(inputs)+1)),'complete_input_sequence_coverage')
    last=worker[-1]if worker else{}
    check(last.get('submitted')==len(inputs),'worker_submitted_count')
    check(last.get('completed')==len(certificates),'worker_completed_count')
    counters['input_rejection_reason'].update(x.get('reason')for x in rejections)
    ratios={}
    for label in ('pose','cloud'):
        c=counters[label+'_source_at_result_receipt'];known=sum(c[k]for k in ('future','expired','within_original_deadline'))
        ratios[label]=dict(expired=c['expired'],known_source_stamp=known,missing_source_stamp=c['not_in_receipts'],
            expired_fraction_of_known=None if not known else c['expired']/known,
            expired_fraction_lower_bound_of_all=None if not certificates else c['expired']/len(certificates))
    report=dict(schema='corridor_receipt_only_audit/v1',run=str(run),status='RECEIPT_BINDINGS_VERIFIED'if not errors else'RECEIPT_ERRORS',
        submitted_inputs=len(inputs),certificate_count=len(certificates),worker_final=last,
        counters={k:dict(v)for k,v in counters.items()},metrics={k:distribution(v)for k,v in metrics.items()},
        source_expiry=ratios,sources=sources,loaded_binary_witnesses=loaded,errors=errors,
        replay_evaluations=0,snapshot_payloads_opened=0,telemetry_or_cloud_array_files_opened=0,
        authority=False,navigation_acceptance='NOT_ASSESSED_BY_CORRIDOR_RECEIPTS',
        limitations=['Snapshot hashes are receipt declarations only; this audit does not read or rehash gzip payloads.',
            'Cloud source timestamps are available only where support_diagnostic contains them; missing timestamps are reported explicitly.',
            'Compute wall includes scheduler/GIL effects; no per-job thread CPU clock was recorded at runtime.',
            'interval/support counters are diagnostic counts, not a count of physical obstacles.',
            'Loaded binary witnesses are verified against their archived runtime-manifest bindings; binaries are not reread or executed.'],
        read_files=reads,script_sha256=sha(Path(__file__).read_bytes()))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(dict(output=str(args.output),report_sha256=sha(args.output.read_bytes()),status=report['status'],
        certificates=len(certificates),worker=last,counters=report['counters'],metrics=report['metrics'],source_expiry=ratios,errors=errors),ensure_ascii=False))

if __name__=='__main__':
    main()
