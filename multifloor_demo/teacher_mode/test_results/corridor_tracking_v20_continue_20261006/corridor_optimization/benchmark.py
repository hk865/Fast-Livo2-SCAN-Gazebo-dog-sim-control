#!/usr/bin/env python3
"""Bounded one-pass offline profiling; no ROS, actuators, truth, or source edits."""
import os
for name in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS', 'NUMEXPR_NUM_THREADS'):
    os.environ[name] = '1'
import argparse
import cProfile
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import pstats
import time

HERE = Path(__file__).resolve().parent
RUN = Path('/home/hyh001/projects/1.Project/go2_teacher_pipeline_v19_20261006_runs/20261006_130405_closed_loop_cascade_clock_hold_v20_exporter_prefix9_r1_39ea')
SAMPLES = {99: 'median recorded wall among stale-free rejections',
           187: 'median recorded wall among support rejections',
           316: 'median recorded wall among occupied rejections'}
RUNTIME_FIELDS = {'observation', 'recorded_clock_ns', 'active_path_id_at_receipt', 'control_authority'}

def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()

def digest(raw):
    return hashlib.sha256(raw).hexdigest()

def load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--variant', choices=('baseline', 'candidate'), required=True)
    args = parser.parse_args()
    output = HERE / (args.variant + '.json')
    if output.exists():
        raise RuntimeError('Refuse repeat timing or overwrite: ' + str(output))
    affinity = sorted(os.sched_getaffinity(0))
    os.sched_setaffinity(0, {affinity[-1]})
    os.nice(15)
    if not (RUN/'runtime_manifest.json').is_file():
        raise RuntimeError('Offline profile requires finished run')
    source_key = 'navigation/corridor_tracking_v20/corridor.py'
    source = RUN/'sources'/source_key
    source_sha = digest(source.read_bytes())
    if json.loads((RUN/'source_manifest.json').read_text())[source_key] != source_sha:
        raise RuntimeError('Frozen source binding mismatch')
    core = load_module(source, 'bounded_frozen_corridor')
    candidate_sha = None
    if args.variant == 'candidate':
        candidate = HERE/'candidate.py'
        candidate_sha = digest(candidate.read_bytes())
        core = load_module(candidate, 'bounded_candidate').install(core)
    inputs = {}
    with (RUN/'corridor_inputs.jsonl').open() as stream:
        for sequence, line in enumerate(stream, 1):
            if sequence in SAMPLES:
                inputs[sequence] = json.loads(line)
    expected = {}
    with (RUN/'corridor_certificates.jsonl').open() as stream:
        for line in stream:
            row = json.loads(line)
            sequence = row['observation']['sequence']
            if sequence in SAMPLES:
                expected[sequence] = row
    report = dict(schema='corridor_bounded_profile/v1', variant=args.variant,
        source=str(source), source_sha256=source_sha, candidate_sha256=candidate_sha,
        run=str(RUN), affinity=sorted(os.sched_getaffinity(0)), nice=os.getpriority(os.PRIO_PROCESS, 0),
        max_baseline_evaluations=3, evaluations_per_selected_input=1,
        timing_note='Both variants measured once with cProfile enabled; profiler overhead is included. Not a runtime latency benchmark.',
        navigation_acceptance='UNVERIFIED', control_authority=False, samples=[])
    for sequence, selection in SAMPLES.items():
        entry = inputs[sequence]
        snapshot_file = (RUN/entry['snapshot_file']).resolve()
        if not snapshot_file.is_relative_to(RUN):
            raise RuntimeError('Snapshot path escapes run')
        with gzip.open(snapshot_file, 'rb') as stream:
            raw = stream.read(2_000_001)
        if len(raw) > 2_000_000 or digest(raw) != entry['snapshot_sha256']:
            raise RuntimeError('Snapshot payload bound or digest mismatch')
        snapshot = json.loads(raw)
        profile = cProfile.Profile()
        wall = time.perf_counter_ns()
        thread = time.thread_time_ns()
        profile.enable()
        result = core.certify_corridor(entry['path'], snapshot, None, entry['actual_state'], entry['limits'], entry['compute_request_clock_ns'])
        profile.disable()
        cpu_ms = (time.thread_time_ns()-thread)/1e6
        wall_ms = (time.perf_counter_ns()-wall)/1e6
        pure = {k:v for k,v in expected[sequence].items() if k not in RUNTIME_FIELDS}
        calls = []
        for (filename, line, function), (primitive, total, self_time, cumulative, callers) in pstats.Stats(profile).stats.items():
            calls.append(dict(file=filename, line=line, function=function, primitive_calls=primitive,
                calls=total, self_profile_s=self_time, cumulative_profile_s=cumulative))
        profile.dump_stats(str(HERE/f'{args.variant}_{sequence:06d}.pstats'))
        report['samples'].append(dict(sequence=sequence, selection=selection,
            snapshot_file=entry['snapshot_file'], snapshot_raw_sha256=digest(raw),
            input_sha256=digest(canonical(entry)), surface_points=len(snapshot.get('surface_points_xyz', [])),
            wall_ms=wall_ms, thread_cpu_ms=cpu_ms, status=result['status'], reason=result['reason'],
            exact_archived_result_match=canonical(result)==canonical(pure), result_sha256=digest(canonical(result)),
            changed_fields=[k for k in sorted(set(result)|set(pure)) if result.get(k)!=pure.get(k)],
            by_cumulative=sorted(calls, key=lambda x:x['cumulative_profile_s'], reverse=True)[:30],
            by_self=sorted(calls, key=lambda x:x['self_profile_s'], reverse=True)[:30]))
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n')
    print(json.dumps(dict(output=str(output), source_sha256=source_sha,
        results=[{k:s[k] for k in ('sequence','wall_ms','thread_cpu_ms','exact_archived_result_match','changed_fields')} for s in report['samples']]), ensure_ascii=False))

if __name__ == '__main__':
    main()
