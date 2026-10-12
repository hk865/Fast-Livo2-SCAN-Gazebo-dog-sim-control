#!/usr/bin/env python3
"""Read-only comparison of actual V4 fsync and V4.1 atomic-file transport logs.

No simulation is started and no prior run summary is overwritten. Observed
timing differences are measurements, not a controlled proof of sole causation.
"""
import argparse
from collections import Counter
import difflib
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
NAMES = [
    '20261004_082258_navigation_slam_scan_roundtrip_v4_r1_9c33',
    '20261004_082715_navigation_slam_scan_roundtrip_v4_confirm_r1_1c57',
    '20261004_083026_navigation_slam_scan_roundtrip_v4_confirm_r2_bd19',
    '20261004_091122_navigation_slam_scan_atomic_transport_v41_r1_dba1',
]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def rows(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def quantiles(values):
    values = np.asarray(values, dtype=float)
    return dict(zip(['p50_s', 'p95_s', 'p99_s', 'maximum_s'],
                    map(float, np.percentile(values, [50, 95, 99, 100]))))


def evaluate(output):
    output.mkdir(parents=True, exist_ok=True)
    original = {}
    records = []
    arrays = {}
    for index, name in enumerate(NAMES):
        run = ROOT / 'runs' / name
        summary = read(run / 'summary_navigation_independent.json')
        commands = rows(run / 'navigation_command_history.jsonl')
        telemetry = rows(run / 'telemetry.jsonl')
        write = np.asarray([r['transport_write_duration_wall_s'] for r in commands])
        queue = np.asarray([r['transport_queue_delay_wall_s'] for r in commands])
        assert len(write) and np.isfinite(write).all() and (write >= 0).all()
        assert np.isfinite(queue).all() and (queue >= 0).all()
        # Actual worker flag means unhealthy OR a source timestamp outside the
        # original inclusive [-.05,.3] bounds. It is not a read_status enum.
        expired = sum(r.get('command_expired') is True for r in telemetry)
        envelopes = [r.get('navigation_envelope', {}) for r in telemetry]
        age_invalid = [not (-.05 <= float(e['wall_age_s']) <= .3 and
                            -.05 <= float(e['sim_age_s']) <= .3) for e in envelopes]
        unhealthy = [e.get('healthy') is not True for e in envelopes]
        status = rows(run / 'navigation_status.jsonl')
        world = (run / 'world.sdf').read_text().replace(str(run), '<RUN>').replace(name, '<RUNID>')
        checks = summary['checks']
        records.append({
            'run_id': name, 'transport': 'fsync_atomic_file' if index < 3 else 'flush_close_atomic_replace',
            'finite_navigation_status': summary['levels']['finite_flat_navigation'],
            'actual_written_envelopes': len(commands), 'policy_samples': len(telemetry),
            'write_duration_wall': quantiles(write), 'queue_delay_wall': quantiles(queue),
            'expired_policy_samples': expired, 'expired_policy_fraction': expired / len(telemetry),
            'worker_flag_semantics': 'command_expired is unhealthy OR original sim/wall source age out of bounds; not all are TTL age expiry',
            'source_age_invalid_samples': sum(age_invalid),
            'source_age_invalid_fraction': sum(age_invalid) / len(telemetry),
            'unhealthy_source_samples': sum(unhealthy),
            'age_invalid_and_unhealthy_samples': sum(a and b for a, b in zip(age_invalid, unhealthy)),
            'disjoint_command_reasons': {
                'source_age_invalid': sum(age_invalid),
                'unhealthy_but_source_age_valid': sum(b and not a for a, b in zip(age_invalid, unhealthy)),
                'accepted_controlled_stop_requested': sum(e.get('read_status') == 'accepted' and e.get('stop_requested') is True for e in envelopes),
                'accepted_without_stop_requested': sum(e.get('read_status') == 'accepted' and e.get('stop_requested') is False for e in envelopes),
            },
            'worker_read_status_counts': dict(Counter(e.get('read_status') for e in envelopes)),
            'worker_command_reason_counts': dict(Counter(r.get('command_reason') for r in telemetry)),
            'recorded_navigation_guard_snapshots': {
                'samples': len(status), 'obstacle_hold_samples': sum(r.get('obstacle_hold') is True for r in status),
                'protected_samples': sum(r.get('protected') is True for r in status),
                'message_counts': dict(Counter(r.get('message') for r in status)),
                'limitation': 'Stop requested may come from heading, dwell, arrival or sensor protection; it is not automatically a cloud obstacle stop. Snapshot counts are not worker-frame counts.',
            },
            'motion': checks['teacher_received_and_executed_motion'],
            'arrivals': [{'goal_id': a['goal_id'], 'start_stamp_ns': a['claimed_receipt']['start_stamp_ns'],
                          'end_stamp_ns': a['claimed_receipt']['stamp_ns'],
                          'measured_dwell_s': a['measured_stamp_dwell_s']} for a in summary['metrics']['arrivals']],
            'post_arrival_stop': checks['post_arrival_actual_teacher_stop'],
            'real_time_factor': summary['metrics']['execution_timing']['real_time_factor_observed']['simulation_seconds_per_wall_second'],
            'world_normalized_sha256': hashlib.sha256(world.encode()).hexdigest(),
            'input_sha256': {n: sha(run / n) for n in ['navigation_command_history.jsonl', 'telemetry.jsonl',
                'summary_navigation_independent.json', 'summary.json', 'world.sdf', 'runtime_manifest.json',
                'navigation_profile.json', 'source_manifest.json']},
        })
        arrays[f'run{index}_written_sim_time'] = np.asarray([r['sim_time'] for r in commands])
        arrays[f'run{index}_write_duration_wall_s'] = write
        arrays[f'run{index}_queue_delay_wall_s'] = queue
        for path in ['summary.json', 'summary_navigation_independent.json']:
            original[run / path] = (run / path).read_bytes()
    old, new = [ROOT / 'runs' / NAMES[i] for i in [0, 3]]
    same_sources = {}
    for folder in ['policy']:
        for path in sorted((old / 'sources' / folder).rglob('*.py')):
            rel = path.relative_to(old / 'sources')
            other = new / 'sources' / rel
            same_sources[str(rel)] = {'old_sha256': sha(path), 'new_sha256': sha(other),
                                      'identical': path.read_bytes() == other.read_bytes()}
    for name in ['simulation/teacher_actuator.cpp', 'simulation/prepare.py']:
        a, b = old / 'sources' / name, new / 'sources' / name
        same_sources[name] = {'old_sha256': sha(a), 'new_sha256': sha(b), 'identical': a.read_bytes() == b.read_bytes()}
    differences = {}
    for name in ['navigation/bridge.py', 'navigation/controller.py', 'navigation/scoped_profile.py',
                 'navigation/stack.launch.py', 'scripts/run_test.py']:
        a, b = old / 'sources' / name, new / 'sources' / name
        diff = ''.join(difflib.unified_diff(a.read_text().splitlines(keepends=True), b.read_text().splitlines(keepends=True),
                                          fromfile='V4/' + name, tofile='V4.1/' + name))
        (output / (name.replace('/', '__') + '.diff')).write_text(diff)
        differences[name] = {'old_sha256': sha(a), 'new_sha256': sha(b),
                             'diff_file': name.replace('/', '__') + '.diff'}
    common_world = len({r['world_normalized_sha256'] for r in records}) == 1
    common_profile = len({r['input_sha256']['navigation_profile.json'] for r in records}) == 1
    outcome = {
        'schema': 1, 'status': 'recorded_comparison', 'runs': records,
        'physical_world_same_after_run_path_normalization': common_world,
        'navigation_profile_byte_identical': common_profile, 'unchanged_teacher_sources': same_sources,
        'navigation_and_runner_source_differences': differences,
        'functional_difference': 'Writer removes os.fsync, retains write/flush/close and atomic replace. '
            'Controller scope string now comes from profile and has identical value for these ordinary flat runs. '
            'Other source differences introduce a separate dynamic branch, unused here.',
        'interpretation': 'The single V4.1 run has a shorter measured write-duration tail and fewer stale-or-unhealthy consumer frames. '
            'Load and callback scheduling differ between runs; this is not proof of a sole cause or elimination of TTL expiry. '
            'Queue maximum and all original timestamps remain reported. Power-loss durability is not an input requirement.',
        'erratum': 'The archived first comparison erroneously searched read_status==expired; that enum does not exist. Corrected to original command_expired flag and separately reports strict timestamp expiry versus unhealthy input. Prior incorrect script/receipt are preserved in incorrect_read_status_expired_v1; no source log or old independent receipt changed.',
        'global_sim2sim': 'failed', 'global_navigation': 'unverified',
        'original_receipt_bytes_preserved': all(p.read_bytes() == data for p, data in original.items()),
        'analyzer_sha256': sha(__file__),
    }
    np.savez_compressed(output / 'transport_actual_arrays.npz', **arrays)
    outcome['actual_arrays_sha256'] = sha(output / 'transport_actual_arrays.npz')
    (output / 'transport_comparison.json').write_text(json.dumps(outcome, indent=2, allow_nan=False) + '\n')
    return outcome


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'test_results/navigation_transport_v41_comparison')
    args = parser.parse_args()
    result = evaluate(args.output)
    print(json.dumps({'output': str(args.output), 'originals_preserved': result['original_receipt_bytes_preserved'],
                      'same_physical_world': result['physical_world_same_after_run_path_normalization'],
                      'same_profile': result['navigation_profile_byte_identical']}))
