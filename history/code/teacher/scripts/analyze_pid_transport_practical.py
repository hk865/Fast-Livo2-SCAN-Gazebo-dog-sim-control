#!/usr/bin/env python3
"""Read actual finished PID transport logs; does not change frozen acceptance."""
from pathlib import Path
from collections import Counter
import hashlib
import json
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RUNS = ['20261004_132148_navigation_teacher_pid_v3_slew_r1_cddc',
        '20261004_135857_navigation_teacher_pid_v4_transport_r1_r1_8c2a']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def rows(path):
    return [json.loads(line) for line in Path(path).open() if line.strip()]


def stats(a):
    a = np.asarray(a, float)
    if not len(a):
        return None
    return {'min': float(a.min()), 'median': float(np.median(a)),
            'p95': float(np.percentile(a, 95)), 'max': float(a.max())}


def case(name):
    run = ROOT / 'runs' / name
    callbacks = rows(run / 'navigation_cloud_callbacks.jsonl')
    accepted = [r for r in callbacks if r['accepted']]
    gate = rows(run / 'navigation_sensor_gate_history.jsonl')
    execution = rows(run / 'telemetry.jsonl')
    summary = load(run / 'summary_pid_navigation_independent.json')
    clock = np.array([r['received_ros_clock_ns'] for r in callbacks], dtype=np.int64)
    stamps = np.array([r['producer_stamp_ns'] for r in callbacks], dtype=np.int64)
    wall = np.array([r['received_monotonic_wall'] for r in callbacks], float)
    reason = Counter(r['reason'] for r in callbacks)
    # Original callback/source age. Actual worker top-level age fields are not interchangeable.
    source_age = (clock - stamps) / 1e9
    stale = [r for r in gate if r.get('initial_warmup_complete') and not r.get('ready')]
    warm = [r for r in gate if r.get('initial_warmup_complete')]
    active = [r for r in execution if r['world_sim_time'] >= load(run / 'navigation_anchor.json')['pose_stamp_ns'] / 1e9]
    paths = [run / f for f in ['navigation_cloud_callbacks.jsonl', 'navigation_sensor_gate_history.jsonl',
                              'telemetry.jsonl', 'navigation_profile.json', 'pid_navigation_protocol.json',
                              'pid_navigation_case.json', 'pid_navigation_freeze.json',
                              'summary_pid_navigation_independent.json', 'navigation_anchor.json', 'runtime_manifest.json']]
    return {'run': name, 'status': summary['status'],
        'actual_cloud_callback_count': len(callbacks), 'accepted_callbacks': len(accepted),
        'original_reasons': dict(reason), 'callback_header_stamps_strictly_preserved': True,
        'cloud_header_gap_sim_s': stats(np.diff(stamps) / 1e9),
        'cloud_callback_gap_sim_s': stats(np.diff(clock) / 1e9),
        'cloud_callback_gap_wall_s': stats(np.diff(wall)),
        'rate_callbacks_per_sim_s': float((len(callbacks) - 1) / ((clock[-1] - clock[0]) / 1e9)),
        'rate_callbacks_per_wall_s': float((len(callbacks) - 1) / (wall[-1] - wall[0])),
        'source_age_at_actual_callback_s': stats(source_age),
        'callback_source_age_invalid': int(((source_age < -.05) | (source_age >= .3)).sum()),
        'callback_processing_wall_s': stats([r['callback_duration_wall_s'] for r in callbacks]),
        'sensor_gate': {'all_records': len(gate), 'ready_records': sum(bool(r.get('ready')) for r in gate),
            'post_initial_warmup_records': len(warm), 'post_initial_warmup_hold_records': len(stale),
            'hold_original_reason_counts': dict(Counter(r['reason'] for r in stale)),
            'coverage': 'Actual100ms gate snapshots; not every50Hz worker sample'},
        'worker_consumer': {'actual_samples': len(execution),
            'expired_or_unhealthy_samples': sum(bool(r.get('command_expired')) for r in execution),
            'post_anchor_samples': len(active),
            'post_anchor_expired_or_unhealthy_samples': sum(bool(r.get('command_expired')) for r in active),
            'original_reason_counts': dict(Counter(r.get('command_reason') for r in execution)),
            'expired_does_not_mean_pure_TTL': True},
        'fixed_acceptance': {'passed_checks': sum(r['status'] == 'passed' for r in summary['checks'].values()),
            'required_checks': len(summary['required_checks']),
            'region_count': sum(r.get('passed') is True for r in summary['metrics']['arrivals']),
            'physical_route': summary['metrics']['route'],
            'forward': summary['checks']['forward_motion_during_drive'],
            'parking': summary['checks']['fixed_final_zero_command_parking']},
        'input_hashes': {str(p): sha(p) for p in paths}}


def main():
    out = ROOT / 'test_results/cloud_transport_actual_20261004/pid_transport_practical_comparison.json'
    a, b = [ROOT / 'runs' / name for name in RUNS]
    cases = [case(name) for name in RUNS]
    af, bf = [load(r / 'pid_navigation_freeze.json')['source_hashes'] for r in (a, b)]
    different = {name: {'old': af.get(name), 'new': bf.get(name)} for name in sorted(set(af) | set(bf)) if af.get(name) != bf.get(name)}
    control_files = ['navigation/pid_mode/controller.py', 'navigation/pid_mode/pid_core.py',
        'navigation/pid_mode/shared_controller.py', 'navigation/pid_mode/teacher_wrapper.py',
        'navigation/pid_mode/bridge.py', 'navigation/pid_mode/sensor_gate.py',
        'policy/worker.py', 'policy/observation.py', 'simulation/teacher_actuator.cpp',
        'simulation/build/libteacher_actuator.so']
    result = {'schema': 'actual_pid_transport_practical_diagnostic/v1', 'cases': cases,
        'frozen_profile_case_protocol_exact_byte_equality': {n: sha(a / n) == sha(b / n) for n in
            ['navigation_profile.json', 'pid_navigation_case.json', 'pid_navigation_protocol.json']},
        'exact_control_source_ref_equality': {n: af.get(n) is not None and af.get(n) == bf.get(n) for n in control_files},
        'all_changed_runtime_source_references': different,
        'scope': 'Finished180sim PID diagnostics with same PID/actor/physical thresholds; no acceptance receipt overwritten',
        'causal_limitations': ['Historical LOCALHOST_ONLY1 implicit transport vs new custom-loopback transport changes discovery/transport construction as well as SHM capacity.',
            'This is not a pure capacity experiment; the paired-QoS stand A/B separately isolates that configuration difference.',
            'One passed PID run does not establish3repeat acceptance, long routes, original-width ramps or multifloor.',
            'No extrapolation from own-probe freshness to another node; these are actual PID callbacks, gate snapshots and worker receipts.',
            'Callback MessageInfo/publication sequence and complete original XYZ payload are unavailable.'],
        'analyzer_sha256': sha(__file__), 'protocol_thresholds_changed': False,
        'raw_data_changed': False, 'old_acceptance_receipts_overwritten': False}
    out.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(str(out))


if __name__ == '__main__':
    main()
