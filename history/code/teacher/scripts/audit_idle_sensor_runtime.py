#!/usr/bin/env python3
"""Read original idle trials; independently recompute forward/stop and receipts."""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import re

import numpy as np


def read(path):
    return json.loads(Path(path).read_text())


def lines(path):
    return [json.loads(s) for s in Path(path).read_text().splitlines() if s.strip()]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(v):
    v = np.asarray(v, dtype=float)
    return dict(p50=float(np.quantile(v, .5)), p95=float(np.quantile(v, .95)), max=float(v.max())) if len(v) else None


def audit(run, baseline):
    rows = lines(run / 'telemetry.jsonl')
    native = lines(run / 'actuator.jsonl')
    steps = [r for r in native if r.get('kind') == 'physics_step']
    meta, runtime, result = [read(run / f) for f in ['policy_manifest.json', 'runtime_manifest.json', 'worker_result.json']]
    protocol = read(run / 'sources/tests/protocol.json')
    t = np.array([r['sim_time'] for r in rows])
    position, command, measured, attitude = [np.array([r[k] for r in rows]) for k in ['position', 'command', 'measured', 'rpy']]
    track = (t >= 5) & (t < 10)
    stop = (t >= 15) & (t <= 18)
    forward = measured[track].mean(axis=0)
    reference = command[track].mean(axis=0)
    tracking_rmse = np.sqrt(np.mean((measured[track] - command[track])**2, axis=0))
    yaw = np.unwrap(attitude[stop, 2])
    stop_rms = np.sqrt(np.mean(measured[stop]**2, axis=0))
    stop_endpoint_drift = float(np.linalg.norm(position[stop][-1, :2] - position[stop][0, :2]))
    stop_max_excursion = float(np.linalg.norm(position[stop][:, :2] - position[stop][0, :2], axis=1).max())
    stop_yaw_endpoint = float(abs(yaw[-1] - yaw[0]))
    stop_yaw_max_excursion = float(abs(yaw - yaw[0]).max())
    lim = protocol['stand_stop']
    vlim = protocol['tracking']
    observed_ages = {key: [] for key in ['imu', 'joints']}
    samples = lines(run / 'sensor_feedback_samples.jsonl')
    for key in observed_ages:
        stream = [s for s in samples if s['stream'] == key and 3005000000 <= s['stamp_ns'] <= 18005000000]
        observed_ages[key] = stats(np.diff([s['received_monotonic_wall'] for s in stream]))
    gpu_samples = [list(map(int, re.match(r'(\d+) MiB, (\d+) %', r['gpu']).groups())) for r in runtime.get('gpu_resource_samples', [])]
    roles = {r['role']: r['returncode'] for r in runtime['owned_processes']}
    nt = np.array([r['t'] for r in steps])
    old_meta = read(baseline / 'policy_manifest.json')
    checks = {
        'cpu_actor_single_thread': meta['inference_device'] == 'cpu' and meta['torch_threads'] == 1,
        'same_frozen_model': meta['checkpoint_sha256'] == old_meta['checkpoint_sha256'],
        'all_five_owned_processes_clean_exit': set(roles) == {'worker', 'gazebo', 'bridge', 'capture', 'sensor_feedback'} and all(v == 0 for v in roles.values()),
        'runtime_and_policy_without_fault': runtime.get('error') is None and result['fault'] is None and not any(r.get('fault') for r in rows),
        'complete_18s_50Hz_policy': len(rows) == 901 and t[-1] == 18 and np.diff(t).min() > 0 and np.diff(t).max() <= .020001,
        'complete_200Hz_physics_and_safe_termination': nt[-1] >= rows[-1]['world_sim_time'] and np.diff(nt).min() > 0 and np.diff(nt).max() <= .005001 and steps[-1]['mode'] == 1 and steps[-1]['terminating'] is True,
        'no_native_latched_fault': not any(r.get('kind') == 'fault' for r in native) and not any(r.get('fault') for r in steps),
        'same_world_geometry_and_physics_bytes': sha(run / 'world.sdf') == sha(baseline / 'world.sdf'),
        'same_frozen_motion_parking_thresholds': sha(run / 'sources/tests/protocol.json') == sha(baseline / 'sources/tests/protocol.json'),
        'same_broker_and_observation_contract': all(sha(run / f) == sha(baseline / f) for f in ['sources/scripts/sensor_feedback.py', 'sources/policy/observation.py', 'sources/policy/contract.json']),
        'same_native_actuator_binary': runtime['plugin_sha256'] == read(baseline / 'runtime_manifest.json')['plugin_sha256'],
        'same_strict_source_ttl_and_activation': meta['actual_sensor_replacement'] == old_meta['actual_sensor_replacement'],
        'requested_direction_and_speed': forward[0] >= reference[0] * vlim['minimum_requested_axis_ratio'],
        'steady_tracking_rmse': tracking_rmse[0] <= vlim['linear_rmse_mps'] and tracking_rmse[1] <= vlim['cross_linear_rms_mps'] and tracking_rmse[2] <= vlim['cross_yaw_rms_radps'],
        'continuous_teacher_zero_command_stop': stop.sum() == 151 and abs(command[stop]).max() == 0 and all(r['actor_inferred_this_frame'] and r['state'] not in ['support_hold', 'support_capture'] for r, s in zip(rows, stop) if s),
        'frozen_legacy_stop_metrics': stop_endpoint_drift <= lim['translation_drift_m'] and stop_yaw_endpoint <= lim['yaw_drift_rad'] and stop_rms[:2].max() <= lim['linear_rms_mps'] and stop_rms[2] <= lim['yaw_rate_rms_radps'],
        'additional_max_excursion_stop_check': stop_max_excursion <= lim['translation_drift_m'] and stop_yaw_max_excursion <= lim['yaw_drift_rad'],
        'no_unknown_or_body_contacts': all(r['contacts'][0] == 0 and min(r['contacts']) >= 0 for r in steps),
    }
    checks = {key: bool(value) for key, value in checks.items()}
    return {'run': str(run.resolve()), 'status': 'verified' if all(checks.values()) else 'failed', 'checks': checks,
            'runtime_exit_codes': roles, 'steady_window_s': [5, 10], 'steady_command_mean': reference.tolist(), 'actual_steady_mean': forward.tolist(), 'steady_rmse': tracking_rmse.tolist(),
            'position_delta_m': (position[-1] - position[0]).tolist(), 'teacher_stop_window_s': [15, 18], 'stop_endpoint_translation_m': stop_endpoint_drift,
            'stop_max_translation_excursion_m': stop_max_excursion, 'stop_endpoint_yaw_rad': stop_yaw_endpoint, 'stop_max_yaw_excursion_rad': stop_yaw_max_excursion,
            'stop_velocity_rms': stop_rms.tolist(), 'thresholds': {k: protocol[k] for k in ['motion_limits', 'tracking', 'stand_stop']},
            'callback_receive_interval_after_activation_s': observed_ages, 'gpu_sample_range': {'MiB': [min(s[0] for s in gpu_samples), max(s[0] for s in gpu_samples)], 'percent': [min(s[1] for s in gpu_samples), max(s[1] for s in gpu_samples)]},
            'source_hashes': {f: sha(run / f) for f in ['telemetry.jsonl', 'actuator.jsonl', 'policy_manifest.json', 'runtime_manifest.json', 'worker_result.json', 'sources/tests/protocol.json', 'sources/policy/worker.py', 'resources_before.json']}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', nargs='+', type=Path)
    parser.add_argument('--baseline', required=True, type=Path)
    parser.add_argument('--interrupted', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError('Keep older audit receipts; use a new output')
    interrupted = args.interrupted
    interruption = read(interrupted / 'runner_interruption_receipt.json')
    new_source = args.runs[0] / 'sources/policy/worker.py'
    old_source = args.baseline / 'sources/policy/worker.py'
    result = {'schema_version': 1, 'scope': 'Idle real-sensor forward repetitions; runtime/parking independent audit only',
              'runs': [audit(run, args.baseline) for run in args.runs],
              'worker_diff_vs_prior_v2_failure': ''.join(difflib.unified_diff(old_source.read_text().splitlines(True), new_source.read_text().splitlines(True), n=3)),
              'interrupted_run': {'run': str(interrupted.resolve()), 'status': 'runtime_unverified_not_counted', 'summary_present': (interrupted / 'summary.json').exists(),
                                  'runtime_manifest_present': (interrupted / 'runtime_manifest.json').exists(), 'receipt': interruption,
                                  'receipt_sha256': sha(interrupted / 'runner_interruption_receipt.json'), 'worker_result_sha256': sha(interrupted / 'worker_result.json'),
                                  'source_and_actor_audit': 'Not included in clean source audit; no fabricated complete-runtime pass'},
              'limits': ['Resources improved contemporaneously with repeated success; GPU utilization is correlated, not proved unique cause',
                         'Old wall-age failed trial retained; strict25ms/300msTTL and190privileged+27controller-known inputs retained',
                         'These trials only forward and stop; no other direction/terrain/complete perception or navigation conclusion'],
              'analyzer_sha256': sha(__file__)}
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'output': str(args.output), 'runs': [{ 'run': r['run'], 'status': r['status'], 'failed': [k for k,v in r['checks'].items() if not v]} for r in result['runs']]}))


if __name__ == '__main__':
    main()
