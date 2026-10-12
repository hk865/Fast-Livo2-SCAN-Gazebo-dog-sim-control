#!/usr/bin/env python3
"""Aggregate completed matched V4 short cases, preserving every original receipt."""
from pathlib import Path
from collections import Counter
import hashlib
import json

ROOT = Path(__file__).resolve().parents[1]
ANALYZER = '80546c42036e29ea2860f06f9146e3ad146692bd925bcee3e7022a397f131762'
SHARED_PROFILE_FIELDS = ['distance_m', 'max_speed_mps', 'max_lateral_speed_mps',
    'max_yaw_rate_radps', 'arrival_radius_m', 'arrival_control_radius_m',
    'arrival_height_half_span_m', 'dwell_sim_s', 'goal_timeout_sim_s', 'pose_cloud_timeout_s',
    'teacher_transition', 'fence', 'arrival_stop_policy', 'pid', 'spawn', 'duration_s']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load(path):
    return json.loads(Path(path).read_text())


def main():
    collected = []
    pending = []
    for controller in ('teacher', 'champ'):
        for run in sorted((ROOT / 'runs').glob(f'*navigation_{controller}_pid_v4_transport_r[123]_r1_*')):
            summary = run / 'summary_pid_navigation_independent.json'
            runtime = run / 'runtime_manifest.json'
            if not summary.exists() or not runtime.exists():
                pending.append(run.name)
                continue
            s = load(summary)
            if s['case_id'] != 'flat_short_1m_roundtrip' or s['analyzer_sha256'] != ANALYZER:
                raise ValueError('Unexpected case or analysis version: ' + run.name)
            r = load(runtime)
            profile = load(run / 'navigation_profile.json')
            freeze = load(run / 'pid_navigation_freeze.json')
            route = s['metrics']['route']
            callback = s['metrics']['actual_cloud_callback_recording']
            collected.append({'controller': controller, 'run': run.name, 'status': s['status'],
                'checks': dict(Counter(v['status'] for v in s['checks'].values())),
                'required_check_status': {k: s['checks'][k]['status'] for k in s['required_checks']},
                'raw_slam_arrivals': s['metrics']['arrivals'], 'physical_route': route,
                'forward_motion': s['checks']['forward_motion_during_drive'],
                'fixed_final_parking': s['checks']['fixed_final_zero_command_parking'],
                'physical_safety': s['checks']['physical_safety_and_clearance'],
                'actual_force_velocity_limits': s['checks']['native_force_and_joint_velocity_limits'],
                'actual_force_phase': s['checks'].get('native_force_capture_phase_and_pairing'),
                'TTL': s['checks']['causal_command_TTL_and_slew'],
                'actual_cloud_callback_recording': callback,
                'source_and_exit': {'all_owned_returncode_zero': all(p['returncode'] == 0 for p in r['owned_processes']),
                    'roles': r['owned_processes'], 'actual_source_provenance': s['checks']['source_provenance'],
                    'clean_launch_children': s['checks']['complete_runtime_and_clean_exit']['launch_clean_children'],
                    'case_sha256': sha(run / 'pid_navigation_case.json'), 'protocol_sha256': sha(run / 'pid_navigation_protocol.json'),
                    'source_ref_hashes': freeze['source_hashes'], 'profile': profile,
                    'cloud_XML_sha256': sha(run / 'cloud_transport.xml'),
                    'segment_capacity_bytes': load(run / 'cloud_transport_manifest.json')['segment_capacity_bytes']},
                'original_summary_filename': str(summary), 'original_summary_sha256': sha(summary),
                'original_arrays_sha256': sha(run / 'pid_navigation_independent_arrays.npz'),
                'original_input_hashes': s['input_hashes'], 'original_receipt_kept': True})
    grouped = {k: [c for c in collected if c['controller'] == k] for k in ('teacher', 'champ')}
    stages = {}
    for k, cases in grouped.items():
        same = bool(cases) and all(c['source_and_exit']['source_ref_hashes'] == cases[0]['source_and_exit']['source_ref_hashes']
            and c['source_and_exit']['profile'] == cases[0]['source_and_exit']['profile']
            and c['source_and_exit']['protocol_sha256'] == cases[0]['source_and_exit']['protocol_sha256']
            and c['source_and_exit']['case_sha256'] == cases[0]['source_and_exit']['case_sha256']
            and c['source_and_exit']['cloud_XML_sha256'] == cases[0]['source_and_exit']['cloud_XML_sha256'] for c in cases)
        good = sum(c['status'] == 'passed' for c in cases)
        stages[k] = {'completed_actual_runs': len(cases), 'passed_actual_runs': good,
            'same_frozen_version_case_profile_transport': same,
            'required_independent_runs': 3,
            'stage_status': 'passed' if len(cases) == 3 and good == 3 and same else 'failed' if any(c['status'] == 'failed' for c in cases) else 'unverified',
            'does_not_use_old_camera_demo_or_older_versions': True}
    if grouped['teacher'] and grouped['champ']:
        tp, cp = [grouped[k][0]['source_and_exit']['profile'] for k in ('teacher', 'champ')]
        matched = {k: tp.get(k) == cp.get(k) for k in SHARED_PROFILE_FIELDS}
    else:
        matched = {k: None for k in SHARED_PROFILE_FIELDS}
    result = {'schema': 'matched_pid_short_actual_campaign/v1', 'cases': collected,
        'stages': stages, 'pending_actual_run_directories': pending,
        'matched_outer_profile_fields': matched,
        'comparison_scope': 'Actual same flat1m-goal-center roundtrip, same scene/sensors/SCAN/PID and original physical/source/TTL/parking criteria; different bottom controllers and initial joint poses',
        'limits': ['Finite body-to-polyline error includes endpoint overrun, not only transverse distance.',
            'Goal centers1m apart do not mean physical body displacement exactly1m.',
            'Teacher PD25/.5+DCMotor envelope vs legacy CHAMP effort PID220.982919/.2/1; not the same motor model.',
            'Failed startup, missing execution cadence, heading or region evidence remains failed/unverified.',
            'Old CHAMP V3 cleared PostUpdate force cannot be combined with V4 captured force for acceptance.',
            'SLAM navigation and native physical routes are separate frames; only an immutable offline anchor registration may compare localization.',
            'Short-case stage pass does not establish6m, ramps, continuous multifloor, all Sim2Sim or hardware deployment.'],
        'continuous_multifloor': 'unverified', 'global_sim2sim': 'unverified', 'real_robot': 'unverified',
        'analyzer_sha256': sha(__file__), 'frozen_acceptance_analyzer_sha256': ANALYZER,
        'protocol_or_thresholds_changed': False, 'original_receipts_overwritten': False}
    out = ROOT / 'test_results/pid_matched_short_v4_20261004'
    out.mkdir(exist_ok=True)
    (out / 'comparison.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'stages': stages, 'matched_fields': matched, 'pending': pending}, indent=2))


if __name__ == '__main__':
    main()
