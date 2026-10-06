#!/usr/bin/env python3
"""Aggregate recorded evidence without rerunning or selectively hiding failures.

Default output is a review PREVIEW. --final explicitly writes acceptance.json.
No simulation, training, policy inference or per-run reevaluation is performed.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
from pathlib import Path
import re
import sys

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
START = '20261003_193541'
TESTS = ('stand', 'forward', 'backward', 'left', 'right', 'turn_positive', 'turn_negative',
         'walk_stop', 'command_timeout', 'switch', 'ramp_up', 'ramp_down', 'step05', 'step10')
FLAT = ('stand', 'forward', 'backward', 'left', 'right', 'turn_positive', 'turn_negative', 'switch', 'command_timeout')
TERRAIN = ('ramp_up', 'ramp_down', 'step05', 'step10')
REF_NAMES = {'turn_positive': 'yaw_positive', 'turn_negative': 'yaw_negative'}
TERRAIN_REFERENCE_DIRS = {'ramp_up': 'isaac_cpu_terrain_ramps_20261003_e',
    'ramp_down': 'isaac_cpu_terrain_ramps_20261003_e', 'step05': 'isaac_cpu_terrain_step05_20261003_f',
    'step10': 'isaac_cpu_terrain_step10_20261003_g'}
SHA = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'


def read(path):
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def digest(path):
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def level(summary, name):
    value = summary.get('levels', {}).get(name)
    if value in ('passed', 'failed'):
        return value
    return 'unverified'


def expected_asset(test):
    terrain = test if test in TERRAIN else 'flat'
    spawn = [4., 2., .6, 0.] if test == 'ramp_up' else [8., 2., 1., math.pi] if test == 'ramp_down' else [6., -.7, .4, 0.]
    return terrain, spawn


def categories(text):
    value = text.lower()
    result = []
    for name, pattern in (
        ('render_capture', r'ogre|egl|render|camera|capture|image'),
        ('process_signal_unknown', r'segmentation|segfault|\-11'),
        ('initialization', r'initializ|bootstrap|before.*ready|pd_init|spawn|no active motion'),
        ('insufficient_duration', r'before required duration|incomplete.*window|duration|wall timeout|ended early'),
        ('physical_fall_contact', r'fallen|clearance|roll.pitch|tilt|body.*contact|collision|fault_damping'),
        ('tracking_parking', r'tracking|drift|residual velocity|direction|speed ratio|stand.stop|parking'),
        ('terrain_crossing', r'ramp|step|tread|uphill|downhill|cross|progress'),
        ('assisted_stop', r'assisted|support_capture|support_hold|support hold'),
        ('transport_execution', r'socket|ipc|disconnect|transport|native.*fault|policy.*exit|gazebo.*exit|returncode'),
        ('interface_provenance', r'interface|hash|finite|dimension|source|provenance|evidence|50hz'),
    ):
        if re.search(pattern, value):
            result.append(name)
    return result or (['uncategorized'] if text else [])


def collect(runs, start=START):
    entries = []
    for directory in sorted(runs.iterdir() if runs.exists() else []):
        if not directory.is_dir() or directory.is_symlink() or not re.match(r'^20\d{6}_\d{6}_', directory.name):
            continue
        summary, asset, policy, runtime = (read(directory / name) for name in ('summary.json', 'asset_manifest.json', 'policy_manifest.json', 'runtime_manifest.json'))
        sources = read(directory / 'source_manifest.json')
        test = policy.get('test', summary.get('test', runtime.get('test')))
        if not test:
            stem = directory.name[16:]
            test = next((name for name in sorted(TESTS, key=len, reverse=True) if stem.startswith(name + '_')), 'unknown')
        checks = summary.get('iface_checks', {})
        reasons = [str(item.get('reason', '')) for item in summary.get('tests', []) if isinstance(item, dict) and item.get('status') == 'failed']
        if runtime.get('error'):
            reasons.append(str(runtime['error']))
        for item in checks.values():
            if isinstance(item, dict) and item.get('passed') is False:
                reasons.append('Interface check failed: ' + str(item.get('reason', item)))
        codes = [item.get('returncode') for item in runtime.get('owned_processes', [])]
        if any(code not in (0, None) for code in codes):
            reasons.append('process returncodes ' + str(codes))
        complete = bool(summary and runtime and len(codes) >= 2 and all(code is not None for code in codes))
        iface = level(summary, 'interface')
        if iface == 'passed' and (not checks or not all(isinstance(item, dict) and item.get('passed') is True for item in checks.values())):
            iface = 'unverified'
            reasons.append('Missing complete named iface_checks receipt')
        exclusion = []
        if directory.name[:15] < start:
            exclusion.append('before_motion_v2_campaign')
        if test not in TESTS:
            exclusion.append('outside_formal_14_tests')
        if asset.get('spawn_override'):
            exclusion.append('spawn_override_diagnostic')
        if asset.get('camera_only'):
            exclusion.append('camera_only_diagnostic')
        if asset.get('sensors'):
            exclusion.append('full_sensor_diagnostic')
        if not asset:
            exclusion.append('asset_manifest_missing')
        elif test in TESTS:
            terrain, spawn = expected_asset(test)
            if asset.get('terrain') != terrain:
                exclusion.append('nonbaseline_terrain')
            actual = asset.get('spawn')
            if not isinstance(actual, list) or len(actual) != 4 or any(not isinstance(a, (float, int)) or abs(a - b) > 1e-8 for a, b in zip(actual, spawn)):
                exclusion.append('nonbaseline_spawn')
        motion = level(summary, 'motion')
        failed = iface == 'failed' or motion == 'failed' or bool(runtime.get('error'))
        process_ids = tuple(item.get('pid') for item in runtime.get('owned_processes', [])[:2])
        entries.append({'run_id': directory.name, 'path': str(directory.resolve()), 'test': test,
            'complete': complete, 'interface': iface, 'motion': motion, 'failed': failed,
            'reason': '; '.join(dict.fromkeys(filter(None, reasons))), 'failure_categories': categories('; '.join(reasons)) if failed else [],
            'formal': not exclusion, 'excluded_from_formal': exclusion,
            'original_origin': asset.get('scenario_label') == 'original_origin' or 'original_origin' in directory.name,
            'independent_process_ids': list(process_ids), 'asset': asset, 'policy': policy, 'runtime': runtime,
            'summary': summary, 'source_manifest': sources,
            'metrics': [item.get('metrics', {}) for item in summary.get('tests', []) if isinstance(item, dict)],
            'capture': {'required': bool(asset.get('camera_only') or asset.get('sensors')),
                'status': 'recorded' if (directory / 'frame.jpg').exists() or (directory / 'frame.png').exists() else
                    'unverified_missing' if asset.get('camera_only') or asset.get('sensors') else 'not_requested',
                'source_receipt': read(directory / 'frame_source.json')},
            'input_sha256': {name: digest(directory / name) for name in
                ('summary.json', 'asset_manifest.json', 'policy_manifest.json', 'runtime_manifest.json', 'source_manifest.json', 'telemetry.jsonl')}})
    return entries


def trace(path, reference=False):
    """Normalize recorded rows only; do not synthesize missing measurements."""
    rows = []
    try:
        if path.suffix == '.json':
            raw = json.loads(path.read_text())
            raw = raw if isinstance(raw, list) else raw.get('rows', raw.get('trace', []))
        else:
            raw = []
            for line in path.read_text().splitlines():
                try:
                    raw.append(json.loads(line))
                except ValueError:
                    continue
        for row in raw:
            if not isinstance(row, dict):
                continue
            t = row.get('t', row.get('sim_time'))
            command = row.get('command')
            measured = row.get('measured')
            if measured is None:
                v = row.get('linear_velocity_body_com')
                w = row.get('angular_velocity_body')
                if isinstance(v, list) and isinstance(w, list) and len(v) >= 2 and len(w) >= 3:
                    measured = [v[0], v[1], w[2]]
            values = [t] + (command if isinstance(command, list) else []) + (measured if isinstance(measured, list) else [])
            if len(values) != 7 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
                continue
            rows.append({'t': float(t), 'command': command, 'measured': measured,
                         'position': row.get('position'), 'rpy': row.get('rpy'),
                         'phase': row.get('state', row.get('controller_mode'))})
    except (OSError, ValueError, AttributeError):
        pass
    return rows


def window_metrics(rows, start, end):
    import numpy as np
    selected = [row for row in rows if start <= row['t'] < end]
    if len(selected) < max(25, int((end - start) * 40)):
        return None
    t = np.asarray([row['t'] for row in selected])
    if len(t) < 2 or not (np.diff(t) > 0).all() or np.diff(t).max() > .041 or t[0] > start + .041 or t[-1] < end - .041:
        return None
    command = np.asarray([row['command'] for row in selected])
    velocity = np.asarray([row['measured'] for row in selected])
    return {'window_s': [start, end], 'samples': len(selected), 'command_mean': command.mean(axis=0).tolist(),
            'measured_mean': velocity.mean(axis=0).tolist(), 'rmse': np.sqrt(np.mean((velocity - command)**2, axis=0)).tolist(),
            'velocity_rms': np.sqrt(np.mean(velocity**2, axis=0)).tolist(),
            'assisted_samples': sum(row['phase'] in ('support_capture', 'support_hold') for row in selected)}


def profile_metrics(gazebo, isaac):
    import numpy as np
    if len(gazebo) < 2 or len(isaac) < 2:
        return {'passed': False, 'reason': 'Command profile evidence missing'}
    rt = np.asarray([row['t'] for row in isaac])
    if not (np.diff(rt) > 0).all():
        return {'passed': False, 'reason': 'Reference timestamps not increasing'}
    candidates = [row for row in gazebo if .12 <= row['t'] <= rt[-1] + 1e-8]
    if not candidates:
        return {'passed': False, 'reason': 'No common command-profile interval'}
    t = np.asarray([row['t'] for row in candidates])
    g = np.asarray([row['command'] for row in candidates])
    r = np.asarray([row['command'] for row in isaac])
    expected = np.column_stack([np.interp(t, rt, r[:, k]) for k in range(3)])
    error = np.max(np.abs(g - expected), axis=0)
    return {'passed': bool((error <= [.021, .021, .031]).all()), 'maximum_command_difference': error.tolist(),
            'tolerance': [.021, .021, .031], 'note': 'Actual recorded slew profile, time-aligned without speed fitting; one policy-period tolerance'}


def parity_receipt(reference_path):
    analysis = read(reference_path / 'matched_analysis.json')
    timing = read(reference_path / 'recorded_timing_diagnostic.json')
    recorded_error = analysis.get('recorded_obs_actor_action_max_error')
    recorded = timing.get('recorded_observation_actor_passed_2e_minus5') is True
    if not timing:
        recorded = isinstance(recorded_error, (int, float)) and recorded_error <= 2e-5
    return {'recorded_input_actor': {'status': 'passed' if recorded else 'failed' if recorded_error is not None else 'unverified',
            'teacher_frames': timing.get('teacher_frames', analysis.get('frames', 0) - analysis.get('pd_init_excluded_from_teacher_actor_parity', 0)),
            'initialization_frames_excluded': analysis.get('pd_init_excluded_from_teacher_actor_parity'),
            'maximum_absolute_action_error': recorded_error, 'tolerance': 2e-5,
            'meaning': 'Frozen actor evaluated on the actual recorded 247-vector, not a reconstructed fresh physics snapshot'},
        'fresh_snapshot_reconstruction': {'status': 'passed' if analysis.get('observation_parity_passed') is True and analysis.get('teacher_actor_parity_passed') is True else
                    'failed' if analysis.get('observation_parity_passed') is False or analysis.get('teacher_actor_parity_passed') is False else 'unverified',
            'maximum_247_absolute_error': analysis.get('all247_observation_max_error'),
            'maximum_reconstructed_action_error': analysis.get('reconstructed_obs_actor_action_max_error'),
            'raw_observation_parity_passed': analysis.get('observation_parity_passed'),
            'raw_reconstructed_teacher_actor_parity_passed': analysis.get('teacher_actor_parity_passed'),
            'timing_diagnostic': timing},
        'source_sha256': {name: digest(reference_path / name) for name in ('matched_analysis.json', 'recorded_timing_diagnostic.json')}}


def terrain_references(runs, supplied=None):
    """Use explicit recorded directories; missing references stay unverified."""
    if supplied and any((supplied / (name + '_trace.json')).exists() for name in TERRAIN):
        return {name: supplied for name in TERRAIN}
    parent = supplied or runs
    return {name: parent / directory for name, directory in TERRAIN_REFERENCE_DIRS.items()}


def compare_reference(formal, reference_path, tests):
    import numpy as np
    paths = reference_path if isinstance(reference_path, dict) else {name: reference_path for name in tests}
    rows, by_test, receipts = [], {}, {}
    for test in tests:
        path = paths[test]
        protocol, analysis, execution = (read(path / name) for name in ('protocol.json', 'terrain_analysis.json' if test in TERRAIN else 'matched_analysis.json', 'execution.json'))
        criteria = read(path / 'analysis_criteria.json')
        checkpoint_ok = protocol.get('checkpoint_sha256') == SHA
        r = trace(path / (REF_NAMES.get(test, test) + '_trace.json'), True)
        native = next((row for row in analysis.get('rows', []) if row.get('name') == REF_NAMES.get(test, test)), {})
        native_pass = native.get('same_numeric_terrain_criteria_passed' if test in TERRAIN else 'control_passed')
        receipts[str(path.resolve())] = {'protocol': protocol, 'execution': execution, 'analysis': analysis,
            'source_sha256': {name: digest(path / name) for name in ('protocol.json', 'matched_analysis.json', 'terrain_analysis.json', 'results.json', 'execution.json', 'analysis_criteria.json')}}
        candidates = [entry for entry in formal if entry['test'] == test]
        case_rows = []
        for entry in candidates:
            g = trace(Path(entry['path']) / 'telemetry.jsonl')
            command = profile_metrics(g, r)
            windows = [] if test == 'stand' else [(5., 7.8), (10., 12.8), (15., 17.8)] if test == 'switch' else [(5., 10.)]
            stop = entry['summary'].get('tests', [{}])[0].get('metrics', {}).get('stop_teacher', {}) if entry['summary'].get('tests') else {}
            reference_parking = native.get('parking', {}).get('window_seconds') or criteria.get('parking_windows', {}).get('stand' if test == 'stand' else 'switch' if test == 'switch' else 'command_timeout' if test == 'command_timeout' else 'single', [14., 18.])
            if isinstance(stop.get('window_s'), list) and len(stop['window_s']) == 2:
                common = [max(stop['window_s'][0], reference_parking[0]), min(stop['window_s'][1] + .02, reference_parking[1])]
                if common[1] > common[0] + .5:
                    windows.append(tuple(common))
            comparisons = []
            for start, end in windows:
                gm, rm = window_metrics(g, start, end), window_metrics(r, start, end)
                if gm is None or rm is None:
                    comparisons.append({'window_s': [start, end], 'passed': False, 'reason': 'Incomplete common recorded window'})
                    continue
                differences = {metric: np.abs(np.asarray(gm[metric]) - np.asarray(rm[metric])).tolist()
                               for metric in ('measured_mean', 'rmse', 'velocity_rms')}
                accepted = all(all(value <= limit for value, limit in zip(values, [.1, .1, .12])) for values in differences.values())
                comparisons.append({'window_s': [start, end], 'passed': accepted and gm['assisted_samples'] == rm['assisted_samples'] == 0,
                    'gazebo': gm, 'isaac': rm, 'abs_differences': differences, 'limits': [.1, .1, .12]})
            row = {'test': test, 'run_id': entry['run_id'], 'reference_trace': str(path / (REF_NAMES.get(test, test) + '_trace.json')),
                'command_profile': command, 'windows': comparisons, 'reference_native_criteria_passed': native_pass,
                'passed': checkpoint_ok and entry['interface'] == 'passed' and entry['motion'] == 'passed'
                    and native_pass is True and command['passed'] and bool(comparisons) and all(item['passed'] for item in comparisons)}
            case_rows.append(row); rows.append(row)
        expected_duration = 25. if test.startswith('step') else 26. if test == 'switch' else 15. if test == 'stand' else 18.
        reference_complete = bool(r) and r[-1]['t'] >= expected_duration - .041 and analysis.get('status') == 'completed' and native_pass is not None
        complete = len(candidates) == 3 and checkpoint_ok and reference_complete and all(entry['complete'] for entry in candidates)
        by_test[test] = {'status': 'passed' if complete and all(row['passed'] for row in case_rows) else
                                  'failed' if native_pass is False or any(entry['failed'] for entry in candidates) or reference_complete and any(not row['passed'] for row in case_rows if row['windows']) else 'unverified',
                         'gazebo_repetitions': len(candidates), 'reference_frames': len(r), 'reference_complete': reference_complete,
                         'reference_path': str(path.resolve()), 'reference_native_criteria': native, 'checkpoint_matches': checkpoint_ok, 'complete': complete}
    complete = all(row['complete'] for row in by_test.values())
    status = 'passed' if complete and all(row['status'] == 'passed' for row in by_test.values()) else 'failed' if any(row['status'] == 'failed' for row in by_test.values()) else 'unverified'
    return {'status': status, 'complete': complete, 'reference_paths': sorted(receipts),
        'checkpoint_matches': all(row['checkpoint_matches'] for row in by_test.values()), 'case_count': len(tests), 'cases': by_test, 'comparisons': rows,
        'reference_receipts': receipts,
        'reference_semantic_parity': parity_receipt(paths[tests[0]]) if not any(test in TERRAIN for test in tests) else {},
        'scope': 'Recorded matched-command performance comparison only; not an overall Sim2Sim approval'}


def configuration(entry):
    asset, policy = entry['asset'], entry['policy']
    return {'plugin': entry['runtime'].get('plugin_sha256'), 'protocol': entry['summary'].get('protocol_sha256'),
            'asset': {key: asset.get(key) for key in ('physics_step_s', 'decimation', 'policy_hz', 'model_mass_kg',
                'friction_baseline', 'base_inertial_pose', 'default_q', 'hard_lower', 'hard_upper', 'exclusive_writer')},
            'policy': {key: policy.get(key) for key in ('checkpoint_sha256', 'inference_device', 'observation_noise',
                'stop_strategy', 'bootstrap_PD_seconds', 'command_start_seconds', 'command_slew_acceleration')}}


def source_versions(entries):
    grouped = defaultdict(lambda: defaultdict(list))
    for entry in entries:
        for name, sha in entry['source_manifest'].items():
            grouped[name][str(sha)].append(entry['run_id'])
    return {'files': {name: [{'sha256': sha, 'run_count': len(ids), 'runs': ids} for sha, ids in hashes.items()]
                      for name, hashes in sorted(grouped.items())},
        'interpretation': 'Source versions are retained separately from motion configuration. Root reports late worker changes only strengthen the unexecuted navigation-command boundary; sensor/renderer branch changes affect diagnostics. Bare baseline math, physics, 247 construction, actor and protocol were unchanged. Full worker/file hash changes alone are not a motion-protocol change.',
        'interpretation_source': 'Root task coordination statement; hashed manifests remain primary receipts'}


def original_origin_reference(runs):
    path = runs / 'isaac_cpu_original_origin_20261003_h'
    analysis, protocol = read(path / 'original_origin_analysis.json'), read(path / 'protocol.json')
    return {'status': 'failed' if analysis.get('physically_failed') is True else 'unverified',
        'path': str(path.resolve()), 'checkpoint_matches': protocol.get('checkpoint_sha256') == SHA,
        'analysis': analysis, 'protocol': protocol,
        'source_sha256': {name: digest(path / name) for name in ('original_origin_analysis.json', 'protocol.json', 'execution.json',
            'default_native_height_termination_receipt.json', 'height_ray_hit_sources.json', 'analytic_ray_boundary_diagnostic.json')},
        'scope': 'Additional original-origin zero-command failure diagnostic, not a completed 15s repetition or navigation run. Native scan-based height termination disabled for this single case; actor overhead scan retained. Counterfactual native termination replay is labelled, not an executed rollout.'}


def aggregate(runs, start=START, flat_reference=None, terrain_reference=None):
    entries = collect(runs, start)
    formal = [entry for entry in entries if entry['formal']]
    count = Counter(entry['test'] for entry in formal)
    completed = [entry for entry in formal if entry['complete']]
    failures = [entry for entry in formal if entry['failed']]
    identities = [tuple(entry['independent_process_ids']) for entry in completed]
    independent = bool(identities) and all(len(value) == 2 and all(isinstance(pid, int) for pid in value) for value in identities) and len(set(identities)) == len(identities)
    fingerprints = [configuration(entry) for entry in completed]
    consistent = bool(fingerprints) and all(item == fingerprints[0] for item in fingerprints)
    enough = len(formal) == len(completed) == 42 and all(count[test] == 3 for test in TESTS) and independent and consistent
    if any(entry['interface'] == 'failed' for entry in formal):
        interface = 'failed'
    elif enough and all(entry['interface'] == 'passed' for entry in formal):
        interface = 'passed'
    else:
        interface = 'unverified'
    motion = 'failed' if failures or completed and (not independent or not consistent) else 'passed' if enough and all(entry['motion'] == 'passed' for entry in formal) and interface == 'passed' else 'unverified'
    original = [entry for entry in entries if entry['original_origin']]
    origin_reference = original_origin_reference(runs)
    origin_status = 'failed' if any(entry['failed'] for entry in original) or origin_reference['status'] == 'failed' else 'passed' if len(original) >= 3 and all(entry['complete'] and entry['interface'] == entry['motion'] == 'passed' for entry in original) else 'unverified'
    flat_reference = flat_reference or runs / 'isaac_cpu_matched_reference_20261003_c'
    flat = compare_reference(formal, flat_reference, FLAT)
    terrain = compare_reference(formal, terrain_references(runs, terrain_reference), TERRAIN)
    semantic = flat['reference_semantic_parity']
    parity_ok = semantic.get('fresh_snapshot_reconstruction', {}).get('status') == 'passed'
    if not terrain.get('complete'):
        sim2sim = 'unverified'
    elif flat['status'] == terrain['status'] == 'passed' and parity_ok and interface == motion == 'passed':
        sim2sim = 'passed'
    else:
        sim2sim = 'failed'
    per_test = {test: {'expected': 3, 'actual': count[test], 'completed': sum(entry['complete'] for entry in formal if entry['test'] == test),
        'passed': sum(entry['interface'] == entry['motion'] == 'passed' and entry['complete'] for entry in formal if entry['test'] == test),
        'failed': sum(entry['failed'] for entry in formal if entry['test'] == test),
        'status': 'failed' if any(entry['failed'] for entry in formal if entry['test'] == test) else
                  'passed' if count[test] == 3 and all(entry['complete'] and entry['interface'] == entry['motion'] == 'passed' for entry in formal if entry['test'] == test) else 'unverified',
        'runs': [entry['run_id'] for entry in formal if entry['test'] == test]} for test in TESTS}
    history_failures = [entry for entry in entries if entry['failed']]
    failure_counts = Counter(category for entry in history_failures for category in entry['failure_categories'])
    overall = 'failed' if motion == 'failed' or interface == 'failed' or sim2sim == 'failed' or origin_status == 'failed' else 'incomplete'
    may_launch_navigation = interface == motion == sim2sim == origin_status == 'passed'
    compact = lambda entry: {key: entry[key] for key in ('run_id', 'test', 'complete', 'interface', 'motion', 'formal',
        'excluded_from_formal', 'original_origin', 'reason', 'failure_categories', 'independent_process_ids', 'input_sha256', 'metrics', 'capture')}
    return {'schema_version': 2, 'status': overall, 'passed': False, 'checkpoint_sha256': SHA,
        'generated_at_local': datetime.now(timezone(timedelta(hours=8))).isoformat(),
        'levels': {'interface': interface, 'motion': motion, 'sim2sim': sim2sim, 'navigation': 'unverified', 'real_robot': 'unverified'},
        'campaign': {'name': 'motion_v2', 'first_run_inclusive': start, 'required_tests': list(TESTS), 'repetitions_per_test': 3,
            'expected_rounds': 42, 'eligible_rounds': len(formal), 'completed_rounds': len(completed), 'failed_rounds': len(failures),
            'exact_42_coverage': enough, 'independent_processes': independent, 'configuration_consistent': consistent,
            'configuration_reference': fingerprints[0] if fingerprints else None, 'cases': per_test,
            'source_versions': source_versions(formal),
            'all_eligible_runs_retained': True, 'selection_rule': 'All baseline campaign attempts, including failures; no best-of or passed-only selection'},
        'additional_scene_gate': {'name': 'original_origin', 'status': origin_status, 'navigation_allowed_for_scene': False,
            'reason': 'Scene failure blocks original-origin navigation; baseline flat spawn cannot replace it',
            'runs': [compact(entry) for entry in original], 'isaac_original_origin_reference': origin_reference},
        'sim2sim': {'flat_nine_case_subgate': flat, 'terrain_subgate': terrain,
            'semantic_parity_resolved': parity_ok, 'overall_reason': 'Terrain coverage and semantic parity required; flat performance comparison alone cannot pass Sim2Sim',
            'controlled_differences': {'gazebo_model_mass_kg': sorted(set(entry['asset'].get('model_mass_kg') for entry in formal if entry['asset'].get('model_mass_kg') is not None)),
                'isaac_nominal_model_mass_kg': 16.087, 'gazebo_robot_friction': .7, 'isaac_robot_friction': 1., 'ground_friction': 1.,
                'renderer': 'Formal Gazebo baseline removes Sensors/IMU rendering; PhysX reference is CPU/headless. Camera/sensor renderer diagnostics are excluded and listed separately.',
                'note': 'Matched commands and PD do not imply identical mass, inertia, contacts, friction or solver dynamics'}},
        'ray_boundary_correction': {'receipt': read(ROOT / 'test_results/raycast_boundary_v1/regression.json'),
            'receipt_sha256': digest(ROOT / 'test_results/raycast_boundary_v1/regression.json'),
            'scope': 'Corrected 1e-9 m XY expansion; offline all42 recorded-state parity does not replace new physical evidence. Original-origin corrected physical rerun is retained in additional_scene_gate.'},
        'sensor_shadow_diagnostics': [{'run_id': entry['run_id'], 'interface': entry['interface'], 'motion': entry['motion'],
            'summary': read(Path(entry['path']) / 'sensor_shadow/summary.json'),
            'summary_sha256': digest(Path(entry['path']) / 'sensor_shadow/summary.json'),
            'phase_diagnostic': read(Path(entry['path']) / 'sensor_phase_diagnostic/summary.json'),
            'phase_diagnostic_sha256': digest(Path(entry['path']) / 'sensor_phase_diagnostic/summary.json'),
            'rendering_manifest': read(Path(entry['path']) / 'rendering_manifest.json'),
            'scope': 'Read-only real sensor shadow; raw LiDAR is not a registered SLAM cloud; no policy observation switch'}
            for entry in entries if (Path(entry['path']) / 'sensor_shadow/summary.json').exists()],
        'history': {'runs_total': len(entries), 'failed_runs_total': len(history_failures), 'formal_excluded_runs': sum(not entry['formal'] for entry in entries),
            'source_versions': source_versions(entries),
            'failure_categories_overlap': dict(failure_counts), 'failure_runs': [compact(entry) for entry in history_failures],
            'all_runs': [compact(entry) for entry in entries]},
        'navigation': {'status': 'unverified', 'executed': False, 'truth_navigation_used': False,
            'allowed': may_launch_navigation,
            'reason': 'Motion and complete Sim2Sim/scene gates passed; SLAM navigation execution remains unverified' if may_launch_navigation else 'Motion, complete Sim2Sim or scene gate not passed; no navigation integration executed',
            'blocked_scenarios': ['original_origin'] if origin_status != 'passed' else [], 'required_regions': 46},
        'scope': 'Simulation only; privileged locomotion inputs. Ramp and low-step tests are not true stair traversal. No real-robot verification.',
        'aggregate_script_sha256': digest(Path(__file__)), 'source_protocol_sha256': digest(ROOT / 'tests/protocol.json')}


def markdown(report):
    c = report['campaign']
    lines = ['# Go2 Teacher 仿真验证报告', '', f"整体状态：**{report['status']}**。此报告读取原始逐轮收据，不运行仿真或重新评估，不挑选通过轮次。", '',
        '| 层级 | 整体结果 |', '|---|---|']
    lines.extend(f'| {name} | {value} |' for name, value in report['levels'].items())
    lines += ['', f"motion_v2 从 `{c['first_run_inclusive']}` 起，共要求14类×3次=42独立轮。当前基准轮 {c['eligible_rounds']}，完成 {c['completed_rounds']}，失败 {c['failed_rounds']}；独立进程={c['independent_processes']}，配置一致={c['configuration_consistent']}。", '',
        '| 正式测试 | 完成 / 要求 | 通过 | 失败 | 结果 |', '|---|---:|---:|---:|---|']
    lines.extend(f"| {test} | {row['completed']} / 3 | {row['passed']} | {row['failed']} | {row['status']} |" for test, row in c['cases'].items())
    scene = report['additional_scene_gate']
    parity = report['sim2sim']['flat_nine_case_subgate']['reference_semantic_parity']
    actor = parity['recorded_input_actor']
    strict = parity['fresh_snapshot_reconstruction']
    lines += ['', f"原起点额外场景门：**{scene['status']}**，该场景导航不获准。它不能因被排除出远离overhead的基准出生点而被忽略。", '',
        '平地九类相同命令对照子门：**' + report['sim2sim']['flat_nine_case_subgate']['status'] + '**；地形对照：**' + report['sim2sim']['terrain_subgate']['status'] + '**。',
        '稳态共同窗口为5–10秒（switch按三段），停车使用两环境原生记录窗口交集；比较真实command profile、三轴均值、RMSE及停车RMS，差值限制0.1 m/s、0.12 rad/s。不能用全程平均覆盖稳态与停车失败。',
        '', f"实际记录247输入→冻结actor：**{actor['status']}**，{actor['teacher_frames']} 个Teacher帧（排除{actor['initialization_frames_excluded']}个PD初始化帧），最大动作误差 {actor['maximum_absolute_action_error']}，预设容差 {actor['tolerance']}。",
        f"最新物理快照重建247/actor的严格parity：**{strict['status']}**，247最大误差 {strict['maximum_247_absolute_error']}，重建输入动作最大误差 {strict['maximum_reconstructed_action_error']}。这不表示实际记录输入的actor计算失败。重置gravity缓存和79个switch扫描帧的一周期延迟已量化，strict失败原记录保留；具体时序解释为来源与观测推断，未用替换输入掩盖失败。",
        '', '已知动力学差异：Gazebo模型16.512 kg，训练参考16.087 kg；机器人摩擦0.7与1.0，地面1.0；基准关闭传感器渲染，相机/全传感器诊断另列。参考是CPU PhysX；此为受控差异，不代表两环境完全相同。',
        '', f"历史共 {report['history']['runs_total']} 轮，失败 {report['history']['failed_runs_total']} 轮。分类可重叠：`{report['history']['failure_categories_overlap']}`。所有历史失败及诊断保留。", '',
        '| 运行 | 测试 | interface / motion | 纳入正式 | 原因 / 排除说明 |', '|---|---|---|---|---|']
    for row in report['history']['all_runs']:
        detail = '; '.join(row['excluded_from_formal']) + ('; ' + row['reason'] if row['reason'] else '')
        lines.append(f"| {row['run_id']} | {row['test']} | {row['interface']} / {row['motion']} | {row['formal']} | {detail.replace('|','/').replace(chr(10),' ')} |")
    lines += ['', '## 地形与原起点的实际对照', '',
        '| 测试 | CPU参考原数值门 | 测量 / 失败原因 | Gazebo正式结果 |', '|---|---|---|---|']
    for test, case in report['sim2sim']['terrain_subgate']['cases'].items():
        native = case['reference_native_criteria']
        terrain = native.get('terrain', {})
        detail = f"progress={terrain.get('progress_m')} / {terrain.get('required_progress_m')} m" if test.startswith('ramp') else f"x={terrain.get('end_median_position', [None])[0]} / {terrain.get('required_end_x_m')} m; height_gain={terrain.get('settled_height_gain_m')} / {terrain.get('minimum_height_gain_m')} m"
        detail += '; ' + '; '.join(native.get('failure_reasons', []))
        lines.append(f"| {test} | {native.get('same_numeric_terrain_criteria_passed', 'unverified')} | {detail} | {c['cases'][test]['status']} |")
    for row in scene['runs']:
        metric = row['metrics'][0] if row['metrics'] else {}
        lines += ['', f"Gazebo原起点 `{row['run_id']}`：{metric.get('duration_s')} s，位移 {metric.get('position_delta_m')} m，yaw变化 {metric.get('total_yaw_delta_rad')} rad；{row['reason']}。安全停车/原生故障锁存及提前结束使该场景interface失败，不能用正式基准42轮interface通过覆盖它。"]
    origin = scene['isaac_original_origin_reference']['analysis']
    snapshot = origin.get('last_snapshot', {})
    lines += ['', f"CPU原起点：{origin.get('frames_recorded')}帧 / {origin.get('duration_recorded_s')} s，实际失稳={origin.get('physically_failed')}，下面支持面间隙 {snapshot.get('independent_below_base_clearance_m')} m，roll {snapshot.get('roll_rad')} rad，位置 {snapshot.get('position')}。初始187scan真实记录范围 {origin.get('first_recorded_raw_height_min_max')}，overhead命中 {origin.get('first_overhead_hits')}。",
        'CPU原起点单项关闭了native scan-height termination以观察实际物理跌倒；actor保持+20m射线含上层遮挡。默认native高度终止的另存AST反事实重放不是实际rollout。扫描几何误差、接触缺失等原始字段完整保留在验收JSON，不能用地形参考的小误差覆盖原起点几何差异。', '',
        '## 版本与失败归因', '',
        '全部源文件SHA按轮次分组存入JSON。较晚worker版本只增强未执行的导航命令校验，传感器/渲染分支影响诊断轮；基准运动协议、物理、247构造与actor没有变化，此解释来自root协调记录，版本哈希不隐藏。',
        '分类是收据文本标签且可重叠。进程signal -11本身归为未知进程信号，不能独自推断渲染根因；未记录图像为unverified_missing。历史辅助停车、初始化和时间不足失败继续保留，后续通过不抹去旧失败。']
    lines += ['', '导航未执行、未验证；必须使用SLAM位置和实测点云，保持原46区域门槛。真机部署未验证，本任务不操作实体机器狗。坡道和低台阶结果不能写成真实楼梯通过。', '',
        '部署射线XY边界1e-9扩张已修正；42轮40692实际记录状态的247构造回放差0，旧构造与旧记录也差0，回归收据保存在ray_boundary_correction。修正后原起点实际复测仍失稳，不能放行导航。', '',
        '真实IMU/关节/LiDAR/RGB旁路在sensor_shadow_diagnostics记录，SLAM与注册点云缺失不补真值。软件Ogre1退出崩溃历史保留；最后NVIDIA/Ogre2真实带传感器前进停车运行全部退出0，Actor继续CPU，图形显存增量约156MiB。只证明该传感器运行，不证明策略观测替换或导航。', '',
        '完整逐轮哈希、对照窗口与数据差异见同目录验收JSON。此报告如标preview仅用于审阅，不能被启动器当正式放行收据。', '']
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runs', type=Path, default=ROOT / 'runs')
    parser.add_argument('--campaign-start', default=START)
    parser.add_argument('--flat-reference', type=Path)
    parser.add_argument('--terrain-reference', type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--prepare', action='store_true', help='Default: write preview only')
    mode.add_argument('--final', action='store_true', help='Write reviewed computed result as formal acceptance; never forces a pass')
    parser.add_argument('--dry-run', action='store_true', help='Print compact computed status and write no files')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--report', type=Path)
    args = parser.parse_args()
    report = aggregate(args.runs.resolve(), args.campaign_start, args.flat_reference, args.terrain_reference)
    report['publication'] = 'final' if args.final else 'preview'
    summary = {'status': report['status'], 'levels': report['levels'], 'publication': report['publication'],
               'formal_rounds': report['campaign']['eligible_rounds'], 'completed': report['campaign']['completed_rounds'],
               'failures': report['campaign']['failed_rounds'], 'original_origin': report['additional_scene_gate']['status'],
               'flat_subgate': report['sim2sim']['flat_nine_case_subgate']['status'], 'terrain_subgate': report['sim2sim']['terrain_subgate']['status']}
    if not args.dry_run:
        output = args.output or args.runs / ('acceptance.json' if args.final else 'acceptance.preview.json')
        destination = args.report or ROOT / ('REPORT.md' if args.final else 'REPORT.preview.md')
        output.parent.mkdir(parents=True, exist_ok=True); destination.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(output.suffix + '.tmp')
        temporary.write_text(json.dumps(clean(report), ensure_ascii=False, indent=2, allow_nan=False) + '\n')
        temporary.replace(output)
        destination.write_text(markdown(report))
        summary.update(output=str(output.resolve()), report=str(destination.resolve()))
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
