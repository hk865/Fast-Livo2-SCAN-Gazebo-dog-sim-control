#!/usr/bin/env python3
"""Read completed CHAMP V4 receipts; never import/start ROS or alter originals."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

PHASE = 'PreUpdate after CM write, before Physics Update'
HERE = Path(__file__).resolve().parent
MODE = HERE.parent.parent
FREEZE = MODE / 'pid_comparison/champ_mode/tests/force_capture_v4/freeze.json'


def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def finite12(values):
    return isinstance(values, list) and len(values) == 12 and all(
        isinstance(value, (int, float)) and not isinstance(value, bool)
        and math.isfinite(value) for value in values)


def check_frame(row):
    issues = []
    if abs(row['dt'] - .005) > 1e-9:
        issues.append('physics_dt')
    if row.get('state_phase') != 'PostUpdate' or row.get('state_time_offset_s') != 0:
        issues.append('state_phase')
    if row.get('torque_source') != 'command_feed_to_physics' or row.get('force_capture_phase') != PHASE:
        issues.append('source_phase')
    if row.get('force_capture_pair_valid') is not True:
        issues.append('pair_flag')
    if row.get('force_command_valid') is not True or row.get('tau_available') != [1] * 12 or not finite12(row.get('tau')):
        issues.append('force_values_available')
    if not isinstance(row.get('physics_iteration'), int) or row.get('physics_iteration') != row.get('force_capture_iteration'):
        issues.append('iteration_pair')
    for key, expected in [('force_capture_sim_time', row['t']), ('force_capture_dt', row['dt']),
                          ('force_application_interval_start_s', row['t'] - row['dt']),
                          ('force_application_interval_end_s', row['t'])]:
        value = row.get(key)
        if not isinstance(value, (int, float)) or not math.isfinite(value) or abs(value - expected) > 1e-9:
            issues.append(key)
    if not finite12(row.get('q')) or not finite12(row.get('qd')):
        issues.append('state_values')
    return issues


def analyze_frames(path):
    counters = Counter()
    examples = []
    first_force, first_nonzero, first_latch, first_violation = None, None, None, None
    max_tau, max_qd = [0.] * 12, [0.] * 12
    step_times, physics_steps = [], []
    groups = {str(i): {'contact_rows': 0, 'contact_pair_records': 0,
        'positions_records': 0, 'normals_records': 0, 'wrenches_records': 0,
        'first_contact_sim_s': None} for i in range(5)}
    header, ownership, stops, other = [], [], [], []
    digest = hashlib.sha256()
    previous_iteration, previous_time = None, None
    with Path(path).open('rb') as stream:
        for line in stream:
            digest.update(line)
            row = json.loads(line)
            kind = row.get('kind')
            if kind != 'physics_step':
                if kind == 'actuator_contract': header.append(row)
                elif kind == 'model_plugin_ownership': ownership.append(row)
                elif kind == 'observer_stop': stops.append(row)
                else: other.append(row)
                continue
            t = row['t']
            counters['physics_rows'] += 1
            step_times.append(t)
            physics_steps.append(row.get('physics_iteration'))
            issues = check_frame(row)
            if previous_iteration is not None:
                if row.get('physics_iteration') != previous_iteration + 1:
                    issues.append('nonconsecutive_physics_iteration')
                if abs(t - previous_time - row['dt']) > 1e-9:
                    issues.append('nonconsecutive_sim_time')
            previous_iteration, previous_time = row.get('physics_iteration'), t
            for issue in issues:
                counters['all/' + issue] += 1
                if t >= .1: counters['after_bootstrap/' + issue] += 1
                if t >= 3.: counters['after_reader_startup/' + issue] += 1
            if issues and len(examples) < 12:
                examples.append({'t': t, 'physics_iteration': row.get('physics_iteration'), 'issues': issues})
            if not issues:
                counters['complete_paired_force_rows'] += 1
                if first_force is None: first_force = t
            if finite12(row.get('tau')):
                values = row['tau']
                for i, value in enumerate(values): max_tau[i] = max(max_tau[i], abs(value))
                if any(abs(value) > 1e-9 for value in values):
                    counters['nonzero_pre_physics_rows'] += 1
                    if first_nonzero is None: first_nonzero = t
                else: counters['allzero_pre_physics_rows'] += 1
                if t >= .1 and any(abs(value) > 23.5 + 1e-6 for value in values):
                    counters['force_limit_violation_rows'] += 1
                    if first_violation is None: first_violation = {'t': t, 'kind': 'force_command_limit_exceeded', 'tau': values}
            post = row.get('tau_postupdate_buffer')
            if finite12(post) and all(value == 0 for value in post):
                counters['allzero_postupdate_buffer_rows'] += 1
            if finite12(row.get('qd')):
                for i, value in enumerate(row['qd']): max_qd[i] = max(max_qd[i], abs(value))
                if t >= .1 and any(abs(value) > 30 + 1e-6 for value in row['qd']):
                    counters['velocity_limit_violation_rows'] += 1
                    if first_violation is None: first_violation = {'t': t, 'kind': 'joint_velocity_limit_exceeded', 'qd': row['qd']}
            if row.get('actuator_limit_violation_latched') is not None:
                counters['latched_violation_rows'] += 1
                if first_latch is None: first_latch = {'t': t, 'reason': row['actuator_limit_violation_latched'],
                    'original_first_time': row.get('actuator_limit_violation_first_time')}
            counts = row.get('contacts')
            if not isinstance(counts, list) or len(counts) != 5 or any(v < 0 for v in counts):
                counters['unavailable_contact_rows'] += 1
            else:
                for i, value in enumerate(counts):
                    if value > 0:
                        groups[str(i)]['contact_rows'] += 1
                        if groups[str(i)]['first_contact_sim_s'] is None: groups[str(i)]['first_contact_sim_s'] = t
            for contact in row.get('contact_pairs', []):
                group = groups.get(str(contact.get('group')))
                if group is None: counters['invalid_contact_group'] += 1; continue
                group['contact_pair_records'] += 1
                for key, raw in [('positions_records', 'positions_world'), ('normals_records', 'normals_world'), ('wrenches_records', 'wrenches_raw')]:
                    if contact.get(raw): group[key] += 1
    return {'original_sha256': digest.hexdigest(), 'counters': dict(counters),
        'first_complete_paired_force_sim_s': first_force, 'first_nonzero_pre_physics_sim_s': first_nonzero,
        'first_violation': first_violation, 'first_native_latch': first_latch,
        'max_abs_tau_by_joint_nm': max_tau, 'max_abs_qd_by_joint_rad_s': max_qd,
        'force_unavailable_examples': examples, 'actual_contact_group_evidence': groups,
        'time_range_s': [min(step_times), max(step_times)] if step_times else None,
        'physics_iteration_range': [physics_steps[0], physics_steps[-1]] if physics_steps else None,
        'actuator_headers': header, 'ownership_receipts': ownership, 'observer_stops': stops,
        'other_native_receipts': other}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    args = parser.parse_args()
    run = args.run.resolve()
    if not (run / 'runtime_manifest.json').is_file():
        raise RuntimeError('Completed runtime manifest required; refuse live run analysis')
    contract = json.loads((run / 'champ_contract.json').read_text())
    freeze = json.loads(FREEZE.read_text())
    mismatch = {str(path): {'expected': expected, 'actual': sha(path)}
        for path, expected in freeze['source_sha256'].items() if sha(path) != expected}
    executed = {}
    for group in ['artifacts', 'executor_source_sha256']:
        executed[group] = {str(path): {'expected': expected, 'actual': sha(path)}
            for path, expected in contract[group].items() if sha(path) != expected}
    native = analyze_frames(run / 'actuator.jsonl')
    telemetry_hash = hashlib.sha256()
    first_reader_fault, first_zero_after_fault, reader_faults = None, None, Counter()
    with (run / 'telemetry.jsonl').open('rb') as stream:
        for line in stream:
            telemetry_hash.update(line)
            row = json.loads(line)
            if row.get('fault'):
                reader_faults[row['fault']] += 1
                if first_reader_fault is None: first_reader_fault = {'sim_time': row['sim_time'], 'fault': row['fault'], 'command': row.get('command')}
                if first_zero_after_fault is None and row.get('command') == [0., 0., 0.]:
                    first_zero_after_fault = row['sim_time']
    plugins = ET.parse(run / 'world.sdf').getroot().findall('.//plugin')
    actual_observer = [{'filename': p.get('filename'), 'name': p.get('name'),
        'priority': p.findtext('{https://gazebosim.org/sdf}system_priority')}
        for p in plugins if p.get('name') == 'champ_compare::NativeObserver']
    refs = {str(run / q): sha(run / q) for q in ['champ_contract.json', 'runtime_manifest.json', 'worker_result.json', 'policy_metadata.json', 'world.sdf']}
    refs[str(run / 'actuator.jsonl')] = native['original_sha256']
    refs[str(run / 'telemetry.jsonl')] = telemetry_hash.hexdigest()
    result = {'schema': 'independent_CHAMP_V4_force_source_audit/v1', 'run': str(run),
        'audit_source': {'path': str(Path(__file__).resolve()), 'sha256': sha(__file__)},
        'frozen_source': {'path': str(FREEZE), 'sha256': sha(FREEZE), 'mismatches': mismatch},
        'contract_source_mismatches': executed, 'force_contract': contract['force_capture'],
        'actuator_safety_contract': contract['actuator_safety'], 'actual_world_observer_plugin': actual_observer,
        'native': native, 'reader_faults': dict(reader_faults), 'first_reader_fault': first_reader_fault,
        'first_zero_command_after_reader_fault_sim_s': first_zero_after_fault,
        'native_latch_to_reader_fault_sim_s': None if not native['first_native_latch'] or not first_reader_fault else
            first_reader_fault['sim_time'] - native['first_native_latch']['t'],
        'worker_result': json.loads((run / 'worker_result.json').read_text()),
        'original_inputs_sha256': refs,
        'scope': 'Force-command feed to physics, not measured motor torque. Foot contact receipts are raw; this source audit does not assign overall route/parking/nav pass.',
        'protection_without_observed_violation': 'If no violation occurs, actual signal/zero reaction remains unexercised; offline cache/guard checks are separate evidence.',
        'runtime_modified': False, 'ROS_or_Gazebo_started_by_auditor': False}
    output = HERE / run.name
    output.mkdir()
    with (output / 'receipt.json').open('x') as stream:
        json.dump(result, stream, indent=2); stream.write('\n')
    print(json.dumps({'receipt': str(output / 'receipt.json'), 'sha256': sha(output / 'receipt.json'),
        'native_counters': native['counters'], 'first_violation': native['first_violation'], 'reader_faults': dict(reader_faults)}))


if __name__ == '__main__':
    main()
