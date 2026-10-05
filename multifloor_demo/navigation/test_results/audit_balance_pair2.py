#!/usr/bin/env python3
"""Read completed pair2 evidence only; no ROS, command or evaluation repair.

Actual adapter commands select physical motion modes. GT interpolation is only
inside recorded brackets with <=.15 s gaps. B failed before an origin existed;
its zero-posture startup is not evidence of active feedback effectiveness.
"""
import ast
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'simulation/test_results'
A = BASE / '20261001_balance_pair2_a_disabled'
B = BASE / '20261001_balance_pair2_b_enabled'
OUT = Path(__file__).with_name('balance_pair2_navigation_review.json')


def rows(path):
    for line in path.open():
        try:
            yield json.loads(line)
        except json.JSONDecodeError:
            pass


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def ranges(records, time_key):
    seq = list(records)
    if len(seq) < 2:
        return dict(count=len(seq), maximum_wall_gap_s=None)
    gap, left, right = max(
        (b[time_key] - a[time_key], a, b) for a, b in zip(seq, seq[1:]))
    return dict(count=len(seq), maximum_wall_gap_s=gap,
                before={k: left.get(k) for k in ('stamp_ns', 'sim', time_key)},
                after={k: right.get(k) for k in ('stamp_ns', 'sim', time_key)})


def physical_windows(result, folder):
    truth = sorted({r['stamp']: r for r in result['truth']}.values(),
                   key=lambda r: r['stamp'])
    times = np.array([r['stamp'] for r in truth])
    yaw = np.unwrap([math.atan2(Rotation.from_quat(r['q']).as_matrix()[1, 0],
                               Rotation.from_quat(r['q']).as_matrix()[0, 0])
                     for r in truth])
    poses = np.column_stack([[r['p'] for r in truth], yaw])
    commands = {r['sim']: r['value'] for r in rows(folder / 'joint_stop_adapter.jsonl')
                if r['kind'] == 'actual_champ_command'}
    ct = np.array(sorted(commands)); cv = np.array([commands[t] for t in ct])
    source = ROOT / 'simulation/test_results/audit_full14_physics.py'
    tree = ast.parse(source.read_text())
    methods = [n for n in tree.body if isinstance(n, ast.FunctionDef)
               and n.name in ('pose', 'category', 'window')]
    scope = dict(np=np, times=times, poses=poses, ct=ct, cv=cv)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), scope)
    starts = {}
    for r in result['statuses']:
        d = r['status']
        if d.get('state') == 'running' and d.get('waypoint_index') is not None:
            starts.setdefault(d['waypoint_index'], r['sim'])
    windows = {str(i): scope['window'](starts[i], starts.get(i+1, result['complete_sim']))
               for i in range(8)}
    for data in windows.values():
        data['walk_command_opposite_actual_yaw_segments'] = [
            s for s in data['segments'] if s['category'] == 'walk'
            and s['yaw_delta_rad'] * s['commanded_yaw_integral_rad'] < 0
            and abs(s['yaw_delta_rad']) > .02
            and abs(s['commanded_yaw_integral_rad']) > .01]
    return dict(index_starts=starts, windows=windows,
                truth_max_adjacent_gap_s=float(np.max(np.diff(times))),
                evaluation_function_source_sha256=sha(source))


def feedback_summary(folder):
    samples = [r for r in rows(folder / 'body_feedback.jsonl')
               if 'actual_published_body_pose_roll_pitch' in r]
    poses = np.array([r['actual_published_body_pose_roll_pitch'] for r in samples])
    pending = [r for r in samples if r.get('neutral_ack_pending')]
    return dict(status_samples=len(samples),
                enabled_values=sorted(set(r['enabled'] for r in samples)),
                actual_published_RP_component_max_abs_rad=np.max(abs(poses), axis=0).tolist(),
                nonzero_published_RP_samples=int(sum(np.any(abs(poses) > 1e-12, axis=1))),
                pending_neutral_ACK_samples=len(pending),
                zero_posture_does_not_validate_active_feedback=True)


def startup_failure():
    previous = []
    first = None
    for r in rows(B / 'body_feedback.jsonl'):
        if r.get('event') != 'input_freshness_tick':
            continue
        if r.get('failed'):
            first = r
            break
        previous = (previous + [r])[-3:]
    before = first['wall_monotonic'] - .06
    after = first['wall_monotonic'] + .35
    selected = {k: [] for k in ('imu', 'joints', 'contact', 'actual_actuator_command', 'adapter')}
    for r in rows(B / 'balance_inputs.jsonl'):
        if before <= r['wall_monotonic'] <= after and r['kind'] in selected:
            selected[r['kind']].append(r)
    joint_edges = []
    for r in selected['joints']:
        joint_edges.append({k: r[k] for k in ('sequence', 'wall_monotonic', 'stamp_ns')})
    controls = {k: [] for k in ('raw_target', 'filtered_target', 'actual_champ_command')}
    stamp = first['predicates']['latest_imu_stamp']
    nonzero = []
    for r in rows(B / 'joint_stop_adapter.jsonl'):
        if r['kind'] == 'actual_champ_command' and any(r['value']):
            nonzero.append({k: r[k] for k in ('sim', 'state', 'value')})
        if stamp-.1 <= r['sim'] <= stamp+.4 and r['kind'] in controls:
            controls[r['kind']].append(r)
    launch = (B / 'ros_logs/latest/launch.log').read_text().splitlines()
    process_edges = [s for s in launch if 'process started' in s or
                     ('python3-16' in s and 'finished cleanly' in s)]
    manifest = json.loads((B / 'first8_manifest.json').read_text())
    return dict(first_failed_tick=first, preceding_ticks=previous,
                independent_joint_receipts_around_failure=joint_edges,
                independent_topic_receipt_gaps={k: ranges(v, 'wall_monotonic')
                                               for k, v in selected.items()},
                actual_CHAMP_and_joint_target_receipt_gaps={k: ranges(v, 'wall')
                                                          for k, v in controls.items()},
                actual_CHAMP_nonzero_commands_all_startup=nonzero,
                launch_process_start_edges=process_edges,
                startup_gate=json.loads((B / 'startup_gate.json').read_text()),
                historical_BLAS_environment_recorded={k: k in manifest['execution_environment']
                                                     for k in ('OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS')},
                interpretation=[
                    'The first rejection is an exact 30 ms integer-stamp boundary represented as >.03 float.',
                    'A following .334 wall-second joint receipt gap is independently recorded; an epsilon change cannot remove this actual gap.',
                    'IMU/contact/CHAMP joint targets remain continuous during the joint-state gap. The gap is not evidence of a whole-Gazebo or whole-Python pause.',
                    'Publisher-vs-DDS-consumer cause is not proven because these are receipt logs, not publication-boundary logs.',
                    'Startup admission launches SLAM/SCAN/Python nodes together; time proximity is not proof of CPU oversubscription.'])


def main():
    ra = json.loads((A / 'first_eight_result.json').read_text())
    rb = json.loads((B / 'first_eight_result.json').read_text())
    report = dict(scope=__doc__, original_results_unchanged=True,
                  baseline=dict(passed=ra['passed'], missing=ra['missing_acceptance'],
                                active_max_tilt=ra['active_max_imu_tilt'], active_holds=ra['active_bridge_holds'],
                                unpaired_truth_rows=ra['unpaired_truth_pose_rows'],
                                actual_NAV_arrivals=ra['nav_confirmations'], physics=physical_windows(ra, A)),
                  candidate=dict(passed=rb['passed'], failure=rb['failure'], origin=rb['origin'],
                                 active_feedback_not_evaluated=True, startup=startup_failure()),
                  published_feedback=dict(baseline=feedback_summary(A), candidate=feedback_summary(B)),
                  cleanup={p.name: json.loads((p / 'process_cleanup.json').read_text()) for p in (A, B)},
                  source_sha256={p.name: {f: sha(p/f) for f in
                      ('first_eight_result.json', 'first8_manifest.json', 'body_feedback.jsonl',
                       'joint_stop_adapter.jsonl', 'staging/body_stabilizer_node.py')}
                      for p in (A, B)}, auditor_sha256=sha(Path(__file__)))
    OUT.write_text(json.dumps(report, indent=2) + '\n')
    print('saved', OUT)
    print('A strict:', report['baseline']['passed'], report['baseline']['missing'])
    for index, window in report['baseline']['physics']['windows'].items():
        print(index, window['seconds'], window['totals'])
    print('B startup failure:', rb['failure'])
    print('feedback:', report['published_feedback'])


if __name__ == '__main__':
    main()
