#!/usr/bin/env python3
"""Read-only actual turnaround diagnosis; no proposed limit is applied to data."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]


def yaw(q):
    w, x, y, z = np.asarray(q).T
    return np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def evaluate(runs, output):
    output.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(4, len(runs), figsize=(7*len(runs), 10), sharex='col', constrained_layout=True)
    axes = np.asarray(axes).reshape(4, len(runs))
    records, archive, originals = [], {}, {}
    for column, run in enumerate(runs):
        run = Path(run).resolve()
        telemetry = rows(run / 'telemetry.jsonl')
        status = rows(run / 'navigation_status.jsonl')
        native = [r for r in rows(run / 'actuator.jsonl') if r.get('kind') == 'physics_step']
        slam = rows(run / 'navigation_slam_poses.jsonl')
        first = next(i for i, r in enumerate(status) if r.get('waypoint_index') == 1)
        start = float(status[first]['ros_sim_time'])
        final = next((r for r in status[first:] if r['state'] in ['failed', 'succeeded']), status[-1])
        end = float(final['ros_sim_time'])
        pt = np.asarray([r['world_sim_time'] for r in telemetry])
        nt = np.asarray([r['t'] for r in native]) - .005
        st = np.asarray([r['stamp_ns']*1e-9 for r in slam])
        cmd = np.asarray([r['command'] for r in telemetry])
        request = np.asarray([r['requested'] for r in telemetry])
        nq = np.asarray([r['quaternion_wxyz'] for r in native])
        sq = np.asarray([r['quaternion'] for r in slam])
        syaw = yaw(sq[:, [3, 0, 1, 2]])
        nyaw = yaw(nq)
        nv = np.asarray([r['body_lin_vel_com'] for r in native])
        nw = np.asarray([r['body_ang_vel'] for r in native])
        pm = (pt >= start) & (pt <= end)
        nm = (nt >= start) & (nt <= end)
        sm = (st >= start) & (st <= end)
        assert pm.any() and nm.any() and sm.any()
        sampled = [r for r, flag in zip(telemetry, pm) if flag]
        history = {r['sequence']: r for r in rows(run / 'navigation_command_history.jsonl')}
        rejected_sources = [history.get(r['navigation_envelope'].get('sequence')) for r in sampled if r.get('command_expired') is True]
        rejected_sources = [r for r in rejected_sources if r is not None]
        invalid = [not(-.05 <= r['navigation_envelope']['wall_age_s'] <= .3 and
                        -.05 <= r['navigation_envelope']['sim_age_s'] <= .3) for r in sampled]
        flag_count = sum(r.get('command_expired') is True for r in sampled)
        transitions = []
        last_phase = None
        for r in status[first:]:
            if r['ros_sim_time'] > end:
                break
            if r.get('alignment_phase') != last_phase:
                transitions.append({'world_s': r['ros_sim_time'], 'phase': r.get('alignment_phase')})
                last_phase = r.get('alignment_phase')
        first_drive = next((r for r in status[first:] if r['ros_sim_time'] <= end and
            r.get('alignment_phase') == 'drive' and r.get('command', [0])[0] > .05), None)
        phase_durations = {}
        for i, r in enumerate(transitions):
            phase_durations[r['phase']] = phase_durations.get(r['phase'], 0.) + (
                transitions[i+1]['world_s'] if i+1 < len(transitions) else end) - r['world_s']
        align = next((r for r in transitions if r['phase'] == 'align'), None)
        settle = next((r for r in transitions if r['phase'] == 'settle'), None)
        align_metric = None
        if align and settle:
            ap = (pt >= align['world_s']) & (pt <= settle['world_s'])
            an = (nt >= align['world_s']) & (nt <= settle['world_s'])
            align_rows = [r for r, flag in zip(telemetry, ap) if flag]
            align_metric = {'duration_sim_s': settle['world_s']-align['world_s'],
                'actual_actor_wz_integral_rad': float(cmd[ap, 2].sum()*.02),
                'actual_native_heading_change_rad': float(nyaw[an][-1]-nyaw[an][0]),
                'actual_actor_abs_wz_above_point08_seconds': float((abs(cmd[ap, 2])>.08).sum()*.02),
                'actual_actor_wz_near_zero_seconds': float((abs(cmd[ap, 2])<.015).sum()*.02),
                'expired_or_unhealthy_flag_samples': sum(r['command_expired'] for r in align_rows),
                'policy_samples': len(align_rows)}
        last_pose = np.asarray([r['position'] for r in slam])[sm][-1]
        goals = json.loads((run / 'navigation_request.json').read_text())['goals']
        goal = np.asarray(goals[1]['center'])
        record = {'run_id': run.name, 'scope': 'Actual return stage only, start/end from recorded navigation status',
            'return_world_window_s': [start, end], 'final_navigation_state': final['state'],
            'final_message': final.get('message'), 'goal_timeout_sim_s': goals[1]['timeout_sim_s'],
            'policy_samples': len(sampled), 'expired_or_unhealthy_flag_samples': flag_count,
            'expired_or_unhealthy_fraction': flag_count/len(sampled), 'strict_source_age_invalid_samples': sum(invalid),
            'command_reason_counts': dict(Counter(r['command_reason'] for r in sampled)),
            'rejected_worker_frames_actual_bridge_reason_counts': dict(Counter(r.get('reason') for r in rejected_sources)),
            'rejected_worker_frames_actual_bridge_state_counts': dict(Counter(r.get('state') for r in rejected_sources)),
            'rejected_worker_frames_history_match_count': len(rejected_sources),
            'actual_actor_pure_turn_seconds': float(((abs(cmd[pm, 2])>.015)&(np.linalg.norm(cmd[pm, :2], axis=1)<.015)).sum()*.02),
            'actual_actor_drive_seconds': float((abs(cmd[pm, 0])>.015).sum()*.02),
            'actual_actor_signed_wz_integral_rad': float(cmd[pm, 2].sum()*.02),
            'native_heading_change_rad': float(nyaw[nm][-1]-nyaw[nm][0]),
            'phase_durations_sim_s': phase_durations, 'phase_transitions': transitions,
            'first_recorded_drive_vx_above_point05_world_s': first_drive['ros_sim_time'] if first_drive else None,
            'first_drive_to_original_goal_deadline_remaining_s': end-first_drive['ros_sim_time'] if first_drive else None,
            'align': align_metric, 'last_raw_slam_return_center_error_xy_m': float(np.linalg.norm(last_pose[:2]-goal[:2])),
            'input_sha256': {n: sha(run / n) for n in ['telemetry.jsonl', 'actuator.jsonl', 'navigation_slam_poses.jsonl',
                'navigation_status.jsonl', 'navigation_request.json', 'navigation_profile.json', 'navigation_command_history.jsonl', 'summary_dynamic_obstacle_independent.json']},
            'interpretation': 'All command integration is actual CPU actor input after consumer health/TTL/slew, not requested time. '
                'Physical heading is native quaternion diagnostic only; navigation heading is original SLAM. '
                'No performance for a proposed faster yaw limit is predicted or claimed.',
        }
        records.append(record)
        axh, axw, axv, axf = axes[:, column]
        axh.plot(nt[nm]-start, nyaw[nm]-nyaw[nm][0], color='#dc2626', linewidth=.8, label='Actual native heading change')
        axh.plot(st[sm]-start, syaw[sm]-syaw[sm][0], color='#2563eb', linewidth=.8, label='Original SLAM heading change')
        axh.set_ylabel('Heading change (rad)')
        axh.set_title(run.name.rsplit('_', 1)[-1] + ': ' + final['state'] + ', return stage')
        axw.plot(pt[pm]-start, request[pm, 2], color='gray', linewidth=.7, label='Original requested wz')
        axw.plot(pt[pm]-start, cmd[pm, 2], color='#2563eb', linewidth=.8, label='Actual actor wz')
        axw.plot(nt[nm]-start, nw[nm, 2], color='#dc2626', alpha=.45, linewidth=.5, label='Actual body wz (200Hz)')
        axw.set_ylabel('Yaw rate (rad/s)')
        axv.plot(pt[pm]-start, cmd[pm, 0], color='#2563eb', linewidth=.8, label='Actual actor vx')
        axv.plot(nt[nm]-start, nv[nm, 0], color='#dc2626', alpha=.5, linewidth=.5, label='Actual body COM vx')
        axv.set_ylabel('Forward body velocity (m/s)')
        flags = np.asarray([r['command_expired'] for r in sampled], dtype=float)
        axf.step(pt[pm]-start, flags, color='#d97706', linewidth=.6, where='post', label='Actual unhealthy-or-stale flag')
        axf.set_ylabel('Flag (1=rejected input)')
        axf.set_yticks([0, 1])
        axf.set_xlabel('Actual simulation seconds since return stage')
        for axis in axes[:, column]:
            axis.set_xlim(0, 90.5)
            axis.grid(alpha=.15)
            if first_drive:
                axis.axvline(first_drive['ros_sim_time']-start, color='green', linestyle='--', linewidth=.8)
            axis.legend(fontsize=7, loc='upper left')
        archive[f'run{column}_policy_world_s'] = pt[pm]
        archive[f'run{column}_requested'] = request[pm]
        archive[f'run{column}_actor_command'] = cmd[pm]
        archive[f'run{column}_rejected_input_flag'] = flags
        archive[f'run{column}_native_world_s'] = nt[nm]
        archive[f'run{column}_native_heading'] = nyaw[nm]
        archive[f'run{column}_native_body_velocity'] = nv[nm]
        archive[f'run{column}_native_body_angular_velocity'] = nw[nm]
        archive[f'run{column}_original_slam_world_s'] = st[sm]
        archive[f'run{column}_original_slam_heading'] = syaw[sm]
        originals[run / 'summary_dynamic_obstacle_independent.json'] = (run / 'summary_dynamic_obstacle_independent.json').read_bytes()
    fig.suptitle('Actual return heading / actor command / physical response / input protection\n'
                 'Green dashed: first recorded drive vx>0.05; exact original 90s deadline retained', fontsize=11)
    plot = output / 'actual_dynamic_return_turnaround_comparison.png'
    fig.savefig(plot, dpi=160)
    plt.close(fig)
    np.savez_compressed(output / 'actual_turnaround_arrays.npz', **archive)
    receipt = {'schema': 1, 'status': 'recorded_diagnostic', 'runs': records,
        'analyzer_sha256': sha(__file__), 'output_sha256': {p.name: sha(p) for p in [plot, output / 'actual_turnaround_arrays.npz']},
        'original_receipt_bytes_preserved': all(p.read_bytes() == b for p, b in originals.items()),
        'truth_scope': 'Only native physical heading/velocity diagnosis; never control, steering, planner or region feedback',
        'proposed_yaw_limit_performance': 'unverified; prospective experiment required', 'no_acceptance_or_runtime_changed': True}
    (output / 'turnaround_diagnosis.json').write_text(json.dumps(receipt, indent=2, ensure_ascii=False, allow_nan=False)+'\n')
    print(json.dumps({'output': str(output), 'runs': [{k: r[k] for k in ['run_id','return_world_window_s','align','actual_actor_drive_seconds']} for r in records]}))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', type=Path, default=ROOT / 'test_results/dynamic_return_turnaround_20261004')
    args = parser.parse_args()
    evaluate(args.runs, args.output)
