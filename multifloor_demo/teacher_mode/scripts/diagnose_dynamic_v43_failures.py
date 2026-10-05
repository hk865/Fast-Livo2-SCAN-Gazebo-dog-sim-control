#!/usr/bin/env python3
"""Append actual V43 failure diagnostics; never changes an acceptance receipt."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def rows(path):
    with Path(path).open() as stream:
        for line in stream:
            if line.strip():
                yield json.loads(line)


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1048576), b''):
            h.update(chunk)
    return h.hexdigest()


def diagnose(run):
    run = Path(run).resolve()
    names = ['summary.json', 'summary_dynamic_obstacle_independent.json']
    originals = {n: (run/n).read_bytes() for n in names}
    summary = json.loads(originals[names[1]])
    output = run/'dynamic_failure_diagnostics'
    if output.exists():
        raise ValueError('Refusing to overwrite earlier failure diagnostics')
    output.mkdir()
    telemetry = list(rows(run/'telemetry.jsonl'))
    guards = list(rows(run/'navigation_guard_history.jsonl'))
    status = list(rows(run/'navigation_status.jsonl'))
    policy_time = np.asarray([r['world_sim_time'] for r in telemetry])
    actor = np.asarray([r['command'] for r in telemetry])
    request = np.asarray([r['requested'] for r in telemetry])
    park = summary['checks']['actual_continuous_teacher_parking_while_box_blocked']
    start, end = park['world_window_s']
    native_compact, native_window = [], []
    for record in rows(run/'actuator.jsonl'):
        if record.get('kind') != 'physics_step':
            continue
        time = record['t']-.005
        velocity = record['body_lin_vel_com']
        native_compact.append([time, velocity[0], velocity[1], record['body_ang_vel'][2]])
        if start-1e-10 <= time <= end+1e-10:
            native_window.append(record)
    speed = np.asarray([np.linalg.norm(r['body_lin_vel_com'][:2]) for r in native_window])
    peak_idx = int(np.argmax(speed))
    peak = native_window[peak_idx]
    peak_time = peak['t']-.005
    near = [r for r in native_window if abs(r['t']-.005-peak_time) <= .050000001]
    pn = [r for r in telemetry if abs(r['world_sim_time']-peak_time) <= .100000001]
    violating = np.flatnonzero(speed > park['limits']['planar_speed_max_mps'])
    parking = {
        'schema': 1, 'status': 'failed', 'scope': 'Original frozen fixed parking window diagnostic only',
        'world_window_s': [start, end], 'native_physics_time': 'Original native PreUpdate t minus0.005s',
        'selection': park['selection'], 'window_was_not_shifted_or_reselected': True,
        'peak_world_s': peak_time, 'peak_planar_speed_mps': float(speed[peak_idx]),
        'frozen_maximum_mps': park['limits']['planar_speed_max_mps'],
        'excess_mps': float(speed[peak_idx]-park['limits']['planar_speed_max_mps']),
        'samples_above_frozen_limit': len(violating), 'sample_count': len(native_window),
        'first_last_violation_world_s': [native_window[int(violating[0])]['t']-.005,
            native_window[int(violating[-1])]['t']-.005] if len(violating) else None,
        'requested_and_actor_zero_in_window': all(max(abs(x) for x in r['requested']+r['command']) < 1e-8
            for r in telemetry if start <= r['world_sim_time'] <= end),
        'cpu_actor_continuous_in_window': all(r['actor_inferred_this_frame'] and r['fault'] is None
            for r in telemetry if start <= r['world_sim_time'] <= end),
        'native_peak_record': peak, 'native_neighbors': near, 'actor_neighbors': pn,
        'interpretation': 'A genuine native COM speed excursion while the continuous Teacher receives zero velocity. '
            'The qualifying initial1s low-speed span does not exempt the next fixed5s window from the maximum speed limit. '
            'No later quieter interval replaces the frozen window; no underlying policy or adapter cause is asserted.'}
    (output/'parking_peak_diagnostic.json').write_text(json.dumps(parking, indent=2)+'\n')
    evidence = summary['checks']['actual_integer_stamped_native_guard_inputs_and_replay']['evidence']
    replay_idx = {r['sequence']: r for r in evidence['replays']}
    selected = [r for r in guards if 25.5 <= r['compute_sim_time_s'] <= 27.8]
    matched = [replay_idx[r['sequence']] for r in selected]
    gap_rows = []
    for before, after in zip(selected, selected[1:]):
        gap = (after['compute_ros_clock_ns']-before['compute_ros_clock_ns'])/1e9
        if gap > .300000001:
            gap_rows.append({'before_sequence': before['sequence'], 'after_sequence': after['sequence'],
                'before_world_s': before['compute_sim_time_s'], 'after_world_s': after['compute_sim_time_s'],
                'actual_gap_s': gap, 'previous_runtime_clear_start_s': before['clear_start_after_s'],
                'after_runtime_clear_start_s': after['clear_start_after_s']})
    trajectories = [json.loads(p.read_text()) for p in sorted((run/'navigation_trajectories').glob('*.json'))]
    clearance = summary['checks']['actual_registered_cloud_corridor_clear_after_withdrawal']
    phase = summary['checks']['actual_box_entry_block_and_withdrawal']['phase_times_s']
    gap_diagnostic = {'schema': 1, 'status': 'failed', 'scope': 'Original frozen clear/newSCAN failure diagnosis',
        'sensor_clear_eligibility_begins_at_leaving_world_s': phase['leaving'],
        'model_fully_withdrawn_world_s_diagnostic_only': phase['clear'],
        'no_model_truth_used_to_fill_sensor_clear': True,
        'original_maximum_actual_guard_gap_s': .300000001,
        'actual_guard_gaps_over_limit': gap_rows,
        'independent_clear_confirmation_world_s': clearance['first_continuous_clear_confirmation_world_s'],
        'actual_after_gap_trajectory_reference_stamps': [{'trajectory_id': p['trajectory_id'],
            'reference_stamp': p['reference_stamp']} for p in trajectories if p.get('waypoint_index') == 0],
        'selected_actual_guard_records': selected, 'original_xyz_independent_replay_receipts': matched,
        'all_selected_actual_xyz_replays_passed': all(r['status'] == 'passed' for r in matched),
        'original_xyz_sha256': {r['original_xyz_file']: r['original_xyz_sha256'] for r in matched if r['status']=='passed'},
        'interpretation': 'Clear is allowed during leaving. The actual0.55s gap resets the independent1s span. '
            'Runtime retained the earlier clear timer and requested post-obstacle SCAN at26.69s, before independently '
            'confirmed27.69s. The frozen failure is not caused by requiring the model to be fully withdrawn.'}
    (output/'guard_gap_release_diagnostic.json').write_text(json.dumps(gap_diagnostic, indent=2)+'\n')
    running = [r for r in status if r['state']=='running' and r.get('waypoint_index') == 0 and r.get('region_arrival_evidence')]
    final = next(r for r in status if r['state']=='failed')
    goal = np.asarray(json.loads((run/'navigation_request.json').read_text())['goals'][0]['center'])
    time = np.asarray([r['ros_sim_time'] for r in running])
    raw_distance = np.asarray([np.linalg.norm(np.asarray(r['pose'])[:2]-goal[:2]) for r in running])
    tracking_distance = np.asarray([np.linalg.norm(np.asarray(r['tracking_pose'])[:2]-goal[:2])
        if r.get('tracking_pose') is not None else np.nan for r in running])
    dwell = np.asarray([r['region_arrival_evidence']['dwell_ns']/1e9 for r in running])
    before = (policy_time>=40)&(policy_time<=final['ros_sim_time'])
    sampled = [r for r, good in zip(telemetry, before) if good]
    command_history = {r['sequence']: r for r in rows(run/'navigation_command_history.jsonl')}
    rejected = [command_history.get(r['navigation_envelope'].get('sequence'))
        for r in sampled if r['command_expired']]
    rejected = [r for r in rejected if r is not None]
    native_compact = np.asarray(native_compact)
    nm = (native_compact[:,0]>=40)&(native_compact[:,0]<=final['ros_sim_time'])
    inside = [r for r in running if r['region_arrival_evidence']['control_region_inside']]
    arrival = {'schema': 1, 'status': 'failed', 'scope': 'Actual outbound stage; original90s deadline and.17m/.6s criteria',
        'final_navigation_world_s': final['ros_sim_time'], 'final_message': final.get('message'),
        'raw_goal_control_radius_m': .17, 'raw_pose_required_dwell_s': .6,
        'minimum_raw_slam_xy_error_m': float(np.min(raw_distance)),
        'minimum_tracking_pose_xy_error_m': float(np.nanmin(tracking_distance)),
        'maximum_recorded_original_raw_stamp_dwell_s': float(np.max(dwell)),
        'running_status_samples': len(running), 'inside_control_band_status_samples': len(inside),
        'inside_control_band_with_zero_controller_command_samples': sum(max(abs(x) for x in r['command'])<1e-8 for r in inside),
        'last_approach_world_window_s': [40., final['ros_sim_time']],
        'actual_actor_vx_mean_mps': float(actor[before,0].mean()),
        'actual_body_com_vx_mean_mps': float(native_compact[nm,1].mean()),
        'expired_or_unhealthy_worker_frames': sum(r['command_expired'] for r in sampled),
        'worker_frames': len(sampled),
        'strict_original_source_age_invalid_frames': sum(not(-.05<=r['navigation_envelope']['wall_age_s']<=.3 and
            -.05<=r['navigation_envelope']['sim_age_s']<=.3) for r in sampled),
        'producer_reason_counts_for_rejected_worker_frames': dict(Counter(r.get('reason') for r in rejected)),
        'raw_arrival_reason_status_counts': dict(Counter(r['region_arrival_evidence']['reason'] for r in running)),
        'actual_return_stage_observed': any(r.get('waypoint_index')==1 for r in status),
        'interpretation': 'The raw SLAM pose genuinely enters the control band but no recorded dwell reaches.6s. '
            'The old controller zeroed on entry before dwell confirmation; boundary excursions and health holds interrupt '
            'continuous arrival. Tracking pose does not supply a false arrival. This is a diagnosis, not a retroactive pass.'}
    (output/'arrival_boundary_diagnostic.json').write_text(json.dumps(arrival, indent=2)+'\n')
    fig, axes = plt.subplots(4,1,figsize=(12,9),sharex=True,constrained_layout=True)
    axes[0].plot(time,raw_distance,label='Original raw SLAM XY goal error',color='#2563eb')
    axes[0].plot(time,tracking_distance,label='Recorded predicted tracking XY goal error',color='#9333ea',alpha=.7)
    axes[0].axhline(.17,color='black',linestyle='--',label='Frozen raw control band0.17m')
    axes[0].set_ylabel('Goal distance (m)');axes[0].set_ylim(.14,.25);axes[0].legend(fontsize=8)
    axes[1].plot(time,dwell,color='#2563eb',label='Dwell from original raw SLAM stamps')
    axes[1].axhline(.6,color='black',linestyle='--');axes[1].set_ylabel('Raw dwell (s)')
    axes[2].plot(policy_time,request[:,0],color='gray',linewidth=.6,label='Requested vx')
    axes[2].plot(policy_time,actor[:,0],color='#2563eb',linewidth=.7,label='Actual CPU actor vx')
    axes[2].plot(native_compact[:,0],native_compact[:,1],color='#dc2626',alpha=.4,linewidth=.5,label='Actual native body COM vx')
    axes[2].set_ylabel('Velocity (m/s)');axes[2].legend(fontsize=8)
    axes[3].step(policy_time,[r['command_expired'] for r in telemetry],color='#d97706',linewidth=.5,where='post')
    axes[3].set_ylabel('Stale or unhealthy');axes[3].set_xlabel('Actual world simulation time (s)')
    for axis in axes:axis.set_xlim(40,final['ros_sim_time']+.5);axis.grid(alpha=.15)
    fig.suptitle('V43 actual outbound boundary/dwell failure; no threshold changed')
    figure=output/'actual_arrival_boundary_failure.png';fig.savefig(figure,dpi=150);plt.close(fig)
    np.savez_compressed(output/'actual_failure_diagnostic_arrays.npz',status_world_s=time,raw_slam_xy_error=raw_distance,
        tracking_xy_error=tracking_distance,raw_stamp_dwell_s=dwell,policy_world_s=policy_time,requested=request,
        actor_command=actor,native_world_s=native_compact[:,0],native_body_com_vx=native_compact[:,1],
        native_body_com_vy=native_compact[:,2],native_wz=native_compact[:,3])
    preserved=all((run/n).read_bytes()==data for n,data in originals.items())
    manifest={'schema':1,'status':'failed','run_dir':str(run),'original_receipts_preserved':preserved,
        'input_sha256':{n:sha(run/n)for n in names+['telemetry.jsonl','actuator.jsonl','navigation_guard_history.jsonl',
            'navigation_status.jsonl','navigation_request.json','navigation_command_history.jsonl','dynamic_protocol.json']},
        'output_sha256':{p.name:sha(p)for p in output.iterdir()if p.is_file()},'analyzer_sha256':sha(__file__),
        'acceptance_re_evaluated_or_overwritten':False,'global_acceptance_overwritten':False}
    (output/'diagnostic_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (output/'diagnose_dynamic_v43_failures.executed.py').write_bytes(Path(__file__).read_bytes())
    print(json.dumps({'output':str(output),'preserved':preserved,'parking_peak_world_s':peak_time,
        'parking_peak_mps':float(speed[peak_idx]),'raw_dwell_max_s':float(np.max(dwell))}))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('run',type=Path)
    diagnose(parser.parse_args().run)
