#!/usr/bin/env python3
"""Read-only SCAN/PID heading diagnosis, with truth used only offline."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
sys.dont_write_bytecode = True
import numpy as np

HERE = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_rows(path):
    return [json.loads(line) for line in Path(path).open() if line.strip()]


def rotation(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
        [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
        [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def steering_geometry(row, samples):
    # Exact frozen follow_trajectory geometry, using actual raw SLAM tracking pose.
    pose = np.asarray(row['control_pose'])
    nearest = int(np.argmin(np.linalg.norm(samples[:, :2]-pose[:2], axis=1)))
    index, length = nearest, 0.
    while index+1 < len(samples) and length < .8:
        length += float(np.linalg.norm(samples[index+1, :2]-samples[index, :2])); index += 1
    projection, target = samples[nearest], samples[index]
    tangent = target[:2]-projection[:2]
    if np.linalg.norm(tangent) < 1e-6:
        previous = nearest-1
        while previous > 0 and np.linalg.norm(projection[:2]-samples[previous, :2]) < .10:
            previous -= 1
        if previous >= 0: tangent = projection[:2]-samples[previous, :2]
    if np.linalg.norm(tangent) < 1e-6:
        return None
    unit = tangent/np.linalg.norm(tangent)
    offset = projection[:2]-pose[:2]
    cross = offset-unit*float(offset@unit)
    direction = .8*unit+cross
    tangent_heading = math.atan2(unit[1], unit[0])
    combined = math.atan2(direction[1], direction[0])
    return {'nearest_index': nearest, 'target_index': index, 'total_samples': len(samples),
        'projection': projection.tolist(), 'target': target.tolist(), 'tangent_xy': tangent.tolist(),
        'tangent_unit_xy': unit.tolist(), 'tangent_heading_raw_SLAM': tangent_heading,
        'cross_track_xy': cross.tolist(), 'cross_track_norm_m': float(np.linalg.norm(cross)),
        'combined_heading_raw_SLAM': combined, 'cross_induced_heading_rad': wrap(combined-tangent_heading),
        'heading_reconstruction_error_rad': wrap(combined-row['heading'])}


def latest(rows, stamp, key):
    return max((row for row in rows if row[key] <= stamp), key=lambda row: row[key])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    run = parser.parse_args().run.resolve()
    if not (run/'runtime_manifest.json').is_file(): raise RuntimeError('Completed run required')
    output = HERE/run.name; output.mkdir()
    pid = read_rows(run/'navigation_pid_history.jsonl')
    execution = read_rows(run/'telemetry.jsonl')
    status = read_rows(run/'navigation_status.jsonl')
    raw_slam = {row['stamp_ns']: row for row in read_rows(run/'navigation_slam_poses.jsonl')}
    anchor = json.loads((run/'navigation_anchor.json').read_text())
    summary = json.loads((run/'summary_pid_navigation_independent.json').read_text())
    with np.load(run/'pid_navigation_independent_arrays.npz', allow_pickle=False) as arrays:
        nt = arrays['native_world_time_s'].copy(); nq = arrays['native_quaternion_wxyz'].copy()
        pos = arrays['native_position'].copy(); drive = arrays['drive_mask'].copy()
        heading_error = arrays['drive_heading_error_rad'].copy()
    nyaw = np.unwrap(np.arctan2(2*(nq[:, 0]*nq[:, 3]+nq[:, 1]*nq[:, 2]), 1-2*(nq[:, 2]**2+nq[:, 3]**2)))
    peak = np.flatnonzero(drive)[np.argmax(heading_error[drive])]
    world = float(nt[peak]); native_stamp = world+.005
    causal_pid = latest(pid, round(world*1e9), 'compute_ros_clock_ns')
    causal_updated = latest([row for row in pid if row['pid'].get('updated') is True], round(world*1e9), 'compute_ros_clock_ns')
    actor = latest(execution, world, 'world_sim_time')
    nav = latest(status, world, 'ros_sim_time')
    # Single frozen full SE3 at existing raw SLAM anchor, strictly offline.
    anchor_raw = raw_slam[anchor['pose_stamp_ns']]
    ai = np.searchsorted(nt, anchor['pose_stamp_ns']/1e9, side='right')-1
    aq = anchor_raw['quaternion']; anchor_rotation = rotation([aq[3], *aq[:3]])
    frame_rotation = rotation(nq[ai])@anchor_rotation.T
    frame_translation = pos[ai]-frame_rotation@np.asarray(anchor_raw['position'])
    def aligned_direction_heading(raw_heading):
        v = frame_rotation@np.array([math.cos(raw_heading), math.sin(raw_heading), 0.])
        return math.atan2(v[1], v[0])
    def aligned_pose_yaw(row):
        matrix = frame_rotation@np.asarray(row['control_rotation'])
        return math.atan2(matrix[1, 0], matrix[0, 0])
    trajectories, points, input_refs = {}, [], {}
    for row in pid:
        path = run/row['trajectory_archive_file']
        if path not in trajectories:
            with np.load(path, allow_pickle=False) as saved: trajectories[path] = saved['samples'].copy()
            input_refs[str(path)] = sha(path)
        geometry = steering_geometry(row, trajectories[path])
        if geometry is None: continue
        source_pose = raw_slam[row['control_pose_stamp_ns']]
        bound_pose_error = float(np.max(np.abs(np.asarray(row['control_pose'])-source_pose['position'])))
        goal_vector = np.asarray(row['goal'])[:2]-np.asarray(anchor['origin'])[:2]
        nominal = math.atan2(goal_vector[1], goal_vector[0])
        unit = np.asarray(geometry['tangent_unit_xy']); normal = np.array([-unit[1], unit[0]])
        point = {'sequence': row['sequence'], 'compute_world_s': row['compute_ros_clock_ns']/1e9,
            'source_pose_s': row['control_pose_stamp_ns']/1e9,
            'SLAM_age_sim_s': (row['compute_ros_clock_ns']-row['control_pose_stamp_ns'])/1e9,
            'IMU_age_sim_s': (row['compute_ros_clock_ns']-row['imu_stamp_ns'])/1e9,
            'mode': row['mode'], 'trajectory_id': row['trajectory_id'], 'pid_updated': row['pid'].get('updated'),
            'yaw_raw_SLAM': row['yaw'], 'desired_heading_raw_SLAM': row['heading'],
            'nominal_route_heading_raw_SLAM': nominal, 'aligned_SLAM_yaw_offline': aligned_pose_yaw(row),
            'aligned_combined_heading_offline': aligned_direction_heading(geometry['combined_heading_raw_SLAM']),
            'aligned_tangent_heading_offline': aligned_direction_heading(geometry['tangent_heading_raw_SLAM']),
            'tangent_vs_nominal_route_rad': wrap(geometry['tangent_heading_raw_SLAM']-nominal),
            'yaw_command': row['command_after_slew'][2], 'body_vx_command': row['command_after_slew'][0],
            'body_vy_command': row['command_after_slew'][1], 'world_command_normal_to_SCAN_mps': float(np.asarray(row['actual_pid_world_direction'])@normal),
            'raw_source_pose_binding_max_abs_error_m': bound_pose_error, **geometry}
        points.append(point)
    by_sequence = {point['sequence']: point for point in points}
    peak_geometry = by_sequence[causal_pid['sequence']]
    updated_geometry = by_sequence[causal_updated['sequence']]
    raw_pose_ns = causal_updated['control_pose_stamp_ns']
    pi = np.searchsorted(nt, raw_pose_ns/1e9, side='right')-1
    raw_pose = np.asarray(causal_updated['control_pose'])
    aligned_position = frame_rotation@raw_pose+frame_translation
    same_state_delta = 1.1*wrap(updated_geometry['tangent_heading_raw_SLAM']-causal_updated['yaw'])-causal_updated['pid']['P_world_xy_yaw'][2]
    exceeded = drive & (heading_error > .35)
    peak_detail = {'native_world_time_s': world, 'native_original_PreUpdate_t_s': native_stamp,
        'state_time_offset_s': -.005, 'native_position': pos[peak].tolist(), 'native_yaw_rad': float(nyaw[peak]),
        'original_max_drive_heading_error_rad': float(heading_error[peak]), 'causal_PID_row': causal_pid,
        'causal_updated_PID_row': causal_updated, 'causal_actor_row': {k: actor.get(k) for k in ['world_sim_time', 'requested', 'command', 'navigation_envelope', 'state', 'body_ang_vel']},
        'causal_navigation_status': {k: nav.get(k) for k in ['ros_sim_time', 'pose_age', 'cloud_age', 'raw_imu_age', 'raw_imu_stamp_age', 'actual_cloud_message_stamp_ns', 'alignment_phase', 'alignment_hold', 'obstacle_hold', 'steering']},
        'reconstructed_SCAN_geometry': peak_geometry,
        'same_source_stamp_native_match_s': float(nt[pi]),
        'same_stamp_full_SE3_aligned_SLAM_native_xy_error_m': float(np.linalg.norm(pos[pi, :2]-aligned_position[:2])),
        'same_stamp_full_SE3_aligned_SLAM_native_yaw_error_rad': wrap(float(nyaw[pi])-aligned_pose_yaw(causal_updated)),
        'same_state_tangent_only_yaw_P_change_radps': same_state_delta,
        'same_state_tangent_only_P_plus_old_I_D_before_new_slew_radps': causal_updated['desired_body_command'][2]+same_state_delta,
        'counterfactual_scope': 'Same recorded state only, old I/D held for diagnosis. It is not a replayed new HeadingGate/PID or a predicted closed-loop result.'}
    result = {'schema': 'Teacher_PID_6m_heading_failure_diagnostic/v1', 'run': str(run),
        'source': {'path': str(Path(__file__).resolve()), 'sha256': sha(__file__)},
        'original_summary_status': summary['status'], 'original_route_check': summary['checks']['physical_route_tracking_and_actual_motion'],
        'original_summary_not_modified': True, 'peak': peak_detail,
        'drive_heading_exceedance': {'native_samples_above_original_0p35': int(exceeded.sum()),
            'observed_duration_sample_sum_s': float(exceeded.sum()*.005),
            'first_world_time_s': float(nt[np.flatnonzero(exceeded)[0]]), 'last_world_time_s': float(nt[np.flatnonzero(exceeded)[-1]])},
        'reconstruction': {'all_PID_pose_binding_max_abs_error_m': max(point['raw_source_pose_binding_max_abs_error_m'] for point in points),
            'drive_heading_reconstruction_max_abs_error_rad': max(abs(point['heading_reconstruction_error_rad']) for point in points if point['mode']=='drive'),
            'raw_checked_SCAN_samples_used': True},
        'offline_only_SE3': {'raw_anchor_stamp_ns': anchor['pose_stamp_ns'], 'native_causal_anchor_world_s': float(nt[ai]),
            'rotation': frame_rotation.tolist(), 'translation': frame_translation.tolist(), 'never_used_in_navigation_or_setpoint': True},
        'direct_evidence': ['The same cross-track vector changes steering desiredYaw and contributes to XY PID target error.',
            'The peak is near the checked SCAN endpoint; the secant tangent itself already bends substantially relative to the frozen raw SLAM mission segment.',
            'Causal source stamps are fresh; exact raw pose binding and same-stamp single-SE3 yaw comparison do not indicate a gross phase/frame mismatch.'],
        'unproven': ['Dual correction alone is not established as the unique failure cause.', 'A tangent-only new closed loop may change future SCAN trajectories; fixed-state tangent replay cannot promise acceptance.', 'Teacher yaw response bias/transient remains possible; this audit does not prove the actor is error-free.'],
        'new_version_plan_without_runtime_edit': ['Prospective Teacher-only V5 heading_reference=route_leg uses the immutable activated actual raw SLAM segment_start-to-goal direction. Original SCAN target and trajectory remain unchanged.',
            'Use that same reference in HeadingGate and yaw PID; keep XY PID target error/lateral command and actual-motion native cloud guard unchanged.',
            'Freeze new provenance/schema and offline tests for duplicate/stale/hold/exhausted/turn/endpoint; unchanged gains, TTL, speed limits, frozen physical route criterion.',
            'Run matched flat_short then 6m/roundtrip trials; log original SCAN heading, immutable actual-SLAM leg source/header, chosen reference and actual response. No truth-derived direction.',
            'The same-state pure-SCAN-tangent alternative still exceeds the nominal route heading criterion near this peak; it is retained only as diagnostic evidence and is not the chosen V5 reference.'],
        'series': points, 'input_sha256': input_refs, 'runtime_source_modified': False, 'ROS_Gazebo_or_signals': False}
    for name in ['pid_navigation_independent_arrays.npz', 'navigation_pid_history.jsonl', 'navigation_slam_poses.jsonl', 'telemetry.jsonl', 'navigation_status.jsonl', 'navigation_anchor.json', 'summary_pid_navigation_independent.json']:
        result['input_sha256'][str(run/name)] = sha(run/name)
    with (output/'diagnostic.json').open('x') as stream: json.dump(result, stream, indent=2); stream.write('\n')
    import matplotlib; matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    window = (nt >= 40) & (nt <= 52)
    axes[0].plot(nt[window], nyaw[window], label='actual native yaw, offline')
    selected = [point for point in points if 40 <= point['compute_world_s'] <= 52]
    pt = [point['compute_world_s'] for point in selected]
    for key, label in [('aligned_combined_heading_offline', 'recorded combined yaw target'), ('aligned_tangent_heading_offline', 'SCAN tangent only'), ('aligned_SLAM_yaw_offline', 'actual SLAM yaw, offline alignment')]:
        axes[0].plot(pt, [point[key] for point in selected], label=label)
    axes[0].axhline(.35, color='red', linestyle='--', label='original drive criterion')
    axes[0].set_ylabel('Heading / yaw (rad)'); axes[0].legend(fontsize=8)
    axes[1].plot(pt, [point['yaw_command'] for point in selected], label='actual yaw command')
    axes[1].plot(pt, [point['body_vy_command'] for point in selected], label='actual body lateral command')
    axes[1].set_ylabel('Command rad/s or m/s'); axes[1].legend(fontsize=8)
    axes[2].plot(pt, [point['cross_induced_heading_rad'] for point in selected], label='cross-track added yaw (rad)')
    axes[2].plot(pt, [point['cross_track_norm_m'] for point in selected], label='same cross-track norm (m)')
    axes[2].plot(pt, [point['tangent_vs_nominal_route_rad'] for point in selected], label='SCAN tangent minus raw mission heading (rad)')
    axes[2].set_xlabel('Actual simulation world time (s)'); axes[2].legend(fontsize=8)
    for axis in axes: axis.axvline(world, color='black', linestyle=':'); axis.grid(alpha=.25)
    fig.suptitle('Frozen Teacher PID V4 6m run: heading diagnosis, no new controller execution')
    fig.tight_layout(); fig.savefig(output/'heading_diagnostic.png', dpi=150); plt.close(fig)
    with (output/'figure_manifest.json').open('x') as stream:
        json.dump({'figure_sha256': sha(output/'heading_diagnostic.png'), 'diagnostic_sha256': sha(output/'diagnostic.json'),
            'scope': 'Standard plot of original receipts and same-state SCAN reconstruction; truth alignment offline only.'}, stream, indent=2); stream.write('\n')
    print(json.dumps({'diagnostic': str(output/'diagnostic.json'), 'sha256': sha(output/'diagnostic.json'),
        'peak_world_s': world, 'max_heading_rad': float(heading_error[peak]),
        'pose_binding_error_m': result['reconstruction']['all_PID_pose_binding_max_abs_error_m'],
        'steering_reconstruction_error_rad': result['reconstruction']['drive_heading_reconstruction_max_abs_error_rad']}))


if __name__ == '__main__': main()
