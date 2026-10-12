#!/usr/bin/env python3
"""Independent truth-feedback Teacher benchmark; never SLAM acceptance."""
from __future__ import annotations
import argparse
import hashlib
import importlib.util
import json
import math
import re
import sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(path):
    output = []
    with Path(path).open() as stream:
        for index, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                output.append(json.loads(line))
            except (ValueError, TypeError) as error:
                raise ValueError(f'{Path(path).name}:{index}: invalid original JSON') from error
    return output


def array(data, key, width=None):
    a = np.asarray([r[key] for r in data], dtype=float)
    if width is not None and a.shape != (len(data), width):
        raise ValueError('Invalid recorded array ' + key)
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite recorded array ' + key)
    return a


def rotation_wxyz(q):
    if q.shape[1:] != (4,) or np.any(abs(np.sum(q*q, axis=1)-1) > .02):
        raise ValueError('Invalid recorded quaternion')
    q = q / np.linalg.norm(q, axis=1)[:, None]
    w, x, y, z = q.T
    return np.stack([1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                     2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                     2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)], axis=1).reshape(-1, 3, 3)


def angular_difference(a, b):
    return np.arctan2(np.sin(a-b), np.cos(a-b))


def polyline_distance(points, route):
    best = np.full(len(points), np.inf)
    segment = np.full(len(points), -1, dtype=int)
    for i, (a, b) in enumerate(zip(route[:-1, :2], route[1:, :2])):
        delta = b-a
        length2 = float(delta @ delta)
        if length2 <= 1e-12:
            raise ValueError('Degenerate requested route XY leg')
        frac = np.clip((points[:, :2]-a) @ delta / length2, 0, 1)
        distance = np.linalg.norm(points[:, :2]-(a+frac[:, None]*delta), axis=1)
        replace = distance < best
        best[replace], segment[replace] = distance[replace], i
    return best, segment


def control_rows(data):
    """Keep actual update/hold/initialization rows; never infer updates from kind."""
    output = []
    for row in data:
        tag = row.get('kind', row.get('type', row.get('event')))
        if tag in ('controller_update', 'controller_hold') or (tag is None and 'control_t_s' in row):
            output.append(row)
    if not output:
        raise ValueError('No original controller update/hold rows')
    for row in output:
        for key in ('control_t_s', 'feedback_time_s', 'elapsed_s'):
            if not math.isfinite(float(row[key])):
                raise ValueError('Nonfinite control ' + key)
        if type(row.get('controller_updated')) is not bool:
            raise ValueError('Actual controller_updated boolean required')
        command = np.asarray(row['command_body'], float)
        if command.shape != (3,) or not np.isfinite(command).all():
            raise ValueError('Invalid command_body')
        if not isinstance(row.get('mode'), str):
            raise ValueError('Missing actual controller mode')
        reference_fields = ('reference_xy', 'reference_yaw', 'reference_velocity_world')
        complete = all(key in row for key in reference_fields)
        if not complete:
            if row['mode'] != 'initializing' or row['controller_updated']:
                raise ValueError('Noninitial control row lacks recorded reference')
            continue
        if not math.isfinite(float(row['reference_yaw'])):
            raise ValueError('Nonfinite reference_yaw')
        for key, width in (('reference_xy', 2), ('reference_velocity_world', 3)):
            v = np.asarray(row[key], float)
            if v.shape != (width,) or not np.isfinite(v).all():
                raise ValueError('Invalid control ' + key)
    return output


def native_evidence(run):
    """Original PreUpdate state is t-dt; tau is the new feed to physics at t.

    Contacts, pose and qd describe the cached previous physics state. The tau
    extrema cover each newly applied command; this is not measured motor torque.
    Clearance is an independent downward all-static SDF ray, not policy scan.
    """
    native = [r for r in rows(run/'actuator.jsonl') if r.get('kind') == 'physics_step']
    if len(native) < 2:
        raise ValueError('Missing original native physics evidence')
    nt = array(native, 't')
    ndt = array(native, 'dt')
    iteration = array(native, 'iteration')
    npos = array(native, 'position', 3)
    nq = array(native, 'quaternion_wxyz', 4)
    nrot = rotation_wxyz(nq)
    nrp = np.column_stack((np.arctan2(nrot[:, 2, 1], nrot[:, 2, 2]), np.arcsin(np.clip(-nrot[:, 2, 0], -1, 1))))
    ncontacts = array(native, 'contacts', 5)
    if np.any(ncontacts < 0):
        raise ValueError('Unavailable native contacts')
    policy_source = run/'sources/policy/observation.py'
    module_name = '_truth_evaluator_terrain_' + hashlib.sha256(str(run).encode()).hexdigest()[:12]
    spec = importlib.util.spec_from_file_location(module_name, policy_source)
    module = importlib.util.module_from_spec(spec)
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
        terrain = module.TerrainHeightMap.from_sdf(run/'world.sdf')
        ground = terrain.height(npos[:, :2], ray_start_z=npos[:, 2])
    finally:
        sys.dont_write_bytecode = previous
        sys.modules.pop(module_name, None)
    if not np.isfinite(ground).all():
        raise ValueError('Unavailable all-static native clearance ray')
    return native, {'world_t': nt, 'state_t': nt-ndt, 'dt': ndt, 'iteration': iteration,
                    'position': npos, 'quaternion_wxyz': nq, 'roll_pitch': nrp,
                    'q': array(native, 'q', 12), 'qd': array(native, 'qd', 12),
                    'tau': array(native, 'tau', 12), 'contacts': ncontacts,
                    'clearance': npos[:, 2]-ground,
                    'body_com': array(native, 'body_lin_vel_com', 3),
                    'body_omega': array(native, 'body_ang_vel', 3)}


def evaluate_run(run, write=True, receipt_suffix=None):
    run = Path(run).resolve()
    if receipt_suffix is not None and not re.fullmatch(r'[A-Za-z0-9_-]+', receipt_suffix):
        raise ValueError('Receipt suffix must contain only ASCII letters, digits, underscore or dash')
    suffix = '.'+receipt_suffix if receipt_suffix else ''
    output_path = run/('summary_truth_pid'+suffix+'.json')
    arrays_path = run/('truth_pid_independent_arrays'+suffix+'.npz')
    if write and (output_path.exists() or arrays_path.exists()):
        raise ValueError('Refusing to replace prior independent truth evidence')
    protocol_path = HERE/'protocol.json'
    protocol = json.loads(protocol_path.read_text())
    source_paths = [protocol_path, Path(__file__).resolve(), run/'truth_profile.json',
                    run/'telemetry.jsonl', run/'control.jsonl', run/'actuator.jsonl',
                    run/'worker_result.json', run/'runtime_manifest.json', run/'world.sdf',
                    run/'sources/policy/observation.py', run/'sources/policy/contract.json']
    result = {'schema': 'independent_truth_teacher_pid_benchmark/v2', 'run': str(run), 'status': 'failed',
              'scope': 'Simulator-feedback outer velocity PID benchmark only; not sensor-SLAM navigation',
              'navigation_ground_truth_used': True, 'SLAM_navigation_verified': False, 'real_robot_verified': False,
              'strict_terrain_motion_verified': False, 'score': None, 'checks': {}, 'receipt_suffix': receipt_suffix,
              'limitations': ['COM velocity is measured physics velocity rotated by actual quaternion; targets are not motion evidence.',
                              'Native tau is the applied force command fed to physics, not measured motor torque.',
                              'Truth feedback explicitly controls this tuning benchmark; it is not SLAM integration.']}
    independent = {}

    def check(name, passed, status=None, **values):
        result['checks'][name] = {'status': status or ('passed' if passed else 'failed'), **values}
        return bool(passed)

    try:
        profile = json.loads((run/'truth_profile.json').read_text())
        data = rows(run/'telemetry.jsonl')
        control = control_rows(rows(run/'control.jsonl'))
        worker = json.loads((run/'worker_result.json').read_text())
        runtime = json.loads((run/'runtime_manifest.json').read_text())
        native, n = native_evidence(run)
        if len(data) < 2:
            raise ValueError('Insufficient original telemetry')
        route = np.asarray(profile['route_world_xyz'], float)
        if route.ndim != 2 or len(route) < 2 or route.shape[1] != 3 or not np.isfinite(route).all():
            raise ValueError('Finite N>=2 route XYZ required')
        speed, duration, feedback_hz = (float(profile[k]) for k in ('desired_speed', 'duration_s', 'feedback_hz'))
        if not all(math.isfinite(v) and v > 0 for v in (speed, duration, feedback_hz)):
            raise ValueError('Invalid frozen profile speed/duration/frequency')
        terrain = str(profile['terrain'])
        result['profile'] = profile
        world_t, t, elapsed = (array(data, k) for k in ('world_sim_time', 'state_physics_world_time', 'sim_time'))
        pos, quat = array(data, 'position', 3), array(data, 'quaternion_wxyz', 4)
        R = rotation_wxyz(quat)
        body_velocity = array(data, 'body_lin_vel', 3)
        world_velocity = np.einsum('nij,nj->ni', R, body_velocity)
        body_omega = array(data, 'body_ang_vel', 3)
        yaw = np.arctan2(R[:, 1, 0], R[:, 0, 0])
        requested, command = array(data, 'requested', 3), array(data, 'command', 3)
        q, qd, tau = (array(data, k, 12) for k in ('q', 'qd', 'applied_torque'))
        clearance = array(data, 'body_clearance')
        faults = [{'index': i, 'physics_time_s': float(t[i]), 'fault': row.get('fault')}
                  for i, row in enumerate(data) if row.get('fault') not in (None, 0, '')]
        bootstrap = float(protocol['time']['bootstrap_excluded_s'])
        audit = t >= bootstrap
        if not audit.any():
            raise ValueError('No physical data beyond bootstrap')
        maxgap = float(np.max(np.diff(t)))
        offset_error = float(np.max(abs(t-(world_t-.005))))
        elapsed_offset = world_t-elapsed
        check('physical_time_and_telemetry_coverage', np.all(np.diff(t) > 0) and
              maxgap <= protocol['time']['maximum_telemetry_gap_s'] and
              offset_error <= protocol['time']['pose_offset_tolerance_s'] and np.ptp(elapsed_offset) <= 1e-8,
              first_physical_time_s=float(t[0]), last_physical_time_s=float(t[-1]), actual_recorded_rows=len(data),
              maximum_gap_s=maxgap, measured_pose_offset_error_s=offset_error,
              elapsed_to_world_offset_s=float(elapsed_offset[0]), duration_s_is_maximum_budget=duration)
        ct, ft, ce = (np.asarray([float(row[k]) for row in control]) for k in ('control_t_s', 'feedback_time_s', 'elapsed_s'))
        updated = np.asarray([row['controller_updated'] for row in control], bool)
        complete_ref = np.asarray(['reference_yaw' in row for row in control], bool)
        cmode = np.asarray([row['mode'] for row in control])
        ref_yaw = np.asarray([float(row.get('reference_yaw', 0.)) for row in control])
        rv = np.asarray([row.get('reference_velocity_world', [0., 0., 0.]) for row in control], float)
        reference_xy = np.asarray([row.get('reference_xy', [0., 0.]) for row in control], float)
        update_times, update_feedback = ct[updated], ft[updated]
        gaps = np.diff(update_times)
        expected = 1./feedback_hz
        cadence_error = float(np.max(abs(gaps-expected))) if len(gaps) else None
        check('original_control_clock_and_feedback_causality', np.all(np.diff(ct) > 0) and np.all(ft <= ct+1e-8) and
              np.all(ct-ft <= protocol['time']['maximum_causal_control_age_s']) and
              np.all(abs((ct-ce)-elapsed_offset[0]) <= 1e-8) and np.all(abs(ct[updated]-ft[updated]-.005) <= 1e-8) and
              len(update_times) > 1 and cadence_error <= 1e-8 and np.all(np.diff(update_feedback) > 0),
              controller_all_rows=len(control), actual_update_rows=int(updated.sum()),
              hold_or_initialization_rows=int((~updated).sum()), feedback_hz_requested=feedback_hz,
              observed_update_rate_hz=float((len(update_times)-1)/(update_times[-1]-update_times[0])) if len(update_times)>1 else None,
              maximum_update_period_error_s=cadence_error, maximum_control_feedback_delay_s=float(np.max(ct-ft)),
              update_selector='controller_updated is exactly true; kind alone does not establish an update')
        # Bind only already-computed rows to earlier cached physics. Initialization
        # legitimately has no route reference. Full hold rows cover slower feedback.
        indexes = np.searchsorted(ct, t, side='right')-1
        bound = np.clip(indexes, 0, len(control)-1)
        age = t-ct[bound]
        valid = (indexes >= 0) & (age >= -1e-8) & (age <= protocol['time']['maximum_causal_control_age_s'])
        mode = cmode[bound]
        needed = audit & (mode != 'initializing')
        ref_valid = valid & complete_ref[bound]
        speed_ref = np.linalg.norm(rv[bound, :2], axis=1)
        moving = ref_valid & audit & np.isin(mode, protocol['speed']['translation_modes']) & (speed_ref > protocol['speed']['active_reference_min_mps'])
        check('causal_control_reference_coverage', np.all(ref_valid[needed]) and moving.any(),
              missing_or_old_reference_rows=int((~ref_valid & needed).sum()),
              maximum_causal_age_s=float(age[valid].max()) if valid.any() else None,
              drive_rows=int(moving.sum()), drive_modes=sorted(set(mode[moving].tolist())),
              excluded_nontranslation_modes=sorted(set(mode[audit & ~moving].tolist())))
        # Match cached telemetry state with exactly the same native cached state;
        # telemetry torque is previous-step feed, so it is not same-phase compared.
        ni = np.searchsorted(n['state_t'], t, side='left')
        ni = np.clip(ni, 0, len(native)-1)
        prev = np.maximum(ni-1, 0)
        ni = np.where(abs(n['state_t'][prev]-t) < abs(n['state_t'][ni]-t), prev, ni)
        native_pair_error = float(abs(n['state_t'][ni]-t).max())
        pose_pair_error = float(abs(n['position'][ni]-pos).max())
        q_pair_error = float(abs(n['q'][ni]-q).max())
        qd_pair_error = float(abs(n['qd'][ni]-qd).max())
        velocity_pair_error = float(abs(n['body_com'][ni]-body_velocity).max())
        quaternion_pair_error = float(abs(n['quaternion_wxyz'][ni]-quat).max())
        angular_velocity_pair_error = float(abs(n['body_omega'][ni]-body_omega).max())
        check('native_physics_coverage_and_state_pairing', np.all(np.diff(n['iteration']) == 1) and
              np.all(abs(n['dt']-.005) <= 1e-8) and np.all(abs(np.diff(n['world_t'])-.005) <= 1e-8) and
              n['state_t'][0] <= t[0]+1e-8 and n['state_t'][-1] >= t[-1]-1e-8 and
              native_pair_error <= 1e-8 and max(pose_pair_error, q_pair_error, qd_pair_error, velocity_pair_error,
                                             quaternion_pair_error, angular_velocity_pair_error) <= 1e-8,
              native_physics_rows=len(native), maximum_native_gap_s=float(np.max(np.diff(n['world_t']))),
              first_native_physical_time_s=float(n['state_t'][0]), last_native_physical_time_s=float(n['state_t'][-1]),
              maximum_time_pair_error_s=native_pair_error, maximum_position_pair_error_m=pose_pair_error,
              maximum_joint_position_pair_error_rad=q_pair_error, maximum_joint_velocity_pair_error_radps=qd_pair_error,
              maximum_COM_velocity_pair_error_mps=velocity_pair_error,
              maximum_quaternion_pair_error=quaternion_pair_error,
              maximum_angular_velocity_pair_error_radps=angular_velocity_pair_error,
              tau_pairing_scope='native tau is current command feed; telemetry applied_torque is previous feed; not falsely compared')
        route_distance, nearest_leg = polyline_distance(pos, route)
        route_max = float(route_distance[audit].max())
        route_rms = float(np.sqrt(np.mean(route_distance[moving]**2))) if moving.any() else None
        check('flat_or_requested_route_distance', moving.any() and route_max <= protocol['route']['maximum_xy_distance_m'] and
              route_rms <= protocol['route']['drive_xy_distance_rms_m'], maximum_xy_distance_m=route_max,
              drive_xy_distance_rms_m=route_rms, maximum_allowed_m=protocol['route']['maximum_xy_distance_m'],
              rms_allowed_m=protocol['route']['drive_xy_distance_rms_m'])
        heading_error = abs(angular_difference(yaw, ref_yaw[bound]))
        heading_max = float(heading_error[moving].max()) if moving.any() else None
        check('drive_heading', heading_max is not None and heading_max <= protocol['route']['maximum_drive_heading_error_rad'],
              maximum_error_rad=heading_max, limit_rad=protocol['route']['maximum_drive_heading_error_rad'],
              reference='causal recorded reference_yaw, translation modes only',
              maximum_error_time_s=float(t[np.flatnonzero(moving)[np.argmax(heading_error[moving])]]) if moving.any() else None)
        direction = np.zeros((len(t), 2))
        np.divide(rv[bound, :2], speed_ref[:, None], out=direction, where=speed_ref[:, None]>1e-12)
        actual_along = np.sum(world_velocity[:, :2]*direction, axis=1)
        first_move = float(t[np.flatnonzero(moving)[0]]) if moving.any() else math.inf
        stable = moving & (t >= first_move+protocol['time']['speed_startup_excluded_s']) & (speed_ref >= speed*protocol['speed']['full_speed_reference_fraction_min'])
        dt = np.r_[np.diff(t), np.median(np.diff(t))]
        stable_duration = float(dt[stable].sum())
        mean_reference = float(np.mean(speed_ref[stable])) if stable.any() else None
        speed_error = float(np.mean(abs(actual_along[stable]-speed_ref[stable]))) if stable.any() else None
        speed_limit = max(protocol['speed']['maximum_absolute_error_floor_mps'], protocol['speed']['maximum_relative_error_fraction']*mean_reference) if mean_reference is not None else None
        check('stable_real_COM_speed', stable_duration >= protocol['time']['minimum_stable_speed_duration_s'] and
              speed_error is not None and speed_error <= speed_limit, desired_profile_speed_mps=speed,
              mean_reference_mps=mean_reference, mean_actual_forward_projection_mps=float(np.mean(actual_along[stable])) if stable.any() else None,
              mean_absolute_error_mps=speed_error, limit_mps=speed_limit, stable_duration_s=stable_duration, stable_rows=int(stable.sum()),
              velocity_source='Actual COM body_lin_vel rotated by quaternion then projected on causal reference XY direction; v_reference_body[2] is angular rate, never vz')
        naudit = n['state_t'] >= bootstrap
        nrpmax = float(abs(n['roll_pitch'][naudit]).max())
        nclear = float(n['clearance'][naudit].min())
        ntau = float(abs(n['tau'][naudit]).max())
        nqd = float(abs(n['qd'][naudit]).max())
        nfaults = [{'world_time_s': row['t'], 'fault': row.get('fault')} for row in native if row.get('fault') not in (None, 0, '')]
        safe = (nrpmax <= protocol['safety']['maximum_abs_roll_pitch_rad'] and nclear >= protocol['safety']['minimum_body_clearance_m'] and
                ntau <= protocol['safety']['maximum_abs_applied_torque_Nm'] and nqd <= protocol['safety']['maximum_abs_joint_velocity_radps'] and
                not np.any(n['contacts'][naudit, 0]>0) and not faults and not nfaults and worker.get('fault') in (None, 0, '') and
                not np.any(cmode == 'feedback_timeout'))
        check('as_recorded_physical_safety', safe, maximum_abs_roll_pitch_rad=nrpmax, minimum_body_clearance_m=nclear,
              maximum_abs_applied_torque_Nm=ntau, maximum_abs_joint_velocity_radps=nqd,
              body_contact_rows=int((n['contacts'][naudit, 0]>0).sum()), telemetry_faults=faults, native_faults=nfaults,
              worker_fault=worker.get('fault'), feedback_timeout_rows=int((cmode=='feedback_timeout').sum()), measured_motor_torque=False,
              coverage='Every original 200Hz native physics_step; bootstrap .1s excluded',
              clearance_source='Archived observation.TerrainHeightMap, unchanged run/world.sdf all static collision surfaces; downward from actual body origin, self excluded')
        cursor = int(np.flatnonzero(audit)[0])
        visits = []
        for number, goal in enumerate(route[1:], 1):
            candidates = np.flatnonzero((np.arange(len(t)) >= cursor) & (np.linalg.norm(pos[:, :2]-goal[:2], axis=1) <= protocol['route']['goal_xy_tolerance_m']))
            if not len(candidates):
                break
            index = int(candidates[0])
            visits.append({'waypoint_index': number, 'physics_time_s': float(t[index]), 'telemetry_index': index,
                           'actual_position': pos[index].tolist(), 'goal': goal.tolist(),
                           'XY_error_m': float(np.linalg.norm(pos[index, :2]-goal[:2])), 'Z_error_m': float(pos[index, 2]-goal[2])})
            cursor = index+1
        completed_route = len(visits) == len(route)-1
        final_error = float(np.linalg.norm(pos[-1, :2]-route[-1, :2]))
        check('ordered_route_and_final_goal', completed_route and final_error <= protocol['route']['goal_xy_tolerance_m'],
              ordered_visits=visits, required_waypoints=len(route)-1, final_goal_xy_error_m=final_error,
              final_goal_z_error_m=float(pos[-1, 2]-route[-1, 2]), arrival_is_true_simulation_pose_not_SLAM=True)
        completed = worker.get('truth_controller_completed_t_s')
        post = profile.get('post_completion_s')
        completion_valid = isinstance(completed, (int, float)) and math.isfinite(completed) and 0 <= completed <= duration
        post_valid = isinstance(post, (int, float)) and math.isfinite(post) and post >= protocol['parking']['fixed_window_s']
        control_completions = [float(row['completed_t_s']) for row in control if row.get('completed_t_s') is not None]
        completion_consistent = completion_valid and len(control_completions)>0 and max(abs(v-completed) for v in control_completions)<=1e-8
        expected_end = min(duration, completed+post) if completion_valid and post_valid else None
        termination_ok = (completion_consistent and post_valid and worker.get('completed') is True and worker.get('samples') == len(data) and
                          worker.get('termination_reason') == 'truth_controller_completed' and
                          abs(float(worker['last_sim_time'])-elapsed[-1]) <= 1e-8 and expected_end is not None and abs(elapsed[-1]-expected_end) <= .021)
        check('declared_completion_and_termination', termination_ok,
              status='passed' if termination_ok else 'unverified' if not post_valid or not completion_valid else 'failed',
              truth_controller_completed_elapsed_s=completed, profile_post_completion_s=post,
              planned_maximum_duration_s=duration, expected_elapsed_termination_s=expected_end, actual_last_elapsed_s=float(elapsed[-1]),
              worker_completed=worker.get('completed'), recorded_termination_reason=worker.get('termination_reason'),
              completion_timestamp_matches_original_control_rows=bool(completion_consistent),
              interface_pilot_only=not post_valid, missing_fields_are_not_backfilled=True)
        zero = (np.max(abs(requested), axis=1)<=1e-9) & (np.max(abs(command), axis=1)<=1e-9) & (speed_ref<=1e-9) & ref_valid & (mode=='parking')
        parking = None
        park_mask = np.zeros(len(t), bool)
        completion_state_time = completed+float(elapsed_offset[0])-.005 if completion_valid else math.inf
        if completed_route and completion_valid:
            ready = np.flatnonzero(zero & (t>=completion_state_time-1e-8) &
                                   (np.arange(len(t))>=visits[-1]['telemetry_index']) &
                                   (np.linalg.norm(pos[:, :2]-route[-1, :2], axis=1)<=protocol['route']['goal_xy_tolerance_m']))
            if len(ready):
                first = int(ready[0])
                start = float(t[first])
                end = start+protocol['parking']['fixed_window_s']
                park_mask = (t>=start-1e-8) & (t<=end+1e-8)
                npark = (n['state_t']>=start-1e-8) & (n['state_t']<=end+1e-8)
                pyaw = np.arctan2(rotation_wxyz(n['quaternion_wxyz'][npark])[:, 1, 0], rotation_wxyz(n['quaternion_wxyz'][npark])[:, 0, 0])
                nfirst = int(np.flatnonzero(npark)[0])
                drift = float(np.linalg.norm(n['position'][npark, :2]-n['position'][nfirst, :2], axis=1).max())
                yd = float(abs(np.unwrap(pyaw)-pyaw[0]).max())
                covered = t[-1]>=end-1e-8 and n['state_t'][-1]>=end-1e-8 and t[park_mask][-1]>=end-.025 and n['state_t'][npark][-1]>=end-1e-8
                parking = {'window_s': [start, end], 'telemetry_rows': int(park_mask.sum()), 'native_rows': int(npark.sum()),
                           'complete_5s': bool(covered), 'xy_drift_m': drift, 'yaw_drift_rad': yd,
                           'continuous_requested_and_actor_command_zero': bool(zero[park_mask].all()),
                           'window_selection': 'First eligible actual parking zero row after completed controller dwell; never a later quiet window',
                           'physical_pose_source': 'Every original native 200Hz state; command zero coverage from original telemetry/causal control rows'}
        park_ok = (parking is not None and parking['complete_5s'] and parking['continuous_requested_and_actor_command_zero'] and
                   parking['xy_drift_m']<=protocol['parking']['maximum_xy_drift_m'] and parking['yaw_drift_rad']<=protocol['parking']['maximum_yaw_drift_rad'])
        check('fixed_final_goal_parking', park_ok, status='passed' if park_ok else 'unverified' if parking is None or not parking['complete_5s'] else 'failed',
              parking=parking, xy_limit_m=protocol['parking']['maximum_xy_drift_m'], yaw_limit_rad=protocol['parking']['maximum_yaw_drift_rad'])
        owned = runtime.get('owned_processes', [])
        required_owned = [r for r in owned if r.get('role') in ('worker', 'gazebo')]
        clean = (runtime.get('error') is None and {r.get('role') for r in required_owned} == {'worker', 'gazebo'} and
                 all(r.get('returncode') == 0 for r in required_owned) and
                 runtime.get('exclusive_writer') == 'teacher_sim::TeacherActuator')
        check('actual_runtime_receipt', clean, owned_processes=owned, error=runtime.get('error'),
              exclusive_writer=runtime.get('exclusive_writer'), no_SLAM_navigation_claim=True,
              required_zero_exit_roles=['worker', 'gazebo'],
              optional_camera_cleanup_returncodes_are_not_actor_faults=True)
        is_flat = terrain in ('flat', 'flat_short', 'flat_long', 'flat_roundtrip')
        geometry_pairs = sum(bool(r.get('contact_pairs')) for r in native)
        if is_flat:
            check('strict_terrain_support_applicability', True, terrain=terrain, meaning='Flat benchmark; no ramp/step support claim')
        else:
            check('strict_terrain_support_applicability', False, status='unverified', terrain=terrain,
                  meaning='Raw native contact geometry may be present, but per-foot requested surface continuity/entry/exit/continuation has not been independently evaluated; no strict terrain pass',
                  native_rows_with_contact_pairs=geometry_pairs, requested_route_length_m=float(np.linalg.norm(np.diff(route, axis=0), axis=1).sum()),
                  body_z_change_diagnostic_m=float(pos[-1, 2]-pos[0, 2]), world_z_gain_ratio_used=False)
            result['limitations'].append('Full 12m ramp and step functional motion require separate actual contact/surface support evidence; body Z rise is diagnostic only.')
        passed = all(row['status']=='passed' for row in result['checks'].values())
        result['status'] = 'passed' if passed else 'failed'
        result['candidate_ranking_eligible'] = passed and profile.get('design') != 'open_loop'
        if result['candidate_ranking_eligible']:
            result['score'] = float(100/(1+route_rms/.08+speed_error/speed_limit+heading_max/.2+parking['xy_drift_m']/.05+parking['yaw_drift_rad']/.1))
        independent = {'state_physics_world_time': t, 'world_sim_time': world_t, 'elapsed_s': elapsed, 'position': pos,
                       'quaternion_wxyz': quat, 'world_COM_velocity': world_velocity, 'body_COM_velocity': body_velocity,
                       'body_angular_velocity': body_omega, 'route_world_xyz': route, 'route_xy_distance': route_distance,
                       'nearest_route_leg': nearest_leg, 'causal_control_index': indexes, 'causal_reference_age_s': age,
                       'causal_reference_valid': ref_valid, 'drive_mask': moving, 'stable_speed_mask': stable,
                       'actual_forward_projection_mps': actual_along, 'reference_speed_mps': speed_ref,
                       'heading_error_rad': heading_error, 'control_t_s': ct, 'feedback_time_s': ft,
                       'controller_updated': updated, 'control_reference_xy': reference_xy,
                       'control_reference_velocity_world': rv, 'control_reference_yaw': ref_yaw,
                       'q': q, 'qd': qd, 'applied_torque_previous_feed': tau, 'body_clearance': clearance,
                       'requested': requested, 'command': command, 'parking_mask': park_mask,
                       'native_state_physics_world_time': n['state_t'], 'native_force_application_world_time': n['world_t'],
                       'native_iteration': n['iteration'], 'native_position': n['position'], 'native_quaternion_wxyz': n['quaternion_wxyz'],
                       'native_q': n['q'], 'native_qd': n['qd'], 'native_applied_force_command': n['tau'],
                       'native_body_contacts': n['contacts'][:, 0], 'native_body_clearance': n['clearance'], 'native_roll_pitch': n['roll_pitch']}
    except (OSError, ValueError, KeyError, TypeError, IndexError, ImportError) as error:
        check('complete_parseable_required_evidence', False, reason=type(error).__name__+': '+str(error))
        result['status'], result['score'] = 'failed', None
    result['failed_checks'] = [key for key, row in result['checks'].items() if row['status']=='failed']
    result['unverified_checks'] = [key for key, row in result['checks'].items() if row['status']=='unverified']
    result['input_source_sha256'] = {str(path): sha(path) for path in source_paths if path.is_file()}
    if write:
        if independent:
            with arrays_path.open('xb') as stream:
                np.savez_compressed(stream, **independent)
            result['independent_arrays'] = {'path': str(arrays_path), 'sha256': sha(arrays_path)}
        with output_path.open('x') as stream:
            json.dump(result, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write('\n')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--receipt-suffix')
    args = parser.parse_args()
    result = evaluate_run(args.run, receipt_suffix=args.receipt_suffix)
    suffix = '.'+args.receipt_suffix if args.receipt_suffix else ''
    print(json.dumps({'status': result['status'], 'score': result['score'],
                      'summary': str(args.run.resolve()/('summary_truth_pid'+suffix+'.json')),
                      'SLAM_navigation_verified': False}, ensure_ascii=False))


if __name__ == '__main__':
    main()
