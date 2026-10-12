#!/usr/bin/env python3
"""Recompute actual native guard records from original sensor XYZ, without truth inputs."""
import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rows(path):
    if not Path(path).exists():
        return []
    return [json.loads(line) for line in Path(path).read_text().splitlines() if line.strip()]


def array_sha(value):
    return hashlib.sha256(np.asarray(value, dtype='<f8').tobytes()).hexdigest()


def result_dict(value):
    return {'blocked': bool(value[0]), 'clearance_m': None if value[1] is None else float(value[1]), 'point_count': int(value[2])}


def same_result(a, b):
    return a['blocked'] == b['blocked'] and a['point_count'] == b['point_count'] and (
        a['clearance_m'] is None and b['clearance_m'] is None or
        a['clearance_m'] is not None and b['clearance_m'] is not None and abs(a['clearance_m']-b['clearance_m']) <= 1e-12)


def evaluate(run, leaving_s, required_clear_s=1., maximum_gap_s=.300000001):
    run = Path(run).resolve()
    core_path = run.parents[2] / 'navigation/control_core.py'
    spec = importlib.util.spec_from_file_location('independent_exact_guard_core', core_path)
    core = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(core)
    guards = rows(run / 'navigation_guard_history.jsonl')
    sensor = rows(run / 'dynamic_sensor_evidence/sensor_inputs.jsonl')
    clouds = {r['stamp_ns']: r for r in rows(run / 'navigation_cloud_history.jsonl')}
    poses = {r['stamp_ns']: r for r in rows(run / 'navigation_slam_poses.jsonl')}
    raw = {r['stamp_ns']: r for r in sensor if r.get('source') == 'cloud' and r.get('xyz_file')}
    writer_path = run / 'navigation_guard_writer_receipt.json'
    writer = json.loads(writer_path.read_text()) if writer_path.exists() else {}
    replayed, errors, missing = [], [], []
    cache = {}
    prior_sequence = None
    prior_clock = None
    original_goals = json.loads((run / 'navigation_request.json').read_text())['goals']
    for record in guards:
        sequence = record['sequence']
        clock = record['compute_ros_clock_ns']
        if record.get('schema') != 'teacher_native_steering_guard/v1' or prior_sequence is None and sequence != 1 or prior_sequence is not None and sequence != prior_sequence+1:
            errors.append({'sequence': sequence, 'error': 'Schema or actual guard sequence discontinuity'})
        if prior_clock is not None and clock <= prior_clock:
            errors.append({'sequence': sequence, 'error': 'Actual compute clock is not strictly increasing'})
        prior_sequence, prior_clock = sequence, clock
        source_stamp = record['cloud_header_stamp_ns']
        control = poses.get(record['control_pose_stamp_ns'])
        filtering = poses.get(record['cloud_filtering_body_stamp_ns'])
        cloud = clouds.get(source_stamp)
        context_ok = control is not None and filtering is not None and cloud is not None
        if context_ok:
            context_ok = (np.array_equal(record['control_pose'], control['position']) and
                np.array_equal(record['control_quaternion'], control['quaternion']) and
                np.array_equal(record['cloud_filtering_body_pose'], filtering['position']) and
                np.max(abs(np.asarray(record['control_rotation'])-core.rotation_xyzw(control['quaternion']))) <= 1e-12 and
                np.max(abs(np.asarray(record['cloud_filtering_body_rotation'])-core.rotation_xyzw(filtering['quaternion']))) <= 1e-12 and
                cloud['nearest_slam_pose_stamp_ns'] == record['cloud_filtering_body_stamp_ns'] and
                cloud['filtered_xyz_float64_sha256'] == record['filtered_xyz_float64_sha256'] and
                cloud['frame_id'] == record['cloud_frame_id'] == 'camera_init' and
                cloud['filtered_points'] == record['filtered_points'] and
                array_sha(record['route']) == record['route_float64_sha256'] and
                np.array_equal(np.asarray(record['route'])[-1], original_goals[record['waypoint_index']]['center']) and
                record['evidence_queue_error'] is None and
                record['compute_sim_time_s'] == clock/1e9 and
                record['compute_monotonic_wall'] >= record['cloud_received_monotonic_wall'] and
                record['compute_monotonic_wall'] >= record['control_pose_received_monotonic_wall'])
        if not context_ok:
            errors.append({'sequence': sequence, 'error': 'Recorded actual integer control/filter/cloud/route context mismatch'})
        payload = raw.get(source_stamp)
        if payload is None:
            missing.append({'sequence': sequence, 'compute_world_s': clock/1e9, 'cloud_stamp_ns': source_stamp,
                            'reason': 'No passive original cloud XYZ for this exact stamp'})
            replayed.append({'sequence': sequence, 'compute_world_s': clock/1e9, 'cloud_stamp_ns': source_stamp,
                             'status': 'unverified', 'waypoint_index': record['waypoint_index']})
            continue
        payload_path = run / 'dynamic_sensor_evidence' / payload['xyz_file']
        if source_stamp not in cache:
            with np.load(payload_path) as data:
                original_xyz = data['xyz']
            filtered, count = core.remove_go2_self_returns(original_xyz, np.asarray(record['cloud_filtering_body_pose']),
                                                          np.asarray(record['cloud_filtering_body_rotation']))
            cache[source_stamp] = (filtered, count, sha(payload_path))
        filtered, count, payload_hash = cache[source_stamp]
        pose = np.asarray(record['control_pose'])
        route = np.asarray(record['route'])
        checked = np.asarray(record['checked_target'])
        direction = np.asarray(record['steering_direction'])
        motion = pose.copy()
        motion[:2] += .8*direction/max(float(np.linalg.norm(direction)), 1e-9)
        goal_result = result_dict(core.obstacle_ahead(filtered, pose, checked, route))
        motion_result = result_dict(core.obstacle_ahead(filtered, pose, motion, route))
        union_result = result_dict(core.steering_obstacle_ahead(filtered, pose, checked, direction, route))
        exact = array_sha(filtered) == record['filtered_xyz_float64_sha256'] and len(filtered) == record['filtered_points'] and count == record['self_filtered_points']
        matched = exact and np.array_equal(motion, record['motion_corridor_target']) and all(same_result(a, record[b]) for a, b in [
            (goal_result, 'goal_corridor_result'), (motion_result, 'motion_corridor_result'), (union_result, 'union_result')])
        status = 'passed' if context_ok and matched else 'failed'
        if not matched:
            errors.append({'sequence': sequence, 'error': 'Independent raw XYZ filtering/actual two corridor results mismatch'})
        replayed.append({'sequence': sequence, 'compute_world_s': clock/1e9, 'cloud_stamp_ns': source_stamp,
            'status': status, 'waypoint_index': record['waypoint_index'], 'original_xyz_file': payload['xyz_file'],
            'original_xyz_sha256': payload_hash, 'exact_filtered_buffer_hash': exact, 'goal_corridor_result': goal_result,
            'motion_corridor_result': motion_result, 'union_result': union_result,
            'cloud_age_sim_s': (clock-source_stamp)/1e9,
            'cloud_age_wall_s': record['compute_monotonic_wall']-record['cloud_received_monotonic_wall'],
            'control_pose_age_sim_s': (clock-record['control_pose_stamp_ns'])/1e9,
            'obstacle_hold_before': record['obstacle_hold_before'], 'obstacle_hold_after': record['obstacle_hold_after'],
            'recorded_clear_start_before_s': record['clear_start_before_s'],
            'recorded_clear_start_after_s': record['clear_start_after_s'],
            'recorded_zero_requested_after_control': record['zero_requested_after_control']})
    # Every observed actual guard participates: a missing/mismatched record or
    # blocked result resets continuity. Duplicate cached clouds are not invented
    # samples; native compute clocks and actual accepted cloud ages are retained.
    start, previous, confirmation, longest = None, None, None, 0.
    for replay in replayed:
        time = replay['compute_world_s']
        if time < leaving_s or replay['waypoint_index'] != 0:
            continue
        valid = (replay['status'] == 'passed' and not replay['goal_corridor_result']['blocked'] and
                 not replay['motion_corridor_result']['blocked'] and not replay['union_result']['blocked'] and
                 -.05 <= replay['cloud_age_sim_s'] <= .3 and -.05 <= replay['cloud_age_wall_s'] <= .3 and
                 -.05 <= replay['control_pose_age_sim_s'] <= .3)
        if not valid:
            start = None
        elif start is None or previous is not None and time-previous > maximum_gap_s:
            start = time
        if valid:
            longest = max(longest, time-start)
            if confirmation is None and time-start >= required_clear_s:
                confirmation = time
        previous = time
    writer_ok = (writer.get('schema') == 'teacher_native_steering_guard_writer/v1' and writer.get('status') == 'drained' and
        writer.get('queue_error') is None and writer.get('expected_records') == len(guards) and
        writer.get('silent_record_loss_permitted') is False)
    if not writer_ok:
        errors.append({'error': 'Actual guard writer drain/count/error receipt missing or mismatched'})
    return {'schema': 1, 'actual_guard_records': len(guards), 'writer_complete': writer_ok, 'writer_receipt': writer,
        'matched_original_xyz_guards': sum(r['status'] == 'passed' for r in replayed),
        'missing_payload_guard_records': missing, 'errors': errors, 'replays': replayed,
        'strict_clear_confirmation_world_s': confirmation, 'longest_corroborated_clear_s': longest,
        'required_clear_s': required_clear_s, 'maximum_compute_gap_s': maximum_gap_s,
        'native_geometry_source_sha256': sha(core_path), 'guard_history_sha256': sha(run/'navigation_guard_history.jsonl') if guards else None,
        'truth_used_for_geometry_or_clear': False, 'no_original_receipt_changed': True}
