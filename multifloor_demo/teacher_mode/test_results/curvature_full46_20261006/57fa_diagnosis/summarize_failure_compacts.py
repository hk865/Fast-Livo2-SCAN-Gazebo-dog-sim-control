#!/usr/bin/env python3
"""Read existing compact files and selected small spline artifacts only.

Never reads original PID/native streams, launches ROS, or writes a frozen run.
"""
import collections
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent
TEACHER = HERE.parents[2]
DEMO = TEACHER.parent
sys.path.insert(0, str(DEMO / 'navigation'))
from goal_regions import parse_goal, contains, contains_control


def read_json(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def percentiles(values):
    return None if not values else dict(zip(('median', 'p95', 'max'), map(float, np.percentile(values, [50, 95, 100]))))


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def main():
    compact_path = HERE / 'NINTH_TENTH_PID_COMPACT.json.gz'
    pid = json.load(gzip.open(compact_path, 'rt'))
    run = Path(pid['run'])
    request_path = run / 'navigation_request.json'
    request = read_json(request_path)
    goal_definition = request['goals'][9]
    goal = parse_goal(goal_definition)
    rows = [r for r in pid['rows'] if r['waypoint_index'] == 9]
    fresh = [r for r in rows if r['math_updated']]
    dt = lambda a, b: (b['clock_ns'] - a['clock_ns']) / 1e9
    mode_seconds = collections.defaultdict(float)
    for a, b in zip(rows, rows[1:]):
        mode_seconds[a['mode'] + '/' + str(a['phase'])] += dt(a, b)

    episodes = []
    start = 0
    for i in range(1, len(rows) + 1):
        if i == len(rows) or rows[i]['phase'] != rows[start]['phase']:
            a, b = rows[start], rows[i - 1]
            next_row = rows[i] if i < len(rows) else b
            episodes.append(dict(phase=a['phase'], start_s=a['clock_ns'] / 1e9,
                end_s=next_row['clock_ns'] / 1e9, duration_s=dt(a, next_row),
                start_sequence=a['pid_sequence'], end_sequence=b['pid_sequence'],
                start_yaw=a['yaw'], end_yaw=b['yaw'], locked_heading=a['locked_heading'],
                next_phase=next_row['phase'] if i < len(rows) else None,
                start_path_id=a['path_id'], end_path_id=b['path_id']))
            start = i
    phase_statistics = {}
    for phase in ('pre_turn', 'align', 'settle', 'drive'):
        selected = [e for e in episodes if e['phase'] == phase]
        durations = [e['duration_s'] for e in selected]
        phase_statistics[phase] = dict(episodes=len(selected), seconds=sum(durations),
            duration_s=percentiles(durations), episodes_le_100ms=sum(d <= .1 for d in durations))

    path_changes = [(a, b) for a, b in zip(fresh, fresh[1:]) if a['path_id'] != b['path_id']]
    heading_jumps = [abs(wrap(b['SCAN_heading'] - a['SCAN_heading'])) for a, b in path_changes
        if a['SCAN_heading'] is not None and b['SCAN_heading'] is not None]

    selected_artifacts = []
    for basename in ('000223_trajectory_225', '000270_trajectory_272', '000478_trajectory_481'):
        array_path = run / 'navigation_trajectories' / (basename + '.npz')
        metadata_path = array_path.with_suffix('.json')
        metadata = read_json(metadata_path)
        planner_metadata = metadata['metadata']
        with np.load(array_path, allow_pickle=False) as archive:
            samples = archive['samples']
            endpoint = samples[-1]
            goal_axes = np.asarray(goal.axes) @ (endpoint - np.asarray(goal.center))
            record = dict(array_path=str(array_path), array_sha256=sha(array_path),
                metadata_path=str(metadata_path), metadata_sha256=sha(metadata_path),
                recorded_array_sha256=metadata['array_sha256'],
                waypoint_index=metadata['waypoint_index'], reference_stamp=metadata['reference_stamp'],
                body_goal=planner_metadata['body_goal'], adjusted_body_goal=planner_metadata['adjusted_body_goal'],
                goal_exactly_matches_request=planner_metadata['body_goal'] == goal_definition['center'],
                adjusted_goal_exactly_matches_request=planner_metadata['adjusted_body_goal'] == goal_definition['center'],
                sample_count=len(samples), start=samples[0].tolist(), endpoint=endpoint.tolist(),
                endpoint_distance_to_goal_m=float(np.linalg.norm(endpoint - np.asarray(goal.center))),
                endpoint_local_axes_m=goal_axes.tolist(), endpoint_inside=contains(goal, endpoint),
                endpoint_inside_control=contains_control(goal, endpoint),
                samples_inside=sum(contains(goal, point) for point in samples),
                direct_start_endpoint_distance_m=float(np.linalg.norm(endpoint - samples[0])))
        if record['array_sha256'] != record['recorded_array_sha256']:
            raise RuntimeError('Selected archived array SHA mismatch')
        selected_artifacts.append(record)

    pairs = []
    by_sequence = {r['pid_sequence']: r for r in rows}
    for before_sequence, after_sequence in ((5006, 5007), (5284, 5285), (5658, 5659), (7392, 7393)):
        a, b = by_sequence[before_sequence], by_sequence[after_sequence]
        metadata_path = (run / b['source_array_file']).with_suffix('.json')
        metadata = read_json(metadata_path)
        reference_stamp_ns = metadata['reference_stamp'][0] * 1_000_000_000 + metadata['reference_stamp'][1]
        keys = ('pid_sequence', 'clock_ns', 'pose_ns', 'mode', 'reason', 'phase', 'yaw',
            'SCAN_heading', 'locked_heading', 'path_id', 'source_array_file', 'pose', 'command_after_slew')
        pairs.append(dict(before={k: a[k] for k in keys}, after={k: b[k] for k in keys},
            elapsed_ms=dt(a, b) * 1000,
            new_reference_stamp_ns=reference_stamp_ns,
            new_reference_stamp_exactly_equals_recovery_tick=reference_stamp_ns == a['clock_ns'],
            actual_pose_change_m=float(np.linalg.norm(np.asarray(b['pose']) - a['pose'])),
            heading_jump_rad=wrap(b['SCAN_heading'] - a['SCAN_heading']),
            new_metadata_path=str(metadata_path), new_metadata_sha256=sha(metadata_path),
            same_goal=metadata['metadata']['body_goal'] == goal_definition['center']))

    native_path = HERE / '57fa_NATIVE_REGION9_10_COMPACT.json.gz'
    native = json.load(gzip.open(native_path, 'rt'))
    physical = [r for r in native['rows'] if rows[0]['clock_ns'] / 1e9 <= r['sim_time'] <= rows[-1]['clock_ns'] / 1e9]
    large_turn = [r for r in physical if abs(r['command'][2]) > .3]
    response = dict(rows=len(physical), actual_window_s=[physical[0]['sim_time'], physical[-1]['sim_time']],
        command_expired_count=sum(bool(r['command_expired']) for r in physical),
        fault_count=sum(bool(r['fault']) for r in physical),
        body_contact_count=sum(r.get('contacts', {}).get('body', 0) > 0 for r in physical),
        command_forward_abs_gt_15mmps_rows=sum(abs(r['command'][0]) > .015 for r in physical),
        abs_yaw_command_gt_03_rows=len(large_turn),
        instantaneous_same_sign_measured_body_wz_rows=sum(r['command'][2] * r['body_ang_vel'][2] > 0 for r in large_turn),
        abs_measured_body_wz_median_for_large_turn_radps=float(np.median([abs(r['body_ang_vel'][2]) for r in large_turn])),
        mean_body_planar_velocity_mps=np.mean([r['body_lin_vel'][:2] for r in physical], axis=0).tolist(),
        maximum_abs_roll_or_pitch_rad=max(max(abs(r['rpy'][0]), abs(r['rpy'][1])) for r in physical),
        interpretation='Observational sign/response evidence, not a plant gain/latency certification or proof of full motion correctness.')

    candidate = TEACHER / 'navigation/corridor_tracking_v23_curvature_full46'
    bindings = {str(path): sha(path) for path in (
        compact_path, native_path, request_path, HERE / 'REGION10_SCAN_GEOMETRY.json',
        HERE.parent / '57fa_FULL46_ACTUAL_EVALUATION.json',
        candidate / 'shared_controller.py', candidate / 'controller.py',
        candidate / 'cascade_core.py', candidate / 'transition_gate.py',
        candidate / 'stack.launch.py',
        DEMO / 'navigation/ros2_ws/src/plan_manage/src/scan_replan_fsm.cpp')}
    result = dict(schema='v23_tenth_region_interlock_diagnosis/v1', run=str(run),
        navigation_ground_truth_used=False, frozen_inputs_modified=False, new_simulation_run=False,
        original_full46_status='FAILED', actual_arrivals=9,
        raw_PID_original_read_passes=pid['read_passes'],
        analysis_inputs='Already saved PID/native compacts plus selected small SCAN arrays/metadata and frozen source.',
        tenth_goal=goal_definition, PID_rows=len(rows), fresh_math_rows=len(fresh),
        PID_window_s=[rows[0]['clock_ns'] / 1e9, rows[-1]['clock_ns'] / 1e9],
        mode_phase_left_sample_occupancy_s=dict(mode_seconds),
        maximum_PID_row_gap_s=max(dt(a, b) for a, b in zip(rows, rows[1:])),
        heading_phase_statistics=phase_statistics, heading_phase_episodes=episodes,
        fresh_path_changes=len(path_changes), path_change_heading_jump_rad=percentiles(heading_jumps),
        path_change_heading_jumps_gt_02=sum(d > .2 for d in heading_jumps),
        path_change_heading_jumps_gt_055=sum(d > .55 for d in heading_jumps),
        recovery_zero_replan_examples=pairs,
        selected_SCAN_artifacts=selected_artifacts,
        final_SCAN_geometry_source=str(HERE / 'REGION10_SCAN_GEOMETRY.json'),
        native_physical_response=response,
        direct_findings=[
            'Current tenth region was accepted by SCAN: selected body_goal and adjusted_body_goal exactly match the requested center.',
            'Early path endpoints are local targets under the actual 2.5m horizon; the final archived path reaches the original oriented goalbox.',
            'Recovery exact-zero immediately after settle-to-drive is mistaken for stalled/end-of-path by shared_controller.py:723-726.',
            'The same tick requests a new reference; a newly accepted heading then exceeds the unchanged 0.2rad drive gate and initiates another measured-stop/turn.',
            'Curvature reference_constraint_hold occupies only 0.25s in this selected window; it is not a persistent interlock.',
            'Large signed yaw commands have physical angular responses, zero expiration and no native fault/body contact in this window.'
        ],
        candidate_minimum_fix=dict(implemented=False, frozen_V23_must_remain_unchanged=True,
            location='new candidate scoped shared_controller low-command replan predicate at inherited line723',
            change='Exclude intentional protect/recovering/reference_constraint_hold command zeros from the generic low-command replan heuristic; retain explicit exhausted-path and intentional constraint-replan paths.',
            preserve=['exact protection zeros', 'two-fresh-source recovery', '300ms sim/wall freshness',
                'measured-stop and 0.2rad/0.1rad heading thresholds', 'raw SLAM region geometry/dwell/90s timeout',
                'unique CPU Teacher executor', 'current native SCAN collision/tilt/fence/ACK gates'],
            limitation='This only removes a confirmed false-trigger. SCAN autonomous replanning and remaining heading changes may still require an independently collision-monitored atomic plan-admission supervisor.',
            finite_cases=['settle-to-drive recovery emits zero without clearing a valid path or publishing a new reference',
                'second causal fresh frame enables the existing drive calculation',
                'true checked-path exhausted/not-arrived still requests a new plan',
                'constraint-hold explicit replan reason still works',
                'goal epoch change, stale/tilt/fence/obstacle/ACK failure still emits exact zero',
                'candidate/active path and native collision-monitor identity cannot diverge']),
        unverified_causes=['Why SCAN alternates between detour and near-straight path shapes: occupancy/optimizer/replan-reason causal replay not completed.',
            'Numerical plant gain and response latency; this diagnosis only establishes observed signed turn response.',
            'Counterfactual full46 success after the proposed fix; no new run or controller edit was performed.'],
        source_bindings_sha256=bindings)
    target = HERE / 'FINAL_INTERLOCK_DIAGNOSIS.json'
    target.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(output=str(target), sha256=sha(target), phase_statistics=phase_statistics,
        response=response, pairs=[dict(elapsed_ms=p['elapsed_ms'], recovery_stamp_match=p['new_reference_stamp_exactly_equals_recovery_tick']) for p in pairs]), ensure_ascii=False))


if __name__ == '__main__':
    main()
