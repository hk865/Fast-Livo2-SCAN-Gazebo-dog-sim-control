#!/usr/bin/env python3
"""Independent V5 route-leg source supplement; no runtime or acceptance edits."""
import argparse
import hashlib
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

def sha(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()

def rows(path):
    with Path(path).open() as stream:
        return [json.loads(line) for line in stream if line.strip()]

def check(value, **evidence):
    return dict(status='passed' if value else 'failed', passed=bool(value), **evidence)

def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))

def audit(run):
    summary_path = run / 'summary_pid_navigation_independent.json'
    original_summary_bytes = summary_path.read_bytes()
    summary = json.loads(original_summary_bytes)
    request = json.loads((run / 'navigation_request.json').read_text())
    profile = json.loads((run / 'navigation_profile.json').read_text())
    freeze = json.loads((run / 'pid_navigation_freeze.json').read_text())
    manifest = json.loads((run / 'navigation_pid_source_manifest.json').read_text())['sources']
    references = rows(run / 'navigation_route_leg_references.jsonl')
    poses = rows(run / 'navigation_slam_poses.jsonl')
    ticks = rows(run / 'navigation_pid_history.jsonl')
    pose_index = {}
    duplicate_pose_stamps = []
    for pose in poses:
        stamp = pose['stamp_ns']
        if stamp in pose_index:
            duplicate_pose_stamps.append(stamp)
        pose_index[stamp] = pose
    ledger = {}
    duplicate_keys = []
    activation_failures = []
    activation_evidence = []
    required_fields = ['schema', 'reference', 'reference_source', 'frame_id', 'origin_source',
        'goal_source', 'request_id', 'waypoint_index', 'leg_start', 'leg_goal',
        'activation_pose_stamp_ns', 'activation_ros_clock_ns', 'activation_quaternion_xyzw',
        'heading_rad', 'navigation_ground_truth_used']
    for reference in references:
        key = reference.get('request_id'), reference.get('waypoint_index')
        if key in ledger:
            duplicate_keys.append(list(key))
        ledger[key] = reference
        failures = []
        missing = [name for name in required_fields if name not in reference]
        if missing:
            activation_failures.append(dict(key=list(key), missing_fields=missing))
            continue
        stamp = reference['activation_pose_stamp_ns']
        clock = reference['activation_ros_clock_ns']
        pose = pose_index.get(stamp)
        index = reference['waypoint_index']
        if (reference['schema'] != 'actual_slam_route_leg_heading/v1'
                or reference['reference'] != 'route_leg'
                or reference['frame_id'] != 'camera_init'
                or reference['origin_source'] != '/demo/slam/body_odom'
                or reference['navigation_ground_truth_used'] is not False
                or reference['request_id'] != request['request_id']):
            failures.append('reference identity/frame/source')
        if (type(stamp) is not int or type(clock) is not int
                or not -50_000_000 <= clock - stamp < 300_000_000):
            failures.append('integer original stamp/fresh activation clock')
        if (pose is None or pose.get('frame_id') != 'camera_init'
                or pose.get('position') != reference['leg_start']
                or pose.get('quaternion') != reference['activation_quaternion_xyzw']
                or type(pose.get('callback_ros_clock_ns')) is not int
                or pose['callback_ros_clock_ns'] > clock):
            failures.append('exact actual raw SLAM activation record')
        if (type(index) is not int or index < 0 or index >= len(request['goals'])
                or request['goals'][index]['center'] != reference['leg_goal']):
            failures.append('original frozen request goal')
        start, goal = reference['leg_start'], reference['leg_goal']
        if (len(start) != 3 or len(goal) != 3
                or not all(math.isfinite(float(v)) for v in start + goal)):
            failures.append('finite geometry')
            recomputed_heading = None
        else:
            dx, dy = goal[0] - start[0], goal[1] - start[1]
            recomputed_heading = math.atan2(dy, dx)
            if (math.hypot(dx, dy) <= 1e-6
                    or abs(wrap(recomputed_heading - reference['heading_rad'])) > 1e-12):
                failures.append('frozen start-to-goal yaw recomputation')
        if failures:
            activation_failures.append(dict(key=list(key), failures=failures))
        activation_evidence.append(dict(key=list(key), activation_pose_stamp_ns=stamp,
            activation_ros_clock_ns=clock, activation_age_s=(clock-stamp)/1e9,
            actual_raw_pose_found=pose is not None, leg_start=start, leg_goal=goal,
            recorded_heading_rad=reference['heading_rad'], recomputed_heading_rad=recomputed_heading))
    tick_failures = []
    used_keys = set()
    for tick in ticks:
        key = tick.get('request_id'), tick.get('waypoint_index')
        used_keys.add(key)
        reference = ledger.get(key)
        reasons = []
        if reference is None or tick.get('heading_reference') != reference:
            reasons.append('PID reference differs from immutable activation ledger')
        elif (type(tick.get('compute_ros_clock_ns')) is not int
                or tick['compute_ros_clock_ns'] < reference['activation_ros_clock_ns']
                or abs(wrap(tick['heading']-reference['heading_rad'])) > 1e-12):
            reasons.append('actual PID yaw or activation causality')
        if (not math.isfinite(float(tick.get('scan_heading_rad', float('nan'))))
                or not math.isfinite(float(tick.get('scan_heading_error_rad', float('nan'))))
                or abs(wrap(tick['scan_heading_rad']-tick['yaw'])-tick['scan_heading_error_rad']) > 1e-10):
            reasons.append('original SCAN yaw evidence absent/inconsistent')
        if tick.get('navigation_ground_truth_used') is not False:
            reasons.append('navigation source exclusion missing')
        if reasons:
            tick_failures.append(dict(sequence=tick.get('sequence'), key=list(key), reasons=reasons))
    expected_keys = {(request['request_id'], i) for i in range(len(request['goals']))}
    snapshot_failures = []
    for name in ['navigation/pid_mode/route_leg_heading.py', 'navigation/pid_mode/controller.py']:
        entry = manifest.get(str(ROOT / name))
        expected = freeze.get('source_hashes', {}).get(name)
        if (not entry or not expected or entry.get('sha256') != expected
                or not Path(entry['snapshot']).is_relative_to(run)
                or not Path(entry['snapshot']).is_file() or sha(entry['snapshot']) != expected):
            snapshot_failures.append(name)
    rejections_path = run / 'navigation_route_leg_rejections.jsonl'
    rejections = rows(rejections_path) if rejections_path.is_file() else []
    checks = {
        'explicit_teacher_only_flat_route_leg_profile': check(
            profile.get('controller_kind') == 'teacher' and profile.get('heading_reference') == 'route_leg'
            and summary.get('case_id') in ['flat_straight_6m', 'flat_short_1m_roundtrip', 'flat_roundtrip_6m'],
            actual_case=summary.get('case_id')),
        'archived_executed_route_leg_sources': check(not snapshot_failures, failures=snapshot_failures),
        'complete_unique_activation_ledger': check(bool(references) and set(ledger) == expected_keys and not duplicate_keys,
            actual_legs=len(references), expected_legs=len(expected_keys), duplicates=duplicate_keys,
            missing_keys=[list(key) for key in sorted(expected_keys-set(ledger))]),
        'exact_original_slam_activation_and_frozen_goal': check(not activation_failures and bool(activation_evidence),
            legs=activation_evidence, failures=activation_failures, duplicate_slam_stamps=duplicate_pose_stamps),
        'all_actual_pid_references_and_yaw_immutable': check(bool(ticks) and not tick_failures and used_keys == expected_keys,
            actual_pid_rows=len(ticks), failures=tick_failures),
        'no_actual_route_leg_rejection': check(not rejections, actual_rejections=rejections),
        'unchanged_independent_xy_pid_and_guard_audits': check(all(summary['checks'].get(name, {}).get('status') == 'passed'
            for name in ['PID_actual_terms_and_bounds', 'PID_measured_source_and_execution',
                         'actual_PID_motion_corridor_guard', 'complete_PID_and_guard_recording']),
            original_summary_sha256=hashlib.sha256(original_summary_bytes).hexdigest()),
    }
    if summary_path.read_bytes() != original_summary_bytes:
        raise RuntimeError('Original formal receipt changed during read-only supplementary audit')
    inputs = [summary_path, run/'pid_navigation_freeze.json', run/'navigation_profile.json',
        run/'navigation_request.json', run/'navigation_pid_source_manifest.json',
        run/'navigation_route_leg_references.jsonl', run/'navigation_slam_poses.jsonl',
        run/'navigation_pid_history.jsonl']
    if rejections_path.is_file():
        inputs.append(rejections_path)
    result = dict(schema='actual_slam_route_leg_source_supplement/v1', run=str(run),
        status='passed' if all(v['passed'] for v in checks.values()) else 'failed', checks=checks,
        original_formal_case_status=summary['status'], formal_25_gates_replaced=False,
        protocol_or_runtime_changed=False, source_geometry_is_navigation_truth=False,
        deliberate_invalid_reference_protection='unverified: not exercised by this actual run',
        global_navigation_or_sim2sim_pass=False, input_sha256={p.name:sha(p) for p in inputs},
        analysis_script_sha256=sha(__file__))
    output = run/'pid_route_leg_source_audit.json'
    output.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output), status=result['status'],
        failed=[name for name, value in checks.items() if not value['passed']])))

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    audit(parser.parse_args().run.resolve())
