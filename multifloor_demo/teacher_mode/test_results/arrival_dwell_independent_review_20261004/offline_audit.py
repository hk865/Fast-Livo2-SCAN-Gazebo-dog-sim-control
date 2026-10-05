#!/usr/bin/env python3
"""Offline actual-method arrival/safety-flow audit, no ROS or simulator imports."""
from __future__ import annotations
import ast
import copy
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def get_class(path, name):
    return next(v for v in ast.parse(path.read_text()).body if isinstance(v, ast.ClassDef) and v.name == name)


def methods(cls, names):
    result = [copy.deepcopy(v) for v in cls.body if isinstance(v, ast.FunctionDef) and v.name in names]
    assert {v.name for v in result} == set(names)
    return result


def main():
    if (OUT / 'review.json').exists():
        raise RuntimeError('Never overwrite original independent evidence')
    sources = OUT / 'sources'
    sources.mkdir(exist_ok=False)
    paths = {'teacher_controller.py': ROOT / 'navigation/controller.py',
        'shared_controller.py': ROOT.parent / 'navigation/controller.py',
        'goal_regions.py': ROOT.parent / 'navigation/goal_regions.py',
        'profile_dwell.json': ROOT / 'navigation/dynamic/flat_dynamic_profile_dwell.json',
        'profile_turn20.json': ROOT / 'navigation/dynamic/flat_dynamic_profile_turn20.json',
        'profile_dynamic12.json': ROOT / 'navigation/dynamic/flat_dynamic_profile.json',
        'profile_flat.json': ROOT / 'navigation/flat_relative_roundtrip.json',
        'worker.py': ROOT / 'policy/worker.py', 'teacher_transition.py': ROOT / 'navigation/teacher_transition.py',
        'guard_audit.py': ROOT / 'navigation/guard_audit.py', 'scoped_profile.py': ROOT / 'navigation/scoped_profile.py'}
    for name, path in paths.items():
        shutil.copyfile(path, sources / name)
    spec = importlib.util.spec_from_file_location('audited_measured_goal_regions', sources / 'goal_regions.py')
    regions = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = regions
    spec.loader.exec_module(regions)
    shared = get_class(sources / 'shared_controller.py', 'Navigation')
    teacher = get_class(sources / 'teacher_controller.py', 'TeacherNavigation')
    base_methods = methods(shared, ['reset_region_arrival', 'measured_region_arrival', 'control'])
    teacher_methods = methods(teacher, ['measured_region_arrival'])
    base_class = ast.ClassDef(name='ActualBaseMethods', bases=[], keywords=[], body=base_methods, decorator_list=[])
    teacher_class = ast.ClassDef(name='ActualTeacherOverride', bases=[ast.Name(id='ActualBaseMethods', ctx=ast.Load())],
        keywords=[], body=teacher_methods, decorator_list=[])
    flow_calls = []
    fake_guard_blocked, fake_exhausted = [False], [False]
    def fake_follow(pose, rotation, samples, goal, **kwargs):
        flow_calls.append('checked_SCAN_follow')
        return np.array([.1, 0.]), .01, np.asarray(goal), {'heading': 0., 'direction': np.array([1., 0.]), 'exhausted': fake_exhausted[0]}
    def fake_guard(*args):
        flow_calls.append('original_native_guard_site')
        return (True, .5, 3) if fake_guard_blocked[0] else (False, None, 0)
    namespace = {'np': np, 'math': math, 'copy': copy, 'ArrivalWindow': regions.ArrivalWindow,
        'goal_contains': regions.contains, 'goal_contains_control': regions.contains_control,
        'follow_trajectory': fake_follow, 'steering_obstacle_ahead': fake_guard}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[base_class, teacher_class], type_ignores=[])), str(sources / 'shared_controller.py'), 'exec'), namespace)
    Base, Teacher = namespace['ActualBaseMethods'], namespace['ActualTeacherOverride']
    goal = regions.Goal('test_goal', (0., 0., 0.), 'disc_prism', radius=.22, height_half_span=.1,
        dwell_sim_s=.6, timeout_sim_s=90., control_extents=(.17, .1))
    dwell_profile = json.loads((sources / 'profile_dwell.json').read_text())
    old_profiles = [json.loads((sources / name).read_text()) for name in ('profile_flat.json', 'profile_dynamic12.json', 'profile_turn20.json')]

    class Heading:
        def __init__(self): self.phase = 'drive'; self.allow = True
        def update(self, *args): flow_calls.append('existing_heading_gate'); return self.allow, .01
        def reset(self): self.phase = 'pre_turn'
        def resume_after_stop(self, *args): return False

    def make(cls=Teacher, profile=None):
        n = cls()
        n.profile = dwell_profile if profile is None else profile
        n.goals = [goal, copy.deepcopy(goal)]
        n.waypoints = np.array([goal.center, goal.center])
        n.waypoint_index = 0
        n.region_raw_pose = np.array([.15, 0., 0.])
        n.pose = n.region_raw_pose.copy()
        n.pose_stamp = 10000000000
        n.region_arrival = regions.ArrivalWindow(goal)
        n.region_arrival_evidence = None
        n.request_id = 'offline_measured_slam'
        n.goals_definition_sha256 = regions.definitions_sha256(n.goals)
        n.obstacle_hold = n.tilt_hold = False
        n.bridge_safety = {'state': 'ready'}
        n.clock_s = n.now = 10.
        n.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(n.clock_s * 1e9)))
        n.state = 'running'
        n.segment_start = n.pose.copy()
        n.segment_started_ros = 10.
        n.pose_updated = n.cloud_updated = 10.
        n.pose_timeout = n.cloud_timeout = .3
        n.raw_imu = SimpleNamespace(fresh=lambda *args: True, stamp=10., tilt=0.)
        n.protect = False
        n.apply_tilt_guard = lambda *args: n.protect
        n.region_arrivals = []
        n.command = [0., 0., 0.]
        n.published = []
        def publish(velocity=None, yaw_rate=0.):
            n.command = [0., 0., 0.] if velocity is None else [float(velocity[0]), float(velocity[1]), float(yaw_rate)]
            n.published.append(n.command.copy())
            n.last_command_time = n.clock_s
        n.publish_command = publish
        n.last_command_time = 10.
        n.heading_gate = Heading()
        n.rotation = np.eye(3)
        n.tracking_pose = n.pose.copy()
        n.samples = np.array([[0., 0., 0.], [1., 0., 0.]])
        n.max_speed = .2
        n.stale_since = None
        n.last_spline_rejected = None
        n.last_reference = 10.
        n.request_plan = lambda: flow_calls.append('request_checked_SCAN')
        n.last_obstacle_check = 0.
        n.cloud = np.array([[.5, 0., .1]] * 3)
        n.cloud_input_context = None
        n.obstacle_resume_pending = False
        n.obstacle_result = (False, None, 0)
        n.obstacle_clear_since = None
        n.obstacle_stops = n.obstacle_resumes = 0
        n.min_obstacle_clearance = None
        n.obstacle_guard_context = None
        n.pending_obstacle_event = None
        n.trajectory_association = SimpleNamespace(reset=lambda: None, reference_stamp=[10, 0])
        n.active_trajectory_id = 1
        n.get_logger = lambda: SimpleNamespace(warning=lambda text: None)
        return n

    checks, scenarios = {}, []
    # Old selectors retain actual native method results across all input/state/stamp variants.
    equality_errors = []
    for profile in old_profiles:
        a, b = make(Base, profile), make(Teacher, profile)
        for i in range(200):
            stamp = 10000000000 + i * 100000000
            if i % 11 == 0: stamp -= 100000000
            x = .15 if i % 7 else .171
            for n in (a, b):
                n.pose_stamp = stamp
                n.region_raw_pose = np.array([x, 0., 0.])
                n.obstacle_hold = i % 19 == 0
                n.bridge_safety = {'state': 'hold' if i % 23 == 0 else 'ready'}
            ra, rb = a.measured_region_arrival(), b.measured_region_arrival()
            if ra != rb or a.region_arrival_evidence != b.region_arrival_evidence:
                equality_errors.append({'profile': profile['experiment'], 'case': i})
    checks['600_old_flat_dynamic12_V43_results_and_receipts_identical'] = not equality_errors
    n = make()
    result = []
    for i in range(7):
        n.pose_stamp = 10000000000 + i * 100000000
        result.append(n.measured_region_arrival())
    checks['new_selector_never_stops_or_arrives_before_actual600ms'] = result[:6] == [(False, False)] * 6 and result[6] == (True, True)
    checks['arrival_receipt_keeps_actual_raw_pose_integer_stamp_radius_dwell_gap'] = (
        n.region_arrival_evidence['raw_position'] == [.15, 0., 0.] and n.region_arrival_evidence['stamp_ns'] == 10600000000
        and n.region_arrival_evidence['dwell_ns'] == 600000000 and n.region_arrival_evidence['max_observation_gap_ns'] == 200000000
        and n.region_arrival_evidence['control_arrival_definition']['radius_m'] == .17)
    scenarios.append({'name': 'continuous_actual_stamp_dwell', 'returns': result, 'receipt': n.region_arrival_evidence})
    for name, changes, expected_reason in [
        ('outside_control_band_but_inside_outer', {'region_raw_pose': np.array([.18, 0., 0.])}, 'outside'),
        ('control_boundary_plus1nm', {'region_raw_pose': np.array([.170000001, 0., 0.])}, 'outside'),
        ('height_outside', {'region_raw_pose': np.array([0., 0., .100000001])}, 'outside'),
        ('obstacle_protected', {'obstacle_hold': True}, 'protected'),
        ('tilt_protected', {'tilt_hold': True}, 'protected'),
        ('bridge_hold_protected', {'bridge_safety': {'state': 'hold'}}, 'protected'),
        ('no_measured_pose', {'region_raw_pose': None}, 'no_measured_pose')]:
        n = make()
        for key, value in changes.items(): setattr(n, key, value)
        answer = n.measured_region_arrival()
        checks[name + '_does_not_arrive_or_adv_goal'] = answer == (False, False) and n.waypoint_index == 0 and n.region_arrival.reason == expected_reason and n.region_arrival.since is None
    n = make()
    n.measured_region_arrival()
    duplicate = [n.measured_region_arrival() for _ in range(50)]
    checks['50_duplicate_pose_stamps_cannot_create_dwell_or_advance'] = duplicate == [(False, False)] * 50 and n.region_arrival.last_stamp == 10000000000 and n.region_arrival.since == 10000000000
    n.pose_stamp = 9999999999
    checks['out_of_order_pose_resets_dwell'] = n.measured_region_arrival() == (False, False) and n.region_arrival.since is None and n.region_arrival.reason == 'out_of_order'
    n = make()
    n.measured_region_arrival()
    n.pose_stamp += 200000001
    checks['gap_above200ms_resets_and_cannot_use_previous_dwell'] = n.measured_region_arrival() == (False, False) and n.region_arrival.since == n.pose_stamp
    n = make()
    n.measured_region_arrival()
    n.pose_stamp += 200000000
    checks['exact200ms_gap_is_inherited_allowed_boundary'] = n.measured_region_arrival() == (False, False) and n.region_arrival.since == 10000000000

    def flow_case(name, mutate, expectation):
        n = make()
        fake_guard_blocked[0] = fake_exhausted[0] = False
        flow_calls.clear()
        mutate(n)
        n.control(n.now)
        ok = expectation(n, flow_calls)
        checks[name] = bool(ok)
        scenarios.append({'name': name, 'calls': flow_calls.copy(), 'requested': n.command, 'state': n.state,
            'waypoint_index': n.waypoint_index, 'region_arrivals': n.region_arrivals.copy(), 'message': getattr(n, 'message', None)})

    flow_case('pending_dwell_keeps_checked_SCAN_heading_and_original_guard', lambda n: None,
        lambda n, calls: calls == ['checked_SCAN_follow', 'existing_heading_gate', 'original_native_guard_site'] and n.command[0] == .1 and n.waypoint_index == 0)
    flow_case('pending_dwell_obstacle_block_still_parks_without_goal_advance', lambda n: fake_guard_blocked.__setitem__(0, True),
        lambda n, calls: 'original_native_guard_site' in calls and n.command == [0., 0., 0.] and n.obstacle_hold and n.waypoint_index == 0)
    flow_case('pending_dwell_heading_hold_still_parks', lambda n: (setattr(n.heading_gate, 'allow', False), setattr(n.heading_gate, 'phase', 'pre_turn')),
        lambda n, calls: 'existing_heading_gate' in calls and 'original_native_guard_site' in calls and n.command == [0., 0., 0.] and n.waypoint_index == 0)
    flow_case('pending_dwell_exhausted_path_still_stops_before_guard_as_original', lambda n: fake_exhausted.__setitem__(0, True),
        lambda n, calls: 'checked_SCAN_follow' in calls and 'original_native_guard_site' not in calls and n.command == [0., 0., 0.] and n.waypoint_index == 0)
    flow_case('pending_dwell_missing_checked_SCAN_still_stops', lambda n: setattr(n, 'samples', None),
        lambda n, calls: 'checked_SCAN_follow' not in calls and n.command == [0., 0., 0.] and n.waypoint_index == 0)
    flow_case('stale_pose_resets_before_arrival_and_parks', lambda n: setattr(n, 'pose_updated', n.now - .31),
        lambda n, calls: not calls and n.command == [0., 0., 0.] and n.region_arrival.reason == 'stale' and n.waypoint_index == 0)
    flow_case('stale_cloud_resets_before_arrival_and_parks', lambda n: setattr(n, 'cloud_updated', n.now - .31),
        lambda n, calls: not calls and n.command == [0., 0., 0.] and n.region_arrival.reason == 'stale' and n.waypoint_index == 0)
    flow_case('stale_IMU_resets_before_arrival_and_parks', lambda n: setattr(n.raw_imu, 'fresh', lambda *a: False),
        lambda n, calls: not calls and n.command == [0., 0., 0.] and n.region_arrival.reason == 'stale' and n.waypoint_index == 0)
    flow_case('protected_command_gate_cannot_advance_goal', lambda n: setattr(n, 'protect', True),
        lambda n, calls: not calls and n.command == [0., 0., 0.] and n.region_arrival.reason == 'protected' and n.waypoint_index == 0)
    flow_case('90second_deadline_is_checked_before_arrival_and_never_extended', lambda n: setattr(n, 'segment_started_ros', n.clock_s - 90.000001),
        lambda n, calls: not calls and n.state == 'failed' and n.command == [0., 0., 0.] and n.waypoint_index == 0)
    # Same-center synthetic next goal specifically challenges stale-stamp reuse.
    n = make()
    for i in range(7):
        n.pose_stamp = 10000000000 + i * 100000000
        n.region_arrival.observe(n.region_raw_pose, n.pose_stamp)
    # Exact arriving observation must be new relative to current last_stamp.
    n.region_arrival.last_stamp -= 1
    fake_guard_blocked[0] = fake_exhausted[0] = False
    n.control(n.now)
    first_adv = n.waypoint_index == 1 and len(n.region_arrivals) == 1 and n.region_arrival.last_stamp == n.pose_stamp
    before = copy.deepcopy(n.region_arrivals)
    n.control(n.now)
    checks['next_goal_cannot_reuse_previous_goal_last_raw_stamp_or_receipt'] = first_adv and n.waypoint_index == 1 and n.region_arrivals == before and n.region_arrival.since is None
    new_profile, old_profile = dwell_profile, json.loads((sources / 'profile_turn20.json').read_text())
    differences = {k: {'before': old_profile.get(k), 'after': new_profile.get(k)} for k in set(old_profile) | set(new_profile) if old_profile.get(k) != new_profile.get(k)}
    checks['new_profile_only_explicit_arrival_stop_policy_scope_metadata'] = set(differences) == {'experiment', 'arrival_stop_policy', 'variant'}
    preserved = ['arrival_control_radius_m', 'dwell_sim_s', 'goal_timeout_sim_s', 'pose_cloud_timeout_s', 'teacher_transition', 'fence', 'max_speed_mps', 'max_yaw_rate_radps']
    checks['all_arrival_safety_deadline_and_velocity_limits_preserved'] = all(new_profile[k] == old_profile[k] for k in preserved)
    v43 = ROOT / 'runs/20261004_100256_navigation_slam_scan_dynamic_flat_v43_turn20_r1_2f70/sources'
    checks['shared_controller_and_regions_bytes_unchanged_from_actual_V43'] = all(
        sha(sources / new) == sha(v43 / old) for new, old in [('shared_controller.py', 'navigation/controller.py'), ('goal_regions.py', 'navigation/goal_regions.py')]
        if (v43 / old).exists())
    # Shared archives live outside Teacher sources in some runner versions, compare freeze refs too in follow-up.
    checks['worker_TTL_command_stop_and_actor_source_bytes_unchanged'] = sha(sources / 'worker.py') == sha(v43 / 'policy/worker.py')
    result = {'schema': 'teacher_arrival_dwell_independent_offline_review/v1', 'asof_utc': datetime.now(timezone.utc).isoformat(),
        'passed': all(checks.values()), 'actual_navigation_pass_asserted': False,
        'scope': 'Actual method AST + actual Goal/ArrivalWindow source with synthetic accepted SLAM contexts and mocked motion/guard call sites, no ROS/sim or source writes',
        'checks': checks, 'old_profile_comparison_errors': equality_errors, 'scenarios': scenarios,
        'profile_diff': differences, 'source_hashes': {name: {'path': str(path), 'snapshot_sha256': sha(sources / name), 'current_equals_snapshot': sha(path) == sha(sources / name)} for name, path in paths.items()},
        'script_sha256': sha(__file__),
        'limits': ['Guard spy validates continued original call site/order and protective branch; no new geometry or physics proof.',
            'Actual ArrivalWindow gap remains200ms, while source TTL remains300ms. Neither is relaxed.',
            'Profile metadata maximum_source_gap_s=.3 denotes source freshness, not the ArrivalWindow200ms gap.',
            'Old profile behavior stays native; new parking waits for actual fresh raw stamp dwell, not wall clock or tracking pose.',
            'Offline success does not imply actual region reach or closed-loop navigation success; finite fence, checked-path exhaustion and90s deadline still apply.']}
    (OUT / 'review.json').write_text(json.dumps(result, indent=2, ensure_ascii=False, allow_nan=False) + '\n')
    print(json.dumps({'passed': result['passed'], 'checks': checks, 'source_controller_sha256': sha(sources / 'teacher_controller.py'),
        'receipt_sha256': sha(OUT / 'review.json'), 'output': str(OUT)}))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
