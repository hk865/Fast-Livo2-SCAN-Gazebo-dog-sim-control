#!/usr/bin/env python3
"""Independent original-prefix9 audit. Full reads are refused until owned cleanup.

Native simulator states are offline evidence only. A limited prefix result never
certifies original46, corridor control, or unsampled 200 Hz physics. This script
does not import a historical full-mission evaluator or write into the run.
"""
from __future__ import annotations
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np

HERE = Path(__file__).resolve().parent
TEACHER = HERE.parents[2]
DEMO = TEACHER.parent
CORE = TEACHER / 'navigation/corridor_tracking_v20'
sys.path[:0] = [str(CORE), str(DEMO)]
from navigation.goal_regions import contains_control, definitions_sha256, parse_request
from route import build_request
from prefix_contract import validate_prefix

FROZEN_TEACHER = 'bfe7fbbffe85fdb60cac590f1a6080a10609011ab6a538a078381261e98fbd34'
DEFAULT_BASELINE = TEACHER / 'runs/20261006_024447_closed_loop_cascade_clock_hold_v19_original46_r3_heading_9779'
NATIVE_FIELDS = ('state_physics_world_time', 'world_sim_time', 'rpy', 'body_lin_vel',
    'body_ang_vel', 'qd', 'applied_torque', 'q_target', 'fault',
    'actor_inferred_this_frame', 'body_clearance', 'contacts', 'command', 'command_expired', 'position')
STATUS_FIELDS = ('request_id', 'state', 'goals_definition_sha256', 'goals_definitions',
    'region_arrivals', 'region_arrival_evidence', 'waypoint_index', 'total', 'ros_sim_time',
    'goal_activated_ros_clock_ns', 'obstacle_hold', 'tilt_hold', 'execution_bridge_safety',
    'pose_age', 'cloud_age', 'raw_imu_age', 'feedback_source', 'command', 'message')
PARK_FIELDS = ('mode', 'parking_hold_declared_pose_stamp_ns', 'parking_hold_declared_clock_ns',
    'parking_hold_declared_wall_ns', 'parking_window_interrupted', 'fixed_goal_sha256',
    'fixed_goal', 'feedback_source', 'navigation_ground_truth_used', 'protection_active', 'failure_latched')


def sha(raw): return hashlib.sha256(raw).hexdigest()
def canonical(value): return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False)
def file_sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as source:
        for raw in iter(lambda: source.read(1 << 20), b''): h.update(raw)
    return h.hexdigest()
def fields(row, names): return {k: row.get(k) for k in names}
def wrap(value): return math.atan2(math.sin(value), math.cos(value))
def yaw_xyzw(q):
    q = np.asarray(q, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or abs(np.linalg.norm(q)-1) > .01:
        raise ValueError('invalid measured quaternion')
    x,y,z,w = q
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
def native_ns(row):
    value = row.get('state_physics_world_time')
    if value is None: value = row.get('world_sim_time')
    if type(value) not in (int, float) or not math.isfinite(value): raise ValueError('missing native time')
    return round(value*1e9)
def status_ns(row): return round(row['ros_sim_time']*1e9)


class Sources:
    def __init__(self): self.bindings = {}; self.errors = []
    def bind(self, path):
        path = Path(path).resolve()
        value = dict(file=str(path), sha256=file_sha(path), bytes=path.stat().st_size)
        self.bindings[str(path)] = value
        return value
    def json(self, path):
        path = Path(path).resolve(); raw = path.read_bytes()
        self.bindings[str(path)] = dict(file=str(path), sha256=sha(raw), bytes=len(raw))
        return json.loads(raw)
    def rows(self, path):
        path = Path(path).resolve(); before = path.stat(); digest = hashlib.sha256(); offset = 0
        with path.open('rb') as source:
            for index, raw in enumerate(source, 1):
                digest.update(raw)
                ref = dict(file=str(path), line=index, offset=offset, length=len(raw), sha256=sha(raw))
                offset += len(raw)
                try:
                    if not raw.endswith(b'\n'): raise ValueError('incomplete final JSONL row')
                    row = json.loads(raw)
                    if not isinstance(row, dict): raise ValueError('nonobject JSONL row')
                    yield row, ref
                except (ValueError, UnicodeError) as error:
                    self.errors.append(dict(reason=str(error), source=ref))
        after = path.stat()
        stable = (before.st_dev, before.st_ino, before.st_size, before.st_mtime_ns) == (after.st_dev, after.st_ino, after.st_size, after.st_mtime_ns)
        self.bindings[str(path)] = dict(file=str(path), sha256=digest.hexdigest(), bytes=offset, stable_during_read=stable)
        if not stable: self.errors.append(dict(reason='source changed during offline read', file=str(path)))


def result(checks, **details):
    return dict(passed=bool(checks) and all(checks.values()), checks=checks, **details)


def coverage(stamps, start, end, max_gap, minimum):
    stamps = list(stamps)
    return (len(stamps) >= minimum and 0 <= stamps[0]-start <= max_gap
        and 0 <= end-stamps[-1] <= max_gap
        and all(0 < b-a <= max_gap for a,b in zip(stamps, stamps[1:])))


def native_safe(row):
    """Recorded-sample safety, explicitly not inter-sample physical safety."""
    try:
        if row.get('fault') is not None or row.get('actor_inferred_this_frame') is not True: return False
        for key, size in [('rpy',3), ('body_lin_vel',3), ('body_ang_vel',3), ('qd',12), ('applied_torque',12), ('q_target',12), ('command',3)]:
            value = np.asarray(row[key], dtype=float)
            if value.shape != (size,) or not np.isfinite(value).all(): return False
        clearance = row['body_clearance']; contact = row['contacts']['body']
        return (math.isfinite(clearance) and clearance >= .18 and math.isfinite(contact) and contact <= 0
            and max(abs(float(x)) for x in row['rpy'][:2]) <= .65
            and max(abs(float(x)) for x in row['qd']) <= 30.001
            and max(abs(float(x)) for x in row['applied_torque']) <= 23.50001)
    except (KeyError, ValueError, TypeError): return False


def validate_dwell(goal, receipt, poses, request_id, goal_hash, index):
    start = receipt.get('start_stamp_ns', -1); end = receipt.get('stamp_ns', -1)
    selected = [p for p in poses if start <= p['stamp_ns'] <= end]
    stamps = [p['stamp_ns'] for p in selected]
    checks = dict(original_goal_id=receipt.get('goal_id') == goal.goal_id,
        ordered_index=receipt.get('waypoint_index') == index, request=receipt.get('request_id') == request_id,
        complete_definition_hash=receipt.get('goals_definition_sha256') == goal_hash,
        original_arrival_definition=receipt.get('arrival_definition') == goal.definition()['arrival'],
        original_control_band=receipt.get('control_arrival_definition') == goal.control_arrival_definition(),
        duration=end-start >= round(goal.dwell_sim_s*1e9) and receipt.get('dwell_ns') == end-start,
        exact_source_endpoints=bool(stamps) and stamps[0] == start and stamps[-1] == end,
        original_max_gap=receipt.get('max_observation_gap_ns') == 200_000_000,
        continuous_raw_samples=coverage(stamps, start, end, 200_000_000, 3),
        original_control_region=bool(selected) and all(contains_control(goal, p['position']) for p in selected),
        raw_arrival_position=bool(selected) and receipt.get('raw_position') == selected[-1]['position'],
        unprotected_receipt=receipt.get('protected') is False and receipt.get('region_inside') is True and receipt.get('control_region_inside') is True and receipt.get('reason') == 'arrived',
        within_original_timeout=isinstance(receipt.get('goal_activated_ros_clock_ns'), int) and
            0 <= end-receipt['goal_activated_ros_clock_ns'] <= round(goal.timeout_sim_s*1e9),
        raw_slam_frame=bool(selected) and all(p.get('frame_id') == 'camera_init' for p in selected))
    return result(checks, goal_id=goal.goal_id, receipt=receipt,
        raw_pose_count=len(selected), actual_max_gap_ns=max(np.diff(stamps).tolist(), default=None),
        raw_pose_sources=[p['_source'] for p in selected])


def evaluate_regions(goals, request_id, statuses, poses):
    goal_hash = definitions_sha256(goals); receipts = []; sources = []; conflicts = []
    by_goal = {}
    for row in statuses:
        if row.get('request_id') != request_id: continue
        values = row.get('region_arrivals') or []
        if not isinstance(values, list): conflicts.append(row['_source']); continue
        for index, receipt in enumerate(values):
            key = receipt.get('goal_id')
            if key in by_goal and by_goal[key] != receipt: conflicts.append(row['_source'])
            if key not in by_goal:
                by_goal[key] = receipt; receipts.append(receipt); sources.append(row['_source'])
            if receipt.get('waypoint_index') != index: conflicts.append(row['_source'])
    individual = []
    for index, (goal, receipt) in enumerate(zip(goals, receipts)):
        detail = validate_dwell(goal, receipt, poses, request_id, goal_hash, index)
        start, end = receipt['start_stamp_ns'], receipt['stamp_ns']
        window = [s for s in statuses if s.get('request_id') == request_id and start <= status_ns(s) <= end]
        detail['checks']['recorded_statuses_unprotected'] = bool(window) and all(
            not s.get('tilt_hold') and not s.get('obstacle_hold') and
            (s.get('execution_bridge_safety') or {}).get('state') == 'ready' for s in window)
        detail['passed'] = all(detail['checks'].values())
        detail['receipt_source'] = sources[index]; individual.append(detail)
    checks = dict(exactly_original_nine=len(goals) == 9 and len(receipts) == 9,
        ordered_unique_receipts=[r.get('goal_id') for r in receipts] == [g.goal_id for g in goals],
        immutable_receipts=not conflicts, all_raw_dwells=bool(individual) and all(r['passed'] for r in individual),
        original_definitions_in_status=all(s.get('goals_definitions') == [g.definition() for g in goals]
            and s.get('goals_definition_sha256') == goal_hash for s in statuses if s.get('request_id') == request_id))
    return result(checks, actual_receipt_count=len(receipts), raw_validated_count=sum(r['passed'] for r in individual),
        goal_definition_sha256=goal_hash, receipts=individual, conflicting_sources=conflicts)


def evaluate_parking(request_id, pid, statuses, poses, native):
    # The first declaration is irrevocable: never search for a later quiet window.
    candidates = [r for r in pid if r.get('request_id') == request_id and r.get('waypoint_index') == 9
        and isinstance(r['cascade'].get('parking_hold_declared_clock_ns'), int)]
    if not candidates:
        # Some adapters keep the final goal index at 8 after arrival.
        candidates = [r for r in pid if r.get('request_id') == request_id and r.get('waypoint_index') == 8
            and isinstance(r['cascade'].get('parking_hold_declared_clock_ns'), int)]
    if not candidates: return result(dict(first_hold_declaration=False), reason='No final-goal active-hold declaration; nine arrivals or stop does not substitute for parking.')
    first = min(candidates, key=lambda r:r['cascade']['parking_hold_declared_clock_ns'])
    declared = first['cascade']; start = declared['parking_hold_declared_clock_ns']; end = start+5_000_000_000
    origin_stamp = declared.get('parking_hold_declared_pose_stamp_ns')
    origin = next((p for p in poses if p['stamp_ns'] == origin_stamp), None)
    ps = [p for p in poses if start <= p['stamp_ns'] <= end]
    ns = [r for r in native if start <= native_ns(r) <= end]
    ss = [r for r in statuses if start <= status_ns(r) <= end]
    cs = [r for r in pid if start <= r['control_stamp_ns'] <= end]
    fixed = declared.get('fixed_goal_sha256')
    checks = dict(origin_exact_actual_slam=origin is not None,
        fixed_first5s_pose_coverage=coverage([p['stamp_ns'] for p in ps],start,end,200_000_000,25),
        fixed_first5s_native_coverage=coverage([native_ns(r) for r in ns],start,end,30_000_000,200),
        fixed_first5s_status_coverage=coverage([status_ns(r) for r in ss],start,end,300_000_000,20),
        fixed_first5s_control_coverage=coverage([r['control_stamp_ns'] for r in cs],start,end,300_000_000,20),
        only_active_hold=bool(cs) and all(r.get('request_id') == request_id and r['cascade'].get('mode') == 'active_hold'
            and r['cascade'].get('parking_window_interrupted') is False
            and r['cascade'].get('parking_hold_declared_clock_ns') == start
            and r['cascade'].get('fixed_goal_sha256') == fixed and not r['cascade'].get('protection_active')
            and not r['cascade'].get('failure_latched') for r in cs),
        succeeded_unprotected=bool(ss) and all(r.get('state') == 'succeeded' and r.get('request_id') == request_id
            and r['cascade_parking'].get('mode') == 'active_hold' and not r.get('obstacle_hold') and not r.get('tilt_hold')
            and (r.get('execution_bridge_safety') or {}).get('state') == 'ready'
            and len(r.get('region_arrivals') or []) == 9 for r in ss),
        recorded_native_safe=bool(ns) and all(native_safe(r) and r.get('command_expired') is False for r in ns),
        actual_pose_callback_fresh=bool(ps) and all(isinstance(p.get('callback_ros_clock_ns'), int)
            and 0 <= p['callback_ros_clock_ns']-p['stamp_ns'] <= 300_000_000 for p in ps))
    metrics = {}
    if origin is not None and ps and len(ns) >= 2:
        try:
            metrics = dict(slam_xy_drift_max_m=max(float(np.linalg.norm(np.asarray(p['position'][:2])-origin['position'][:2])) for p in ps),
                slam_yaw_drift_max_rad=max(abs(wrap(yaw_xyzw(p['quaternion'])-yaw_xyzw(origin['quaternion']))) for p in ps),
                native_planar_speed_max_mps=max(float(np.linalg.norm(r['body_lin_vel'][:2])) for r in ns),
                native_wz_max_radps=max(abs(r['body_ang_vel'][2]) for r in ns))
            t = np.asarray([native_ns(r) for r in ns],dtype=float)/1e9
            yd = np.diff(np.unwrap([r['rpy'][2] for r in ns]))/np.diff(t)
            metrics['native_yaw_dot_max_radps'] = float(np.max(abs(yd))) if np.isfinite(yd).all() else None
            limits = dict(slam_xy_drift_max_m=.05,slam_yaw_drift_max_rad=.1,native_planar_speed_max_mps=.08,
                native_wz_max_radps=.1,native_yaw_dot_max_radps=.1)
            checks['all_original_parking_thresholds'] = all(metrics.get(k) is not None and math.isfinite(metrics[k]) and metrics[k] <= v for k,v in limits.items())
        except (ValueError, TypeError, KeyError, IndexError): checks['all_original_parking_thresholds'] = False
    else: checks['all_original_parking_thresholds'] = False
    return result(checks, clock_interval_ns=[start,end], origin_pose_stamp_ns=origin_stamp,
        first_declaration_source=first['_source'], origin_source=origin['_source'] if origin else None,
        metrics=metrics, counts=dict(poses=len(ps),native=len(ns),statuses=len(ss),control=len(cs)),
        pose_sources=[p['_source'] for p in ps], native_sources=[r['_source'] for r in ns],
        status_sources=[r['_source'] for r in ss], control_sources=[r['_source'] for r in cs],
        native_scope='Recorded 50 Hz native snapshots only; 200 Hz inter-sample physics remains unverified.')


def pid_projection(row, ref):
    d = fields(row, ('request_id','waypoint_index','control_stamp_ns','source_pose_stamp_ns','state',
        'trajectory_id','control_pose','command_after_slew','navigation_ground_truth_used','compute_monotonic_wall'))
    d['cascade'] = fields(row.get('cascade') or {}, PARK_FIELDS)
    d['heading'] = fields(row.get('heading_gate_reference') or {}, ('phase','locked_heading','steering'))
    d['feedback'] = fields(row.get('feedback') or {}, ('stamp_ns','received_wall_ns','frame_id','callback_ros_clock_ns'))
    d['_source'] = ref
    return d


def ninth_diagnostics(pid, statuses, poses, goal_id='exploration:8'):
    values = [r for r in pid if r.get('waypoint_index') == 8]
    events = []; durations = defaultdict(float); unobserved = 0.; modes = Counter()
    for previous, current in zip(values, values[1:]):
        dt = (current['control_stamp_ns']-previous['control_stamp_ns'])/1e9
        phase = previous['heading'].get('phase'); next_phase = current['heading'].get('phase')
        if 0 < dt <= .2: durations[str(phase)] += dt
        elif dt > 0: unobserved += dt
        if phase == 'drive' and next_phase == 'pre_turn':
            events.append(dict(previous_source=previous['_source'], current_source=current['_source'],
                control_stamp_ns=current['control_stamp_ns'], new_path=current.get('trajectory_id') != previous.get('trajectory_id'),
                steering=current['heading'].get('steering')))
    for r in values: modes[str(r['cascade'].get('mode'))] += 1
    observations = [r for r in statuses if r.get('waypoint_index') == 8]
    activation = next((r['goal_activated_ros_clock_ns'] for r in observations if isinstance(r.get('goal_activated_ros_clock_ns'),int)), None)
    receipt = next((x for r in statuses for x in r.get('region_arrivals') or [] if x.get('goal_id') == goal_id), None)
    failed = next((r for r in observations if r.get('state') == 'failed'), None)
    terminal = receipt.get('stamp_ns') if receipt else status_ns(failed) if failed else None
    route = [p for p in poses if activation is not None and p['stamp_ns'] >= activation and (terminal is None or p['stamp_ns'] <= terminal)]
    gaps = []; length = 0.
    for a,b in zip(route,route[1:]):
        dt = b['stamp_ns']-a['stamp_ns']
        if 0 < dt <= 200_000_000: length += float(np.linalg.norm(np.asarray(b['position'])-a['position']))
        else: gaps.append(dict(previous=a['_source'],current=b['_source'],gap_ns=dt))
    return dict(goal_id=goal_id, PID_rows=len(values), drive_to_pre_turn_count=len(events),
        new_path_reset_count=sum(e['new_path'] for e in events), phase_durations_sim_s=dict(durations),
        phase_uncovered_intervals_s=unobserved, duration_scope='Adjacent PID rows with 0 < gap <= 0.2 s; held command time beyond gaps not inferred.',
        activation_sim_s=activation/1e9 if activation is not None else None,
        terminal_sim_s=terminal/1e9 if terminal is not None else None,
        observed_elapsed_until_arrival_or_failure_s=(terminal-activation)/1e9 if terminal is not None and activation is not None else None,
        outcome='original_region_arrived' if receipt else 'failed' if failed else 'unverified',
        route_length_observed_slam_3d_m=length, route_gap_sources=gaps,
        first_actual_slam_position=route[0]['position'] if route else None,
        last_actual_slam_position=route[-1]['position'] if route else None,
        mode_counts=dict(modes), reset_events=events)


def load_logs(run, sources, native=True):
    poses = []
    for row, ref in sources.rows(run/'navigation_slam_poses.jsonl'):
        value = fields(row, ('stamp_ns','frame_id','position','quaternion','received_monotonic_wall','callback_ros_clock_ns','sim_age_at_callback_s'))
        value['_source'] = ref; poses.append(value)
    statuses = []
    for row, ref in sources.rows(run/'navigation_status.jsonl'):
        value = fields(row, STATUS_FIELDS); value['_source'] = ref
        value['cascade_parking'] = fields(row.get('cascade_parking') or {}, PARK_FIELDS); statuses.append(value)
    pid = [pid_projection(row, ref) for row,ref in sources.rows(run/'navigation_pid_history.jsonl')]
    natives = []
    if native:
        for row, ref in sources.rows(run/'telemetry.jsonl'):
            value = fields(row, NATIVE_FIELDS); value['_source'] = ref; natives.append(value)
    return poses, statuses, pid, natives


def verify_archive(run, sources, manifest):
    checks = []; hashes = set(); failures = []
    # Absolute manifest aliases are not read from the mutable live checkout.
    for key, expected in manifest.items():
        if Path(key).is_absolute(): continue
        path = (run/'sources'/key).resolve()
        if not path.is_relative_to((run/'sources').resolve()) or not path.is_file():
            failures.append(dict(key=key, reason='missing archived source')); continue
        binding = sources.bind(path); checks.append(binding['sha256'] == expected)
        if binding['sha256'] != expected: failures.append(dict(key=key,reason='archived source hash mismatch'))
        else: hashes.add(expected)
    for key, expected in manifest.items():
        if Path(key).is_absolute() and expected not in hashes:
            failures.append(dict(key=key,reason='absolute source alias has no matching immutable archive'))
    helpers = [CORE/'route.py', CORE/'prefix_contract.py', CORE/'mission46_profile.py',
        DEMO/'navigation/goal_regions.py', DEMO/'mission/route_regions.py']
    helper_bindings = [sources.bind(p) for p in helpers]
    helper_match = all(b['sha256'] in hashes for b in helper_bindings)
    return result(dict(all_archived_sources=bool(checks) and all(checks) and not failures,
        imported_geometry_helpers_identical_to_archive=helper_match), archived_unique_hashes=len(hashes),
        helper_bindings=helper_bindings, failures=failures)


def owned_processes_stopped(runtime):
    """Allow post-abort analysis without changing cleanup failure into PASS."""
    checks=[]
    for saved in runtime.get('owned_processes') or []:
        pid=saved.get('pid'); start=saved.get('start_ticks')
        if not isinstance(pid,int) or not isinstance(start,int): return False,checks
        path=Path('/proc')/str(pid)/'stat'
        try:
            text=path.read_text(); parts=text[text.rfind(')')+2:].split()
            current_start=int(parts[19]); state=parts[0]
            alive=current_start == start and state != 'Z'
        except FileNotFoundError: alive=False; current_start=None; state=None
        checks.append(dict(saved_identity=saved,current_start_ticks=current_start,current_state=state,same_process_live=alive))
    for child in (runtime.get('launch_child_exit_evidence') or {}).get('children') or []:
        # A clean exit receipt is required for children whose start tick is not archived here.
        if child.get('clean_exit') is not True: return False,checks
    return bool(checks) and not any(r['same_process_live'] for r in checks),checks


def audit(run, baseline=DEFAULT_BASELINE):
    run = Path(run).resolve(); sources = Sources()
    if not (run/'runtime_manifest.json').is_file(): raise RuntimeError('Full audit is refused before final runtime manifest; use --status-only while running.')
    runtime = sources.json(run/'runtime_manifest.json')
    stopped,process_evidence=owned_processes_stopped(runtime)
    if runtime.get('all_owned_and_children_clean') is not True and not stopped:
        raise RuntimeError('Full audit requires stopped owned processes; cleanup failure is retained independently.')
    profile = sources.json(run/'navigation_profile.json'); anchor = sources.json(run/'navigation_anchor.json')
    request = sources.json(run/'navigation_request.json'); manifest = sources.json(run/'source_manifest.json')
    archive = verify_archive(run,sources,manifest)
    validate_prefix(profile)
    expected_request = build_request(run.name,anchor,profile,None)
    rid, goals = parse_request(expected_request)
    policy = sources.json(run/'policy_manifest.json'); asset = sources.json(run/'asset_manifest.json')
    slam = sources.json(run/'navigation_slam_contract.json')
    worker = sources.json(run/'worker_result.json') if (run/'worker_result.json').exists() else {}
    pose_rows,statuses,pid,native = load_logs(run,sources)
    geometry = result(dict(prefix_profile=True, exact_original_transformed_request=request == expected_request,
        sensor_slam_anchor=anchor.get('source') == '/demo/slam/body_odom' and anchor.get('ground_truth_navigation_used') is False,
        pose_source_present=bool(pose_rows) and all(r.get('frame_id') == 'camera_init' for r in pose_rows),
        slam_sensor_contract=slam.get('ground_truth_used') is False,
        no_truth_in_controller=bool(pid) and all(r.get('navigation_ground_truth_used') is False and r['feedback'].get('frame_id') == 'camera_init' for r in pid)),
        original_world_goals=profile['original_scenario']['route_goals']['exploration'][:9],
        transformed_goals=[g.definition() for g in goals], request_id=rid)
    regions = evaluate_regions(goals,rid,statuses,pose_rows)
    parking = evaluate_parking(rid,pid,statuses,pose_rows,native)
    bad_native = [r for r in native if not native_safe(r)]
    times = [native_ns(r) for r in native]
    sampled = result(dict(actual_native_samples=bool(native), no_recorded_fault_or_safety_violation=not bad_native,
        native_time_increases=all(b>a for a,b in zip(times,times[1:]))),
        samples=len(native), bad_sample_count=len(bad_native), bad_sources=[r['_source'] for r in bad_native[:100]],
        worker_completion_status='recorded' if worker else 'unverified_missing_after_abort',
        worker_fault=worker.get('fault'), worker_result_present=bool(worker),
        body_contact_samples=sum(bool(r.get('contacts',{}).get('body',0)>0) for r in native),
        actor_fault_samples=sum(r.get('fault') is not None for r in native),
        observed_native_gap_max_ns=max(np.diff(times).tolist(),default=None), all_200hz_physics_verified=False)
    actuator = None
    # Bind the entire actuator log but avoid retaining it in memory.
    for row,ref in sources.rows(run/'actuator.jsonl'):
        if actuator is None: actuator = dict(data=row,source=ref)
    data = actuator['data'] if actuator else {}
    execution = result(dict(frozen_teacher=policy.get('checkpoint_sha256') == FROZEN_TEACHER,
        CPU_single_thread=policy.get('inference_device') == 'cpu' and policy.get('torch_threads') == 1,
        original_actuator=asset.get('exclusive_writer') == 'teacher_sim::TeacherActuator' and runtime.get('exclusive_writer') == 'teacher_sim::TeacherActuator',
        native_writer_contract=data.get('writer') == 'teacher_sim::TeacherActuator sole JointForceCmd writer',
        no_body_pose_resets=data.get('body_pose_resets') == 0,
        original_rates=asset.get('physics_step_s') == .005 and asset.get('policy_hz') == 50 and data.get('expected_dt') == .005 and data.get('decimation') == 4,
        original_spawn=asset.get('spawn') == [0.,0.,.3,0.],
        owned_runtime_clean=runtime.get('all_owned_and_children_clean') is True,
        runtime_error_absent=runtime.get('error') is None,
        final_worker_no_fault=bool(worker) and worker.get('fault') is None), actuator_contract=actuator,
        current_owned_process_stop_evidence=process_evidence,
        scope='Archived configuration, native writer contract and process cleanup; not an exhaustive 200 Hz physical trace.')
    # Pair every PID source stamp with the original raw SLAM record; no nearest-time substitution.
    by_stamp = {p['stamp_ns']:p for p in pose_rows}; unmatched = []
    fresh_failures = []
    for row in pid:
        f = row['feedback']; p = by_stamp.get(f.get('stamp_ns'))
        if p is None or row.get('control_pose') != p.get('position'): unmatched.append(row['_source'])
        # Freshness is evaluated only while a nonzero command is actually published.
        command = row.get('command_after_slew')
        if command is not None and np.linalg.norm(command) > 1e-10:
            wall = row.get('compute_monotonic_wall'); receipt = f.get('received_wall_ns')
            if not (isinstance(f.get('stamp_ns'),int) and 0 <= row['control_stamp_ns']-f['stamp_ns'] <= 300_000_000
                and isinstance(wall,(int,float)) and isinstance(receipt,int) and 0 <= round(wall*1e9)-receipt <= 300_000_000): fresh_failures.append(row['_source'])
    provenance = result(dict(actual_slam_exact_stamp_and_position=bool(pid) and not unmatched,
        nonzero_commands_have_fresh_pose=not fresh_failures), unmatched_sources=unmatched[:100],freshness_failures=fresh_failures[:100],
        note='Cloud/IMU protections are separately recorded; this is exact pose provenance, not a full controller replay.')
    diagnostics = ninth_diagnostics(pid,statuses,pose_rows)
    comparison = dict(baseline_status='unverified',
        candidate_region9_exposed=diagnostics['PID_rows']>0,
        zero_reset_count_without_region9_exposure_is_not_improvement=diagnostics['PID_rows']==0,
        terminal_policy_difference='V20 prefix9 makes original exploration:8 the final parking goal; r3 original46 continues to exploration:9. Region geometry/dwell match; terminal braking/parking do not form an identical-control A/B test.')
    if baseline is not None:
        baseline = Path(baseline).resolve()
        base_runtime = sources.json(baseline/'runtime_manifest.json')
        if base_runtime.get('all_owned_and_children_clean') is not True: raise RuntimeError('Baseline is not stopped')
        bp,bs,bc,_ = load_logs(baseline,sources,native=False)
        original_profile = sources.json(baseline/'navigation_profile.json')
        comparison.update(baseline_status='independently_recomputed_from_archived_raw_logs', baseline_run=str(baseline),
            baseline=ninth_diagnostics(bc,bs,bp),candidate=diagnostics,
            original_world_first9_identical=original_profile['original_scenario']['route_goals']['exploration'][:9] == profile['original_scenario']['route_goals']['exploration'][:9])
    checks = dict(immutable_source_archive=archive['passed'],original_geometry_and_slam=geometry['passed'],
        all_original_nine_dwells=regions['passed'],first_fixed5s_parking=parking['passed'],
        sampled_native_safety=sampled['passed'],execution_contract=execution['passed'],actual_pose_provenance=provenance['passed'],
        source_read_errors_absent=not sources.errors)
    return dict(schema='independent_original46_prefix9_actual_run/v1',run=str(run),run_id=run.name,
        result_status='limited_pass' if all(checks.values()) else 'not_completed_resource_abort' if 'GPU headroom' in str(runtime.get('error')) else 'not_passed',
        prefix9_limited_pass=all(checks.values()),checks=checks,geometry=geometry,archive=archive,
        runtime_termination=dict(status=runtime.get('runtime_status'),error=runtime.get('error'),
            recorded_clean_cleanup=runtime.get('all_owned_and_children_clean'),
            resource_abort='GPU headroom' in str(runtime.get('error')),
            ninth_region_controller_failure_established=diagnostics['outcome']=='failed'),
        original_regions=regions,first5s_parking=parking,sampled_native_safety=sampled,
        execution_contract=execution,actual_slam_provenance=provenance,ninth_region_diagnostics=diagnostics,
        r3_comparison=comparison,source_bindings=list(sources.bindings.values()),source_errors=sources.errors,
        evaluator=dict(file=str(Path(__file__).resolve()),sha256=file_sha(__file__)),
        unverified=['Full original46 mission, return, current RGB save and subsequent navigation',
            'All 200 Hz physics steps and inter-sample contacts/torques/transitions; native evidence is 50 Hz snapshots',
            'Corridor controlling motion or retaining a previous plan: implemented runtime is shadow only',
            'Hardware deployment and real stairs', 'Full closed-loop counterfactual causal attribution of V20 vs V19'],
        full46_pass=False,whole_200hz_physical_acceptance=False,corridor_control_verified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    parser.add_argument('--baseline',type=Path,default=DEFAULT_BASELINE)
    parser.add_argument('--skip-baseline',action='store_true')
    parser.add_argument('--status-only',action='store_true',help='Read only the atomic small status JSON; no acceptance claim.')
    args = parser.parse_args()
    if args.status_only:
        row=json.loads((args.run/'navigation_status.json').read_text())
        print(json.dumps(dict(state=row.get('state'),waypoint_index=row.get('waypoint_index'),total=row.get('total'),
            observed_region_receipts=len(row.get('region_arrivals') or []),mode=(row.get('cascade_parking') or {}).get('mode'),
            acceptance='unverified while running'),indent=2));return
    if args.output is None: parser.error('--output is required for a full offline audit')
    if args.output.resolve().is_relative_to(args.run.resolve()): parser.error('Evaluation output must be outside the run archive')
    report=audit(args.run,None if args.skip_baseline else args.baseline)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(args.output),prefix9_limited_pass=report['prefix9_limited_pass'],checks=report['checks']),indent=2))


if __name__ == '__main__': main()
