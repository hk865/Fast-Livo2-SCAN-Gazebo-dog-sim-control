"""Observe original controller evidence and stop one first-goal experiment.

No commands, new arrival semantics, receipt creation or estimator state changes.
The controller remains the sole producer of ordered-pass evidence.
"""
from pathlib import Path
import hashlib
import json
import math
import numpy as np

def rows_with_refs(path):
    path = Path(path)
    result = []
    try:
        with path.open('rb') as stream:
            while True:
                offset = stream.tell(); raw = stream.readline()
                if not raw: break
                try: value = json.loads(raw)
                except ValueError: break
                result.append((value, dict(file=str(path), offset=offset,
                    bytes=len(raw), sha256=hashlib.sha256(raw).hexdigest())))
    except OSError: pass
    return result

def verified_first_goal(run, status, actual, requests):
    """No nearest pose, stored scalar count or trajectory admission shortcut."""
    from navigation.goal_regions import parse_request, definitions_sha256, contains, contains_control
    from continuous_route import sha as route_sha
    from startup_trace import actual_scan_association
    try:
        rid, goals = parse_request(actual)
        goal_hash = definitions_sha256(goals)
        if (len(goals) != 18 or goals[0].goal_id != 'exploration:0'
            or not rid.startswith(Path(run).name + ':exploration:')): return None
        if (status.get('request_id') != rid or status.get('state') != 'running'
            or status.get('goals_definition_sha256') != goal_hash
            or not any(x.get('request') == actual and x.get('original_goals_hash') == goal_hash for x in requests)):
            return None
        route = status.get('continuous_route') or {}
        points = np.asarray(route.get('route_points_xyz'), dtype=float)
        next_index = route.get('next_coverage_index')
        if (route.get('schema') != 'original46_continuous_route/v1'
            or route.get('request_id') != rid or route.get('goals_definition_sha256') != goal_hash
            or route.get('navigation_ground_truth_used') is not False
            or type(next_index) is not int or next_index < 1
            or points.shape != (19, 3) or not np.isfinite(points).all()
            or not np.array_equal(points[1:], np.asarray([g.center for g in goals]))
            or route_sha(points.tolist()) != route.get('route_sha256')): return None
        arrivals = status.get('region_arrivals')
        if not isinstance(arrivals, list) or not arrivals or not isinstance(arrivals[0], dict): return None
        receipt = arrivals[0]
        stamp, previous = receipt.get('stamp_ns'), receipt.get('previous_observation_stamp_ns')
        raw = np.asarray(receipt.get('raw_position'), dtype=float)
        if (receipt.get('schema') != 'original_region_ordered_pass/v1'
            or receipt.get('completion_semantics') != 'ordered_pass_without_dwell'
            or receipt.get('reason') != 'passed' or receipt.get('request_id') != rid
            or receipt.get('goal_id') != 'exploration:0' or type(receipt.get('waypoint_index')) is not int
            or receipt.get('waypoint_index') != 0 or receipt.get('goals_definition_sha256') != goal_hash
            or receipt.get('route_sha256') != route.get('route_sha256')
            or receipt.get('navigation_ground_truth_used') is not False
            or receipt.get('protected') is not False or receipt.get('region_inside') is not True
            or receipt.get('control_region_inside') is not True
            or type(stamp) is not int or type(previous) is not int or not 0 < stamp-previous <= 200_000_000
            or receipt.get('max_observation_gap_ns') != 200_000_000
            or raw.shape != (3,) or not np.isfinite(raw).all()
            or not contains(goals[0], raw) or not contains_control(goals[0], raw)
            or receipt.get('arrival_definition') != goals[0].definition()['arrival']
            or receipt.get('control_arrival_definition') != goals[0].control_arrival_definition()
            or receipt.get('original_dwell_satisfied') is not False): return None
        vertex = float(np.linalg.norm(points[1, :2]-points[0, :2]))
        projection = receipt.get('route_projection') or {}
        projected_arc = projection.get('arc_m')
        if (type(projected_arc) not in (int, float) or not math.isfinite(projected_arc)
            or abs(projected_arc-vertex) > .5
            or receipt.get('route_vertex_arc_m') != vertex): return None
        archived = [(x, ref) for x, ref in rows_with_refs(Path(run)/'navigation_region_passes.jsonl') if x == receipt]
        if not archived: return None
        feedback = [(x, ref) for x, ref in rows_with_refs(Path(run)/'navigation_feedback_history.jsonl')
            if x.get('stamp_ns') == stamp and x.get('position_world_xyz') == receipt['raw_position']
            and x.get('navigation_ground_truth_used') is False]
        previous_feedback = [(x, ref) for x, ref in rows_with_refs(Path(run)/'navigation_feedback_history.jsonl')
            if x.get('stamp_ns') == previous and x.get('navigation_ground_truth_used') is False]
        if not feedback or not previous_feedback: return None
        association = actual_scan_association(run, rid, admitted_only=True)
        if not association: return None
        return dict(actual_request_id=rid, actual_controller_goals_definition_sha256=goal_hash,
            actual_first_goal='exploration:0', actual_first_goal_receipt=receipt,
            receipt_source_line=archived[0][1], actual_feedback_source_line=feedback[0][1],
            previous_actual_feedback_source_line=previous_feedback[0][1],
            exact_same_integer_stamp_and_position=True,
            next_coverage_index=next_index, SCAN_response_association=association,
            goal_semantics='original control-volume ordered pass; no ordinary dwell or parking claimed',
            navigation_PASS=False, first_goal_ordered_pass_verified=True,
            full46_or_parking_verified=False)
    except (ValueError, TypeError, KeyError, IndexError, OverflowError): return None

def observe_native_tail(tail, previous_ns, maximum_chunks=4):
    """Bounded catch-up of original complete lines; no success over backlog."""
    from mission46_runtime_evidence import native_ns
    latest = previous_ns
    fault = None
    chunks = 0
    for _ in range(maximum_chunks):
        records = tail.poll()
        chunks += 1
        for record in records:
            latest = native_ns(record['data'])
            if record['data'].get('fault') is not None and fault is None:
                fault = record
        if fault is not None: break
        if tail.path.exists() and tail.offset == tail.path.stat().st_size: break
        if not records: break  # Partial line stays pending, never treated as EOF.
    caught_up = tail.path.exists() and tail.offset == tail.path.stat().st_size
    return dict(native_ns=latest, fault_record=fault, caught_up=caught_up,
        chunks=chunks, complete_source_bytes_checked=tail.offset)

def initialization_rejection_diagnostic(run, result):
    """Only a proven short pose history may wait; never grants admission."""
    run = Path(run)
    metrics = result.get('predicates')
    if not isinstance(metrics, dict):
        return dict(state='REJECTED', recoverable_wait=False, marker='INIT_REJECT_INVALID_DIAGNOSTIC', navigation_PASS=False)
    checks = metrics.get('predicates')
    if not isinstance(checks, dict):
        return dict(state='REJECTED', recoverable_wait=False, marker='INIT_REJECT_INVALID_DIAGNOSTIC', navigation_PASS=False)
    failed = sorted(k for k, value in checks.items() if value is not True)
    diagnostic = dict(state='REJECTED', recoverable_wait=False, marker='INIT_REJECT_NONRECOVERABLE',
        failed_predicates=failed, original_exception=result.get('original_exception'),
        job_token=result.get('job_token'), actual_submit_clock_ns=result.get('actual_submit_clock_ns'),
        actual_values={k: metrics.get(k) for k in ('counts', 'imu', 'pose', 'native')},
        original_thresholds=metrics.get('original_thresholds'), snapshot=result.get('snapshot'),
        navigation_PASS=False, admission_by_diagnostic=False)
    expected = {'imu_count', 'pose_count', 'native_count', 'stationary_estimator_gate_enabled',
        'imu_span', 'imu_strictly_ordered', 'imu_max_gap', 'gyro_finite', 'acceleration_finite',
        'gyro_norm', 'acceleration_norm', 'acceleration_axis_std', 'pose_span', 'pose_max_gap',
        'pose_speed', 'pose_displacement', 'native_span', 'native_max_gap', 'native_original_safe',
        'sensor_health_ready', 'sensor_health_no_truth', 'registration_no_truth', 'registration_frozen'}
    try:
        if (result.get('schema') != 'R33_actual_initialization_result/v1'
            or set(checks) != expected or any(type(value) is not bool for value in checks.values())
            or failed != ['pose_span'] or metrics.get('diagnostic_only') is not True
            or metrics.get('thresholds_modified') is not False
            or result.get('original_exception') != 'ValueError:Actual SLAM body did not remain stably stopped'
            or result.get('snapshot_complete') is not True or result.get('snapshot_error') is not None
            or result.get('original_tuple_reconstructed_or_changed') is not False
            or result.get('admission_by_diagnostic') is not False):
            return diagnostic
        token = result['job_token']; suffix = token.removeprefix(run.name + ':init:')
        if token == suffix or not suffix.isdigit() or int(suffix) < 1: return diagnostic
        source = result['snapshot']; path = run/'R33_INITIALIZATION_JOBS.jsonl'
        offset, size = source['offset'], source['bytes']
        if (source['file'] != str(path) or type(offset) is not int or offset < 0
            or type(size) is not int or not 0 < size <= 1_572_864): return diagnostic
        with path.open('rb') as stream:
            stream.seek(offset); raw = stream.read(size)
        if len(raw) != size or hashlib.sha256(raw).hexdigest() != source['sha256']: return diagnostic
        job = json.loads(raw)
        if (job.get('schema') != 'R33_actual_initialization_submit_snapshot/v1'
            or job.get('job_token') != token or job.get('predicates') != metrics
            or job.get('actual_submit_clock_ns') != result.get('actual_submit_clock_ns')
            or job.get('original_tuple_reconstructed_or_changed') is not False
            or job.get('admission_by_diagnostic') is not False): return diagnostic
        scope_path = run/'navigation_scope.json'; config_path = run/'navigation_fastlivo.yaml'
        scope_raw = scope_path.read_bytes(); config_sha = hashlib.sha256(config_path.read_bytes()).hexdigest()
        if (hashlib.sha256(scope_raw).hexdigest() != job.get('scope_file_sha256')
            or config_sha != job.get('config_file_sha256')
            or json.loads(scope_raw)['references'].get(str(config_path)) != config_sha): return diagnostic
        pose = metrics['pose']; threshold = metrics['original_thresholds']
        if (threshold.get('pose_span_ns') != 2_900_000_000
            or threshold.get('pose_gap_limit_ns') != 200_000_000
            or threshold.get('pose_speed_limit') != .03
            or threshold.get('pose_displacement_limit') != .08): return diagnostic
        poses = job['actual_pose_tuple']; stamps = [row['stamp_ns'] for row in poses]
        if (len(poses) != metrics['counts']['pose'] or len(poses) < 10
            or len(job['actual_IMU_tuple']) < int(job['actual_config']['/**']['ros__parameters']['imu']['imu_int_frame'])
            or len(job['actual_native_tuple_exact_source_references']) < 150
            or len(job['actual_IMU_tuple']) != metrics['counts']['imu']
            or len(job['actual_native_tuple_exact_source_references']) != metrics['counts']['native']
            or any(type(stamp) is not int or stamp < 0 for stamp in stamps)
            or any(a >= b for a, b in zip(stamps, stamps[1:]))
            or type(result['actual_submit_clock_ns']) is not int
            or not result['actual_submit_clock_ns'] - 3_200_000_000 <= stamps[0] <= stamps[-1] <= result['actual_submit_clock_ns']
            or stamps[0] != pose['first_ns'] or stamps[-1] != pose['last_ns']
            or type(pose['span_ns']) is not int or pose['span_ns'] != stamps[-1] - stamps[0]
            or not 0 <= pose['span_ns'] < 2_900_000_000
            or pose['max_gap_ns'] != max(b-a for a, b in zip(stamps, stamps[1:]))
            or pose['max_gap_ns'] > 200_000_000): return diagnostic
        positions = np.asarray([row['position'] for row in poses], dtype=float)
        speeds = np.asarray([row['body_velocity'] for row in poses], dtype=float)
        if (positions.shape != (len(poses), 3) or speeds.shape != positions.shape
            or not np.isfinite(positions).all() or not np.isfinite(speeds).all()
            or float(max(np.linalg.norm(speeds, axis=1))) != pose['max_speed']
            or float(max(np.linalg.norm(positions-positions[0], axis=1))) != pose['max_displacement']
            or pose['max_speed'] > .03 or pose['max_displacement'] > .08): return diagnostic
        diagnostic.update(state='WAIT_INITIALIZATION', marker='INIT_WAIT_POSE_SPAN',
            recoverable_wait=True, actual_pose_span_ns=pose['span_ns'],
            minimum_pose_span_ns=2_900_000_000, shortfall_ns=2_900_000_000-pose['span_ns'])
    except (OSError, ValueError, TypeError, KeyError, IndexError, OverflowError):
        diagnostic['diagnostic_validation_failed'] = True
    return diagnostic


def record_initialization_wait(run, diagnostic):
    """Small append-only diagnostic record once per exact rejected job."""
    path = Path(run)/'R35_INITIALIZATION_WAIT.jsonl'
    raw = path.read_bytes() if path.exists() else b''
    if len(raw) > 65_536: raise ValueError('Initialization wait diagnostic cap reached')
    for line in raw.splitlines():
        old = json.loads(line)
        if old.get('job_token') == diagnostic['job_token']:
            if old != diagnostic: raise ValueError('Rejected initialization diagnostic changed')
            return
    row = (json.dumps(diagnostic, sort_keys=True, separators=(',', ':'))+'\n').encode()
    if len(raw)+len(row) > 65_536: raise ValueError('Initialization wait diagnostic cap reached')
    with path.open('ab') as stream: stream.write(row)


def first_goal_stop_decision(run, clock_ns, wall_elapsed, run_output_bytes):
    from startup_trace import read_live_json
    run = Path(run)
    # Caps and original failures precede a success observation.
    if run_output_bytes >= 10_000_000_000:
        return dict(stop=True, reason='new_output_cap', navigation_PASS=False)
    result = read_live_json(run/'R33_INITIALIZATION_RESULT.json')
    waiting = None
    if isinstance(result, dict) and result.get('original_verifier_passed') is False:
        waiting = initialization_rejection_diagnostic(run, result)
        if not waiting['recoverable_wait']:
            return dict(stop=True, reason='actual_initialization_nonrecoverable_rejection',
                initialization_diagnostic=waiting, job_result=result, navigation_PASS=False)
        try:
            record_initialization_wait(run, waiting)
        except (OSError, ValueError) as error:
            return dict(stop=True, reason='initialization_diagnostic_record_failed',
                diagnostic_error=type(error).__name__+':'+str(error), navigation_PASS=False)
    status = read_live_json(run/'navigation_status.json')
    actual = read_live_json(run/'navigation_request.json')
    mission = read_live_json(run/'mission46_status.json')
    if (isinstance(mission, dict) and mission.get('run_id') == run.name
        and mission.get('stage') in ('failed', 'aborted')):
        return dict(stop=True, reason='actual_original_mission_failed', actual_mission_status=mission, navigation_PASS=False)
    if isinstance(status, dict) and status.get('state') in ('failed', 'aborted'):
        return dict(stop=True, reason='actual_original_navigation_failed', actual_status=status, navigation_PASS=False)
    if clock_ns >= 1_500_000_000_000:
        return dict(stop=True, reason='native1500s_original_route_total_cutoff', navigation_PASS=False)
    if wall_elapsed >= 9090:
        return dict(stop=True, reason='wall9090s_original_route_total_cutoff', navigation_PASS=False)
    # Event sampling is evidence only. Completion or expiry must not stop the
    # autonomous route; rejected updates remain rejected by known_scene_runtime.
    # Original coordinator and worker retain all route and completion semantics.
    return dict(stop=False, navigation_PASS=False, initialization_wait=waiting)


def first_goal_runtime_decision(run, observed_native, wall_elapsed, output_bytes):
    """The actual runner's fault/backlog choice, independently testable on CPU."""
    if observed_native['fault_record'] is not None:
        return dict(stop=True, reason='actual_original_native_fault',
            actual_fault_record=observed_native['fault_record'], navigation_PASS=False)
    decision = first_goal_stop_decision(run, observed_native['native_ns'], wall_elapsed, output_bytes)
    if decision.get('first_goal_ordered_pass_verified') and not observed_native['caught_up']:
        return dict(stop=False, navigation_PASS=False, pending_native_backlog=True)
    return decision
