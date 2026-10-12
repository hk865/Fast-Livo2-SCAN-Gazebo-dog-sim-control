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

def first_goal_stop_decision(run, clock_ns, wall_elapsed, run_output_bytes):
    from startup_trace import read_live_json
    run = Path(run)
    # Caps and original failures precede a success observation.
    if run_output_bytes >= 2_000_000_000:
        return dict(stop=True, reason='new_output_cap', navigation_PASS=False)
    result = read_live_json(run/'R33_INITIALIZATION_RESULT.json')
    if isinstance(result, dict) and result.get('original_verifier_passed') is False:
        return dict(stop=True, reason='actual_initialization_first_rejection', job_result=result, navigation_PASS=False)
    status = read_live_json(run/'navigation_status.json')
    actual = read_live_json(run/'navigation_request.json')
    if isinstance(status, dict) and status.get('state') in ('failed', 'aborted'):
        return dict(stop=True, reason='actual_original_navigation_failed', actual_status=status, navigation_PASS=False)
    if clock_ns >= 60_000_000_000:
        return dict(stop=True, reason='native60s_first_goal_cutoff', navigation_PASS=False)
    if wall_elapsed >= 240:
        return dict(stop=True, reason='wall240s_first_goal_cutoff', navigation_PASS=False)
    required = ['mission46_initialization_receipt.json', 'mission46_initialization_evidence.json', 'navigation_anchor.json']
    if (isinstance(result, dict) and result.get('original_verifier_passed') is True
        and result.get('snapshot_complete') is True and all((run/n).is_file() for n in required)
        and isinstance(status, dict) and isinstance(actual, dict)):
        initialization = read_live_json(run/'mission46_initialization_receipt.json')
        anchor = read_live_json(run/'navigation_anchor.json')
        if (isinstance(initialization, dict) and initialization.get('passed') is True
            and initialization.get('navigation_ground_truth_used') is False
            and initialization.get('binding_verified') is True
            and initialization.get('run_id') == run.name
            and result.get('original_receipt') == initialization
            and str(result.get('job_token', '')).startswith(run.name + ':init:')
            and isinstance(anchor, dict) and anchor.get('ground_truth_navigation_used') is False):
            requests = [x for x, _ in rows_with_refs(run/'mission46_requests.jsonl')]
            proof = verified_first_goal(run, status, actual, requests)
            if proof is not None and proof['actual_first_goal_receipt']['stamp_ns'] <= clock_ns:
                return dict(stop=True, reason='real_first_goal_original_ordered_pass', **proof)
    return dict(stop=False, navigation_PASS=False)


def first_goal_runtime_decision(run, observed_native, wall_elapsed, output_bytes):
    """The actual runner's fault/backlog choice, independently testable on CPU."""
    if observed_native['fault_record'] is not None:
        return dict(stop=True, reason='actual_original_native_fault',
            actual_fault_record=observed_native['fault_record'], navigation_PASS=False)
    decision = first_goal_stop_decision(run, observed_native['native_ns'], wall_elapsed, output_bytes)
    if decision.get('first_goal_ordered_pass_verified') and not observed_native['caught_up']:
        return dict(stop=False, navigation_PASS=False, pending_native_backlog=True)
    return decision
