"""R31E-style diagnostic_check_invalid ledger; no control or truth source."""
from pathlib import Path
import hashlib
import json
import os
import time

_last = {}
MAX_FILE_BYTES = 8 * 1024 * 1024
MAX_LINE_BYTES = 16 * 1024


def enabled(profile):
    value = (profile or {}).get('diagnostic_observe_only')
    return (isinstance(value, dict) and type(value.get('enabled')) is bool
            and value['enabled'] is True)



def strict_formal_enabled(profile):
    """Select strict F1->F3 configuration only; grants no motion or PASS.

    Diagnostic execution permissions still depend solely on enabled(profile).
    Profiles without this option keep every legacy selector behavior.
    """
    if not isinstance(profile, dict) or 'r85_formal_regression' not in profile:
        return False
    value=profile['r85_formal_regression']
    if type(value) is dict and set(value)=={'enabled'} and value['enabled'] is False:
        return False
    expected=dict(schema='r85_strict_formal_f1_f3/v1',enabled=True,kind='f1_to_f3',
        original_single_scope_dynamic_required=True,business_rejections_must_stop=True)
    segmented=type(value) is dict and value.get('schema')=='r85_strict_formal_f1_f3/v3'
    coordinate=type(value) is dict and value.get('schema')=='r85_strict_formal_f1_f3/v4'
    event_window=type(value) is dict and value.get('schema')in ('r85_strict_formal_f1_f3/v2','r85_strict_formal_f1_f3/v3')
    if event_window:
        # The event owns its real completed window; current control still owns
        # the current frame/route. This selector never qualifies either one.
        expected=dict(schema='r85_strict_formal_f1_f3/v2',enabled=True,kind='f1_to_f3',
            dynamic_event_lifetime='single_completed_event_window/v1',business_rejections_must_stop=True)
    if segmented:expected=dict(schema='r85_strict_formal_f1_f3/v3',enabled=True,kind='f1_to_f3',
        dynamic_event_lifetime='single_world_event_pre_stop_nav_segment/v2',business_rejections_must_stop=True)
    if coordinate:expected=dict(schema='r85_strict_formal_f1_f3/v4',enabled=True,kind='f1_to_f3',
        dynamic_event_lifetime='single_world_event_coordinate_handoffs/v3',business_rejections_must_stop=True)
    if (type(value) is not dict or set(value)!=set(expected)
            or any(type(value[k]) is not type(v) or value[k]!=v for k,v in expected.items())):
        raise ValueError('Exact typed strict formal F1-to-F3 scope required')
    diagnostic=profile.get('diagnostic_observe_only')
    dynamic=profile.get('r85_dynamic_event_contract')
    asynchronous=profile.get('r85_completed_history_async_qualification')
    session=profile.get('r85_session',{})
    task=profile.get('r85_task',{})
    auto=profile.get('r85_auto_reference_contract',{})
    cascade=profile.get('cascade')
    lower=cascade.get('r85_student_lower_loop_ack') if type(cascade) is dict else None
    continuous=profile.get('continuous_route_contract')
    expected_dynamic=(dict(schema='single_completed_event_window/v1',completed_event_window=True,
        whole_task_single_scope=False,completed_same_geometry_preservation=False)
        if event_window else dict(completed_same_geometry_preservation=False))
    if segmented:expected_dynamic=dict(schema='single_world_event_pre_stop_nav_segment/v2',completed_event_window=True,
        whole_task_single_scope=False,completed_same_geometry_preservation=False,maximum_pre_stop_navigation_switches=1)
    if coordinate:expected_dynamic=dict(schema='single_world_event_coordinate_handoffs/v3',completed_event_window=True,
        whole_task_single_scope=False,completed_same_geometry_preservation=False,maximum_coordinate_handoffs=2)
    dynamic_matches=(type(dynamic) is dict and set(dynamic)==set(expected_dynamic)
        and all(type(dynamic[k]) is type(v) and dynamic[k]==v for k,v in expected_dynamic.items()))
    if (type(diagnostic) is not dict or diagnostic.get('enabled') is not False
            or diagnostic.get('formal_acceptance_pass_allowed') is not False
            or diagnostic.get('schema')!='r85_existing_diagnostic_business_observation/v1'
            or not dynamic_matches
            or type(asynchronous) is not dict or set(asynchronous)!={'enabled'}
            or asynchronous['enabled'] is not False
            or type(session) is not dict or session.get('enabled') is not True
            or session.get('schema')!='r85_pct_scan_student_session/v1'
            or session.get('simulation_only') is not True
            or profile.get('actor_kind')!='student' or profile.get('controller_kind')!='student'
            or profile.get('navigation_ground_truth_used') is not False
            or profile.get('mission46_required') is not False
            or type(continuous) is not dict or continuous.get('enabled') is not False
            or type(task) is not dict or task.get('kind')!='f1_to_f3'
            or type(auto) is not dict or auto.get('enabled') is not True
            or auto.get('schema')!='r85_persistent_auto_reference/v1'
            or auto.get('require_child_odometry_consumption_ack') is not True
            or auto.get('require_reference_protocol_ack') is not True
            or auto.get('terminal_heading_from_pct_approach') is not True
            or type(lower) is not dict or lower.get('enabled') is not True):
        raise ValueError('Strict formal scope must disable diagnostic execution and preserve R85 source ACKs')
    return True

def record(run, profile, gate, reason, evidence=None):
    if not enabled(profile):
        return False
    if not isinstance(gate, str) or not gate or not isinstance(reason, str) or not reason:
        raise ValueError('diagnostic gate/reason identity required')
    now = time.monotonic()
    key = (str(run), gate, reason)
    if key in _last and now - _last[key] < 1.:
        return True
    raw_evidence = json.dumps(evidence, sort_keys=True, allow_nan=False)
    if len(raw_evidence.encode()) > 8192:
        evidence = dict(bounded_summary=True, original_bytes=len(raw_evidence.encode()),
                        sha256=hashlib.sha256(raw_evidence.encode()).hexdigest())
    row = dict(schema='r85_diagnostic_check_invalid/v1', event='diagnostic_check_invalid',
               run_id=Path(run).name, pid=os.getpid(), monotonic_wall=now,
               gate=gate, reason=reason, original_check_passed=False,
               diagnostic_continued=True, formal_acceptance_pass=False,
               evidence=evidence)
    line = (json.dumps(row, allow_nan=False, separators=(',', ':')) + '\n').encode()
    if len(line) > MAX_LINE_BYTES:
        raise RuntimeError('bounded diagnostic line exceeded')
    path = Path(run) / 'R85_DIAGNOSTIC_BUSINESS_OBSERVATIONS.jsonl'
    fd = os.open(path, os.O_CREAT | os.O_APPEND | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
    try:
        if os.fstat(fd).st_size + len(line) > MAX_FILE_BYTES:
            raise RuntimeError('bounded diagnostic ledger exceeded')
        if os.write(fd, line) != len(line):
            raise RuntimeError('incomplete diagnostic evidence write')
    finally:
        os.close(fd)
    _last[key] = now
    return True
