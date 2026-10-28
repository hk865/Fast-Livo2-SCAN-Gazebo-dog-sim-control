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
    value=profile.get('r85_formal_regression') if isinstance(profile,dict) else None
    if value is None or isinstance(value,dict) and value.get('enabled') is False:return False
    if not isinstance(value,dict) or value.get('enabled') is not True:
        raise ValueError('Formal regression requires a boolean selection')
    auto=profile.get('r85_auto_reference_contract')or{}
    dynamic=profile.get('r85_dynamic_event_contract')or{}
    if (profile.get('diagnostic_observe_only',{}).get('enabled') is not False
        or value.get('business_rejections_must_stop') is not True
        or profile.get('actor_kind')!='student' or profile.get('controller_kind')!='student'
        or profile.get('navigation_ground_truth_used') is not False
        or profile.get('r85_task',{}).get('kind')!='f1_to_f3'
        or profile.get('r85_completed_history_async_qualification',{}).get('enabled') is not False
        or auto.get('enabled') is not True
        or any(auto.get(key) is not True for key in ('require_child_odometry_consumption_ack',
            'require_reference_protocol_ack','terminal_heading_from_pct_approach'))
        or profile.get('cascade',{}).get('r85_student_lower_loop_ack',{}).get('enabled') is not True
        or dynamic.get('completed_same_geometry_preservation') is not False):
        raise ValueError('Formal task must retain real source ACKs and stop on business rejection')
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
