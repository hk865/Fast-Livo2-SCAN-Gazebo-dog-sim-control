"""Typed recorder termination; task failure is never relabeled diagnostic-only.

This protocol closes evidence. It grants no command or safety authority.
"""
import json
import math
import time

REASON_KINDS={
    'R77_terminal_native_backlog_unverified':'diagnostic',
    'native1500s_original_route_total_cutoff':'diagnostic',
    'wall9090s_original_route_total_cutoff':'diagnostic',
    'R77_native1500s_full_cap':'diagnostic',
    'R77_wall4350s_full_cap':'diagnostic',
    'R77_predictive_round_stop':'diagnostic',
    'R77_night_cleanup_deadline':'diagnostic',
    'R77_original_run_terminated':'diagnostic',
    'actual_original_mission_failed':'failed_task',
    'actual_original_navigation_failed':'failed_task',
    'actual_original_native_fault':'hard_fault',
    'actual_initialization_nonrecoverable_rejection':'hard_fault',
    'R77_exceptional_stop':'hard_fault',
    'new_output_cap':'hard_fault',
    'initialization_diagnostic_record_failed':'hard_fault',
}
OWNER_STOP_REASONS=frozenset(REASON_KINDS)

def validate_owner_stop(value):
    if not isinstance(value,dict):
        raise RuntimeError('Owner stop must be a typed record')
    reason=value.get('reason');kind=REASON_KINDS.get(reason)
    if not (value.get('schema')=='typed_capture_owner_stop/v2'
            and value.get('stop')is True and kind is not None
            and value.get('stop_kind')==kind
            and value.get('diagnostic_only')is (kind=='diagnostic')
            and value.get('navigation_PASS')is False
            and type(value.get('actual_native_ns'))is int and value['actual_native_ns']>=0
            and type(value.get('actual_monotonic_wall'))in(int,float)
            and math.isfinite(value['actual_monotonic_wall']) and value['actual_monotonic_wall']>=0):
        raise RuntimeError('Owner stop category or source clock is invalid')
    if kind=='failed_task':
        evidence=value.get('original_failure_evidence')
        if not (isinstance(evidence,dict) and evidence.get('state')in('failed','aborted')
                and isinstance(evidence.get('source_file'),str)
                and isinstance(evidence.get('sha256'),str) and len(evidence['sha256'])==64):
            raise RuntimeError('Task-failure stop requires original failure evidence')
    return dict(value)

def make_owner_stop(reason,ns,now,**evidence):
    kind=REASON_KINDS[reason]
    value=dict(schema='typed_capture_owner_stop/v2',stop=True,reason=reason,
               stop_kind=kind,diagnostic_only=kind=='diagnostic',navigation_PASS=False,
               actual_native_ns=ns,actual_monotonic_wall=now,**evidence)
    return validate_owner_stop(value)

def read_owner_stop(path):
    with path.open('rb')as stream:raw=stream.read(16385)
    if len(raw)>16384:raise RuntimeError('Owner stop marker exceeds bounded size')
    return validate_owner_stop(json.loads(raw))

def publish_owner_stop(run,value):
    receipt=validate_owner_stop(value)
    raw=json.dumps(receipt,allow_nan=False)+'\n'
    if len(raw.encode())>16384:raise RuntimeError('Owner stop marker exceeds bounded size')
    marker=run/'CAPTURE_OWNER_STOP.json';pending=run/'CAPTURE_OWNER_STOP.pending'
    if marker.exists():raise RuntimeError('Recorder stop already published')
    with pending.open('x')as stream:stream.write(raw)
    pending.rename(marker)

def spin_until_owner_stop(node,run,api):
    marker=run/'CAPTURE_OWNER_STOP.json'
    if marker.exists():raise RuntimeError('Recorder stop exists at startup')
    next_check=0.
    while api.ok():
        now=time.monotonic()
        if now>=next_check:
            next_check=now+.05
            if marker.exists():return read_owner_stop(marker)
        api.spin_once(node,timeout_sec=.05)
    raise RuntimeError('ROS ended before typed owner stop')
