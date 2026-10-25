"""Default-OFF scalar source-age evidence; never a time or gate authority.

Each process retains one 512-row tail and first near-limit / source-rejection
windows (512 prior + 64 post). The original clocks/predicates are passed in;
optional observation-clock reads never feed business calculations. Exit-only IO.
"""
from collections import deque
import hashlib
import json
import os
import sys
from pathlib import Path
import threading
import time

KEY = 'R118_SOURCE_AGE_PROBE'
OPTION = os.environ.get(KEY, '0')
if OPTION not in ('0', '1'):
    raise ValueError(KEY + ' must be 0 or 1')
ENABLED = OPTION == '1'
CAPACITY, POST = 512, 64
COLUMNS = ('stage', 'mono_ns', 'thread_cpu_ns', 'tid', 'original_clock_ns',
           'observation_clock_ns', 'source_ns', 'control_ns', 'cascade_sequence',
           'producer_sequence', 'source_received_wall_ns', 'nonzero', 'status',
           'source_predicate_rejected', 'source_age_ns', 'control_age_ns',
           'pose_imu_gap_ns', 'pose_control_order_ok', 'control_upper_bound_ok',
           'observation_source_age_ns')
PROBE = None


def integer(value):
    return value if type(value) is int and -(1<<63) <= value < (1<<63) else None


def command_nonzero(values):
    # Refused/malformed reader envelopes are also observed. Never traverse an
    # arbitrary decoded container while collecting an optional scalar.
    return bool(any(values)) if type(values) in (tuple, list) and len(values) == 3 else None


class Recorder:
    def __init__(self, role, binding):
        self.role, self.binding = role, binding
        self.rows = deque(maxlen=CAPACITY)
        self.windows = {}
        self.lock = threading.Lock()
        self.counts = dict(recorded=0, evicted=0, errors=0, near_limit=0,
                           source_rejected=0, flush_errors=0, external_trigger_calls=0)
        self.external_trigger = None
        self.stages = {}

    def note(self, stage, clock, lineage=None, *, observation_clock=None,
             producer_sequence=None, nonzero=None, status=None, source_rejected=False):
        try:
            d = lineage if type(lineage) is dict else {}
            pose, control, gyro = (integer(d.get(k)) for k in
                ('source_pose_stamp_ns', 'control_stamp_ns', 'paired_imu_stamp_ns'))
            clock = integer(clock)
            observation_clock = integer(observation_clock)
            age = None if clock is None or pose is None else clock-pose
            control_age = None if clock is None or control is None else clock-control
            gap = None if pose is None or gyro is None else pose-gyro
            row = (stage, time.monotonic_ns(), time.thread_time_ns(), threading.get_native_id(),
                   clock, integer(observation_clock), pose, control, integer(d.get('cascade_sequence')),
                   integer(producer_sequence), integer(d.get('source_pose_received_wall_ns')),
                   nonzero if type(nonzero) is bool else None,
                   status[:120] if type(status) is str else None, bool(source_rejected), age,
                   control_age, gap, None if pose is None or control is None else pose <= control,
                   None if control is None or clock is None else control <= clock+50_000_000,
                   None if pose is None or type(observation_clock) is not int else observation_clock-pose)
            with self.lock:
                self.counts['recorded'] += 1
                self.counts['evicted'] += int(len(self.rows) == CAPACITY)
                self.stages[stage] = self.stages.get(stage, 0)+1
                self.rows.append(row)
                for window in self.windows.values():
                    if len(window['post']) < POST:
                        window['post'].append(row)
                near = any(type(value) is int and value >= 200_000_000 for value in (age, row[-1]))
                self.counts['near_limit'] += int(near)
                self.counts['source_rejected'] += int(source_rejected)
                for kind, active in (('first_near_limit', near), ('first_source_rejected', source_rejected)):
                    if active and kind not in self.windows:
                        self.windows[kind] = dict(trigger=row, prior=tuple(self.rows), post=[])
        except Exception:
            self.counts['errors'] += 1

    def receipt(self):
        with self.lock:
            return dict(schema='r118_source_age_observation/v1', role=self.role,
                binding=self.binding, selection=dict(key=KEY, value=OPTION, enabled=ENABLED,
                    process_fixed=True, default_off=True), columns=COLUMNS,
                counts=dict(self.counts), stages=dict(self.stages), tail=list(self.rows),
                windows={k:dict(v) for k,v in self.windows.items()}, capacity=CAPACITY, post=POST,
                near_limit_observation_threshold_ns=200_000_000,
                external_trigger=self.external_trigger,
                original_source_limit_ns=300_000_000, original_business_clocks_unchanged=True,
                no_future_acceptance_or_wait=True, bounded_scalar_only=True,
                scope_or_navigation_PASS=False)


def bind(role, run):
    """Startup-only actual source binding, before any evidence hook is used."""
    global PROBE
    if not ENABLED:
        return None
    if role not in ('controller', 'bridge', 'reader') or PROBE is not None:
        raise RuntimeError('R118 source-age probe has invalid/duplicate process owner')
    run = Path(run).resolve()
    path = Path(__file__).resolve()
    scope = run/'navigation_scope.json'
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if json.loads(scope.read_bytes())['references'].get(str(path)) != digest:
        raise RuntimeError('R118 actual helper source is not in frozen navigation scope')
    binding = dict(role=role, pid=os.getpid(), run=str(run), source=str(path), sha256=digest,
        scope_sha256=hashlib.sha256(scope.read_bytes()).hexdigest(),
        boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'), before_business=True)
    PROBE = Recorder(role, binding)
    return dict(binding)


def controller(node, stage, original_clock, row=None, *, feedback=False):
    try:
        if PROBE is None:
            return
        if feedback:
            f = getattr(node, 'feedback', None) or {}
            i = getattr(node, 'paired_imu', None) or {}
            lineage = dict(source_pose_stamp_ns=f.get('stamp_ns'),
                paired_imu_stamp_ns=i.get('stamp_ns'), control_stamp_ns=original_clock,
                source_pose_received_wall_ns=f.get('received_wall_ns'))
        else:
            r = row if type(row) is dict else {}
            f = r.get('feedback') or {}
            lineage = dict(source_pose_stamp_ns=r.get('source_pose_stamp_ns'),
                paired_imu_stamp_ns=r.get('paired_imu_stamp_ns'),control_stamp_ns=r.get('control_stamp_ns'),
                cascade_sequence=r.get('sequence'),source_pose_received_wall_ns=f.get('received_wall_ns'))
        observed = node.get_clock().now().nanoseconds
        PROBE.note(stage, original_clock, lineage, observation_clock=observed,
            nonzero=command_nonzero(getattr(node, 'command', ())), status=getattr(node, 'state', None))
        source = integer(lineage.get('source_pose_stamp_ns'))
        upstream = sys.modules.get('r118_feedback_evidence')
        if (PROBE.external_trigger is None and source is not None and type(observed) is int
                and observed-source >= 200_000_000 and upstream is not None):
            # Same-process optional evidence hook. This is a near-limit signal,
            # not a native-reader rejection or proof its window covers one.
            upstream.trigger(source, reason=1)
            PROBE.counts['external_trigger_calls'] += 1
            PROBE.external_trigger = dict(source_ns=source, reason=1,
                meaning='controller_near_limit_200ms', returned=True,
                native_reader_rejection_claimed=False)
    except Exception:
        if PROBE is not None:PROBE.counts['errors'] += 1


def bridge_received(clock, source, command, error):
    try:
        if PROBE is not None:
            PROBE.note('bridge.command_received', clock, source, nonzero=command_nonzero(command),
                status=error or 'received')
    except Exception:
        if PROBE is not None:PROBE.counts['errors'] += 1


def envelope(stage, value):
    try:
        if PROBE is None:return
        stamp = value.get('stamp') or {}
        PROBE.note(stage, stamp.get('ros_sim_time_ns'), value.get('controller_source'),
            producer_sequence=value.get('sequence'), nonzero=command_nonzero(value.get('command', ())),
            status=value.get('state'), source_rejected=False)
    except Exception:
        if PROBE is not None:PROBE.counts['errors'] += 1


def reader_result(record):
    try:
        if PROBE is None:return
        data = record.get('decoded_envelope') or {}
        reason = record.get('reason', '')
        PROBE.note('reader.result', record.get('read_clock_ns'), data.get('controller_source'),
            producer_sequence=data.get('sequence'), nonzero=command_nonzero(data.get('command', ())),
            status=reason, source_rejected=reason == 'ValueError: Source age/causality rejected')
    except Exception:
        if PROBE is not None:PROBE.counts['errors'] += 1


def finish(run):
    """After owner/pool joins. Diagnostic failure never changes shutdown state."""
    if PROBE is None:return
    try:
        path = Path(run)/('r118_source_age_'+PROBE.role+'.json')
        with path.open('x') as stream:
            json.dump(PROBE.receipt(), stream, allow_nan=False)
            stream.write('\n')
    except Exception:
        PROBE.counts['flush_errors'] += 1
