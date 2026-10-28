"""Default-OFF scalar SCAN observer; no message parsing, hashing or replay.

3072 packed Python rows: immutable first 1024 plus last 2048.
Only Admission.close writes a file. Exceptions never replace business results.
"""
from array import array
import json
import os
from pathlib import Path
import time
import threading
from contextvars import ContextVar
from functools import wraps

TAKE_CONTEXT = ContextVar("r107_scan_take", default=None)
PREFIX = 1024
HIST_LIMITS_NS = (10000, 50000, 100000, 500000, 1000000, 5000000, 10000000, 20000000, 50000000, 100000000, 200000000, 500000000, 1000000000, 2000000000)

ENVIRONMENT_KEY = 'R107_SCAN_TRANSPORT_PROBE'
CAPACITY = 3072
COLUMNS = ('seq', 'stage', 'mono_ns', 'tid', 'event_index', 'kind', 'size',
           'queue_count', 'queue_bytes', 'job_token', 'worker_busy', 'outcome',
           'identity_status', 'epoch', 'generation', 'revision', 'planner_pid',
           'attempt_id', 'traj_id', 'start_sec', 'start_nsec', 'order',
           'pos_count', 'knot_count', 'yaw_count', 'original_receive_mono_ns', 'take_seq', 'take_before_mono', 'take_after_mono',
           'callback_enter_mono', 'callback_enter_thread_cpu', 'mark_thread_cpu')
WIDTH = len(COLUMNS)
STAGES = ('unused', 'receive_enter', 'size_reject', 'size_validated', 'append',
          'popleft', 'consume_enter', 'consume_return_or_raise', 'overflow',
          'invalidated_event', 'submitted', 'result_to_original_commit',
          'owner_invalidated', 'fault', 'discarded', 'commit_return_or_raise', 'take_before', 'take_after',
          'callback_enter', 'callback_return_or_raise', 'ready_check_enter',
          'ready_check_exit', 'ready_poll_enter', 'ready_poll_exit', 'batch_enter', 'batch_exit')
KINDS = {'attempt': 1, 'spline': 2, 'metadata': 3, 'definition': 4}
TOPICS = {'attempt': 'reference_attempt', 'spline': 'bspline',
          'metadata': 'trajectory_metadata'}
OUTCOMES = {'pending_or_no_match': 1, 'rejected': 2, 'matched_candidate_only': 3}
FILE_LIMIT = 3 * 1024 * 1024


def _integer(value):
    return value if type(value) is int and -(1 << 63) <= value < (1 << 63) else -1


def _process_identity():
    raw = Path('/proc/self/stat').read_text()
    return dict(pid=os.getpid(), start_ticks=int(raw[raw.rfind(')') + 2:].split()[19]),
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def _serial(method):
    @wraps(method)
    def call(self,*args,**kwargs):
        with self.lock:
            return method(self,*args,**kwargs)
    return call


class Recorder:
    def __init__(self, node):
        self.lock = threading.RLock()
        self.node = node
        self.identity = _process_identity()
        self.run = Path(node.run)
        self.buffer = array('q', [0]) * (CAPACITY * WIDTH)
        self.attempted = self.recorded = self.errors = self.unresolved = 0
        self.cost_wall_ns = self.cost_cpu_ns = self.cost_wall_max_ns = 0
        self.stage_counts = [0] * len(STAGES)
        self.closed = False
        self.distributions = {key: {'n': 0, 'sum_ns': 0, 'max_ns': 0,
                              'bins': [0] * (len(HIST_LIMITS_NS) + 1)}
            for key in ('receive_to_business', 'business_callback', 'commit_callback',
                        'take_call', 'take_to_callback', 'callback_to_receive')}
        self.consume_start = self.commit_start = None
        self.boundary_mismatch = 0
        # Payload-free identity sidecar: 64 slots exceed 32 queued + one current.
        # Every lookup verifies event_index; eviction never borrows ambient take.
        self.anomaly = array('q', [0]) * (1024 * WIDTH)
        self.anomaly_rows = self.anomaly_post = 0
        self.anomaly_trigger = None
        self.event_take = array('q', [0]) * (64 * 6)
        self.take_identity_unavailable = 0

    def _offset(self, sequence):
        slot = sequence if sequence < PREFIX else PREFIX + (sequence - PREFIX) % (CAPACITY - PREFIX)
        return slot * WIDTH

    @_serial
    def duration(self, key, ns):
        if ns < 0:
            self.boundary_mismatch += 1
            return
        item = self.distributions[key]
        item['n'] += 1
        item['sum_ns'] += ns
        item['max_ns'] = max(item['max_ns'], ns)
        if ns >= 50_000_000:
            self.freeze(key, ns)
        bucket = 0
        while bucket < len(HIST_LIMITS_NS) and ns > HIST_LIMITS_NS[bucket]:
            bucket += 1
        item['bins'][bucket] += 1

    @_serial
    def freeze(self, reason, elapsed_ns):
        if self.anomaly_trigger is not None:
            return
        self.anomaly_trigger = dict(reason=reason, elapsed_ns=elapsed_ns,
            mono_ns=time.monotonic_ns(), recorded_before=self.recorded)
        for sequence in range(max(0, self.recorded - 256), self.recorded):
            src = self._offset(sequence)
            dst = self.anomaly_rows * WIDTH
            self.anomaly[dst:dst + WIDTH] = self.buffer[src:src + WIDTH]
            self.anomaly_rows += 1

    @_serial
    def prior_context(self, kind):
        """Existing bounded observer context only; never creates an observer."""
        try:
            observer = getattr(self.node, '_r85_protocol_intake_observer', None)
            return None if observer is None else observer.last.get(TOPICS.get(kind))
        except Exception:
            self.errors += 1
            return None

    @_serial
    def completed_context(self, event, prior):
        try:
            context = self.prior_context(event['kind'])
            # begin() creates a new context. A stale observer entry is not ours.
            if context is prior or not isinstance(context, dict):
                return None
            receipt = context.get('original_entry_receipt', {})
            if receipt.get('entry_monotonic_wall_ns', -1) < event['received_wall_ns']:
                return None
            source = context.get('source_identity')
            if not isinstance(source, dict) or source.get('run_id') != self.run.name:
                return None
            return context
        except Exception:
            self.errors += 1
            return None

    @_serial
    def mark(self, admission, stage, event=None, *, kind=None, msg=None,
             size=-1, outcome=0, context=None, job_token=None,
             event_index=-1, received_mono=-1):
        """Only bounded scalar reads. String payloads are never inspected."""
        if self.closed:
            return
        begin = time.monotonic_ns()
        cpu = time.thread_time_ns()
        self.attempted += 1
        try:
            if event is not None:
                kind, msg, size = event['kind'], event['msg'], event['size']
            identity_status = 0  # unresolved; never inferred from callback success
            epoch = generation = revision = planner = attempt = trajectory = -1
            sec = nano = order = positions = knots = yaws = -1
            if kind == 'spline' and msg is not None:
                identity_status = 1  # partial spline identity, not a payload hash
                trajectory = _integer(getattr(msg, 'traj_id', None))
                stamp = getattr(msg, 'start_time', None)
                sec = _integer(getattr(stamp, 'sec', None))
                nano = _integer(getattr(stamp, 'nanosec', None))
                order = _integer(getattr(msg, 'order', None))
                positions, knots, yaws = len(msg.pos_pts), len(msg.knots), len(msg.yaw_pts)
            elif context is not None:
                source = context['source_identity']
                epoch = _integer(source.get('reference_epoch'))
                generation = _integer(source.get('localization_generation'))
                revision = _integer(source.get('committed_reference_revision'))
                planner = _integer(source.get('planner_process_id'))
                attempt = _integer(source.get('attempt', {}).get('id'))
                curve = source.get('trajectory', {})
                trajectory = _integer(source.get('trajectory_id',curve.get('traj_id')))
                stamp = source.get('trajectory_start_time',curve.get('start_time'))
                if isinstance(stamp, (tuple, list)) and len(stamp) == 2:
                    sec, nano = _integer(stamp[0]), _integer(stamp[1])
                identity_status = 2 if min(epoch, generation, revision, planner, attempt) >= 0 else 0
                if outcome == 1:
                    outcome = 10 + OUTCOMES.get(context.get('association_outcome'), 0)
            if identity_status == 0:
                self.unresolved += 1
            current = admission.current
            token = (-1 if current is None else _integer(current.get('token'))
                     ) if job_token is None else _integer(job_token)
            take = TAKE_CONTEXT.get()
            identity = _integer(event_index if event is None else event['index'])
            if stage == 1 and identity > 0 and take is not None:
                take_offset = (identity % 64) * 6
                self.event_take[take_offset] = identity
                for column in range(5):
                    self.event_take[take_offset + 1 + column] = _integer(take[column])
            take_fields = (-1,) * 5
            if stage in (16, 17, 18, 19):
                if take is not None:
                    take_fields = tuple(take[:5])
            elif identity > 0:
                take_offset = (identity % 64) * 6
                if self.event_take[take_offset] == identity:
                    take_fields = tuple(self.event_take[take_offset + 1:take_offset + 6])
                else:
                    self.take_identity_unavailable += 1
            if stage == 6 and event is not None:
                self.duration('receive_to_business', begin - event['received_wall_ns'])
                self.consume_start = (event['index'], begin)
            elif stage == 7 and event is not None:
                if self.consume_start is not None and self.consume_start[0] == event['index']:
                    self.duration('business_callback', begin - self.consume_start[1])
                else:
                    self.boundary_mismatch += 1
                self.consume_start = None
            elif stage == 11:
                self.commit_start = (token, begin)
            elif stage == 15:
                if self.commit_start is not None and self.commit_start[0] == token:
                    self.duration('commit_callback', begin - self.commit_start[1])
                else:
                    self.boundary_mismatch += 1
                self.commit_start = None
            if stage == 1 and take is not None:
                self.duration('callback_to_receive', begin - take[3])
            queue_count, queue_bytes = admission.queue_snapshot()
            values = (self.recorded + 1, stage, begin, threading.get_native_id(),
                      event_index if event is None else event['index'], KINDS.get(kind, 0), size,
                      queue_count, queue_bytes, token,
                      int(bool(admission.worker.busy)), outcome, identity_status,
                      epoch, generation, revision, planner, attempt, trajectory, sec,
                      nano, order, positions, knots, yaws,
                      received_mono if event is None else event['received_wall_ns'],
                      *take_fields, cpu)
            offset = self._offset(self.recorded)
            for index, value in enumerate(values):
                self.buffer[offset + index] = _integer(value)
            if stage in (2, 8, 13):
                self.freeze('exception_or_bound_stage_' + str(stage), 0)
            if self.anomaly_trigger is not None and self.anomaly_post < 768:
                dst = self.anomaly_rows * WIDTH
                self.anomaly[dst:dst+WIDTH] = self.buffer[offset:offset+WIDTH]
                self.anomaly_rows += 1
                self.anomaly_post += 1
            self.recorded += 1
            self.stage_counts[stage] += 1
        except Exception:
            self.errors += 1
        finally:
            elapsed = time.monotonic_ns() - begin
            self.cost_wall_ns += elapsed
            self.cost_wall_max_ns = max(self.cost_wall_max_ns, elapsed)
            self.cost_cpu_ns += time.thread_time_ns() - cpu

    @_serial
    def close(self):
        if self.closed:
            return dict(enabled=True, already_closed=True)
        self.closed = True
        begin = time.monotonic_ns()
        result = dict(schema='r107_scan_transport_python/v1', enabled=True,
                      identity=self.identity, run_id=self.run.name,
                      columns=COLUMNS, stages=STAGES, kinds=KINDS,
                      retention='first1024_tail2048_first_anomaly_pre256_post768_not_all_events',
                      histogram_limits_ns=HIST_LIMITS_NS, distributions=self.distributions,
                      boundary_mismatch=self.boundary_mismatch,
                      event_take_sidecar_bytes=len(self.event_take) * self.event_take.itemsize,
                      take_identity_unavailable=self.take_identity_unavailable,
                      take_fields_are_original_event_take_not_ambient=True,
                      outcomes={'consume_return_or_raise': {
                          '-1': 'original_callback_raised', '1': 'callback_returned_identity_unresolved',
                          '10': 'callback_returned_association_outcome_unknown',
                          '11': 'pending_or_no_match', '12': 'association_rejected',
                          '13': 'matched_candidate_only'},
                          'overflow': {'1': 'count_bound', '2': 'bytes_bound', '3': 'both_bounds'},
                          'commit_return_or_raise': {'-1': 'original_commit_callback_raised',
                                                     '1': 'original_commit_callback_returned'}},
                      callback_return_is_business_acceptance=False,
                      identity_status={'0': 'unresolved', '1': 'partial_spline',
                                       '2': 'original_parser_semantics'},
                      anomaly_trigger=self.anomaly_trigger,
                      anomaly_rows=[list(self.anomaly[i*WIDTH:(i+1)*WIDTH]) for i in range(self.anomaly_rows)],
                      anomaly_post_rows=self.anomaly_post,
                      anomaly_buffer_bytes=len(self.anomaly)*self.anomaly.itemsize,
                      first_slow_window_not_all_slow_events=True,
                      capacity=CAPACITY, recorded=self.recorded,
                      attempted=self.attempted, evicted=max(0, self.recorded - CAPACITY),
                      errors=self.errors, unresolved_rows=self.unresolved,
                      stage_counts=self.stage_counts, buffer_bytes=len(self.buffer) * self.buffer.itemsize,
                      cost_wall_ns=self.cost_wall_ns, cost_cpu_ns=self.cost_cpu_ns,
                      cost_wall_max_ns=self.cost_wall_max_ns,
                      no_payload_parse_or_hash=True, monotonic_clock='CLOCK_MONOTONIC',
                      unique_cross_process_attribution_established=False)
        try:
            # JSON streaming only during close; no whole-ring dict/list clone.
            path = self.run / ('r107_scan_transport_python_' + str(self.identity['pid']) + '.json')
            flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW
            with os.fdopen(os.open(path, flags, 0o600), 'w') as stream:
                text = json.dumps(result, separators=(',', ':'))
                stream.write(text[:-1] + ',"rows":[')
                sequences = list(range(min(PREFIX, self.recorded))) + list(range(
                    max(PREFIX, self.recorded - (CAPACITY - PREFIX)), self.recorded))
                for index, sequence in enumerate(sequences):
                    offset = self._offset(sequence)
                    row = json.dumps(list(self.buffer[offset:offset + WIDTH]), separators=(',', ':'))
                    if stream.tell() + len(row) + 3 > FILE_LIMIT:
                        raise ValueError('r107_exit_file_limit')
                    stream.write((',' if index else '') + row)
                stream.write(']}\n')
            result.update(path=str(path), file_bytes=path.stat().st_size, file_complete=True)
        except Exception as error:
            self.errors += 1
            result.update(file_complete=False, write_error=type(error).__name__)
        result['flush_wall_ns'] = time.monotonic_ns() - begin
        return result


class Unavailable:
    def mark(self, *args, **kwargs):
        pass

    def prior_context(self, kind):
        return None

    def completed_context(self, event, prior):
        return None

    def close(self):
        return dict(schema='r107_scan_transport_python/v1', enabled=True,
                    available=False, initialization_failed=True)


def create(node):
    if os.environ.get(ENVIRONMENT_KEY, '0') != '1':
        return None
    try:
        return Recorder(node)
    except Exception:
        return Unavailable()
