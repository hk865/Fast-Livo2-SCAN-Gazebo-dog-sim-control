"""Default-OFF, version-bound observation of the existing TimeSource subscription.

No new subscription, clock producer, thread, hot-path I/O or message retention.
Only original take/execute/callback are called, once each. All recorded rows are
immutable scalar tuples. Clock reads are observations, never freshness inputs.
"""
from collections import deque
from contextvars import ContextVar
from functools import wraps
import hashlib
import inspect
import json
import os
from pathlib import Path
import threading
import time
from types import MethodType
import weakref

KEY = 'R99_CLOCK_PROBE'
OPTION = os.environ.get(KEY, '0')
if OPTION not in ('0', '1'):
    raise ValueError(KEY + ' must be 0 or 1')
READ_KEY = 'R102_CLOCK_READ_INTERVAL_PROBE'
READ_OPTION = os.environ.get(READ_KEY, '0')
if READ_OPTION not in ('0', '1'):
    raise ValueError(READ_KEY + ' must be 0 or 1')
# Three distinct tails plus two post windows fit the measured 4 MiB budget.
CAPACITY, NOTE_CAPACITY, POST, NOTE_POST = 1024, 256, 128, 64
MAX_OUTPUT_BYTES = 20 * 1024 * 1024
EXPECTED_SHA256 = {
    'executors.py': 'f0d655bdd32d4cc735a37fc46f0147b339b28aaacaa0de0d1ca8b3512de97002',
    'subscription.py': 'b21a0f4ef3bd60583b569d912ab60a20377ba7b0f3213af43d29403382630ef9',
    'time_source.py': '9aacb85102019116edc6eb74a6993ca7efa3296273bfa53c30cebbb792d446f5',
}
CLOCK_COLUMNS = (
    'callback_id', 'take_id', 'take_before_wall_ns', 'take_before_thread_cpu_ns',
    'take_before_tid', 'take_after_wall_ns', 'take_after_thread_cpu_ns',
    'take_after_tid', 'callback_enter_wall_ns', 'callback_enter_thread_cpu_ns',
    'callback_enter_tid', 'original_clock_before_ns', 'message_clock_ns',
    'original_callback_exit_wall_ns', 'original_callback_exit_thread_cpu_ns',
    'original_callback_exit_tid', 'original_clock_after_ns',
    'override_completed', 'original_exception_type',
    'override_completed_wall_ns', 'override_completed_thread_cpu_ns', 'override_completed_tid',
)
NOTE_COLUMNS = (
    'note_id', 'original_gate_wall_ns', 'observer_wall_ns', 'observer_thread_cpu_ns',
    'observer_tid', 'original_header_stamp_ns', 'original_gate_clock_ns',
    'prior_pose_stamp_ns', 'original_future_predicate', 'last_completed_callback_id',
    'original_clock_read_before_wall_ns', 'original_clock_read_interval_valid',
)
_TAKE = ContextVar('r99_original_clock_take', default=None)
PROBE = None
_stopped = None
_binding = None
_receipt = None


class Recorder:
    def __init__(self, node):
        self.node = weakref.ref(node)
        self.pid = os.getpid()
        self.rows, self.notes = deque(maxlen=CAPACITY), deque(maxlen=NOTE_CAPACITY)
        self.windows = {}
        self.take_serial = self.callback_serial = self.note_serial = 0
        self.take_attempts = self.take_empty = self.take_business_errors = 0
        self.recorded = self.evicted = self.dropped = self.unmatched_callbacks = 0
        self.note_recorded = self.note_evicted = self.note_dropped = 0
        self.future_wait_calls = 0
        self.read_before_attempted = self.read_before_errors = 0
        self.owner = {}
        self.closed = False
        self.restore_callback = self.restore_take = 'not_installed'
        self.errors = {key: 0 for key in (
            'sample', 'clock_read', 'message', 'take_observation', 'context',
            'callback_record', 'note', 'future_wait', 'freeze', 'restore_callback',
            'restore_take', 'detach', 'finish', 'identity')}

    def bind_owner(self, node, callback):
        """Setup-only identity; no other process data or environment is read."""
        try:
            fields = Path('/proc/self/stat').read_text().rsplit(')', 1)[1].split()
            self.owner = dict(pid=self.pid, start_ticks=int(fields[19]),
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                run=str(Path(node.run).resolve()), node_id=id(node),
                node_clock_id=id(node.get_clock()),
                original_timesource_owner_id=id(getattr(callback, '__self__', None)),
                object_ids_are_process_local=True,
                candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'))
        except Exception:
            self.error('identity')

    def error(self, key):
        try:
            self.errors[key] += 1
        except Exception:
            pass

    def sample(self):
        try:
            return time.monotonic_ns(), time.thread_time_ns(), threading.get_native_id()
        except Exception:
            self.error('sample')
            return None, None, None

    def clock(self):
        try:
            node = self.node()
            value = node.get_clock().now().nanoseconds
            if type(value) is not int:
                raise TypeError('integer local clock required')
            return value
        except Exception:
            self.error('clock_read')
            return None

    def message_clock(self, msg):
        try:
            sec, nanosec = msg.clock.sec, msg.clock.nanosec
            if type(sec) is not int or type(nanosec) is not int:
                raise TypeError('integer message clock required')
            return sec*1_000_000_000+nanosec
        except Exception:
            self.error('message')
            return None

    def take_before(self):
        try:
            self.take_attempts += 1
            self.take_serial += 1
            return (self.take_serial, *self.sample())
        except Exception:
            self.error('take_observation')
            return None

    def take_after(self, before, result):
        try:
            if result is None:
                self.take_empty += 1
            return None if before is None else (*before, *self.sample())
        except Exception:
            self.error('take_observation')
            return None

    def callback(self, original, msg):
        if self.closed:
            return original(msg)
        entered, prior, source, context = (None, None, None), None, None, None
        serial = None
        try:
            self.callback_serial += 1
            serial = self.callback_serial
            entered = self.sample()
            prior = self.clock()
            source = self.message_clock(msg)
            context = _TAKE.get()
            if context is None:
                self.unmatched_callbacks += 1
        except Exception:
            self.error('callback_record')
        returned, error = False, None
        try:
            value = original(msg)
            returned = True
            return value
        except BaseException as failure:
            error = type(failure).__name__
            raise
        finally:
            # The original callback has returned/raised before these reads.
            try:
                exited = self.sample()
                after = self.clock()
                take = (None,)*7 if context is None else context
                row = (serial, *take, *entered, prior, source, *exited, after, returned, error,
                       *(exited if returned else (None, None, None)))
                if len(row) != len(CLOCK_COLUMNS):
                    raise ValueError('clock row shape')
                self.evicted += int(len(self.rows) == CAPACITY)
                self.rows.append(row)
                self.recorded += 1
                for window in tuple(self.windows.values()):
                    if len(window['post_rows']) < POST:
                        window['post_rows'].append(row)
            except Exception:
                self.dropped += 1
                self.error('callback_record')

    def freeze(self, kind, trigger, trigger_kind=None):
        try:
            if kind not in ('first_future', 'first_hard_fault'):
                raise ValueError('Only two bounded original failure windows are supported')
            if kind not in self.windows:
                self.windows[kind] = dict(trigger=trigger, trigger_kind=trigger_kind,
                    rows=tuple(self.rows),
                    notes=tuple(self.notes), post_rows=[], post_notes=[])
        except Exception:
            self.error('freeze')

    def note(self, stamp, originalclock, wall, priorpose, read_before=None):
        try:
            if self.closed:
                self.note_dropped += 1
                return
            if not all(type(value) is int for value in (stamp, originalclock, wall, priorpose)):
                raise TypeError('original gate scalar integers required')
            if read_before is not None and type(read_before) is not int:
                self.read_before_errors += 1
                read_before = None
            self.note_serial += 1
            observed = self.sample()
            row = (self.note_serial, wall, *observed, stamp, originalclock, priorpose,
                   stamp > originalclock, self.rows[-1][0] if self.rows else None,
                   read_before, type(read_before) is int and read_before <= wall)
            self.note_evicted += int(len(self.notes) == NOTE_CAPACITY)
            self.notes.append(row)
            self.note_recorded += 1
            for window in tuple(self.windows.values()):
                if len(window['post_notes']) < NOTE_POST:
                    window['post_notes'].append(row)
            if stamp > originalclock:
                self.freeze('first_future', row, 'pid_future_gate')
        except Exception:
            self.note_dropped += 1
            self.error('note')

    def future_wait(self, stamp, originalclock, wall):
        try:
            self.future_wait_calls += 1
            if self.closed:
                return
            if not all(type(value) is int for value in (stamp, originalclock, wall)):
                raise TypeError('original owner future scalar integers required')
            if stamp <= originalclock or 'first_future' in self.windows:
                return
            self.freeze('first_future', (stamp, originalclock, wall, *self.sample()),
                        'owner_future_deferral')
        except Exception:
            self.error('future_wait')

    def status(self):
        return dict(pid=self.pid, closed=self.closed, capacity=CAPACITY,
            note_capacity=NOTE_CAPACITY, take_attempts=self.take_attempts,
            take_empty=self.take_empty, take_business_errors=self.take_business_errors,
            callbacks_attempted=self.callback_serial, callbacks_recorded=self.recorded,
            evicted=self.evicted, dropped=self.dropped,
            unmatched_callbacks=self.unmatched_callbacks, notes_recorded=self.note_recorded,
            note_evicted=self.note_evicted, note_dropped=self.note_dropped,
            owner_future_wait_calls=self.future_wait_calls,
            read_interval=dict(environment_key=READ_KEY, environment_value=READ_OPTION,
                enabled=READ_OPTION == '1', attempted=self.read_before_attempted,
                errors=self.read_before_errors,
                after_endpoint='original_gate_wall_ns, sampled after original now()',
                before_endpoint='original_clock_read_before_wall_ns, sampled before original now()'),
            owner=dict(self.owner), maximum_output_bytes=MAX_OUTPUT_BYTES,
            errors=dict(self.errors), restore_callback=self.restore_callback,
            restore_take=self.restore_take)

    def snapshot(self):
        result = self.status()
        result.update(schema='r99_original_clock_take_callback_observation/v1',
            clock_columns=list(CLOCK_COLUMNS), note_columns=list(NOTE_COLUMNS),
            rows=list(self.rows), notes=list(self.notes), windows=self.windows,
            maximum_retained_clock_rows=3*CAPACITY+2*POST,
            maximum_retained_note_rows=3*NOTE_CAPACITY+2*NOTE_POST,
            one_combined_immutable_scalar_row_per_clock_callback=True,
            take_is_original_executor_take=True, take_is_ready_or_DDS_arrival_time=False,
            no_new_subscription=True, no_hot_path_disk_IO=True,
            no_message_payloads_or_locals=True, no_original_timestamps_changed=True,
            no_clock_or_GC_policy_change=True, control_or_navigation_PASS=False,
            limits=[
                'Ordered wall/thread CPU/clock samples are not atomic.',
                'Take endpoints observe only the original take call, not publisher/transport/DDS arrival/ready.',
                'override_completed marks successful original callback return; clock_after is sampled afterwards.',
                'Cross-thread note and callback recording order need endpoint comparison, not row order alone.',
                'Only update completion strictly before read_before proves completion before the original read; overlapping intervals remain unresolved.',
                'Read interval endpoints are optional observations, not business freshness inputs.',
                'The pinned synchronous TimeSource callback cannot yield in await_or_execute.',
                'Probe allocations and clock reads perturb scheduling; no clock-delivery cause is assumed.'])
        return result


def _compatibility(node, executor):
    # Setup only. Importing this module while OFF has no rclpy dependency.
    import rclpy.executors as execution
    import rclpy.subscription as subscription
    import rclpy.time_source as time_source
    sources = {}
    for module in (execution, subscription, time_source):
        path = Path(module.__file__).resolve()
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if path.name not in EXPECTED_SHA256 or digest != EXPECTED_SHA256[path.name]:
            raise RuntimeError('R99 unsupported Jazzy source: '+path.name)
        sources[path.name] = dict(path=str(path), sha256=digest)
    matches = [sub for sub in node.subscriptions if sub.topic_name == '/clock']
    if len(matches) != 1:
        raise RuntimeError('R99 requires exactly one existing /clock subscription')
    sub = matches[0]
    if type(sub) is not subscription.Subscription:
        raise RuntimeError('R99 requires the pinned public Subscription API')
    descriptor = inspect.getattr_static(subscription.Subscription, 'callback')
    if not isinstance(descriptor, property) or descriptor.fget is None or descriptor.fset is None:
        raise RuntimeError('R99 public callback property/setter unavailable')
    callback = sub.callback
    if (not inspect.ismethod(callback) or type(callback.__self__) is not time_source.TimeSource
            or callback.__func__ is not time_source.TimeSource.clock_callback
            or inspect.iscoroutinefunction(callback)
            or sub._callback_type is not subscription.Subscription.CallbackType.MessageOnly
            or sub.raw):
        raise RuntimeError('R99 requires original synchronous TimeSource MessageOnly callback')
    inspect.signature(callback).bind(object())
    take = executor._take_subscription
    if (not inspect.ismethod(take) or take.__self__ is not executor
            or take.__func__ is not execution.Executor._take_subscription
            or inspect.iscoroutinefunction(take)):
        raise RuntimeError('R99 executor take method differs from pinned Jazzy shape')
    inspect.signature(take).bind(sub)
    # The whole pinned files additionally bind _execute/await_or_execute shape:
    # original take returns async closure; synchronous callback path cannot yield.
    return sub, sources


def _attach(node, executor, sub, sources):
    """Internal already-verified attachment; helper tests use explicit fake APIs."""
    global PROBE, _binding, _receipt
    probe = Recorder(node)
    callback, take = sub.callback, executor._take_subscription
    probe.bind_owner(node, callback)
    had_take = '_take_subscription' in vars(executor)
    previous_take = vars(executor).get('_take_subscription')

    @wraps(callback)
    def observed_callback(msg):
        return probe.callback(callback, msg)

    @wraps(take.__func__)
    def observed_take(self, incoming):
        if incoming is not sub or probe.closed:
            return take(incoming)
        try:
            before = probe.take_before()
        except Exception:
            probe.error('take_observation')
            before = None
        try:
            execute = take(incoming)
        except BaseException:
            try:
                probe.take_business_errors += 1
            except Exception:
                probe.error('take_observation')
            raise
        try:
            context = probe.take_after(before, execute)
        except Exception:
            probe.error('take_observation')
            context = None
        if execute is None:
            return None

        @wraps(execute)
        async def observed_execute():
            token = None
            try:
                token = _TAKE.set(context)
            except Exception:
                probe.error('context')
            try:
                return await execute()
            finally:
                if token is not None:
                    try:
                        _TAKE.reset(token)
                    except Exception:
                        probe.error('context')
        return observed_execute

    wrapped_take = MethodType(observed_take, executor)
    PROBE = probe
    _binding = (sub, callback, observed_callback, executor, had_take, previous_take, wrapped_take)
    try:
        sub.callback = observed_callback
        probe.restore_callback = 'installed'
        executor._take_subscription = wrapped_take
        probe.restore_take = 'installed'
    except BaseException:
        detach()
        raise
    _receipt = dict(enabled=True, installed=True, environment_key=KEY,
        environment_value=OPTION, pid=probe.pid, sources=sources,
        public_subscription_callback_setter=True, original_timesource_callback=True,
        version_bound_executor_instance_take=True, one_clock_subscription=True,
        context_var_execute_link=True, capacity=CAPACITY, note_capacity=NOTE_CAPACITY,
        read_interval_environment_key=READ_KEY, read_interval_environment_value=READ_OPTION,
        owner=dict(probe.owner), maximum_output_bytes=MAX_OUTPUT_BYTES,
        candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'))
    return status()


def install(node, executor):
    global _receipt
    if os.environ.get(KEY, '0') != OPTION:
        raise RuntimeError('R99 environment differs from imported selection')
    if os.environ.get(READ_KEY, '0') != READ_OPTION:
        raise RuntimeError('R102 read interval environment differs from imported selection')
    if READ_OPTION == '1' and (OPTION != '1' or os.environ.get('R99_CLOCK_FEEDBACK', '0') != '0'):
        raise RuntimeError('R102 interval observation requires R99 probe ON and feedback wait OFF')
    if _binding is not None:
        if _binding[0] in node.subscriptions and _binding[3] is executor:
            return dict(status(), already_installed=True)
        raise RuntimeError('R99 already installed on another node/executor')
    if OPTION == '0':
        _receipt = dict(enabled=False, installed=False, environment_key=KEY, environment_value=OPTION)
        return status()
    sub, sources = _compatibility(node, executor)
    return _attach(node, executor, sub, sources)


def read_before(node):
    """Optional endpoint only; caller keeps its original now()/wall/predicate order."""
    probe = PROBE
    if READ_OPTION == '1' and probe is not None and probe.node() is node:
        try:
            probe.read_before_attempted += 1
            return time.monotonic_ns()
        except Exception:
            probe.read_before_errors += 1
    return None


def note(node, stamp, originalclock, wall, priorpose, read_before=None):
    probe = PROBE
    if probe is not None:
        try:
            if probe.node() is node:
                probe.note(stamp, originalclock, wall, priorpose, read_before)
        except Exception:
            probe.error('note')


def hard_fault(node, reason, wall=None):
    probe = PROBE
    if probe is not None:
        try:
            if probe.node() is node:
                original_wall = wall if type(wall) is int else None
                text = reason if type(reason) is str else type(reason).__name__
                probe.freeze('first_hard_fault', (original_wall, *probe.sample(), text[:240]), 'hard_fault')
        except Exception:
            probe.error('freeze')


def future_wait(node, stamp, original_clock, wall):
    """Owner's original future-defer predicate, never a fabricated PID gate note.

    Shares the sole first_future snapshot with note(); first observed wins.
    Trigger tuple: (header_stamp, original_owner_clock, original_owner_wall,
                    observer_wall, observer_thread_cpu, observer_tid).
    """
    probe = PROBE
    if probe is not None:
        try:
            if probe.node() is node:
                probe.future_wait(stamp, original_clock, wall)
        except Exception:
            probe.error('future_wait')


def status():
    result = dict(_receipt or dict(enabled=False, installed=False,
        environment_key=KEY, environment_value=OPTION))
    probe = PROBE if PROBE is not None else _stopped
    if probe is not None:
        result['observation'] = probe.status()
    return result


def _detach():
    """Call after callback join; restore only bindings still owned by this probe."""
    global PROBE, _stopped, _binding
    probe, PROBE = PROBE, None
    if probe is not None:
        _stopped = probe
        probe.closed = True
    binding, _binding = _binding, None
    if binding is None:
        return
    sub, original, wrapped, executor, had_take, previous, wrapped_take = binding
    try:
        if sub.callback is wrapped:
            sub.callback = original
            if probe is not None:
                probe.restore_callback = 'restored'
        elif probe is not None:
            probe.restore_callback = 'changed_by_another_owner'
    except Exception:
        if probe is not None:
            probe.restore_callback = 'restore_failed'
            probe.error('restore_callback')
    try:
        if vars(executor).get('_take_subscription') is wrapped_take:
            if had_take:
                executor._take_subscription = previous
            else:
                del executor._take_subscription
            if probe is not None:
                probe.restore_take = 'restored'
        elif probe is not None:
            probe.restore_take = 'changed_by_another_owner'
    except Exception:
        if probe is not None:
            probe.restore_take = 'restore_failed'
            probe.error('restore_take')


def detach():
    try:
        _detach()
    except Exception:
        probe = PROBE if PROBE is not None else _stopped
        if probe is not None:
            probe.error('detach')


def finish(node):
    """Exit-only flush; observation/restore errors must not replace business errors."""
    global _stopped
    try:
        detach()
        probe = _stopped
        if probe is None:
            return status()
        value = probe.snapshot()
        value['installation'] = status()
        path = Path(node.run)/'r99_clock_timeline.json'
        partial = path.with_suffix('.json.partial')
        written = 0
        with partial.open('xb') as stream:
            for chunk in json.JSONEncoder(allow_nan=False).iterencode(value):
                encoded = chunk.encode('utf-8')
                if written + len(encoded) + 1 > MAX_OUTPUT_BYTES:
                    raise OverflowError('R99 clock output byte limit; bounded partial retained')
                stream.write(encoded)
                written += len(encoded)
            stream.write(b'\n')
            written += 1
        partial.replace(path)
        _stopped = None
        return dict(enabled=True, flushed=True, path=str(path), output_bytes=written,
            maximum_output_bytes=MAX_OUTPUT_BYTES)
    except Exception as error:
        if _stopped is not None:
            _stopped.error('finish')
        return dict(enabled=OPTION == '1', flushed=False, error=type(error).__name__,
            maximum_output_bytes=MAX_OUTPUT_BYTES, partial_output_may_exist=True)
