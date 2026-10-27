"""Default-OFF bounded controller CPU/GC evidence; no navigation authority.

No thread, hot-path file I/O, payload copy, GC policy change or clock rewrite.
All clocks are sampled in order, so wall/CPU endpoints are not simultaneous.
GC events observe only this process and may themselves perturb allocation/GC.
"""
from collections import deque
from contextlib import contextmanager
from functools import wraps
import gc
import json
import os
import sys
import threading
import time
from types import MethodType

KEY = 'R96_CONTROL_CPU_PROBE'
OPTION = os.environ.get(KEY, '0')
if OPTION not in ('0', '1'):
    raise ValueError(KEY + ' must be 0 or 1')
CAPACITY = 8192
GC_CAPACITY = 1024
POST = 256
MAX_THREADS = 8
MAX_DEPTH = 32
PROBE = None
_receipt = None
_stopped = None
_zero_binding = None
_cleanup_errors = dict(detach=0, callback_remove=0, restore=0, finish=0)
_restore_status = 'not_installed'


class Recorder:
    def __init__(self, capacity=CAPACITY, gc_capacity=GC_CAPACITY):
        self.pid = os.getpid()
        self.capacity, self.gc_capacity = capacity, gc_capacity
        self.rows, self.gc_rows = deque(maxlen=capacity), deque(maxlen=gc_capacity)
        self.states, self.gc_active = {}, {}
        self.bookkeeping = {}
        self.serial = self.tick_serial = 0
        self.recorded = self.evicted = self.dropped = self.errors = 0
        self.gc_recorded = self.gc_evicted = self.gc_dropped = self.gc_errors = 0
        self.gc_unmatched_stop = self.gc_unclosed_start = 0
        self.longest_ns = 0
        self.slow = self.exception = None
        self.slow_post, self.exception_post = [], []
        self.closed = False
        self.zero_rows, self.zero_first = deque(maxlen=256), []
        self.zero_counts = {}
        self.zero_attempted = self.zero_recorded = self.zero_evicted = self.zero_errors = 0
        self.zero_caller_overflow = 0

    def begin(self, stage, node=None):
        tid = None
        try:
            if self.closed or os.getpid() != self.pid:
                self.dropped += 1
                return None
            tid = threading.get_native_id()
            if tid in self.bookkeeping or len(self.bookkeeping) < MAX_THREADS:
                self.bookkeeping[tid] = True
            state = self.states.get(tid)
            if state is None:
                if len(self.states) >= MAX_THREADS:
                    self.dropped += 1
                    return None
                state = self.states[tid] = []
            if len(state) >= MAX_DEPTH:
                self.dropped += 1
                return None
            self.serial += 1
            if stage == 'control.tick':
                self.tick_serial += 1
            source = getattr(node, 'pose_stamp', None) if node is not None else None
            source = source if type(source) is int else None
            token = (self.serial, state[-1][0] if state else None,
                     self.tick_serial if state or stage == 'control.tick' else None,
                     tid, stage, source, time.monotonic_ns(), time.thread_time_ns())
            state.append(token)
            return token
        except Exception:
            self.errors += 1
            return None
        finally:
            if tid in self.bookkeeping:
                self.bookkeeping[tid] = False

    def end(self, token, error=None):
        if token is None:
            return
        try:
            self.bookkeeping[token[3]] = True
            wall, cpu = time.monotonic_ns(), time.thread_time_ns()
            state = self.states.get(token[3])
            if state and state[-1] is token:
                state.pop()
            else:
                self.errors += 1
            row = token + (wall, cpu, error)
            self.evicted += int(len(self.rows) == self.capacity)
            self.rows.append(row)
            self.recorded += 1
            if self.slow is not None and len(self.slow_post) < POST:
                self.slow_post.append(row)
            if self.exception is not None and len(self.exception_post) < POST:
                self.exception_post.append(row)
            if token[4] == 'control.tick' and wall-token[6] > self.longest_ns:
                self.longest_ns = wall-token[6]
                # Bounded shallow references to scalar rows only, never graphs.
                self.slow = (tuple(self.rows), tuple(self.gc_rows), row)
                self.slow_post.clear()
            if error is not None and self.exception is None:
                self.exception = (tuple(self.rows), tuple(self.gc_rows), row)
        except Exception:
            self.errors += 1
        finally:
            self.bookkeeping[token[3]] = False

    def gc_callback(self, phase, info):
        try:
            if self.closed or os.getpid() != self.pid:
                self.gc_dropped += 1
                return
            tid = threading.get_native_id()
            wall, cpu = time.monotonic_ns(), time.thread_time_ns()
            generation = info.get('generation')
            if phase == 'start':
                if len(self.gc_active) >= MAX_THREADS or tid in self.gc_active:
                    self.gc_dropped += 1
                    return
                state = self.states.get(tid)
                active = state[-1] if state else None
                self.gc_active[tid] = (tid, generation, wall, cpu,
                    active[0] if active else None, active[2] if active else None,
                    active[4] if active else None, self.bookkeeping.get(tid, False))
            elif phase == 'stop':
                before = self.gc_active.pop(tid, None)
                if before is None:
                    self.gc_unmatched_stop += 1
                    return
                row = before + (wall, cpu, info.get('collected'), info.get('uncollectable'))
                self.gc_evicted += int(len(self.gc_rows) == self.gc_capacity)
                self.gc_rows.append(row)
                self.gc_recorded += 1
        except Exception:
            self.gc_errors += 1

    def zero_begin(self, node, caller):
        self.zero_attempted += 1
        key = (caller.f_code.co_filename.rsplit('/', 1)[-1][:120],
               caller.f_code.co_name[:120], caller.f_lineno)
        if key in self.zero_counts or len(self.zero_counts) < 64:
            self.zero_counts[key] = self.zero_counts.get(key, 0)+1
        else:
            self.zero_caller_overflow += 1
        tid = threading.get_native_id()
        state = self.states.get(tid)
        return (self.zero_attempted, key, tid, state[-1][2] if state else None,
                time.monotonic_ns(), time.thread_time_ns(), self.zero_state(node))

    @staticmethod
    def zero_state(node):
        def scalar(value):
            return value if type(value) in (bool, int, float) or value is None else None
        command = getattr(node, 'command', None)
        command = tuple(scalar(command[i]) for i in range(3)) if command is not None and len(command)==3 else None
        feedback = getattr(node, 'feedback', None)
        return (scalar(getattr(node, 'publication_records', None)),
                scalar(getattr(node, 'pose_stamp', None)),
                scalar(getattr(node, 'cloud_stamp', None)),
                scalar(feedback.get('stamp_ns')) if type(feedback) is dict else None,
                scalar(getattr(node, 'last_publication_ros_clock_ns', None)),
                scalar(getattr(node, 'last_command_time', None)), command)

    def zero_end(self, token, node, error):
        if token is None:
            return
        row = token + (time.monotonic_ns(), time.thread_time_ns(), self.zero_state(node), error)
        self.zero_evicted += int(len(self.zero_rows) == 256)
        self.zero_rows.append(row)
        if len(self.zero_first) < 64:
            self.zero_first.append(row)
        self.zero_recorded += 1

    def status(self):
        return dict(enabled=True, pid=self.pid, closed=self.closed,
            capacity=self.capacity, gc_capacity=self.gc_capacity, post_capacity=POST,
            recorded=self.recorded, evicted=self.evicted, dropped=self.dropped,
            errors=self.errors, gc_recorded=self.gc_recorded, gc_evicted=self.gc_evicted,
            gc_dropped=self.gc_dropped, gc_errors=self.gc_errors,
            gc_unmatched_stop=self.gc_unmatched_stop,
            gc_unclosed_start=self.gc_unclosed_start, longest_tick_wall_ns=self.longest_ns,
            active_spans=sum(len(s) for s in self.states.values()),
            gc_callback_installed=self.gc_callback in gc.callbacks,
            zero_attempted=self.zero_attempted, zero_recorded=self.zero_recorded,
            zero_evicted=self.zero_evicted, zero_errors=self.zero_errors,
            zero_caller_overflow=self.zero_caller_overflow)

    def snapshot(self):
        value = self.status()
        value.update(schema='r96_bounded_controller_cpu_gc/v1', switch=KEY,
            row_columns=['span_id','parent_span_id','tick_id','native_tid','stage',
                'source_pose_stamp_ns','wall_begin_ns','thread_cpu_begin_ns',
                'wall_end_ns','thread_cpu_end_ns','exception_type'],
            gc_columns=['native_tid','generation','wall_begin_ns','thread_cpu_begin_ns',
                'active_span_id','tick_id','active_stage','observer_bookkeeping',
                'wall_end_ns','thread_cpu_end_ns',
                'collected','uncollectable'],
            rows=list(self.rows), gc_rows=list(self.gc_rows),
            gc_unclosed_starts=list(self.gc_active.values()),
            zero_columns=['zero_id','caller_basename_function_lineno','native_tid','tick_id',
                'wall_begin_ns','thread_cpu_begin_ns','state_before','wall_end_ns',
                'thread_cpu_end_ns','state_after','exception_type'],
            zero_state_columns=['publication_records','pose_stamp','cloud_stamp','feedback_stamp_ns',
                'last_publication_ros_clock_ns','last_command_time','command_xyz'],
            zero_tail=list(self.zero_rows), zero_first=self.zero_first,
            zero_caller_counts=[(key,value) for key,value in self.zero_counts.items()],
            maximum_retained_zero_rows=320, maximum_zero_caller_keys=64,
            slow_window=self.slow, slow_post=self.slow_post,
            exception_window=self.exception, exception_post=self.exception_post,
            maximum_retained_span_rows=3*self.capacity+2*POST,
            maximum_retained_gc_rows=3*self.gc_capacity,
            no_hot_path_io=True, no_payloads=True, gc_policy_unchanged=True,
            control_or_navigation_PASS=False,
            clock_basis='Ordered monotonic/thread CPU samples from recorded TID; not atomic.',
            attribution_limit='A GC interval overlaps the recorded stage; causation requires event alignment. Probe allocations perturb GC.')
        return value


@contextmanager
def span(stage, node=None):
    probe = PROBE
    token = probe.begin(stage, node) if probe is not None else None
    error = None
    try:
        yield
    except BaseException as exception:
        error = type(exception).__name__
        raise
    finally:
        if probe is not None:
            probe.end(token, error)


def observe(stage):
    def decorate(function):
        if OPTION == '0':
            return function
        @wraps(function)
        def observed(self, *args, **kwargs):
            if PROBE is None:
                return function(self, *args, **kwargs)
            with span(stage, self):
                return function(self, *args, **kwargs)
        return observed
    return decorate


def receipt_copy(function, receipt, node, stage='control.final_bundle_receipt_deepcopy'):
    """Observation-only hook around the final_bundle receipt builder.

    The caller now passes the read-only snapshot builder and its own stage
    label; the original deepcopy label stays the default so historical analysis
    names remain attributable. Nothing here copies or rewrites the receipt.
    """
    with span(stage, node):
        return function(receipt)


def install(node):
    global PROBE, _receipt, _zero_binding, _restore_status
    if _receipt is not None:
        return dict(_receipt, already_installed=True)
    _receipt = dict(enabled=False, environment_key=KEY, environment_value=OPTION,
                    installed=False, gc_policy_before=list(gc.get_threshold()))
    if OPTION == '0':
        return dict(_receipt)
    try:
        PROBE = Recorder()
        gc.callbacks.append(PROBE.gc_callback)
        if hasattr(node, 'publish_command'):
            original = node.publish_command
            had_instance = 'publish_command' in vars(node)
            previous = vars(node).get('publish_command')
            @wraps(original)
            def observed_zero(self, *args, **kwargs):
                probe = PROBE
                velocity = args[0] if args else kwargs.get('velocity')
                if probe is None or velocity is not None:
                    return original(*args, **kwargs)
                token = None
                error = None
                try:
                    caller = sys._getframe(1)
                    try:token = probe.zero_begin(self, caller)
                    finally:del caller
                except Exception:
                    probe.zero_errors += 1
                try:
                    return original(*args, **kwargs)
                except BaseException as exception:
                    error = type(exception).__name__
                    raise
                finally:
                    try:probe.zero_end(token, self, error)
                    except Exception:probe.zero_errors += 1
            wrapper = MethodType(observed_zero, node)
            node.publish_command = wrapper
            _zero_binding = (node, had_instance, previous, wrapper)
            _restore_status = 'installed'
        _receipt.update(enabled=True, installed=True, pid=PROBE.pid,
            native_tid=threading.get_native_id(), gc_enabled_before=gc.isenabled(),
            capacity=CAPACITY, gc_capacity=GC_CAPACITY,
            zero_publication_wrapper_installed=_zero_binding is not None,
            candidate_commit=os.environ.get('R87_CANDIDATE_COMMIT'))
    except Exception as error:
        _receipt['installation_error'] = type(error).__name__
        detach()
    return dict(_receipt)


def status():
    return dict(_receipt or dict(enabled=False, installed=False,
                environment_key=KEY, environment_value=OPTION),
                cleanup_errors=dict(_cleanup_errors), zero_restore_status=_restore_status)


def _detach():
    global PROBE, _stopped, _zero_binding, _restore_status
    probe, PROBE = PROBE, None
    if probe is not None:
        _stopped = probe
        probe.closed = True
        probe.gc_unclosed_start = len(probe.gc_active)
    # A prior failed removal remains recoverable at the later finish call.
    probe = _stopped
    try:
        if probe is not None and probe.gc_callback in gc.callbacks:
            gc.callbacks.remove(probe.gc_callback)
    except Exception:
        _cleanup_errors['callback_remove'] += 1
    binding, _zero_binding = _zero_binding, None
    if binding is not None:
        node, had_instance, previous, wrapper = binding
        try:
            if vars(node).get('publish_command') is wrapper:
                if had_instance:node.publish_command = previous
                else:del node.publish_command
                _restore_status = 'restored'
                if _receipt is not None:_receipt['zero_publication_wrapper_restored'] = True
            else:
                _restore_status = 'changed_by_another_owner'
        except Exception:
            _restore_status = 'restore_failed'
            _cleanup_errors['restore'] += 1
            if _receipt is not None:_receipt['zero_publication_restore_error'] = True


def detach():
    """Nonthrowing after callback join, before exit evidence graph construction."""
    try:
        _detach()
    except Exception:
        _cleanup_errors['detach'] += 1


def finish(node):
    global _stopped
    try:
        detach()
        probe, _stopped = _stopped, None
        if probe is None:
            return status()
        value = probe.snapshot()
        value['installation'] = status()
        (node.run/'r96_control_cpu_timeline.json').write_text(json.dumps(value, allow_nan=False)+'\n')
        return dict(enabled=True, flushed=True, path=str(node.run/'r96_control_cpu_timeline.json'))
    except Exception as error:
        _cleanup_errors['finish'] += 1
        return dict(enabled=True, flushed=False, error=type(error).__name__)
