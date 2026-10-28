"""Default-off bounded diagnostic timeline. No control authority or hot-path I/O.

Enable explicitly with R87_DIAGNOSTIC_PROBE=1 before process startup. Endpoints
reuse the R86 wall/thread/process/rusage sampler. Only scalar identities enter
the ring; messages and point arrays are never copied or retained here.
"""
from collections import deque
from contextlib import contextmanager
from functools import wraps
from itertools import count
import os
import math
import threading
import time
import json
import sys
import experiments.diagnostics.r89_thread_probe as _r89_threads

from experiments.diagnostics.r86_stage_probe import sample

KEY = 'R87_DIAGNOSTIC_PROBE'
CAPACITY = 32768


class Recorder:
    def __init__(self, capacity=CAPACITY):
        self.capacity = capacity
        self.rows = deque(maxlen=capacity)
        self.frozen = None
        self.trigger = None
        self.pending_anomaly = {}
        self.guard = threading.Lock()
        self.ids = count(1)
        self.attempts = count(1)
        self.drops = count(1)
        self.errors = count(1)
        self.attempted = self.dropped = self.error_count = self.recorded = self.evicted = 0
        self.slow = deque(maxlen=64)

    def mark(self, stage, source_ns=None, **fields):
        try:
            self.attempted = next(self.attempts)
            row = sample()
            row.update(stage=str(stage)[:120], source_ns=source_ns)
            for key, value in fields.items():
                if type(value) in (bool, int, float, str) or value is None:
                    if type(value) is float and not math.isfinite(value):
                        value = None
                    row[str(key)[:80]] = value[:240] if type(value) is str else value
            if not self.guard.acquire(False):
                self.dropped = next(self.drops)
                return row
            try:
                self.evicted += int(len(self.rows) == self.capacity)
                self.rows.append(row)
                self.recorded += 1
                self._freeze_pending()
            finally:
                self.guard.release()
            return row
        except Exception:
            self.error_count = next(self.errors)
            return None

    def anomaly(self, reason, source_ns=None):
        # CPython dict.setdefault selects one first request atomically without
        # waiting on the diagnostic lock; the next successful writer freezes it.
        self.pending_anomaly.setdefault('first', dict(reason=str(reason)[:240],
            source_ns=source_ns, monotonic_wall_ns=time.monotonic_ns()))
        self.mark('anomaly', source_ns, reason=reason)

    def _freeze_pending(self):
        if self.frozen is None and self.pending_anomaly:
            self.frozen = tuple(self.rows)
            self.trigger = dict(self.pending_anomaly['first'],
                                frozen_monotonic_wall_ns=time.monotonic_ns())

    def snapshot(self):
        # Exit-only, after executor callback threads have joined.
        with self.guard:
            self._freeze_pending()
            return dict(schema='r87_bounded_diagnostic_timeline/v1', enabled=True,
                switch=KEY, capacity=self.capacity, recorded=self.recorded,
                attempted=self.attempts.__reduce__()[1][0]-1, evicted=self.evicted,
                dropped_lock_contention=self.drops.__reduce__()[1][0]-1,
                probe_errors=self.errors.__reduce__()[1][0]-1,
                counter_basis='monotonic attempted/drop/error sequence high-water; endpoint samples are not atomic',
                rows=list(self.rows), anomaly_window=list(self.frozen or ()),
                anomaly_trigger=self.trigger, slow_spans=list(self.slow),
                pending_anomaly_request=self.pending_anomaly.get('first'),
                no_hot_path_disk_io=True, no_message_or_cloud_payloads=True,
                maximum_retained_endpoints=2*self.capacity+128,
                control_or_navigation_PASS=False)


_option = os.environ.get(KEY, '0')
if _option not in ('0', '1'):
    raise ValueError(KEY + ' must be 0 or 1')
RECORDER = Recorder() if _option == '1' else None


def enabled():
    return RECORDER is not None


def mark(stage, source_ns=None, **fields):
    if RECORDER is not None:
        return RECORDER.mark(stage, source_ns, **fields)


def anomaly(reason, source_ns=None):
    try:
        _r89_threads.freeze(reason, source_ns=source_ns)
    except Exception:
        pass
    if RECORDER is not None:
        try:
            RECORDER.anomaly(reason, source_ns)
        except Exception:
            RECORDER.error_count = next(RECORDER.errors)


def flush(path):
    """Exit-only best effort for this optional artifact; preserve node cleanup."""
    if RECORDER is None:
        return
    try:
        path.write_text(json.dumps(RECORDER.snapshot(),allow_nan=False)+'\n')
    except Exception as error:
        # A single shutdown diagnostic, never a high-frequency hot-path print.
        print('R87 diagnostic timeline flush failed: '+type(error).__name__+': '+str(error),
              file=sys.stderr)


@contextmanager
def span(stage, source_ns=None, **fields):
    recorder = RECORDER
    if recorder is None:
        yield
        return
    serial = next(recorder.ids)
    before = mark(stage + '.enter', source_ns, span_id=serial, **fields)
    exception = None
    try:
        yield
    except BaseException as error:
        exception = type(error).__name__
        raise
    finally:
        after = mark(stage + '.exit', source_ns, span_id=serial, exception=exception, **fields)
        try:
            if before and after and after['wall_begin_ns']-before['wall_end_ns'] >= 50_000_000:
                if recorder.guard.acquire(False):
                    try: recorder.slow.append((before, after))
                    finally: recorder.guard.release()
                else: recorder.dropped = next(recorder.drops)
        except Exception:
            recorder.error_count = next(recorder.errors)


@contextmanager
def lock(original, stage, source_ns=None):
    if RECORDER is None:
        with original:
            yield
        return
    mark(stage + '.wait', source_ns)
    original.acquire()
    mark(stage + '.acquired', source_ns)
    try:
        yield
    finally:
        # Separate release call bounds: acquired->release_before is lock held;
        # release_after cannot be treated as the exact unlock instant.
        mark(stage + '.release_before', source_ns)
        original.release()
        mark(stage + '.release_after', source_ns)


def observe(stage):
    def decorate(function):
        @wraps(function)
        def wrapped(*args, **kwargs):
            if RECORDER is None:
                return function(*args, **kwargs)
            source_ns = None
            fields = {}
            try:
                message = args[1] if len(args)>1 else args[0]
                trajectory = getattr(message, 'traj_id', None)
                if type(trajectory) in (str, int):
                    fields['trajectory_id'] = trajectory
                stamp = getattr(getattr(message, 'header', None), 'stamp', None)
                if stamp is not None:
                    source_ns = int(stamp.sec)*1_000_000_000+int(stamp.nanosec)
            except Exception:
                pass
            with span(stage, source_ns, **fields):
                return function(*args, **kwargs)
        return wrapped
    return decorate
