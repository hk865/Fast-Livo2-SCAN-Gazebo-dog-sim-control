"""Bounded, optional worker evidence; never part of the wire protocol.

The child attempts one file write at Python exit. This can use some of the
unchanged 250 ms close grace; SIGTERM/SIGKILL can leave no evidence at all.
"""
from __future__ import annotations

from collections import deque
import functools
import hashlib
import json
import os
from pathlib import Path
import threading
import time

CAPACITY = 64
HIST_LIMITS_NS = (10000,100000,1000000,10000000,50000000,100000000,500000000,1000000000,2000000000)


def _safe(method):
    @functools.wraps(method)
    def call(self, *args, **kwargs):
        try:
            return method(self, *args, **kwargs)
        except Exception:
            try:
                self.observation_errors += 1
            except Exception:
                pass
            return None
    return call


def _token(value):
    if type(value) is int and 0 <= value < 2**63:
        return dict(type='int', value=value)
    if type(value) is str and 0 < len(value.encode('utf-8')) <= 512:
        return dict(type='str', value=value)
    return None


class Probe:
    enabled = True

    def __init__(self, role, run, worker_path):
        self.role = role
        self.run = Path(run)
        self.completed = deque(maxlen=CAPACITY)
        self.active = None
        self.background = dict(kind='background', stages={})
        self.sequence = self.evicted = self.observation_errors = 0
        self.exit_write_attempts = 0
        self.eof_read = None
        self.completed_phase_totals = {}
        stat = Path("/proc/self/stat").read_text()
        self.identity = dict(pid=os.getpid(), tid=threading.get_native_id(),
            parent_pid=os.getppid(), role=role, run_path=str(self.run),
            boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            start_ticks=int(stat[stat.rfind(')')+2:].split()[19]),
            monotonic_clock=time.get_clock_info('monotonic').implementation,
            thread_cpu_clock='CLOCK_THREAD_CPUTIME_ID; subtract only within same TID',
            worker_source_sha256=hashlib.sha256(Path(worker_path).read_bytes()).hexdigest(),
            probe_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())

    @_safe
    def new_job(self, token=None):
        if self.active is not None:
            self.finish('superseded_observation')
        self.sequence += 1
        self.active = dict(sequence=self.sequence, token=_token(token),
                           state='active', stages={})

    @_safe
    def set_token(self, token):
        if self.active is not None:
            self.active['token'] = _token(token)

    @_safe
    def annotate(self, **values):
        # Call sites pass only fixed scalar fields, never payloads or proofs.
        (self.active if self.active is not None else self.background).update(values)

    @_safe
    def begin(self, name):
        row = self.active if self.active is not None else self.background
        phase = row['stages'].setdefault(name, dict(calls=0, returned=0,
            errors=0, would_block=0, bytes=0, wall_total_ns=0, thread_cpu_total_ns=0))
        wall, cpu = time.monotonic_ns(), time.thread_time_ns()
        phase.setdefault('first_entered_monotonic_ns', wall)
        phase.update(entered_monotonic_ns=wall, entered_thread_cpu_ns=cpu,
                     returned_monotonic_ns=None, returned_thread_cpu_ns=None)
        phase['calls'] += 1
        return row

    @_safe
    def end(self, name, *, row=None, byte_count=None, outcome='returned'):
        row = row if row is not None else (
            self.active if self.active is not None else self.background)
        phase = row['stages'][name]
        wall, cpu = time.monotonic_ns(), time.thread_time_ns()
        phase.update(returned_monotonic_ns=wall, returned_thread_cpu_ns=cpu,
                     last_outcome=outcome)
        phase['returned'] += 1
        phase['wall_total_ns'] += wall - phase['entered_monotonic_ns']
        phase['thread_cpu_total_ns'] += cpu - phase['entered_thread_cpu_ns']
        if byte_count is not None:
            phase['bytes'] += byte_count
            phase['last_bytes'] = byte_count
            if byte_count > 0:
                phase.setdefault('first_progress_entered_monotonic_ns', phase['entered_monotonic_ns'])
                phase.setdefault('first_progress_returned_monotonic_ns', wall)
                phase.update(last_progress_entered_monotonic_ns=phase['entered_monotonic_ns'],
                             last_progress_returned_monotonic_ns=wall)
        if outcome == 'would_block':
            phase['would_block'] += 1
        elif outcome == 'error':
            phase['errors'] += 1

    @_safe
    def error(self, error):
        row = self.active if self.active is not None else self.background
        row['error_type'] = type(error).__name__[:128]
        for name, phase in row['stages'].items():
            if phase.get('returned_monotonic_ns') is None:
                self.end(name, row=row, outcome='error')

    @_safe
    def finish(self, state):
        if self.active is None:
            return
        self.active['state'] = state
        if len(self.completed) == CAPACITY:
            self.evicted += 1
        for name, phase in self.active['stages'].items():
            totals = self.completed_phase_totals.setdefault(name, dict(jobs=0,
                calls=0, wall_ns=0, thread_cpu_ns=0, max_job_wall_ns=0,
                wall_bins=[0]*(len(HIST_LIMITS_NS)+1)))
            elapsed = phase['wall_total_ns']
            totals['jobs'] += 1; totals['calls'] += phase['calls']
            totals['wall_ns'] += elapsed
            totals['thread_cpu_ns'] += phase['thread_cpu_total_ns']
            totals['max_job_wall_ns'] = max(totals['max_job_wall_ns'], elapsed)
            bucket=0
            while bucket<len(HIST_LIMITS_NS) and elapsed>HIST_LIMITS_NS[bucket]:bucket+=1
            totals['wall_bins'][bucket]+=1
        self.completed.append(self.active)
        self.active = None

    @_safe
    def eof(self):
        self.eof_read, self.active = self.active, None
        if self.eof_read is not None:
            self.eof_read['state'] = 'eof'
            self.eof_read['sequence'] = None
            self.sequence -= 1

    @_safe
    def snapshot(self):
        return dict(schema='r93_worker_timeline/v1', enabled=True,
            identity=self.identity, capacity=CAPACITY, evicted=self.evicted,
            completed_phase_totals=self.completed_phase_totals,
            histogram_limits_ns=HIST_LIMITS_NS,
            totals_scope='all finished local jobs; per-job phase sum, not every individual call',
            observation_errors=self.observation_errors, sequence=self.sequence,
            completed=list(self.completed), active=self.active,
            background=self.background, eof_read=self.eof_read,
            exit_write_attempts=self.exit_write_attempts,
            scope='local call brackets; read/write may overlap across processes',
            limitations=['exit write uses original close grace',
                        'forced termination may leave no child file',
                        'write/flush return does not establish parent consumption'])

    @_safe
    def write_exit(self, error=None):
        if self.exit_write_attempts:
            return
        self.exit_write_attempts = 1
        if error is not None:
            self.error(error)
        snapshot = self.snapshot()
        if snapshot is None:
            return
        snapshot['exit_reason'] = 'python_exception' if error is not None else 'returned'
        # One best-effort write, no fsync, no per-job file I/O. Existing run only.
        (self.run / 'r93_worker_timeline.json').write_text(
            json.dumps(snapshot, allow_nan=False, separators=(',', ':')) + '\n')


def create_probe(role, run, worker_path):
    """Configuration is checked by the caller; runtime observation is optional."""
    try:
        return Probe(role, run, worker_path)
    except Exception:
        return None
