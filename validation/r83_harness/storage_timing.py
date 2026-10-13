"""Bounded best-effort scalar I/O diagnostics; no budget or health authority."""
from contextlib import contextmanager
from functools import wraps
from pathlib import Path
import fcntl
import json
import os
import threading
import time

LIMIT_BYTES = 8 * 1024 * 1024
_context = threading.local()


@contextmanager
def scope(run):
    previous = getattr(_context, 'run', None)
    _context.run = Path(run)
    try:
        yield
    finally:
        _context.run = previous


def _record(phase, started, ended):
    run = getattr(_context, 'run', None)
    if run is None:
        return
    row = dict(schema='R77_storage_io_timing/v1', phase=phase, pid=os.getpid(),
               thread=threading.current_thread().name, started_wall=started,
               ended_wall=ended, elapsed_s=ended-started,
               dropped_before=getattr(_context, 'dropped', 0),
               diagnostic_only=True, budget_authority=False)
    data = (json.dumps(row, allow_nan=False)+'\n').encode()
    fd = None
    try:
        if len(data) > 2048:
            raise ValueError('Diagnostic record oversized')
        fd = os.open(run/'R77_storage_io_timing.jsonl',
                     os.O_WRONLY | os.O_CREAT | os.O_APPEND | os.O_NOFOLLOW, 0o600)
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        value = os.fstat(fd)
        block = os.fstatvfs(fd).f_frsize
        predicted = ((value.st_size+len(data)+block-1)//block)*block
        if max(value.st_blocks*512, predicted) > LIMIT_BYTES:
            raise ValueError('Diagnostic file cap reached')
        if os.write(fd, data) != len(data):
            raise OSError('Partial diagnostic write')
    except Exception:
        _context.dropped = getattr(_context, 'dropped', 0)+1
    finally:
        if fd is not None:
            os.close(fd)


@contextmanager
def span(phase):
    try:
        started = time.monotonic()
    except Exception:
        started = None
    try:
        yield
    finally:
        try:
            if started is not None:
                _record(phase, started, time.monotonic())
        except Exception:
            pass  # Observation failure never replaces the original result/error.


def timed(phase):
    def decorate(operation):
        @wraps(operation)
        def invoke(*args, **kwargs):
            with span(phase):
                return operation(*args, **kwargs)
        return invoke
    return decorate
