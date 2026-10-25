"""Bounded caller-owned execution samples; no hot-path I/O or control authority.

Two evidence levels:
- sample(): optional native TID, thread/process CPU and RUSAGE_THREAD counters
  (original behaviour, always available through the four stage markers).
- guard_window_begin/end(): default-OFF window around the native geometry guard.
  It snapshots per-thread /proc CPU/scheduler counters and gc counters at both
  ends, and arms CPython's C-level faulthandler watchdog so that only a guard
  longer than 50 ms dumps thread stacks (repeat 50 ms, capped at 100 blocks /
  256 KiB). No Python thread is added, no guard math/cache/protection changes.
"""
import faulthandler
import gc
import os
import tempfile
import threading
import time

try:
    import resource
except ImportError:
    resource = None

STAGES = frozenset((
    'native_geometry_guard_before', 'native_geometry_guard_after',
    'remember_native_guard_before', 'remember_native_guard_after',
))
GUARD_STAGES = frozenset(('native_geometry_guard_before', 'native_geometry_guard_after'))
ENV_KEY = 'R86_GUARD_WINDOW_PROBE'
ENABLED = os.environ.get(ENV_KEY, '0') == '1'
if os.environ.get(ENV_KEY, '0') not in ('0', '1'):
    raise ValueError(ENV_KEY + ' must be 0 or 1')
GUARD_FIRST_DUMP_S = .05
GUARD_DUMP_MAX_BYTES = 256 * 1024
GUARD_DUMP_MAX_BLOCKS = 100
MAX_THREADS = 16
RECENT_WINDOWS = 16
_recent = []
_armed_window = None


def sample():
    """Keep partial evidence on optional-counter failure; never replace clocks."""
    row = {'schema': 'r86_execution_sample/v1', 'wall_begin_ns': time.monotonic_ns()}
    failures = {}
    for name, reader in (
        ('native_tid', threading.get_native_id),
        ('thread_cpu_ns', time.thread_time_ns),
        ('process_cpu_ns', time.process_time_ns),
    ):
        try:
            row[name] = reader()
        except Exception as error:
            row[name] = None
            failures[name] = type(error).__name__
    try:
        if resource is None or not hasattr(resource, 'RUSAGE_THREAD'):
            raise NotImplementedError('RUSAGE_THREAD unavailable')
        usage = resource.getrusage(resource.RUSAGE_THREAD)
        row['thread_usage'] = {name: getattr(usage, 'ru_' + name) for name in
            ('nvcsw', 'nivcsw', 'minflt', 'majflt', 'inblock', 'oublock')}
    except Exception as error:
        row['thread_usage'] = None
        failures['thread_usage'] = type(error).__name__
    row['wall_end_ns'] = time.monotonic_ns()
    row['errors'] = failures
    return row


def _threads():
    """Per-thread CPU from stat and switch counters from status.

    The /proc/<tid>/stat tail layout differs across kernels, so only the two
    documented, empirically verified CPU fields are parsed there. Context
    switches are read from the self-describing /proc/<tid>/status keys.
    """
    rows = {}
    try:
        tids = sorted(os.listdir('/proc/self/task'), key=int)
    except OSError:
        return None
    for name in tids[:MAX_THREADS]:
        try:
            with open('/proc/self/task/' + name + '/stat', 'rb') as stream:
                raw = stream.read()
            fields = raw[raw.rindex(b')') + 2:].split()
            # After comm: state=0 ... utime=11 stime=12 (verified by CPU delta).
            row = dict(state=fields[0].decode('ascii'),
                       utime_ticks=int(fields[11]), stime_ticks=int(fields[12]),
                       nvcsw=None, nivcsw=None)
            try:
                with open('/proc/self/task/' + name + '/status') as stream:
                    for line in stream:
                        if line.startswith('voluntary_ctxt_switches:'):
                            row['nvcsw'] = int(line.split(':', 1)[1])
                        elif line.startswith('nonvoluntary_ctxt_switches:'):
                            row['nivcsw'] = int(line.split(':', 1)[1])
            except (OSError, ValueError):
                pass
            rows[int(name)] = row
        except (OSError, ValueError, IndexError):
            continue
    return rows


def _gc():
    try:
        return dict(count=list(gc.get_count()),
                    stats=[dict(collections=row['collections'],
                                collected=row['collected'],
                                uncollectable=row['uncollectable'])
                           for row in gc.get_stats()],
                    enabled=gc.isenabled())
    except Exception:
        return None


def _ticks_per_second():
    try:
        return float(os.sysconf('SC_CLK_TCK'))
    except (OSError, ValueError, AttributeError):
        return 100.


def _thread_delta(before, after, ticks):
    delta = {}
    if not before or not after:
        return None
    for tid, row in after.items():
        old = before.get(tid)
        if old is None:
            continue
        nvcsw = (None if row['nvcsw'] is None or old['nvcsw'] is None
                 else row['nvcsw'] - old['nvcsw'])
        nivcsw = (None if row['nivcsw'] is None or old['nivcsw'] is None
                  else row['nivcsw'] - old['nivcsw'])
        delta[str(tid)] = dict(
            cpu_ns=int(round((row['utime_ticks'] + row['stime_ticks'] -
                              old['utime_ticks'] - old['stime_ticks']) * 1e9 / ticks)),
            nvcsw=nvcsw, nivcsw=nivcsw, state=row['state'])
    return delta


def _gc_delta(before, after):
    if not before or not after:
        return None
    return dict(collections=[after['stats'][i]['collections'] - before['stats'][i]['collections']
                             for i in range(3)],
                collected=[after['stats'][i]['collected'] - before['stats'][i]['collected']
                           for i in range(3)])


def _parse_dump(raw):
    """Bound the C watchdog output to scalar block headers and a byte cap."""
    text = raw[:GUARD_DUMP_MAX_BYTES].decode('utf-8', 'replace')
    blocks = [line for line in text.splitlines() if line.startswith('Thread 0x')]
    return dict(dump_bytes=min(len(raw), GUARD_DUMP_MAX_BYTES),
                truncated=len(raw) > GUARD_DUMP_MAX_BYTES,
                block_count=min(len(blocks), GUARD_DUMP_MAX_BLOCKS),
                blocks=blocks[:GUARD_DUMP_MAX_BLOCKS],
                text=None if len(blocks) > GUARD_DUMP_MAX_BLOCKS else text)


def guard_window_begin():
    """Arm the slow-guard window; off by default and never re-entrant."""
    global _armed_window
    if not ENABLED or _armed_window is not None:
        return None
    try:
        handle = tempfile.TemporaryFile(prefix='r86_guard_stacks_', suffix='.log')
        window = dict(started_ns=time.monotonic_ns(), thread_cpu_begin_ns=time.thread_time_ns(),
                      handle=handle, threads=_threads(), gc=_gc(),
                      ticks=_ticks_per_second(), armed=False, error=None)
        try:
            faulthandler.dump_traceback_later(GUARD_FIRST_DUMP_S, repeat=True,
                                              file=handle)
            window['armed'] = True
        except Exception as error:
            window['error'] = type(error).__name__
        _armed_window = window
        return window
    except Exception:
        return None


def guard_window_end(window, call=None):
    """Cancel and summarise one guard window; bounded memory and no retry."""
    global _armed_window
    if window is None:
        return None
    if _armed_window is window:
        _armed_window = None
    summary = dict(schema='r86_guard_window/v1', armed=bool(window.get('armed')),
                   arm_error=window.get('error'))
    try:
        try:
            faulthandler.cancel_dump_traceback_later()
        except Exception as error:
            summary['cancel_error'] = type(error).__name__
        ended = time.monotonic_ns()
        summary.update(wall_ns=ended - window['started_ns'],
                       thread_cpu_ns=time.thread_time_ns() - window['thread_cpu_begin_ns'])
        threads_after, gc_after = _threads(), _gc()
        summary['thread_delta'] = _thread_delta(window['threads'], threads_after,
                                                window['ticks'])
        summary['gc_delta'] = _gc_delta(window['gc'], gc_after)
        summary['gc_after'] = gc_after
        handle = window['handle']
        if summary['armed']:
            try:
                handle.flush()
                handle.seek(0)
                summary['stacks'] = _parse_dump(handle.read(GUARD_DUMP_MAX_BYTES + 1))
            except Exception as error:
                summary['stacks'] = dict(error=type(error).__name__)
        else:
            summary['stacks'] = None
        try:
            handle.close()
        except Exception:
            pass
    except Exception as error:
        summary['error'] = type(error).__name__
    if len(_recent) >= RECENT_WINDOWS:
        _recent.pop(0)
    _recent.append(summary)
    if isinstance(call, dict):
        call['r86_guard_window'] = summary
    return summary


def recent_windows():
    return list(_recent)
