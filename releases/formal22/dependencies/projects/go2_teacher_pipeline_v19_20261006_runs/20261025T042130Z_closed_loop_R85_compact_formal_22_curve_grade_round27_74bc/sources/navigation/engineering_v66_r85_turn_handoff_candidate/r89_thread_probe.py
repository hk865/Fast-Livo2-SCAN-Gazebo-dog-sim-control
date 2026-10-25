"""Default-off, self-process-only bounded thread CPU and Python stack evidence.

The observer has no control authority. Its caller supplies an exit-only JSON
sink; this module never writes files, prints, changes affinity or thread limits,
or reads another process. Python stacks contain code locations, never locals.
Sample timestamps bracket each read: these are not simultaneous CPU snapshots.
Linux schedstat runqueue wait excludes sleeping/blocked time, and a Python frame
inside a native call does not establish GIL ownership or identify native frames.
"""
from collections import deque
from contextlib import contextmanager
import importlib
import os
import sys
import threading
import time

KEY = 'R89_THREAD_PROBE'
CAPACITY = 512
MAX_CAPACITY = 1024
MAX_THREADS = 64
MAX_FRAMES = 12
POST_SAMPLES = 5
INTERVAL_NS = 50_000_000
SLOW_ZONE_NS = 200_000_000
ZONE_CAPACITY = 1024
REGISTRY_MAX_AGE_NS = 2_000_000_000
_TASK_ROOT = '/proc/self/task'
_READ_LIMIT = 16_384
_ERROR_KINDS = ('denied', 'gone', 'invalid', 'other')
_ERROR_FILES = ('task_list', 'stat', 'schedstat', 'status', 'wchan', 'frames',
                'thread_map', 'sample', 'startup', 'thread_start', 'zone', 'snapshot')


def _text(value, limit):
    return value[:limit] if isinstance(value, str) else str(value)[:limit]


def _read(path, limit=_READ_LIMIT):
    # All production callers use literal /proc/self paths. No arbitrary PID API.
    with open(path, 'r', encoding='utf-8', errors='replace') as stream:
        value = stream.read(limit + 1)
    if len(value) > limit:
        raise ValueError('bounded proc read exceeded')
    return value


def _stat(value):
    # Linux comm can contain spaces and parentheses; the final ')' terminates it.
    end = value.rfind(')')
    begin = value.find('(')
    if begin < 0 or end <= begin:
        raise ValueError('invalid task stat')
    fields = value[end + 2:].split()
    if len(fields) < 20:
        raise ValueError('short task stat')
    return dict(comm=value[begin + 1:end][:80], state=fields[0][:1],
                user_ticks=int(fields[11]), system_ticks=int(fields[12]),
                starttime_ticks=int(fields[19]))


def _status(value):
    keys = {'voluntary_ctxt_switches', 'nonvoluntary_ctxt_switches'}
    result = {}
    for line in value.splitlines():
        key, separator, tail = line.partition(':')
        if separator and key in keys:
            result[key] = int(tail.strip())
    if len(result) != 2:
        raise ValueError('missing task context switch counts')
    return result


def _schedstat(value):
    fields = value.split()
    if len(fields) != 3:
        raise ValueError('invalid task schedstat')
    run, wait, timeslices = map(int, fields)
    if min(run, wait, timeslices) < 0:
        raise ValueError('negative task schedstat')
    return dict(runtime_ns=run, runqueue_wait_ns=wait, timeslices=timeslices)


def _locations(frame):
    result = []
    while frame is not None and len(result) < MAX_FRAMES:
        code = frame.f_code
        result.append((_text(code.co_filename, 240), _text(code.co_name, 120),
                       int(frame.f_lineno)))
        frame = frame.f_back
    return result


def _thread_clock_ns(tid):
    # Linux include/linux/posix-timers.h make_thread_cpuclock(tid, CPUCLOCK_SCHED):
    # CPUCLOCK_SCHED=2, CPUCLOCK_PERTHREAD_MASK=4. Unlike pthread_getcpuclockid,
    # this does not dereference a potentially expired pthread_t. The Linux
    # kernel rejects non-current-thread-group TIDs (tested with an owned child).
    if type(tid) is not int or not 0 < tid < (1 << 28):
        raise ValueError('invalid bounded native tid')
    return time.clock_gettime_ns(((~tid) << 3) | 6)


class ThreadProbe:
    def __init__(self, *, interval_ns=INTERVAL_NS, capacity=CAPACITY,
                 max_threads=MAX_THREADS):
        if type(interval_ns) is not int or not 20_000_000 <= interval_ns <= 50_000_000:
            raise ValueError('sample interval must be 20 through 50 ms')
        if type(capacity) is not int or not 1 <= capacity <= MAX_CAPACITY:
            raise ValueError('invalid bounded sample capacity')
        if type(max_threads) is not int or not 1 <= max_threads <= MAX_THREADS:
            raise ValueError('invalid bounded thread count')
        self.pid = os.getpid()
        self.interval_ns = interval_ns
        self.capacity = capacity
        self.max_threads = max_threads
        self.clock_ticks_per_second = os.sysconf('SC_CLK_TCK')
        self.rows = deque(maxlen=capacity)
        self.frozen = None
        self.slow_window = None
        self.frozen_post = deque(maxlen=POST_SAMPLES)
        self.slow_post = deque(maxlen=POST_SAMPLES)
        self.freeze_trigger = None
        self.slow_trigger = None
        self.requests = {}
        self.zones = {}
        self.longest_zone_ns = 0
        self.guard = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = None
        self.started = self.finished = False
        self.sampler_native_tid = None
        self.attempted = self.recorded = self.evicted = self.sample_errors = 0
        self.missed_intervals = self.truncated_thread_samples = 0
        self.omitted_thread_observations = self.identity_changed = 0
        self.dropped_lock_contention = self.zone_drops = 0
        self.max_sample_wall_ns = self.max_sample_cpu_ns = 0
        self.sample_wall_ns = self.sample_cpu_ns = self.sampler_lifetime_cpu_ns = 0
        self.startup = None
        self.clock_available = False
        self.registry = ()
        self.registry_generation = 0
        self.zone_rows = deque(maxlen=ZONE_CAPACITY)
        self.frozen_zones = self.slow_zones = None
        self.frozen_zone_post = deque(maxlen=POST_SAMPLES)
        self.slow_zone_post = deque(maxlen=POST_SAMPLES)
        self.zone_recorded = self.zone_evicted = self.zone_errors = 0
        self.boundary_clock_reads = self.boundary_clock_unknown = 0
        self.boundary_wall_ns = self.boundary_cpu_ns = 0
        self.errors = {name + ':' + kind: 0 for name in _ERROR_FILES for kind in _ERROR_KINDS}

    def _error(self, name, error):
        kind = ('denied' if isinstance(error, PermissionError) else
                'gone' if isinstance(error, (FileNotFoundError, ProcessLookupError)) else
                'invalid' if isinstance(error, (ValueError, TypeError, IndexError)) else 'other')
        key = name + ':' + kind
        self.errors[key if key in self.errors else 'sample:other'] += 1

    def _startup(self):
        before, cpu = time.monotonic_ns(), time.thread_time_ns()
        result = dict(pid=self.pid, process_starttime_ticks=None, numerical_libraries=[],
                      libraries_truncated=False, threadpools=[], threadpool_status=None,
                      thread_limit_environment={}, changed_thread_limits=False)
        try:
            before_clock = time.thread_time_ns()
            linux_clock = _thread_clock_ns(threading.get_native_id()) if sys.platform == 'linux' else -1
            after_clock = time.thread_time_ns()
            self.clock_available = before_clock <= linux_clock <= after_clock
            result['thread_cpu_clock_self_check'] = dict(available=self.clock_available,
                before_ns=before_clock, encoded_clock_ns=linux_clock, after_ns=after_clock,
                encoding='Linux make_thread_cpuclock: ((~tid)<<3)|6')
        except Exception as error:
            result['thread_cpu_clock_self_check'] = dict(available=False, error=type(error).__name__)
        try:
            result['process_starttime_ticks'] = _stat(_read('/proc/self/stat'))['starttime_ticks']
        except Exception as error:
            self._error('startup', error)
        for name in ('OMP_NUM_THREADS', 'OPENBLAS_NUM_THREADS', 'MKL_NUM_THREADS',
                     'NUMEXPR_NUM_THREADS', 'BLIS_NUM_THREADS', 'VECLIB_MAXIMUM_THREADS'):
            result['thread_limit_environment'][name] = _text(os.environ.get(name, ''), 80)
        try:
            names = set()
            with open('/proc/self/maps', 'r', encoding='utf-8', errors='replace') as stream:
                # Bounded startup-only read. Store only numerical/native-library paths.
                for index, line in enumerate(stream):
                    if index >= 4096:
                        result['libraries_truncated'] = True
                        break
                    fields = line.rstrip().split(None, 5)
                    if len(fields) != 6:
                        continue
                    path = fields[-1]
                    lower = path.lower()
                    if '.so' in lower and any(key in lower for key in
                            ('blas', 'lapack', 'mkl', 'libomp', 'libgomp', 'blis', 'numexpr')):
                        if len(names) < 64:
                            names.add(path[:240])
                        else:
                            result['libraries_truncated'] = True
            result['numerical_libraries'] = sorted(names)
        except Exception as error:
            self._error('startup', error)
        try:
            # Introspection only. Does not import numpy or call threadpool_limits.
            module = importlib.import_module('threadpoolctl')
            pools = module.threadpool_info()
            result['threadpool_status'] = 'available'
            result['threadpools_truncated'] = len(pools) > 32
            allowed = ('user_api', 'internal_api', 'num_threads', 'prefix', 'filepath',
                       'version', 'threading_layer', 'architecture')
            for pool in pools[:32]:
                result['threadpools'].append({key: (value if type(value) in (int, bool)
                    or value is None else _text(value, 240)) for key in allowed
                    if (value := pool.get(key)) is not None})
        except Exception as error:
            self._error('startup', error)
            result['threadpool_status'] = type(error).__name__
        result.update(wall_ns=time.monotonic_ns() - before,
                      thread_cpu_ns=time.thread_time_ns() - cpu,
                      captured_once=True, configuration_can_change_after_startup=True)
        return result

    def start(self):
        if self.pid != os.getpid() or self.started:
            return False
        self.started = True
        try:
            self.startup = self._startup()
            self.thread = threading.Thread(target=self._run, name='r89-thread-probe', daemon=True)
            self.thread.start()
            return True
        except Exception as error:
            self._error('thread_start', error)
            self.finished = True
            return False

    def _sample(self, scheduled_ns):
        before, cpu = time.monotonic_ns(), time.thread_time_ns()
        self.attempted += 1
        row = dict(sequence=self.attempted, wall_begin_ns=before, scheduled_ns=scheduled_ns,
                   process_cpu_begin_ns=time.process_time_ns(), threads=[])
        try:
            tids, total = [], 0
            with os.scandir(_TASK_ROOT) as entries:
                for entry in entries:
                    if entry.name.isdecimal():
                        total += 1
                        if len(tids) < self.max_threads:
                            tids.append(int(entry.name))
            row['observed_task_count'] = total
            row['omitted_task_count'] = max(0, total - len(tids))
            self.omitted_thread_observations += row['omitted_task_count']
            self.truncated_thread_samples += int(row['omitted_task_count'] > 0)
        except Exception as error:
            self._error('task_list', error)
            tids = []
            row.update(observed_task_count=None, omitted_task_count=None)
        # Establish task creation identity before Python identity/stack capture.
        # Final stat below rejects exit/reuse during the entire capture window.
        initial = {}
        for tid in tids:
            item = dict(native_tid=tid, wall_begin_ns=time.monotonic_ns())
            try:
                item.update(_stat(_read(_TASK_ROOT + '/' + str(tid) + '/stat')))
                item['stat_begin_read_end_ns'] = time.monotonic_ns()
                initial[tid] = item
            except Exception as error:
                self._error('stat', error)
        mapping = {}
        row['thread_map_capture_begin_ns'] = time.monotonic_ns()
        try:
            selected = set(initial)
            for thread in threading.enumerate():
                if thread.native_id in selected:
                    mapping[thread.native_id] = (thread.ident, _text(thread.name, 80))
        except Exception as error:
            self._error('thread_map', error)
        row['thread_map_capture_end_ns'] = time.monotonic_ns()
        locations = {}
        active_zones = {}
        row['python_frames_capture_begin_ns'] = time.monotonic_ns()
        try:
            frames = sys._current_frames()
            try:
                row['zones_capture_begin_ns'] = time.monotonic_ns()
                active_zones = dict(self.zones)
                row['zones_capture_end_ns'] = time.monotonic_ns()
                for ident, _ in mapping.values():
                    if ident in frames:
                        locations[ident] = _locations(frames[ident])
            finally:
                del frames  # Never retain frames or anything reachable via frame locals.
        except Exception as error:
            self._error('frames', error)
        row['python_frames_capture_end_ns'] = time.monotonic_ns()
        for tid, item in initial.items():
            base = _TASK_ROOT + '/' + str(tid) + '/'
            ident, role = mapping.get(tid, (None, None))
            item.update(python_ident=ident, python_thread_name=role,
                        python_stack=locations.get(ident, []))
            active = active_zones.get(tid)
            if active is not None and active['python_ident'] == ident:
                item['zone'] = active
            for name, parser in (('schedstat', _schedstat), ('status', _status),
                                 ('wchan', lambda value: value.strip()[:80])):
                item[name + '_wall_begin_ns'] = time.monotonic_ns()
                try:
                    item[name] = parser(_read(base + name))
                except Exception as error:
                    item[name] = None
                    self._error(name, error)
                finally:
                    item[name + '_wall_end_ns'] = time.monotonic_ns()
            try:
                item['stat_end_read_begin_ns'] = time.monotonic_ns()
                check = _stat(_read(base + 'stat'))
                if check['starttime_ticks'] != item['starttime_ticks']:
                    self.identity_changed += 1
                    continue
                item['cpu_end'] = {key: check[key] for key in ('user_ticks', 'system_ticks')}
            except Exception as error:
                self._error('stat', error)
                continue
            item['wall_end_ns'] = time.monotonic_ns()
            row['threads'].append(item)
        row.update(wall_end_ns=time.monotonic_ns(), process_cpu_end_ns=time.process_time_ns(),
                   sampler_thread_cpu_ns=time.thread_time_ns() - cpu)
        self.registry_generation += 1
        self.registry = tuple(dict(native_tid=item['native_tid'],
            starttime_ticks=item['starttime_ticks'], confirmed_wall_ns=item['wall_end_ns'],
            identity_read_begin_ns=item['wall_begin_ns'],
            python_ident=item['python_ident'], python_thread_name=item['python_thread_name'],
            registry_generation=self.registry_generation) for item in row['threads'])
        self._record(row)
        wall, spent = time.monotonic_ns() - before, time.thread_time_ns() - cpu
        self.sample_wall_ns += wall
        self.sample_cpu_ns += spent
        self.max_sample_wall_ns = max(self.max_sample_wall_ns, wall)
        self.max_sample_cpu_ns = max(self.max_sample_cpu_ns, spent)

    def _record(self, row):
        if not self.guard.acquire(False):
            self.dropped_lock_contention += 1
            return
        try:
            self.evicted += int(len(self.rows) == self.capacity)
            self.rows.append(row)
            self.recorded += 1
            request = self.requests.get('freeze')
            if request is not None and self.frozen is None:
                self.frozen = tuple(self.rows)
                self.frozen_zones = tuple(self.zone_rows)
                self.freeze_trigger = dict(request, capture_wall_ns=time.monotonic_ns())
            elif self.frozen is not None and len(self.frozen_post) < POST_SAMPLES:
                self.frozen_post.append(row)
            slow = self.requests.get('slow')
            if slow is not None and (self.slow_trigger is None or
                                     slow['duration_ns'] > self.slow_trigger['duration_ns']):
                self.slow_window = tuple(self.rows)
                self.slow_zones = tuple(self.zone_rows)
                self.slow_zone_post.clear()
                self.slow_trigger = dict(slow, capture_wall_ns=time.monotonic_ns())
                self.slow_post.clear()
            elif self.slow_window is not None and len(self.slow_post) < POST_SAMPLES:
                self.slow_post.append(row)
        finally:
            self.guard.release()

    def _run(self):
        lifetime_cpu = time.thread_time_ns()
        self.sampler_native_tid = threading.get_native_id()
        deadline = time.monotonic_ns()
        try:
            while not self.stop_event.is_set():
                try:
                    self._sample(deadline)
                except Exception as error:
                    self.sample_errors += 1
                    self._error('sample', error)
                deadline += self.interval_ns
                now = time.monotonic_ns()
                if now > deadline:
                    skipped = (now - deadline) // self.interval_ns + 1
                    self.missed_intervals += skipped
                    deadline += skipped * self.interval_ns
                self.stop_event.wait(max(0, deadline - time.monotonic_ns()) / 1e9)
        finally:
            self.sampler_lifetime_cpu_ns = time.thread_time_ns() - lifetime_cpu
            self.finished = True

    def freeze(self, reason, source_ns=None):
        try:
            if self.pid == os.getpid():
                self.requests.setdefault('freeze', dict(reason=_text(reason, 240),
                    source_ns=source_ns if type(source_ns) is int else None,
                    request_wall_ns=time.monotonic_ns()))
        except Exception as error:
            self._error('zone', error)

    def _cpu_snapshot(self):
        # Bounded no-I/O zone endpoint. Only a previously confirmed self-task
        # registry may reach this private Linux clock call; no caller-supplied
        # PID/TID parameter exists. CPU clock reads do not atomically validate
        # task starttime, so that residual identity limitation stays explicit.
        before, own_cpu = time.monotonic_ns(), time.thread_time_ns()
        registry = self.registry
        result = dict(wall_begin_ns=before, process_cpu_begin_ns=time.process_time_ns(),
            registry_generation=self.registry_generation, registered_tasks=len(registry),
            registry_max_age_ns=REGISTRY_MAX_AGE_NS, threads=[],
            clock_available=self.clock_available, identity_at_clock_read_unverified=True,
            coverage='bounded latest self-task registry; newly created tasks may be absent')
        if self.clock_available:
            for entry in registry[:self.max_threads]:
                item = dict(entry)
                item['wall_begin_ns'] = time.monotonic_ns()
                try:
                    if item['wall_begin_ns'] - entry['confirmed_wall_ns'] > REGISTRY_MAX_AGE_NS:
                        item['unknown'] = 'stale_registry'
                        self.boundary_clock_unknown += 1
                    else:
                        item['cpu_ns'] = _thread_clock_ns(entry['native_tid'])
                        self.boundary_clock_reads += 1
                except Exception as error:
                    item['unknown'] = type(error).__name__
                    self.boundary_clock_unknown += 1
                item['wall_end_ns'] = time.monotonic_ns()
                result['threads'].append(item)
        result.update(wall_end_ns=time.monotonic_ns(), process_cpu_end_ns=time.process_time_ns(),
                      caller_thread_cpu_ns=time.thread_time_ns()-own_cpu)
        self.boundary_wall_ns += result['wall_end_ns']-before
        self.boundary_cpu_ns += result['caller_thread_cpu_ns']
        return result

    @contextmanager
    def zone(self, stage, source_ns=None):
        previous = active = None
        cpu_enter = None
        body_entered = None
        tid = None
        try:
            if self.pid == os.getpid():
                tid = threading.get_native_id()
                previous = self.zones.get(tid)
                if previous is not None or len(self.zones) < self.max_threads:
                    active = dict(stage=_text(stage, 120),
                        source_ns=source_ns if type(source_ns) is int else None,
                        python_ident=threading.get_ident(), native_tid=tid,
                        entered_wall_ns=time.monotonic_ns())
                    self.zones[tid] = active
                    cpu_enter = self._cpu_snapshot()
                    body_entered = time.monotonic_ns()
                else:
                    self.zone_drops += 1
        except Exception as error:
            self._error('zone', error)
        try:
            yield
        finally:
            try:
                if active is not None:
                    end = time.monotonic_ns()
                    duration = end - active['entered_wall_ns']
                    try:
                        cpu_exit = self._cpu_snapshot()
                        self.zone_evicted += int(len(self.zone_rows) == ZONE_CAPACITY)
                        completed = dict(active, exited_wall_ns=end,
                            duration_ns=duration, body_entered_wall_ns=body_entered,
                            body_exited_wall_ns=end,
                            body_duration_ns=None if body_entered is None else end-body_entered,
                            cpu_enter=cpu_enter, cpu_exit=cpu_exit)
                        self.zone_rows.append(completed)
                        if self.frozen_zones is not None and len(self.frozen_zone_post) < POST_SAMPLES:
                            self.frozen_zone_post.append(completed)
                        if self.slow_zones is not None and len(self.slow_zone_post) < POST_SAMPLES:
                            self.slow_zone_post.append(completed)
                        self.zone_recorded += 1
                    except Exception:
                        self.zone_errors += 1
                    if duration >= SLOW_ZONE_NS and duration > self.longest_zone_ns:
                        self.longest_zone_ns = duration
                        self.requests['slow'] = dict(active, exited_wall_ns=end, duration_ns=duration)
                    if self.zones.get(tid) is active:
                        if previous is None:
                            self.zones.pop(tid, None)
                        else:
                            self.zones[tid] = previous
            except Exception as error:
                self._error('zone', error)

    def status(self):
        # Scalar-only, nonblocking and safe to call while the sampler runs.
        return dict(schema='r89_bounded_self_thread_evidence/v1', enabled=True, pid=self.pid,
            started=self.started, finished=self.finished,
            running=self.thread is not None and self.thread.is_alive(),
            sampler_native_tid=self.sampler_native_tid, interval_ns=self.interval_ns,
            capacity=self.capacity, max_threads=self.max_threads, max_frames=MAX_FRAMES,
            post_samples=POST_SAMPLES, clock_ticks_per_second=self.clock_ticks_per_second,
            attempted=self.attempted, recorded=self.recorded, evicted=self.evicted,
            missed_intervals=self.missed_intervals, sample_errors=self.sample_errors,
            dropped_lock_contention=self.dropped_lock_contention,
            truncated_thread_samples=self.truncated_thread_samples,
            omitted_thread_observations=self.omitted_thread_observations,
            identity_changed=self.identity_changed, zone_drops=self.zone_drops,
            sample_wall_ns=self.sample_wall_ns, sample_cpu_ns=self.sample_cpu_ns,
            max_sample_wall_ns=self.max_sample_wall_ns, max_sample_cpu_ns=self.max_sample_cpu_ns,
            sampler_lifetime_cpu_ns=self.sampler_lifetime_cpu_ns,
            errors=dict(self.errors), longest_zone_ns=self.longest_zone_ns,
            cpu_clock_available=self.clock_available, zone_capacity=ZONE_CAPACITY,
            zone_recorded=self.zone_recorded, zone_evicted=self.zone_evicted,
            zone_errors=self.zone_errors, boundary_clock_reads=self.boundary_clock_reads,
            boundary_clock_unknown=self.boundary_clock_unknown,
            boundary_wall_ns=self.boundary_wall_ns, boundary_cpu_ns=self.boundary_cpu_ns,
            no_payloads_or_locals=True, no_disk_writes=True,
            target='self process only', configured_maximum_retained_samples=3*self.capacity+2*POST_SAMPLES,
            configured_maximum_retained_zone_records=3*ZONE_CAPACITY+2*POST_SAMPLES,
            timestamp_basis='Each sample and each task read is bracketed; cross-thread counters are not simultaneous.',
            cpu_basis='stat ticks and schedstat cumulative task CPU; sum of thread CPU may exceed elapsed wall time.',
            attribution_limit='Python code locations do not identify native stacks or GIL ownership. schedstat wait is runnable wait, not blocked time.')

    def stop(self, timeout=2.0):
        # Exit-only. A timeout is evidence, never a license to kill a thread.
        self.stop_event.set()
        try:
            if self.thread is not None and self.thread is not threading.current_thread():
                self.thread.join(timeout=max(0., min(5., float(timeout))))
        except Exception as error:
            self._error('snapshot', error)
        result = self.status()
        result['startup'] = self.startup
        result['stop_complete'] = not result['running']
        if not self.guard.acquire(False):
            result['snapshot_unavailable_lock_contention'] = True
            return result
        try:
            # A stop may arrive before the next scheduled sample. Preserve its
            # request and existing pre-window without collecting on this thread.
            if self.frozen is None and 'freeze' in self.requests:
                self.frozen = tuple(self.rows)
                self.frozen_zones = tuple(self.zone_rows)
                self.freeze_trigger = dict(self.requests['freeze'],
                                           capture_wall_ns=time.monotonic_ns(), exit_capture=True)
            slow = self.requests.get('slow')
            if slow and (self.slow_trigger is None or
                         slow['duration_ns'] > self.slow_trigger['duration_ns']):
                self.slow_window = tuple(self.rows)
                self.slow_zones = tuple(self.zone_rows)
                self.slow_zone_post.clear()
                self.slow_trigger = dict(slow, capture_wall_ns=time.monotonic_ns(), exit_capture=True)
                self.slow_post.clear()
            result.update(rows=list(self.rows), anomaly_window=list(self.frozen or ()),
                anomaly_post=list(self.frozen_post), anomaly_trigger=self.freeze_trigger,
                slow_window=list(self.slow_window or ()), slow_post=list(self.slow_post),
                slow_trigger=self.slow_trigger, zone_rows=list(self.zone_rows),
                anomaly_zones=list(self.frozen_zones or ()), slow_zones=list(self.slow_zones or ()),
                anomaly_zone_post=list(self.frozen_zone_post), slow_zone_post=list(self.slow_zone_post))
        except Exception as error:
            self._error('snapshot', error)
            result['snapshot_error'] = type(error).__name__
        finally:
            self.guard.release()
        return result


PROBE = None
_start_attempted = False
_start_error = None


def enabled():
    return os.environ.get(KEY, '0') == '1'


def start():
    """Explicit post-node-construction startup; importing creates no thread."""
    global PROBE, _start_attempted, _start_error
    if _start_attempted:
        return status()
    _start_attempted = True
    if not enabled():
        return status()
    try:
        PROBE = ThreadProbe()
        PROBE.start()
    except Exception as error:
        _start_error = type(error).__name__
    return status()


def freeze(reason, source_ns=None):
    if PROBE is not None:
        try:
            PROBE.freeze(reason, source_ns)
        except Exception:
            pass


@contextmanager
def zone(stage, source_ns=None):
    probe = PROBE
    if probe is None:
        yield
    else:
        with probe.zone(stage, source_ns):
            yield


def status():
    try:
        if PROBE is not None:
            return PROBE.status()
    except Exception as error:
        return dict(enabled=True, status_error=type(error).__name__, running=None)
    return dict(enabled=False, switch=KEY, start_attempted=_start_attempted,
                start_error=_start_error, invalid_switch=os.environ.get(KEY, '0') not in ('0', '1'))


def stop(timeout=2.0):
    if PROBE is not None:
        try:
            return PROBE.stop(timeout)
        except Exception as error:
            return dict(enabled=True, stop_error=type(error).__name__, stop_complete=False)
    return status()
