"""Default-OFF bounded scalar feedback trace; no ROS actions or hot-path I/O.

Local identity is (run, boot, process-start, source_ns). token_namespace is
registered from already verified bundle tokens; it is never inferred by stamp.
first256 + tail1024 + first-trigger pre512/post512; all arrays preallocated.
"""
from array import array
import json
import os
from pathlib import Path
import threading
import time

KEY = 'R118_FEEDBACK_EVIDENCE'
FIRST, TAIL, WINDOW, PRE = 256, 1024, 1024, 512
COLUMNS = ('seq', 'stage', 'source_ns', 'mono_ns', 'thread_cpu_ns', 'tid',
           'role', 'arrival_ns', 'prepared_ns', 'generation', 'token_namespace',
           'token_sequence', 'batch_id', 'ready_count', 'retained_bytes', 'outcome')
STAGES = ('unused', 'receive', 'parts_joined', 'ready', 'batch_member',
          'commit_enter', 'commit_exit', 'committed', 'retired', 'release', 'fault', 'external_trigger')
LIMITS = (100000, 1000000, 5000000, 10000000, 20000000, 50000000,
          100000000, 200000000, 300000000, 500000000, 1000000000)
WIDTH = len(COLUMNS)
RECORDER = None


def _identity():
    raw = Path('/proc/self/stat').read_text()
    return dict(pid=os.getpid(), start_ticks=int(raw[raw.rfind(')')+2:].split()[19]),
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())


class Recorder:
    def __init__(self):
        self.run = Path(os.environ['DEMO_RUN_DIR'])
        if not self.run.is_absolute() or not self.run.is_dir():
            raise ValueError('existing absolute DEMO_RUN_DIR required')
        self.identity = _identity()
        self.ring = array('q', [0]) * ((FIRST + TAIL) * WIDTH)
        self.window = array('q', [0]) * (WINDOW * WIDTH)
        self.external_window = array('q', [0]) * (WINDOW * WIDTH)
        self.external_rows = self.external_pre = self.external_post = 0
        self.external_trigger = None
        self.lock = threading.Lock()
        self.recorded = self.attempted = self.dropped = self.errors = 0
        self.window_rows = self.window_pre = self.window_post = 0
        self.trigger_info = None
        self.stage_counts = [0] * len(STAGES)
        self.namespaces = []  # at most 64 already parsed token strings
        self.namespace_overflow = 0
        self.cost_wall_ns = self.cost_thread_cpu_ns = self.cost_wall_max_ns = 0
        self.distributions = {k: dict(n=0, sum_ns=0, max_ns=0, negative=0,
                                     bins=[0]*(len(LIMITS)+1))
                              for k in ('intake_to_prepare', 'ready_residence', 'intake_to_commit')}
        self.closed = False

    def offset(self, seq):
        return (seq if seq < FIRST else FIRST+(seq-FIRST)%TAIL)*WIDTH

    def duration(self, key, value):
        d = self.distributions[key]
        if value < 0:
            d['negative'] += 1
            return
        d['n'] += 1; d['sum_ns'] += value; d['max_ns'] = max(d['max_ns'], value)
        bucket = 0
        while bucket < len(LIMITS) and value > LIMITS[bucket]:bucket += 1
        d['bins'][bucket] += 1

    def freeze(self, source_ns, reason, mono):
        if self.trigger_info is not None:return
        self.trigger_info = dict(source_ns=source_ns, reason_code=reason, mono_ns=mono,
                                 scope='first trigger only, no complete-run claim')
        # At most 512 * 16 scalar copies, only on first trigger; no payload.
        start = max(0, self.recorded-PRE)
        for seq in range(start, self.recorded):
            dst = self.window_rows*WIDTH; src = self.offset(seq)
            self.window[dst:dst+WIDTH] = self.ring[src:src+WIDTH]
            self.window_rows += 1
        self.window_pre = self.window_rows

    def mark(self, stage, source_ns=-1, *, bundle=None, role=-1, arrival_ns=-1,
             batch_id=-1, ready_count=-1, retained_bytes=-1, outcome=0):
        if self.closed:return
        wall, cpu = time.monotonic_ns(), time.thread_time_ns()
        self.attempted += 1
        if not self.lock.acquire(False):
            self.dropped += 1
            return
        try:
            prepared = generation = namespace = token_seq = -1
            if bundle is not None:
                source_ns, prepared, generation = bundle.source_ns, bundle.prepared_ns, bundle.generation
                arrival_ns = min(bundle.arrivals_ns)
                token = bundle.token
                if type(token) is tuple and len(token) == 2 and type(token[0]) is str:
                    if token[0] not in self.namespaces:
                        if len(self.namespaces) < 64:self.namespaces.append(token[0])
                        else:self.namespace_overflow += 1
                    if token[0] in self.namespaces:namespace = self.namespaces.index(token[0])
                    if type(token[1]) is int:token_seq = token[1]
                if stage == 3:self.duration('intake_to_prepare', prepared-arrival_ns)
                if stage == 4:
                    residence = wall-prepared
                    self.duration('ready_residence', residence)
                    if residence >= 50_000_000:self.freeze(source_ns, 1, wall)
                if stage == 7:self.duration('intake_to_commit', wall-arrival_ns)
            if stage == 11 and self.external_trigger is None:
                self.external_trigger = dict(source_ns=source_ns, reason_code=outcome, mono_ns=wall)
                for seq in range(max(0,self.recorded-PRE),self.recorded):
                    src=self.offset(seq); dst=self.external_rows*WIDTH
                    self.external_window[dst:dst+WIDTH]=self.ring[src:src+WIDTH]
                    self.external_rows+=1
                self.external_pre=self.external_rows
            if stage in (8, 10, 11):self.freeze(source_ns, stage, wall)
            values = (self.recorded+1, stage, source_ns, wall, cpu, threading.get_native_id(),
                      role, arrival_ns, prepared, generation, namespace, token_seq,
                      batch_id, ready_count, retained_bytes, outcome)
            offset = self.offset(self.recorded)
            for index,value in enumerate(values):
                self.ring[offset+index] = value if type(value) is int and -(1<<63)<=value<(1<<63) else -1
            if self.trigger_info is not None and self.window_post < WINDOW-PRE:
                dst = self.window_rows*WIDTH
                self.window[dst:dst+WIDTH] = self.ring[offset:offset+WIDTH]
                self.window_rows += 1; self.window_post += 1
            if self.external_trigger is not None and self.external_post < WINDOW-PRE:
                dst=self.external_rows*WIDTH
                self.external_window[dst:dst+WIDTH]=self.ring[offset:offset+WIDTH]
                self.external_rows+=1; self.external_post+=1
            self.recorded += 1; self.stage_counts[stage] += 1
        except Exception:
            self.errors += 1
        finally:
            elapsed = time.monotonic_ns()-wall
            self.cost_wall_ns += elapsed; self.cost_thread_cpu_ns += time.thread_time_ns()-cpu
            self.cost_wall_max_ns = max(self.cost_wall_max_ns, elapsed)
            self.lock.release()

    def close(self):
        if self.closed:return dict(enabled=True, already_closed=True)
        self.closed = True
        start = time.monotonic_ns()
        summary = dict(schema='r118_feedback_evidence/v1', enabled=True,
            run_id=self.run.name, identity=self.identity, columns=COLUMNS, stages=STAGES,
            attempted=self.attempted, recorded=self.recorded, errors=self.errors,
            dropped_lock_contention=self.dropped,
            evicted=max(0,self.recorded-FIRST-TAIL), stage_counts=self.stage_counts,
            token_namespaces=self.namespaces, namespace_overflow=self.namespace_overflow,
            time_domains=dict(mono='CLOCK_MONOTONIC', cpu='CLOCK_THREAD_CPUTIME_ID',
                             source='ROS SIM header; never subtract from MONO'),
            histogram_limits_ns=LIMITS, distributions=self.distributions,
            buffer_bytes=(len(self.ring)+len(self.window)+len(self.external_window))*self.ring.itemsize,
            cost_wall_ns=self.cost_wall_ns, cost_thread_cpu_ns=self.cost_thread_cpu_ns,
            cost_wall_max_ns=self.cost_wall_max_ns, trigger=self.trigger_info,
            window_pre_rows=self.window_pre, window_post_rows=self.window_post,
            external_trigger=self.external_trigger, external_pre_rows=self.external_pre,
            external_post_rows=self.external_post,
            retention='first256_tail1024_first_slow_pre512_post512_first_external_pre512_post512',
            no_payload_copy=True, navigation_PASS=False)
        indices = list(range(min(FIRST,self.recorded))) + list(range(max(FIRST,self.recorded-TAIL),self.recorded))
        summary['rows'] = [list(self.ring[self.offset(i):self.offset(i)+WIDTH]) for i in indices]
        summary['window_rows'] = [list(self.window[i*WIDTH:(i+1)*WIDTH]) for i in range(self.window_rows)]
        summary['external_window_rows'] = [list(self.external_window[i*WIDTH:(i+1)*WIDTH]) for i in range(self.external_rows)]
        try:
            raw = json.dumps(summary, separators=(',',':'), allow_nan=False)
            if len(raw) > 2*1024*1024:raise ValueError('exit file limit')
            path = self.run / ('r118_feedback_evidence_'+str(self.identity['pid'])+'.json')
            flags = os.O_WRONLY|os.O_CREAT|os.O_EXCL|os.O_NOFOLLOW
            with os.fdopen(os.open(path,flags,0o600),'w') as stream:stream.write(raw+'\n')
            return dict(enabled=True, path=str(path), file_complete=True, bytes=len(raw)+1,
                        flush_wall_ns=time.monotonic_ns()-start)
        except Exception as error:
            return dict(enabled=True,file_complete=False,write_error=type(error).__name__)


def create():
    global RECORDER
    value=os.environ.get(KEY,'0')
    if value not in ('0','1'):raise ValueError(KEY+' must be 0 or 1')
    if value == '0':return None
    # Initialization errors are explicit; no silently enabled-but-unavailable trace.
    if RECORDER is not None:raise RuntimeError('one prepared Mailbox per process required')
    RECORDER=Recorder()
    return RECORDER


def trigger(source_ns, reason=0):
    """Optional C probe bridge. Frozen window is global time, exact ID stays scalar."""
    if RECORDER is not None:
        try:
            RECORDER.mark(11,source_ns,outcome=reason)
        except Exception:
            RECORDER.errors += 1


def finish():
    return dict(enabled=False) if RECORDER is None else RECORDER.close()
