"""Default-OFF caller-owned low-frequency Linux resource observations.

No threads, sleeps, process actions, command arguments/environ reads or writes.
All deltas are adjacent observations, never admission/resource authority. The
original load guard stays separate. Partial discovery is never full coverage.
"""
from collections import deque
import math
import os
from pathlib import Path
import time

KEY = 'R100_RESOURCE_OBSERVER'
MIN_INTERVAL_NS = 2_000_000_000
MAX_SCAN = 4096
MAX_DIRECTORY_ENTRIES = MAX_SCAN + 256
READ_LIMIT = 16384
READ_BUDGET = 16 * 1024 * 1024
IO_KEYS = ('rchar', 'wchar', 'syscr', 'syscw', 'read_bytes', 'write_bytes', 'cancelled_write_bytes')
CPU_KEYS = ('user', 'nice', 'system', 'idle', 'iowait', 'irq', 'softirq', 'steal')
FILES = ('boot_id', 'stat', 'loadavg', 'cpu_psi', 'io_psi', 'process_stat', 'process_io', 'process_status', 'directory', 'input', 'sample')
KINDS = ('denied', 'gone', 'invalid', 'other')


def enabled():
    option = os.environ.get(KEY, '0')
    if option not in ('0', '1'):
        raise ValueError(KEY + ' must be 0 or 1')
    return option == '1'


def _nonnegative(text):
    value = int(text)
    if value < 0:
        raise ValueError('negative counter')
    return value


def _process_stat(text):
    left, right = text.find('('), text.rfind(')')
    if left < 1 or right <= left:
        raise ValueError('invalid process stat')
    f = text[right+2:].split()
    if len(f) < 22:
        raise ValueError('short process stat')
    return dict(pid=_nonnegative(text[:left].strip()), comm=text[left+1:right][:80],
        state=f[0][:1], ppid=_nonnegative(f[1]), pgid=_nonnegative(f[2]), session=_nonnegative(f[3]),
        user_ticks=_nonnegative(f[11]), system_ticks=_nonnegative(f[12]),
        thread_count=_nonnegative(f[17]), start_ticks=_nonnegative(f[19]), rss_pages=_nonnegative(f[21]))


def _system_stat(text):
    row = next((s for s in text.splitlines() if s.startswith('cpu ')), None)
    if row is None:
        raise ValueError('missing aggregate cpu')
    values = [_nonnegative(x) for x in row.split()[1:]]
    if len(values) < 4:
        raise ValueError('short aggregate cpu')
    values = (values + [0]*8)[:8]
    return dict(zip(CPU_KEYS, values))  # guest/guest_nice already included in user/nice


def _loadavg(text):
    f = text.split()
    if len(f) < 4:
        raise ValueError('short loadavg')
    loads = [float(v) for v in f[:3]]
    if any(not math.isfinite(v) or v < 0 for v in loads):
        raise ValueError('invalid load')
    runnable, total = map(_nonnegative, f[3].split('/'))
    return dict(load_1_5_15=loads, runnable_tasks=runnable, total_tasks=total)


def _psi(text):
    out = {}
    for line in text.splitlines():
        fields = line.split()
        if not fields or fields[0] not in ('some', 'full'):
            continue
        raw = dict(s.split('=', 1) for s in fields[1:])
        row = {k: float(raw[k]) for k in ('avg10', 'avg60', 'avg300')}
        if any(not math.isfinite(v) or not 0 <= v <= 100 for v in row.values()):
            raise ValueError('invalid PSI average')
        row['total_us'] = _nonnegative(raw['total'])
        out[fields[0]] = row
    if 'some' not in out:
        raise ValueError('missing PSI some')
    return out


def _io(text):
    raw = dict(line.split(':', 1) for line in text.splitlines() if ':' in line)
    return {k: _nonnegative(raw[k].strip()) for k in IO_KEYS}


def _identity(row, boot):
    return row['pid'], row['start_ticks'], boot


class Observer:
    def __init__(self, max_processes=128, max_foreign=8, *, _proc_root='/proc',
                 _clock=None, _thread_clock=None, _clock_ticks=None, _page_size=None):
        if type(max_processes) is not int or not 1 <= max_processes <= 128:
            raise ValueError('max_processes must be 1..128')
        if type(max_foreign) is not int or not 0 <= max_foreign <= 8:
            raise ValueError('max_foreign must be 0..8')
        self.enabled = enabled()
        self.max_processes, self.max_foreign = max_processes, max_foreign
        self._proc = Path(_proc_root)
        self._clock = time.monotonic_ns if _clock is None else _clock
        self._thread_clock = time.thread_time_ns if _thread_clock is None else _thread_clock
        self._hz = os.sysconf('SC_CLK_TCK') if _clock_ticks is None else _clock_ticks
        self._page = os.sysconf('SC_PAGE_SIZE') if _page_size is None else _page_size
        if type(self._hz) is not int or self._hz <= 0 or type(self._page) is not int or self._page <= 0:
            raise ValueError('positive integer clock ticks/page size required')
        self._last_begin = self._boot = self._system = None
        self._processes = {}
        self._owned_keys = set()
        self._pid_anchors = {}
        self._psi_previous = {}
        self.calls = self.samples = self.throttled = self.clock_rejected = 0
        self.errors = {f+'.'+k: 0 for f in FILES for k in KINDS}
        self._read_bytes = self._read_attempts = 0

    def _error(self, field, error):
        kind = ('denied' if isinstance(error, PermissionError) else
                'gone' if isinstance(error, (FileNotFoundError, ProcessLookupError)) else
                'invalid' if isinstance(error, (ValueError, TypeError, KeyError, IndexError, OverflowError)) else 'other')
        self.errors[field+'.'+kind] += 1

    def _read(self, path, field):
        self._read_attempts += 1
        try:
            if self._read_bytes >= READ_BUDGET:
                raise ValueError('sample read budget exhausted')
            with path.open('rb') as stream:
                raw = stream.read(READ_LIMIT+1)
            self._read_bytes += len(raw)
            if len(raw) > READ_LIMIT or self._read_bytes > READ_BUDGET:
                raise ValueError('bounded proc read exceeded')
            return raw.decode('utf-8', errors='strict')
        except Exception as error:
            self._error(field, error)
            return None

    def _parse(self, path, field, parser):
        text = self._read(path, field)
        if text is None:
            return None
        try:
            return parser(text)
        except Exception as error:
            self._error(field, error)
            return None

    def _roots(self, supplied, pids):
        result = []
        source = supplied.values() if isinstance(supplied, dict) else supplied
        try:
            for item in source:
                if len(result) >= self.max_processes:
                    return result, True
                if type(item) is int:
                    result.append(dict(pid=item, basis='pid_only_readonly_not_authority'))
                elif isinstance(item, dict):
                    fields = ('pid', 'start_ticks', 'boot_id', 'pgid', 'session')
                    result.append(dict((k, item.get(k)) for k in fields))
                    result[-1]['basis'] = 'provided_stable_identity_readonly_not_authority'
                else:
                    raise ValueError('invalid owner input')
            for pid in pids:
                if len(result) >= self.max_processes:
                    return result, True
                result.append(dict(pid=pid, basis='pid_only_readonly_not_authority'))
            if any(type(r.get('pid')) is not int or not 0 < r['pid'] < 2**31 for r in result):
                raise ValueError('invalid PID')
        except Exception as error:
            self._error('input', error)
            return [], True
        return result, False

    def _ids(self, roots):
        pids = set(r['pid'] for r in roots)
        truncated = False
        entries = numeric = 0
        try:
            with os.scandir(self._proc) as scan:
                for entry in scan:
                    entries += 1
                    if entries > MAX_DIRECTORY_ENTRIES:
                        truncated = True
                        break
                    if entry.name.isascii() and entry.name.isdigit():
                        numeric += 1
                        if len(pids) >= MAX_SCAN and int(entry.name) not in pids:
                            truncated = True
                            break
                        pids.add(int(entry.name))
        except Exception as error:
            self._error('directory', error)
            truncated = True
        seeds = [r['pid'] for r in roots]
        return list(dict.fromkeys(seeds + sorted(pids))), dict(directory_entries_examined=entries,
            numeric_entries_examined=numeric, scan_truncated=truncated, maximum_scanned_processes=MAX_SCAN)

    def _one_process(self, pid, boot):
        begin = self._clock()
        path = self._proc/str(pid)
        first = self._parse(path/'stat', 'process_stat', _process_stat)
        if first is None or first['pid'] != pid:
            return None
        io = self._parse(path/'io', 'process_io', _io)
        last = self._parse(path/'stat', 'process_stat', _process_stat)
        if last is None:
            return None
        if _identity(first, boot) != _identity(last, boot):
            self._error('process_stat', ValueError('identity changed during read'))
            return None
        last.update(boot_id=boot, io=io, read_begin_ns=begin, read_end_ns=self._clock(),
                    rss_bytes=last.pop('rss_pages')*self._page)
        previous = self._processes.get(pid)
        last['delta'] = None
        last['delta_status'] = 'baseline_none'
        if previous is not None:
            if _identity(previous, previous['boot_id']) != _identity(last, boot):
                last['delta_status'] = 'identity_changed_no_delta'
            else:
                elapsed = last['read_end_ns']-previous['read_end_ns']
                cpu_delta = last['user_ticks']+last['system_ticks']-previous['user_ticks']-previous['system_ticks']
                if elapsed <= 0 or last['user_ticks'] < previous['user_ticks'] or last['system_ticks'] < previous['system_ticks']:
                    last['delta_status'] = 'counter_or_clock_rollback_rejected'
                else:
                    io_delta = None
                    io_status = 'unavailable'
                    if io is not None and previous['io'] is not None:
                        if any(io[k] < previous['io'][k] for k in IO_KEYS):
                            io_status = 'counter_rollback_rejected'
                        else:
                            io_delta = {k: io[k]-previous['io'][k] for k in IO_KEYS}
                            io_status = 'same_identity_delta'
                    last['delta'] = dict(elapsed_ns=elapsed, cpu_ticks=cpu_delta,
                        cpu_core_percent=cpu_delta/self._hz*1e9/elapsed*100,
                        io=io_delta, io_status=io_status,
                        io_bytes_per_s=None if io_delta is None else
                            (io_delta['read_bytes']+io_delta['write_bytes'])*1e9/elapsed)
                    last['delta_status'] = 'same_identity_delta'
        return last

    def _owned(self, processes, roots, boot):
        seeds = set()
        anchors = []
        by_pid = {r['pid']:r for r in processes}
        for root in roots:
            pid = root['pid'];live = by_pid.get(pid)
            saved = root
            if root['basis'].startswith('pid_only'):
                saved = self._pid_anchors.get(pid)
                if saved is None and live is not None:
                    if len(self._pid_anchors) < self.max_processes:
                        saved = {k:live[k] for k in ('pid','start_ticks','boot_id','pgid','session')}
                        self._pid_anchors[pid] = saved
            valid = bool(saved and saved.get('boot_id') == boot and
                type(saved.get('start_ticks')) is int and type(saved.get('pgid')) is int and
                type(saved.get('session')) is int)
            reused = live is not None and valid and any(live[k] != saved[k] for k in
                ('pid','start_ticks','boot_id','pgid','session'))
            if reused:
                valid = False
            anchors.append(dict(pid=pid, basis=root['basis'], valid=valid,
                leader_present=live is not None, leader_missing=live is None, identity_mismatch=reused))
            if not valid:
                continue
            if live is not None:
                seeds.add(pid)
                # Numeric PGID/session alone cannot prove continued ownership
                # after the stable leader disappears: both numbers can be reused.
                seeds.update(r['pid'] for r in processes if r['pgid']==saved['pgid'] and
                    r['session']==saved['session'] and r['start_ticks']>=saved['start_ticks'])
        # A previously proven descendant can stay attributed after reparenting,
        # only while its own boot/PID/start identity remains unchanged.
        seeds.update(r['pid'] for r in processes if _identity(r, boot) in self._owned_keys)
        children = {}
        for row in processes:
            children.setdefault(row['ppid'], []).append(row['pid'])
        queue = deque(seeds)
        while queue:
            for child in children.get(queue.popleft(), ()):
                if child not in seeds:
                    seeds.add(child);queue.append(child)
        return seeds, anchors

    def _affinity(self, row):
        path = self._proc/str(row['pid'])
        text = self._read(path/'status', 'process_status')
        affinity = None
        if text is not None:
            for line in text.splitlines():
                if line.startswith('Cpus_allowed_list:'):
                    affinity = line.split(':', 1)[1].strip()[:256]
            if affinity is None:
                self._error('process_status', ValueError('affinity missing'))
        check = self._parse(path/'stat', 'process_stat', _process_stat)
        if check is None or check['start_ticks'] != row['start_ticks'] or check['pid'] != row['pid']:
            self._error('process_status', ValueError('identity changed around status'))
            return None
        return dict(row, affinity_cpu_list=affinity)

    def _sample(self, supplied, pids, now):
        roots, roots_truncated = self._roots(supplied, pids)
        boot = self._read(self._proc/'sys/kernel/random/boot_id', 'boot_id')
        boot = None if boot is None else boot.strip()
        if (boot is None or len(boot) != 36 or
                any(boot[i] != '-' for i in (8,13,18,23)) or
                any(c not in '0123456789abcdefABCDEF' for i,c in enumerate(boot) if i not in (8,13,18,23))):
            self._error('boot_id', ValueError('invalid boot identity'))
            return dict(status='boot_identity_unavailable')
        if self._boot is not None and boot != self._boot:
            return dict(status='boot_identity_changed_rejected', original_boot_id=self._boot, observed_boot_id=boot)
        self._boot = boot
        system = self._parse(self._proc/'stat', 'stat', _system_stat)
        cpu_delta = None;cpu_status = 'unavailable' if system is None else 'baseline_none'
        if system is not None and self._system is not None:
            previous = self._system['counters']
            diffs = {k: system[k]-previous[k] for k in CPU_KEYS}
            total = sum(diffs.values())
            if any(v < 0 for v in diffs.values()) or total <= 0:
                cpu_status = 'counter_rollback_or_zero_interval_rejected'
            else:
                busy = sum(diffs[k] for k in ('user','nice','system','irq','softirq','steal'))
                cpu_delta = dict(elapsed_ns=now-self._system['wall_ns'],total_ticks=total,
                    busy_ticks=busy,idle_ticks=diffs['idle'],iowait_ticks=diffs['iowait'],
                    busy_percent=busy/total*100,idle_percent=diffs['idle']/total*100,
                    iowait_percent=diffs['iowait']/total*100,fields=diffs)
                cpu_status = 'adjacent_counter_delta'
        if system is not None:
            self._system = dict(counters=system,wall_ns=now)
        psi = {}
        for kind in ('cpu','io'):
            value = self._parse(self._proc/'pressure'/kind,kind+'_psi',_psi)
            if value is not None:
                for name,row in value.items():
                    key = kind+'.'+name;previous = self._psi_previous.get(key)
                    row['delta_total_us'] = None if previous is None or row['total_us'] < previous[0] else row['total_us']-previous[0]
                    row['delta_status'] = ('baseline_none' if previous is None else
                        'counter_rollback_rejected' if row['total_us'] < previous[0] else 'adjacent_counter_delta')
                    row['elapsed_ns'] = None if previous is None else now-previous[1]
                    self._psi_previous[key] = row['total_us'],now
            psi[kind] = value
        ids, coverage = self._ids(roots)
        processes = []
        for pid in ids[:MAX_SCAN]:
            if self._read_bytes >= READ_BUDGET:
                coverage['scan_truncated'] = True;break
            row = self._one_process(pid,boot)
            if row is not None:
                processes.append(row)
        owned_ids, anchors = self._owned(processes,roots,boot)
        owned = sorted((r for r in processes if r['pid'] in owned_ids),key=lambda r:r['pid'])
        foreign = [r for r in processes if r['pid'] not in owned_ids]
        cpu_rank = sorted((r for r in foreign if r['delta'] is not None),
            key=lambda r:(-r['delta']['cpu_core_percent'],r['pid']))
        io_rank = sorted((r for r in foreign if r['delta'] is not None and r['delta']['io'] is not None),
            key=lambda r:(-r['delta']['io_bytes_per_s'],r['pid']))
        chosen = []
        seen = set()
        for index in range(max(len(cpu_rank),len(io_rank))):
            for ranking in (cpu_rank,io_rank):
                if index < len(ranking) and ranking[index]['pid'] not in seen and len(chosen)<self.max_foreign:
                    seen.add(ranking[index]['pid']);chosen.append(ranking[index])
            if len(chosen)>=self.max_foreign:
                break
        owned_out = [x for row in owned[:self.max_processes] if (x:=self._affinity(row)) is not None]
        foreign_out = [x for row in chosen if (x:=self._affinity(row)) is not None]
        self._owned_keys = {_identity(r,boot) for r in owned[:self.max_processes]}
        self._processes = {r['pid']:dict((k,r[k])for k in ('pid','start_ticks','boot_id','user_ticks','system_ticks','io','read_end_ns'))
                           for r in processes[:MAX_SCAN]}
        coverage.update(roots_truncated=roots_truncated,processes_read=len(processes),
            process_cpu_delta_rejections=sum(r['delta_status']=='counter_or_clock_rollback_rejected' for r in processes),
            process_identity_changed_no_delta=sum(r['delta_status']=='identity_changed_no_delta' for r in processes),
            process_io_delta_rejections=sum(r['delta'] is not None and
                r['delta']['io_status']=='counter_rollback_rejected' for r in processes),
            owned_discovered=len(owned),owned_returned=len(owned_out),owned_truncated=len(owned)>self.max_processes,
            anchor_leaders_missing=sum(a['leader_missing'] for a in anchors),
            foreign_returned=len(foreign_out),foreign_total_cap=self.max_foreign,
            cached_processes=len(self._processes),maximum_cached_processes=MAX_SCAN,
            discovery_complete_within_scan=not coverage['scan_truncated'] and not roots_truncated,
            complete_owned_observation=not coverage['scan_truncated'] and not roots_truncated and
                len(owned)==len(owned_out) and all(a['valid'] and a['leader_present'] for a in anchors),
            attribution_is_not_ownership_authority=True)
        return dict(status='sampled',boot_id=boot,system_cpu=dict(counters=system,delta=cpu_delta,delta_status=cpu_status,
            guest_accounting='guest/guest_nice not added again to user/nice'),
            loadavg=self._parse(self._proc/'loadavg','loadavg',_loadavg),pressure=psi,
            owned_roots=anchors,owned_processes=owned_out,foreign_top=foreign_out,
            foreign_selection='CPU and IO delta rankings alternated, deduplicated, total capped; baseline/missing deltas not ranked',
            coverage=coverage)

    def sample(self, owned_parent_identities=(), *, pids=()):
        self.calls += 1
        begin = self._clock();cpu_begin = self._thread_clock()
        before = dict(self.errors)
        self._read_bytes = self._read_attempts = 0
        if not self.enabled:
            result = dict(status='disabled')
        elif type(begin) is not int or begin < 0 or (self._last_begin is not None and begin < self._last_begin):
            self.clock_rejected += 1;result = dict(status='monotonic_clock_rollback_rejected')
        elif self._last_begin is not None and begin-self._last_begin<MIN_INTERVAL_NS:
            self.throttled += 1
            result = dict(status='throttled',next_eligible_wall_ns=self._last_begin+MIN_INTERVAL_NS)
        else:
            self._last_begin = begin;self.samples += 1
            try:
                result = self._sample(owned_parent_identities,pids,begin)
            except Exception as error:
                self._error('sample',error);result = dict(status='observer_error')
        end = self._clock();cpu_end = self._thread_clock()
        errors = {k:self.errors[k]-before[k] for k in self.errors if self.errors[k]!=before[k]}
        if 'coverage' in result:
            coverage = result['coverage']
            discovery_errors = any(k.split('.')[0] in ('directory','input','process_stat') for k in errors)
            metric_errors = any(k.split('.')[0] in ('process_io','process_status') for k in errors)
            coverage['discovery_complete_within_scan'] &= not discovery_errors
            coverage['complete_owned_observation'] &= not discovery_errors and not metric_errors
            coverage['discovery_read_errors'] = discovery_errors
            coverage['metric_read_errors'] = metric_errors
            coverage['sequential_nonatomic_observation'] = True
        result.update(schema='r100_low_frequency_resource_observation/v1',enabled=self.enabled,
            environment_key=KEY,minimum_interval_ns=MIN_INTERVAL_NS,sample_begin_wall_ns=begin,
            sample_end_wall_ns=end,observer_wall_ns=max(0,end-begin),observer_thread_cpu_ns=max(0,cpu_end-cpu_begin),
            reads=self._read_attempts,read_bytes=self._read_bytes,
            sample_errors=errors,
            cumulative_errors=dict(self.errors),calls=self.calls,samples=self.samples,throttled=self.throttled,
            clock_rejected=self.clock_rejected,no_threads_or_writes=True,
            no_process_actions_or_command_arguments_or_proc_environ_reads=True,
            source='Linux procfs sequential reads, not atomic snapshots',
            limits='iowait/PSI are not exact task IO attribution; process IO accounting is not operation latency; host load is not CPU percent; observation never changes original resource guards')
        return result
