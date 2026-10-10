# Public learning snapshot 2026-10-10: privacy/path metadata or documentation links adjusted; control algorithms and numeric gates unchanged.
"""One conservative storage budget for the whole V33 session, never deletion.

Known outputs are counted without following symlinks. Filesystem free-space
consumption also counts writes outside the declared roots (including /tmp,
ROS/Gazebo caches, and concurrent tasks). This is a supervisor with a 512 MiB
stop reserve, not an operating-system quota or a proof of write attribution.
"""
from pathlib import Path
import json
import os
import threading
import time

GIB = 1024**3
SESSION = Path('external/omitted-history/STORAGE_SESSION.json')
HARD_BYTES = 80_000_000_000  # User-approved decimal80GB; authority AUTHORITY.json,2026-10-09
SOFT_BYTES = HARD_BYTES - 512 * 1024**2
HOME_RESERVE = 100 * GIB
ROOT_RESERVE = 80 * GIB
MIN_PREDICTIVE_RATE = 17 * 1024**2  # max old log row x measured Hz, plus 1.5x allowance
MAX_SINGLE_ATOMIC_WRITE = 128 * 1024**2  # original RGB-map 8M points x16 bytes


def capacity(path):
    path = Path(path).resolve()
    s = os.statvfs(path)
    return dict(path=str(path), device=os.stat(path).st_dev,
                available_bytes=s.f_bavail*s.f_frsize)


def usage(paths, *, allowed_missing=()):
    roots = sorted(set(Path(x).resolve() for x in paths), key=lambda p: len(p.parts))
    roots = [p for p in roots if not any(p != q and p.is_relative_to(q) for q in roots)]
    seen = set(); logical = 0; allocated = 0; count = 0
    for root in roots:
        if not root.is_dir():
            if str(root) in allowed_missing and not root.exists() and not root.is_symlink():
                continue  # Explicit recovery contract; prior bytes remain charged.
            raise RuntimeError('Storage output root absent: '+str(root))
        for base, dirs, names in os.walk(root, followlinks=False):
            dirs[:] = [d for d in dirs if not Path(base,d).is_symlink()]
            for name in names:
                p = Path(base,name)
                try:
                    st = p.lstat()
                    if p.is_symlink() or not p.is_file(): continue
                    key = (st.st_dev,st.st_ino)
                    if key in seen: continue
                    seen.add(key); logical += st.st_size; allocated += st.st_blocks*512; count += 1
                except FileNotFoundError:
                    continue
    return dict(logical_bytes=logical, allocated_bytes=allocated, files=count,
                roots=[str(x) for x in roots])


def initialize(roots):
    """Call exactly once before any V33 simulator; preserve this baseline."""
    if SESSION.exists():
        raise RuntimeError('Existing session budget must be reused, never reset')
    d = dict(schema='go2_v33_whole_session_storage/v1', created_utc=time.time(),
        roots=[str(Path(x).resolve()) for x in roots], initial_usage=usage(roots),
        initial_filesystems=[capacity('/home'),capacity('/tmp')],
        hard_bytes=HARD_BYTES, soft_bytes=SOFT_BYTES, cleanup_reserve_bytes=HARD_BYTES-SOFT_BYTES,
        filesystem_minimums={'/home':HOME_RESERVE,'/tmp':ROOT_RESERVE},
        poll_s=1., maximum_predictive_stop_grace_s=120., deleted_existing_files=False,
        limitation='Free-space deltas include unrelated writes; conservative stop, no attribution or OS quota')
    with SESSION.open('x') as f: json.dump(d,f,indent=2);f.write('\n')
    return d


def register(run):
    d = json.loads(SESSION.read_text()); name = str(Path(run).resolve())
    if name not in d['roots']:
        d['roots'].append(name)
        temp=SESSION.with_suffix('.tmp');temp.write_text(json.dumps(d,indent=2)+'\n');temp.replace(SESSION)
    return d


def check(snapshot, previous=None):
    reasons=[]
    if snapshot['conservative_session_growth_bytes'] >= SOFT_BYTES:
        reasons.append('whole_session_soft_stop')
    for fs in snapshot['filesystems']:
        reserve = HOME_RESERVE if fs['path']=='/home' else ROOT_RESERVE
        if fs['available_bytes'] < reserve: reasons.append('filesystem_reserve_'+fs['path'])
    observed_rate=0.
    if previous:
        dt=snapshot['monotonic_wall']-previous['monotonic_wall']
        growth=max(0,snapshot['conservative_session_growth_bytes']-previous['conservative_session_growth_bytes'])
        observed_rate=growth/max(dt,1e-6)
    snapshot['observed_growth_bytes_per_s']=observed_rate
    rate=max(observed_rate,MIN_PREDICTIVE_RATE)
    snapshot['predictive_growth_allowance_bytes_per_s']=rate
    snapshot['single_atomic_write_allowance_bytes']=MAX_SINGLE_ATOMIC_WRITE
    # Start with a measured-history floor, not merely react after a first burst.
    if snapshot['conservative_session_growth_bytes']+rate*122+MAX_SINGLE_ATOMIC_WRITE >= SOFT_BYTES:
        reasons.append('projected_stop_grace_would_exhaust_soft_budget')
    snapshot['stop_reasons']=reasons
    return snapshot


def conservative_growth(known, initial, filesystem_consumption):
    seed=max(initial['logical_bytes'],initial['allocated_bytes'])
    return max(max(known['logical_bytes'],known['allocated_bytes']),
        filesystem_consumption+seed)


def observe(run,profile,starting=False,count_files=True,previous=None):
    d=json.loads(SESSION.read_text())
    if (d.get('hard_bytes'), d.get('soft_bytes'), d.get('cleanup_reserve_bytes')) != (HARD_BYTES, SOFT_BYTES, HARD_BYTES-SOFT_BYTES):
        raise RuntimeError('Shared session budget/source authorization mismatch')
    policy=profile['raw_storage_budget']
    if policy != dict(max_run_bytes=HARD_BYTES,
            minimum_start_available_bytes=HOME_RESERVE,
            minimum_runtime_available_bytes=HOME_RESERVE):
        raise RuntimeError('Explicit V33 session storage contract differs')
    if str(Path(run).resolve()) not in d['roots']:
        raise RuntimeError('Run not registered in the shared storage session')
    from storage_recovery import observe_recovery
    current,filesystems,recovery=observe_recovery(Path(__file__).resolve().parent,d,usage,capacity,conservative_growth)
    fs_by_device={x['device']:x for x in d['initial_filesystems']}
    deltas={x['device']:max(0,fs_by_device[x['device']]['available_bytes']-x['available_bytes']) for x in filesystems}
    # These are exclusively newly-created V33 roots; count the initial code/test
    # copies too. Their budget is never reset between experiments.
    owned=max(current['logical_bytes'],current['allocated_bytes'])
    result=dict(schema='go2_v33_storage_observation/v1',monotonic_wall=time.monotonic(),
        session_file=str(SESSION),known_outputs=current,filesystems=filesystems,
        known_output_growth_bytes=owned,all_filesystem_consumption_bytes=sum(deltas.values()),
        conservative_session_growth_bytes=conservative_growth(current,d['initial_usage'],sum(deltas.values())),
        hard_bytes=HARD_BYTES,soft_stop_bytes=SOFT_BYTES,cleanup_reserve_bytes=HARD_BYTES-SOFT_BYTES,
        size_check_interval_wall_s=1.,includes_all_declared_runs_agents_clouds_and_unattributed_fs_writes=True,
        no_deletion=True)
    if recovery is not None:
        result['conservative_session_growth_bytes']=max(result['conservative_session_growth_bytes'],recovery['charged_bytes'])
        result['reboot_recovery']=recovery
    result=check(result,previous)
    if result['stop_reasons']:
        raise StorageStop(result)
    return result


class StorageStop(RuntimeError):
    def __init__(self,observation):
        self.observation=observation
        super().__init__('V33 shared storage stop: '+', '.join(observation['stop_reasons']))


class Watchdog:
    """Independent of ROS startup and runner waits; signals only its own runner."""
    def __init__(self,run,profile,stop_runner):
        self.run=Path(run);self.profile=profile;self.stop_runner=stop_runner
        self.done=threading.Event();self.thread=None;self.last=None;self.error=None
    def start(self):
        self.last=observe(self.run,self.profile,starting=True)
        self.thread=threading.Thread(target=self._loop,name='v33_storage_watchdog',daemon=True)
        self.thread.start();return self
    def _loop(self):
        try:
            with (self.run/'storage_observations.jsonl').open('x',buffering=1) as f:
                f.write(json.dumps(self.last)+'\n')
                while not self.done.wait(1.):
                    row=observe(self.run,self.profile,previous=self.last);self.last=row
                    f.write(json.dumps(row)+'\n')
        except Exception as e:
            self.error=str(e)
            row=e.observation if isinstance(e,StorageStop) else dict(error=str(e))
            row.update(schema='go2_v33_storage_stop/v1',monotonic_wall=time.monotonic(),
                stop_only_current_owned_runner=True,no_old_file_deleted=True)
            if not self.done.is_set():
                # Stop first: even a hung filesystem receipt must not delay it.
                self.stop_runner()
                try:
                    (self.run/'storage_stop.json').write_text(json.dumps(row,indent=2)+'\n')
                except Exception as receipt_error:
                    self.error += '; stop receipt unavailable: '+str(receipt_error)
    def close(self):
        self.done.set()
        if self.thread:self.thread.join(timeout=5)
