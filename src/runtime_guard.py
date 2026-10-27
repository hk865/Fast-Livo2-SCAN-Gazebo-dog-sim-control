"""Owned physics lock, bounded wall time and current retained resource census."""
from pathlib import Path
import datetime as dt
import fcntl,json,os,subprocess
from project_config import ROOT,read
from storage_usage import usage
FINISH_RESERVE_S=300
LOCK=Path('/tmp/go2_pid_physics_serial.lock')
def authorized_user_deadline(profile=None):
    from runtime_limits import configuration
    return dt.datetime.fromisoformat(configuration(profile)['deadline_utc'])
def session_deadline(profile=None):return authorized_user_deadline(profile)
def session_deadline_utc(profile=None):return session_deadline(profile).isoformat()
def remaining_seconds(profile=None):return (session_deadline(profile)-dt.datetime.now(dt.timezone.utc)).total_seconds()
def work_deadline_withdrawn(profile=None):return False
def census():
    roots=[ROOT/Path(p)for p in read()['resource_count_roots']]
    optional=ROOT/'.local/census_roots.json'
    if optional.is_file():roots += [Path(p)for p in json.loads(optional.read_text())]
    roots.append(ROOT)
    for parent in ('/tmp','/var/tmp'):
        with os.scandir(parent)as entries:roots.extend(Path(e.path)for e in entries if 'go2'in e.name.lower())
    return usage(roots)
def processes():
    raw=subprocess.check_output(['ps','-eo','pid,ppid,pgid,comm,args'],text=True)
    matches=[]
    for line in raw.splitlines()[1:]:
        x=line.split(None,4)
        if len(x)!=5:continue
        pid,ppid,pgid,comm,args=x
        if int(pid) in (os.getpid(),os.getppid()) or comm in ('bash','ps','timeout','codex'):continue
        if any(k in args for k in ('gz sim','gzserver','gzclient','train.py','student_worker.py',
                'fastlivo_mapping','scan_planner_node','ros2 launch','ipc_request_window.py')):
            matches.append(dict(pid=int(pid),ppid=int(ppid),pgid=int(pgid),comm=comm,args=args))
    return dict(host_process_count=len(raw.splitlines())-1,matching_runtime=matches)

def acquire(profile=None):
    """Caller retains returned fd throughout its owned round; never break a lock."""
    f=LOCK.open('a+');extra=[]
    try:
        fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
        for lock_path in ('/tmp/go2_model_pid_simulation.lock','/tmp/go2_navigation_dispatch.lock'):
            handle=open(lock_path,'a+');extra.append(handle);fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        p=processes();u=census()
        if p['host_process_count']<30:raise RuntimeError('Host process visibility is required')
        if p['matching_runtime']:raise RuntimeError('External runtime active: '+json.dumps(p['matching_runtime']))
        if u['scan_errors']:raise RuntimeError('Retained census errors')
        # Keep the existing minimum predictive drain reserve, not merely free bytes.
        reserve=17*1024**2*122+134217728+536870912
        from runtime_limits import retained_hard_bytes
        if u['current_retained_budget_bytes']+reserve>=retained_hard_bytes(profile):
            raise RuntimeError('Insufficient retained capacity including drain reserve')
        withdrawn=work_deadline_withdrawn(profile)
        if not withdrawn and remaining_seconds(profile)<=FINISH_RESERVE_S:raise RuntimeError('Overnight finish reserve reached')
        return LockGroup([f,*extra]),dict(utc=dt.datetime.now(dt.timezone.utc).isoformat(),processes=p,
            retained=u,deadline_utc=session_deadline_utc(profile),
            finish_reserve_s=0 if withdrawn else FINISH_RESERVE_S,
            self_imposed_work_deadline_withdrawn=withdrawn,
            serial_lock=str(LOCK),physical_runtime_started=False)
    except BaseException:
        for handle in [f,*extra]:handle.close()
        raise


class LockGroup:
    def __init__(self,handles):self.handles=handles
    def close(self):
        for handle in reversed(self.handles):handle.close()
