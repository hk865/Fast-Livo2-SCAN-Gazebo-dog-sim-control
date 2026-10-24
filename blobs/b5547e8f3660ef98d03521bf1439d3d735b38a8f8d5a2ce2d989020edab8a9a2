"""Independent bounded owner monitor; bootstrap gates every real role launch."""
from pathlib import Path
import argparse
import contextlib
import datetime as dt
import hashlib
import json
import os
import signal
import subprocess
import sys
import threading
import time
import uuid

BOOT = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
MARKER = 'TEACHER_OWNED_RUN_TOKEN'

def ident(pid):
    try:
        text = Path(f'/proc/{pid}/stat').read_text()
        f = text[text.rfind(')')+2:].split()
        return dict(pid=int(pid), state=f[0], ppid=int(f[1]), pgid=int(f[2]),
                    session=int(f[3]), start_ticks=int(f[19]), boot_id=BOOT)
    except (OSError, ValueError): return None

def same(a, b):
    return bool(a and b and all(a.get(k)==b.get(k) for k in
                ('pid','start_ticks','pgid','session','boot_id')))

def atomic(path, data, *, exclusive=False):
    path = Path(path)
    temp = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    with temp.open('x') as f:
        json.dump(data, f, ensure_ascii=False, allow_nan=False)
        f.write('\n'); f.flush(); os.fsync(f.fileno())
    if exclusive:
        os.link(temp, path); temp.unlink()
    else: temp.replace(path)

def members(pgid):
    out=[]
    for p in Path('/proc').iterdir():
        if p.name.isdigit():
            x=ident(int(p.name))
            if x and x['pgid']==pgid and x['state']!='Z': out.append(x)
    return out

def validated_members(saved):
    """A reused leader or foreign/older member prevents any group signal."""
    leader=ident(saved['pid'])
    if leader and not same(leader,saved): raise RuntimeError('Owned leader identity changed')
    rows=members(saved['pgid']);valid=[]
    for row in rows:
        if row['session']!=saved['session'] or row['start_ticks']<saved['start_ticks']:
            raise RuntimeError('Foreign process-group member')
        # start_new_session makes this recorded bootstrap the unique session
        # leader. A process from another session cannot join this process group.
        # Recheck the run-owned registration and causal launch identity instead
        # of reading any process environment, including an owned environment.
        owner=json.loads((Path(saved['owned_run'])/'guardian_owner.json').read_text())
        role=saved['role']
        if (role not in owner['allowed_roles'] or saved['ownership_token']!=owner['token']
                or saved['boot_id']!=BOOT or saved['pid']!=saved['pgid']
                or saved['session']!=saved['pid'] or saved['ppid']!=owner['parent']['pid']
                or json.loads((Path(saved['owned_run'])/'owned_roles'/(role+'.identity.json')).read_text())!=saved):
            raise RuntimeError('Owned bootstrap/session registration differs; no group signal')
        valid.append(row)
    return valid

@contextlib.contextmanager
def bounded_shutdown_signals():
    """Discard repeat INT/TERM only during the existing bounded drain/close."""
    old={}
    if threading.current_thread() is threading.main_thread():
        old={s:signal.signal(s,signal.SIG_IGN) for s in (signal.SIGINT,signal.SIGTERM)}
    try: yield
    finally:
        for s,h in old.items(): signal.signal(s,h)

def stop_group(saved, *, parent_first=False, grace=(15.,5.,3.)):
    sent=[]
    for sig,timeout in zip((signal.SIGINT,signal.SIGTERM,signal.SIGKILL),grace):
        rows=validated_members(saved)
        if not rows: break
        leader=ident(saved['pid'])
        if parent_first and sig==signal.SIGINT and leader and leader['state']!='Z':
            os.kill(saved['pid'],sig)
        else: os.killpg(saved['pgid'],sig)
        sent.append(int(sig)); end=time.monotonic()+timeout
        while time.monotonic()<end:
            if not validated_members(saved): break
            time.sleep(.05)
    return dict(sent_signals=sent,remaining_members=validated_members(saved))

def bootstrap(run, role, command):
    run=Path(run); owner=json.loads((run/'guardian_owner.json').read_text())
    deadline=time.monotonic()+10.
    while time.monotonic()<deadline:
        if not same(ident(owner['parent']['pid']),owner['parent']): return 75
        permit=run/'owned_roles'/(role+'.permit.json')
        if permit.exists():
            p=json.loads(permit.read_text()); me=ident(os.getpid())
            heartbeat=json.loads((run/'guardian_heartbeat.json').read_text())
            if (same(me,p['identity']) and p['token']==owner['token'] and
                time.monotonic()-heartbeat['monotonic_wall']<3. and
                heartbeat['state']=='RUNNING'):
                os.execvpe(command[0],command,os.environ)
            return 76
        time.sleep(.02)
    return 77

def watch(run, storage_check=None):
    run=Path(run); owner=json.loads((run/'guardian_owner.json').read_text())
    plan=json.loads((run/'runtime_plan.json').read_text())
    if storage_check is None:
        from storage_guard import observe
        storage_check=lambda previous:observe(run,plan['profile'],previous=previous)
    for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP): signal.signal(s,signal.SIG_IGN)
    mine=ident(os.getpid()); last_storage=None; storage_at=0.; roles={}; reason=None
    atomic(run/'guardian_ready.json',dict(identity=mine,token=owner['token']),exclusive=True)
    try:
        while True:
            now=time.monotonic()
            atomic(run/'guardian_heartbeat.json',dict(monotonic_wall=now,state='RUNNING'))
            for file in sorted((run/'owned_roles').glob('*.identity.json')):
                role=file.name.removesuffix('.identity.json')
                if role not in owner['allowed_roles']: raise RuntimeError('Undeclared owned role')
                saved=json.loads(file.read_text())
                if saved['ownership_token']!=owner['token']: raise RuntimeError('Wrong owner token')
                if role not in roles:
                    validated_members(saved); roles[role]=saved
                    atomic(file.with_name(role+'.permit.json'),
                           dict(identity=saved,token=owner['token']),exclusive=True)
            release=run/'guardian_release.json'
            if release.exists():
                if any(validated_members(x) for x in roles.values()):
                    raise RuntimeError('Release while owned processes remain')
                atomic(run/'guardian_result.json',dict(status='parent_cleanup_verified',
                    utc=dt.datetime.now(dt.timezone.utc).isoformat(),roles=roles,
                    all_owned_groups_absent=True,original_runner_exitcode='not_recorded_by_guardian'),exclusive=True)
                return 0
            parent=ident(owner['parent']['pid'])
            if not same(parent,owner['parent']) or parent['state']=='Z':
                reason='parent_identity_lost'; break
            heartbeat=json.loads((run/'runner_heartbeat.json').read_text())
            if now-heartbeat['monotonic_wall']>owner['heartbeat_timeout_s']:
                reason='parent_heartbeat_stale'; break
            if now>=owner['wall_deadline']:
                reason='independent_wall_budget'; break
            if now-storage_at>=1.:
                last_storage=storage_check(last_storage); storage_at=now
                atomic(run/'guardian_storage_last.json',last_storage)
            time.sleep(.1)
    except Exception as error:
        reason=type(error).__name__+': '+str(error)
    # A surviving parent gets a bounded chance to use its normal cleanup.
    parent=ident(owner['parent']['pid'])
    if same(parent,owner['parent']) and parent['state']!='Z':
        os.kill(parent['pid'],signal.SIGTERM)
        until=time.monotonic()+owner['parent_cleanup_grace_s']
        while time.monotonic()<until:
            if (run/'guardian_release.json').exists() or not same(ident(parent['pid']),parent): break
            time.sleep(.1)
    atomic(run/'guardian_recovery_intent.json',dict(reason=reason,roles=roles,
        utc=dt.datetime.now(dt.timezone.utc).isoformat(),original_exitcode='unknown'),exclusive=True)
    pipeline=None
    witness=run/'slam_loaded_binary.json'
    if witness.exists() and 'navigation_stack' in roles:
        from pipeline_lifecycle import request_normal_stop
        pipeline=request_normal_stop(run,json.loads(witness.read_text()),roles['navigation_stack'],ident,
                                     receipt_name='pipeline_guardian_stop.json')
    cleanup={}
    for role in ('shadow_sampler','navigation_stack','capture','bridge','gazebo','worker'):
        if role in roles:
            try:
                if role=='worker':
                    try:
                        from guardian_archive_shutdown import observe_archive_close
                        archive_close=observe_archive_close(run,reason,owner,plan,roles,cleanup,pipeline,
                            storage_check,last_storage,ident,validated_members)
                        atomic(run/'guardian_worker_archive_close.json',archive_close,exclusive=True)
                    except Exception as e:
                        try:
                            atomic(run/'guardian_worker_archive_close_error.json',dict(
                                error=type(e).__name__+': '+str(e),worker_returncode='UNVERIFIED',
                                navigation_verified=False),exclusive=True)
                        except Exception: pass
                cleanup[role]=stop_group(roles[role],parent_first=role=='navigation_stack',
                                         grace=(15. if role=='navigation_stack' else 2.,3.,2.))
            except Exception as e: cleanup[role]=dict(error=type(e).__name__+': '+str(e))
    remaining={role:members(x['pgid']) for role,x in roles.items()}
    if 'worker' in roles:
        try:
            from guardian_archive_shutdown import inspect_archive_after_recovery
            archive_integrity=inspect_archive_after_recovery(run,reason,roles,cleanup,pipeline,validated_members)
            atomic(run/'guardian_worker_archive_integrity.json',archive_integrity,exclusive=True)
        except Exception as e:
            try:
                atomic(run/'guardian_worker_archive_integrity_error.json',dict(
                    error=type(e).__name__+': '+str(e),worker_returncode='UNVERIFIED',
                    navigation_verified=False),exclusive=True)
            except Exception: pass
    atomic(run/'guardian_result.json',dict(status='recovered_failed',reason=reason,
        utc=dt.datetime.now(dt.timezone.utc).isoformat(),cleanup=cleanup,pipeline=pipeline,
        remaining=remaining,all_owned_groups_absent=not any(remaining.values()),
        original_runner_exitcode='unknown',navigation_verified=False),exclusive=True)
    parent=ident(owner['parent']['pid'])
    if same(parent,owner['parent']) and not (run/'guardian_release.json').exists():
        os.kill(parent['pid'],signal.SIGKILL)
    return 1

class Guardian:
    def __init__(self,run,plan,*,module_path=None):
        self.run=Path(run); self.script=Path(module_path or __file__).resolve()
        self.done=threading.Event(); self.error=None
        self.token=uuid.uuid4().hex; self.parent=ident(os.getpid()); self.children={}
        (self.run/'owned_roles').mkdir()
        atomic(self.run/'guardian_owner.json',dict(parent=self.parent,token=self.token,
            wall_deadline=time.monotonic()+plan['wall_budget_s'],heartbeat_timeout_s=10.,
            parent_cleanup_grace_s=60.,allowed_roles=['worker','bridge','capture','navigation_stack','gazebo','shadow_sampler']),exclusive=True)
        atomic(self.run/'runner_heartbeat.json',dict(monotonic_wall=time.monotonic()))
        self.log=(self.run/'guardian.log').open('x')
        self.proc=subprocess.Popen([sys.executable,'-B',str(self.script),'--watch',str(self.run)],
                                   stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True)
        until=time.monotonic()+10.
        while not (self.run/'guardian_ready.json').exists():
            if self.proc.poll() is not None or time.monotonic()>until:
                raise RuntimeError('Independent guardian failed before role admission')
            time.sleep(.02)
        self.saved=ident(self.proc.pid)
        self.thread=threading.Thread(target=self._heartbeat,name='owned-guardian-monitor',daemon=True)
        self.thread.start()
    def _heartbeat(self):
        try:
            while not self.done.wait(.5):
                atomic(self.run/'runner_heartbeat.json',dict(monotonic_wall=time.monotonic()))
                if self.proc.poll() is not None: raise RuntimeError('Independent guardian exited')
                p=json.loads((self.run/'guardian_heartbeat.json').read_text())
                if time.monotonic()-p['monotonic_wall']>3.: raise RuntimeError('Guardian heartbeat stale')
        except Exception as e:
            self.error=str(e)
            if not self.done.is_set(): os.kill(self.parent['pid'],signal.SIGTERM)
    def start_role(self,role,command,env,log):
        if self.error or self.proc.poll() is not None: raise RuntimeError('Guardian unavailable')
        child=subprocess.Popen([sys.executable,'-B',str(self.script),'--bootstrap',str(self.run),
            '--role',role,'--',*command],env={**env,MARKER:self.token},stdout=log,
            stderr=subprocess.STDOUT,start_new_session=True)
        saved=ident(child.pid)
        if saved is None: raise RuntimeError('Bootstrap identity unavailable')
        saved.update(ownership_token=self.token,role=role,owned_run=str(self.run))
        self.children[role]=(child,saved)
        atomic(self.run/'owned_roles'/(role+'.identity.json'),saved,exclusive=True)
        until=time.monotonic()+10.
        while not (self.run/'owned_roles'/(role+'.permit.json')).exists():
            if child.poll() is not None or self.proc.poll() is not None or time.monotonic()>until:
                raise RuntimeError('Role admission failed '+role)
            time.sleep(.02)
        return child,saved
    def close(self):
        self.done.set(); self.thread.join(timeout=2.)
        if any(validated_members(saved) for _,saved in self.children.values()):
            raise RuntimeError('Owned role remains before guardian release')
        atomic(self.run/'guardian_release.json',dict(parent=self.parent,monotonic_wall=time.monotonic()),exclusive=True)
        self.proc.wait(timeout=5.); self.log.close()
        result=json.loads((self.run/'guardian_result.json').read_text())
        if self.proc.returncode!=0 or not result['all_owned_groups_absent']:
            raise RuntimeError('Guardian release incomplete')
        return dict(identity=self.saved,returncode=self.proc.returncode,result=result,error=self.error)

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--watch'); ap.add_argument('--bootstrap')
    ap.add_argument('--role'); ap.add_argument('command',nargs=argparse.REMAINDER); a=ap.parse_args()
    if a.watch: raise SystemExit(watch(Path(a.watch)))
    command=a.command[1:] if a.command[:1]==['--'] else a.command
    raise SystemExit(bootstrap(a.bootstrap,a.role,command))
