"""Independent bounded owner monitor; bootstrap gates every real role launch."""
from pathlib import Path
import argparse
import contextlib
import datetime as dt
import hashlib
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time
import types
import uuid

BOOT = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
MARKER = 'TEACHER_OWNED_RUN_TOKEN'


def load_pipeline_lifecycle(run,plan):
    """Load only the hash-bound sibling bytes, independent of legacy import state."""
    run=Path(run)
    path=Path(__file__).resolve().parent/'pipeline_lifecycle.py'
    manifest_path=run/'source_manifest.json'
    manifest_raw=manifest_path.read_bytes()
    manifest_sha=hashlib.sha256(manifest_raw).hexdigest()
    if manifest_sha!=plan.get('source_manifest_sha256'):
        raise RuntimeError('Guardian pipeline source manifest binding differs')
    expected=json.loads(manifest_raw)[str(path)]
    source=path.read_bytes();actual=hashlib.sha256(source).hexdigest()
    if actual!=expected:
        raise RuntimeError('Guardian sibling pipeline source hash differs')
    module=types.ModuleType('_r134_guardian_pipeline_lifecycle')
    module.__file__=str(path)
    # Compile these verified bytes; do not consume a cached module or stale pyc.
    exec(compile(source,str(path),'exec'),module.__dict__)
    functions={}
    for name in ('reuse_normal_stop','request_normal_stop','check_summary'):
        function=getattr(module,name)
        if Path(function.__code__.co_filename).resolve()!=path:
            raise RuntimeError('Guardian actual pipeline function code differs: '+name)
        functions[name]=function.__code__.co_filename
    witness=dict(schema='r134_guardian_actual_pipeline_loaded/v1',
        actual_module_file=str(path),actual_source_sha256=actual,source_manifest_sha256=manifest_sha,
        actual_function_code=functions,verified=True,module_cache_bypassed=True,
        verified_source_bytes_compiled=True,before_guardian_ready=True,navigation_verified=False)
    return module,witness


class GuardianOperationalStop(RuntimeError):
    """Owned cleanup proof never turns an operational stop into runtime success."""
    def __init__(self,result):
        self.observation=result
        super().__init__('Guardian operational stop: '+str(result.get('reason'))+
                         '; cleanup_verified='+str(result.get('cleanup_verified') is True))


def released_cleanup_proof(run,owner,roles,pipeline_module=None):
    """Validate original parent release, all registered owners, and prior drain."""
    run=Path(run)
    release=json.loads((run/'guardian_release.json').read_text())
    if not same(release.get('parent'),owner['parent']):
        raise RuntimeError('Guardian release parent identity differs')
    wall=release.get('monotonic_wall')
    if type(wall) not in (int,float) or not math.isfinite(wall) or wall>time.monotonic():
        raise RuntimeError('Guardian release time differs')
    registered={p.name.removesuffix('.identity.json'):json.loads(p.read_text())
                for p in (run/'owned_roles').glob('*.identity.json')}
    if registered!=roles:
        raise RuntimeError('Guardian release registered role set/identity differs')
    for role,saved in registered.items():
        if (role not in owner['allowed_roles'] or saved.get('ownership_token')!=owner['token']
                or saved.get('role')!=role or saved.get('owned_run')!=str(run)
                or saved.get('boot_id')!=BOOT or saved.get('pid')!=saved.get('pgid')
                or saved.get('session')!=saved.get('pid') or saved.get('ppid')!=owner['parent']['pid']):
            raise RuntimeError('Guardian release role owner differs')
        if validated_members(saved):
            raise RuntimeError('Guardian release while owned processes remain')
    if 'navigation_stack' not in roles:
        raise RuntimeError('No owned navigation role for completed drain proof')
    if pipeline_module is None:
        pipeline_module,_=load_pipeline_lifecycle(run,json.loads((run/'runtime_plan.json').read_text()))
    return pipeline_module.reuse_normal_stop(run,json.loads((run/'slam_loaded_binary.json').read_text()),
                                            roles['navigation_stack'],wall)


def soft_cleanup_room(observation,*,initial=False):
    """Classify an unchanged observation for stopped-input owner cleanup only."""
    if type(observation) is not dict:return False
    row=observation.get('R85_round_budget',{})
    reasons=observation.get('stop_reasons')
    if reasons not in ([],['fresh_round_predictive_stop']):return False
    if initial and reasons!=['fresh_round_predictive_stop']:return False
    predictive=reasons==['fresh_round_predictive_stop']
    if (observation.get('hard_bytes')!=170000000000
            or observation.get('soft_stop_bytes')!=169463129088
            or observation.get('budget_basis')!='current_retained_experiment_data'
            or observation.get('historical_counter_used_for_capacity') is not False
            or row.get('hard_B')!=10000000000 or row.get('hard_stop') is not False
            or row.get('predictive_soft_stop') is not predictive
            or row.get('soft_stop_requires_owner_handshake') is not True
            or row.get('atomic_allowance_B')!=134217728 or row.get('cleanup_reserve_B')!=536870912
            or row.get('predictive_grace_s')!=122.):return False
    numbers=[observation.get('current_retained_budget_bytes'),row.get('charged_B'),
             row.get('run_B'),row.get('engineering_B')]
    if any(type(value) is not int or value<0 for value in numbers):return False
    retained,charged,run_bytes,engineering=numbers
    projected=row.get('projected_full_stop_B')
    if type(projected) is not int or projected<0 or (projected>=10000000000)!=predictive:return False
    reserve=134217728+536870912  # Existing atomic and cleanup reserves, unchanged.
    if (retained+reserve>=169463129088 or charged+reserve>=10000000000
            or run_bytes>=6800000000 or engineering>=268435456):return False
    fs={item.get('path'):item.get('available_bytes') for item in observation.get('filesystems',[])}
    if any(type(fs.get(path)) is not int or fs[path]<minimum for path,minimum in
           (('/home',107374182400),('/tmp',85899345920))):return False
    return True


def wait_soft_parent_cleanup(run,owner,plan,roles,observation,pipeline_module,storage_check,until):
    """Keep the live soft-stop supervisor visible while the owner drains/closes.

    No archive eligibility/returncode is granted here. The parent's unchanged
    native and archive gates retain that decision, and the stop remains fatal.
    """
    if not soft_cleanup_room(observation,initial=True):return None
    run=Path(run);started=time.monotonic()
    try:
        proof=pipeline_module.reuse_normal_stop(run,json.loads((run/'slam_loaded_binary.json').read_text()),
                                              roles['navigation_stack'],started)
        utc=dt.datetime.fromisoformat(plan['profile']['r85_session']['deadline_utc'])
        if utc.tzinfo is None:raise RuntimeError('Soft cleanup requires original zoned UTC deadline')
        deadline=min(until,owner['wall_deadline'],started+122.,
                     started+(utc-dt.datetime.now(dt.timezone.utc)).total_seconds())
        if deadline<=started:raise RuntimeError('Soft cleanup original deadline exhausted')
    except Exception as error:
        return dict(entered=False,rejected=type(error).__name__+': '+str(error))
    evidence=dict(schema='r134_soft_stop_owner_cleanup/v1',started_monotonic_wall=started,
        deadline_monotonic=deadline,initial_observation=observation,normal_pipeline_proof=proof,
        source_plan_sha256=owner['runtime_plan_sha256'],parent=owner['parent'],
        original_prediction_unchanged=True,archive_eligibility_granted=False,
        original_worker_returncode_known=False,operational_stop=True,navigation_verified=False)
    atomic(run/'r134_soft_stop_handshake.json',evidence,exclusive=True)
    previous=observation;storage_at=started;abort=None;released=False
    while time.monotonic()<deadline:
        now=time.monotonic()
        parent=ident(owner['parent']['pid'])
        if not same(parent,owner['parent']) or parent['state']=='Z':
            abort='soft_cleanup_parent_identity_lost';break
        if now-json.loads((run/'runner_heartbeat.json').read_text())['monotonic_wall']>owner['heartbeat_timeout_s']:
            abort='soft_cleanup_parent_heartbeat_stale';break
        if (run/'guardian_release.json').exists():released=True;break
        registered={p.name.removesuffix('.identity.json'):json.loads(p.read_text())
                    for p in (run/'owned_roles').glob('*.identity.json')}
        if registered!=roles:abort='soft_cleanup_registered_role_identity_changed';break
        try:
            live={role:validated_members(saved) for role,saved in roles.items()}
        except Exception as error:
            abort='soft_cleanup_owned_identity: '+type(error).__name__+': '+str(error);break
        atomic(run/'guardian_heartbeat.json',dict(monotonic_wall=now,state='STOPPING_SOFT',
            non_worker_groups_absent=not any(v for k,v in live.items() if k!='worker'),
            operational_stop=True,archive_eligibility_granted=False))
        if now-storage_at>=1.:
            try:
                current=storage_check(previous)
            except Exception as error:
                if type(error).__name__!='StorageStop':
                    abort='soft_cleanup_storage_error: '+type(error).__name__+': '+str(error);break
                current=getattr(error,'observation',None)
            after=time.monotonic()
            atomic(run/'r134_soft_stop_storage_last.json',current)
            if after>=deadline or after-now>3.:
                abort='soft_cleanup_storage_callback_overdue';break
            if not soft_cleanup_room(current):
                abort='soft_cleanup_storage_hard_FS_deadline_or_other_stop';break
            previous=current;storage_at=now  # Same callback-start cadence as the running monitor.
        time.sleep(min(.1,max(0.,deadline-time.monotonic())))
    if not released:
        abort=abort or 'soft_cleanup_original_deadline_exhausted'
        atomic(run/'guardian_heartbeat.json',dict(monotonic_wall=time.monotonic(),
            state='STOPPING_ABORT',reason=abort,operational_stop=True))
    result=dict(entered=True,released=released,abort_reason=abort,
        ended_monotonic_wall=time.monotonic(),deadline_monotonic=deadline,
        original_prediction_unchanged=True,archive_eligibility_granted=False,navigation_verified=False)
    atomic(run/'r134_soft_stop_wait_result.json',result,exclusive=True)
    return result

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

def _watch(run, storage_check=None):
    run=Path(run); owner=json.loads((run/'guardian_owner.json').read_text())
    plan_raw=(run/'runtime_plan.json').read_bytes();plan=json.loads(plan_raw)
    if (plan.get('profile',{}).get('r85_session',{}).get('enabled')is True
            and owner.get('runtime_plan_sha256')!=hashlib.sha256(plan_raw).hexdigest()):
        raise RuntimeError('R85 independent guardian frozen plan binding differs')
    pipeline_module,pipeline_loaded=load_pipeline_lifecycle(run,plan)
    atomic(run/'r134_guardian_pipeline_loaded.json',pipeline_loaded,exclusive=True)
    if storage_check is None:
        from storage_guard import observe
        storage_check=lambda previous:observe(run,plan['profile'],previous=previous)
    for s in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP): signal.signal(s,signal.SIG_IGN)
    mine=ident(os.getpid()); last_storage=None; storage_at=0.; roles={}; reason=None
    stop_observation=None;stop_evidence_error=None;release_proof_error=None;soft_wait=None
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
        try:
            stop_observation=getattr(error,'observation',None)
            atomic(run/'guardian_stop_cause.json',dict(reason=reason,
                caught_monotonic_wall=time.monotonic(),observation=stop_observation,
                last_successful_storage_monotonic_wall=None if last_storage is None else last_storage.get('monotonic_wall'),
                navigation_verified=False),exclusive=True)
        except Exception as receipt_error:
            stop_evidence_error=type(receipt_error).__name__+': '+str(receipt_error)
            print('guardian_stop_cause_write_failed: '+stop_evidence_error,file=sys.stderr,flush=True)
    # A surviving parent gets a bounded chance to use its normal cleanup.
    parent=ident(owner['parent']['pid'])
    if same(parent,owner['parent']) and parent['state']!='Z':
        os.kill(parent['pid'],signal.SIGTERM)
        until=time.monotonic()+owner['parent_cleanup_grace_s']
        try:
            soft_wait=wait_soft_parent_cleanup(run,owner,plan,roles,stop_observation,
                                               pipeline_module,storage_check,until)
        except Exception as error:
            soft_wait=dict(entered=True,abort_reason=type(error).__name__+': '+str(error))
            atomic(run/'guardian_heartbeat.json',dict(monotonic_wall=time.monotonic(),
                state='STOPPING_ABORT',reason=soft_wait['abort_reason'],operational_stop=True))
        if not soft_wait or not soft_wait.get('entered'):
            while time.monotonic()<until:
                if (run/'guardian_release.json').exists() or not same(ident(parent['pid']),parent): break
                time.sleep(.1)
    if (run/'guardian_release.json').exists():
        try:
            pipeline=released_cleanup_proof(run,owner,roles,pipeline_module)
        except Exception as proof_error:
            release_proof_error=type(proof_error).__name__+': '+str(proof_error)
        else:
            atomic(run/'pipeline_guardian_stop.json',pipeline,exclusive=True)
            atomic(run/'guardian_result.json',dict(status='parent_cleanup_verified_after_stop',
                reason=reason,utc=dt.datetime.now(dt.timezone.utc).isoformat(),
                cleanup_verified=True,operational_stop=True,resource_observation=stop_observation,
                soft_stop_owner_wait=soft_wait,
                stop_evidence_error=stop_evidence_error,pipeline=pipeline,
                remaining={role:[] for role in roles},all_owned_groups_absent=True,
                original_runner_exitcode='unknown',navigation_verified=False),exclusive=True)
            return 1  # Cleanup succeeded; resource/other operational stop remains fatal.
    atomic(run/'guardian_recovery_intent.json',dict(reason=reason,roles=roles,
        utc=dt.datetime.now(dt.timezone.utc).isoformat(),original_exitcode='unknown'),exclusive=True)
    pipeline=None
    witness=run/'slam_loaded_binary.json'
    if witness.exists() and 'navigation_stack' in roles:
        pipeline=pipeline_module.request_normal_stop(run,json.loads(witness.read_text()),roles['navigation_stack'],ident,
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
        cleanup_verified=False,operational_stop=True,resource_observation=stop_observation,
        soft_stop_owner_wait=soft_wait,
        stop_evidence_error=stop_evidence_error,parent_release_proof_error=release_proof_error,
        utc=dt.datetime.now(dt.timezone.utc).isoformat(),cleanup=cleanup,pipeline=pipeline,
        remaining=remaining,all_owned_groups_absent=not any(remaining.values()),
        original_runner_exitcode='unknown',navigation_verified=False),exclusive=True)
    parent=ident(owner['parent']['pid'])
    if same(parent,owner['parent']) and not (run/'guardian_release.json').exists():
        os.kill(parent['pid'],signal.SIGKILL)
    return 1


def terminal_facts(run, observer_error=None):
    """Record shutdown evidence, including missing archives, without a PASS.

    The guardian is not the worker's parent. Its normal release proves owned
    cleanup only, and recovery never invents the original runner exit code.
    Large streams/arrays are inventoried, not rewritten or fully read here.
    """
    run=Path(run)
    receipt=dict(schema='owned_guardian_terminal_facts/v1',run_id=run.name,
        utc=dt.datetime.now(dt.timezone.utc).isoformat(),observer='independent_owned_guardian',
        observer_error=observer_error,navigation_verified=False,worker_PASS_created=False,
        original_artifacts_repaired=False,artifacts={},all_owned_groups_absent=None)
    for name in ('guardian_result.json','guardian_recovery_intent.json',
                 'guardian_worker_archive_close.json','guardian_worker_archive_integrity.json',
                 'worker_result.json','worker_archive_integrity.json','r85_task_result.json',
                 'runtime_manifest.json','run_result.json','observations_actions.npz'):
        path=run/name
        row=dict(exists=path.exists(),regular_non_symlink=False)
        receipt['artifacts'][name]=row
        try:
            if path.is_symlink() or not path.is_file():continue
            row.update(regular_non_symlink=True,bytes=path.stat().st_size)
            if name.endswith('.json') and row['bytes']<=2*1024*1024:
                raw=path.read_bytes();row['sha256']=hashlib.sha256(raw).hexdigest()
                data=json.loads(raw)
                row['recorded_fields']={key:data[key] for key in (
                    'status','state','completed','samples','fault','passed','failure_reason',
                    'reason','verified','error','all_owned_groups_absent',
                    'original_runner_exitcode','worker_returncode','navigation_verified')if key in data}
                if name=='guardian_result.json':
                    receipt['guardian_status']=data.get('status')
                    receipt['all_owned_groups_absent']=data.get('all_owned_groups_absent')
                    receipt['original_runner_exitcode']=data.get('original_runner_exitcode','unknown')
        except Exception as error:
            row['inspection_error']=type(error).__name__+': '+str(error)
    atomic(run/'terminal_facts.json',receipt,exclusive=True)
    return receipt


def watch(run, storage_check=None):
    observer_error=None
    try:
        return _watch(run,storage_check)
    except BaseException as error:
        observer_error=type(error).__name__+': '+str(error)
        raise
    finally:
        try:terminal_facts(run,observer_error)
        except Exception as error:
            # Filesystem failure cannot be represented as a successful archive.
            print('terminal_facts_write_failed: '+type(error).__name__+': '+str(error),
                  file=sys.stderr,flush=True)


class Guardian:
    def __init__(self,run,plan,*,module_path=None):
        self.run=Path(run); self.script=Path(module_path or __file__).resolve()
        self.done=threading.Event(); self.error=None
        self.token=uuid.uuid4().hex; self.parent=ident(os.getpid()); self.children={}
        (self.run/'owned_roles').mkdir()
        atomic(self.run/'guardian_owner.json',dict(parent=self.parent,token=self.token,
            runtime_plan_sha256=hashlib.sha256((self.run/'runtime_plan.json').read_bytes()).hexdigest(),
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
                if p.get('state')=='STOPPING_ABORT':raise RuntimeError('Guardian cleanup aborted: '+str(p.get('reason')))
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
        if self.proc.returncode!=0 and result.get('reason'):
            raise GuardianOperationalStop(result)
        if self.proc.returncode!=0 or not result['all_owned_groups_absent']:
            raise RuntimeError('Guardian release incomplete')
        return dict(identity=self.saved,returncode=self.proc.returncode,result=result,error=self.error)

if __name__=='__main__':
    ap=argparse.ArgumentParser(); ap.add_argument('--watch'); ap.add_argument('--bootstrap')
    ap.add_argument('--role'); ap.add_argument('command',nargs=argparse.REMAINDER); a=ap.parse_args()
    if a.watch: raise SystemExit(watch(Path(a.watch)))
    command=a.command[1:] if a.command[:1]==['--'] else a.command
    raise SystemExit(bootstrap(a.bootstrap,a.role,command))
