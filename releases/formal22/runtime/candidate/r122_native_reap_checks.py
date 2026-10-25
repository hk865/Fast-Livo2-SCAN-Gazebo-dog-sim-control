"""Original stop/archive control with scheduled owned-process fixtures; no OS launch."""
from pathlib import Path
import argparse
import ast
import hashlib
import json
import math
import signal
import subprocess
import types
from unittest.mock import patch

import owned_guardian
import worker_archive_shutdown as archive

HERE=Path(__file__).resolve().parent
TREE=ast.parse((HERE/'run.py').read_text())
STOP=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='stop_owned')
ABORT=next(n for n in ast.walk(TREE) if isinstance(n,ast.FunctionDef) and n.name=='archive_abort_reason')


class World:
    def __init__(self,*,gone=.1,waitable=.4,foreign=False,native_rc=0,worker_rc=0,reap_error=False):
        self.t=0.;self.gone=gone;self.waitable=waitable;self.foreign=foreign
        self.native_rc=native_rc;self.worker_rc=worker_rc;self.worker_done=1.1
        self.reap_error=reap_error
        self.worker_interrupted=False;self.fixture_archive_completed=False;self.signals=[];self.waits=[]
        self.native=Process(self,100,'gazebo');self.worker=Process(self,200,'worker')
    def now(self):return self.t
    def advance(self,seconds):
        self.t+=seconds
        if self.t>=self.worker_done and not self.worker_interrupted and self.worker_rc==0:
            self.fixture_archive_completed=True
    def identity(self,pid):
        if pid==100 and (self.gone is None or self.t<self.gone):
            value=self.native.saved.copy()
            if self.foreign:value['start_ticks']=999
            return value
        if pid==200 and self.worker.poll() is None:return self.worker.saved.copy()
        return None
    def members(self,pgid):
        if pgid==100:return [self.native.saved] if self.gone is None or self.t<self.gone else []
        if pgid==200:return [self.worker.saved] if self.worker.poll() is None else []
        return []
    def validated(self,saved):
        current=self.identity(saved['pid'])
        if current and any(current[k]!=saved[k] for k in ('pid','start_ticks','pgid','session','boot_id')):
            raise RuntimeError('fixture foreign process identity')
        return self.members(saved['pgid'])
    def killpg(self,pgid,sig):
        self.signals.append(dict(pgid=pgid,signal=int(sig),monotonic_wall=self.t))
        if pgid==200:self.worker_interrupted=True
    def stop(self,node=STOP):
        scope=dict(time=types.SimpleNamespace(monotonic=self.now,sleep=self.advance),
            math=math,signal=signal,subprocess=subprocess,identity=self.identity,group_members=self.members,
            os=types.SimpleNamespace(killpg=self.killpg,kill=self.killpg))
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(HERE/'run.py'),'exec'),scope)
        return scope['stop_owned']
    def abort(self,cleanup,*,storage_error=None,runner_deadline=100.):
        scope=dict(startup_stop=dict(reason='actual_R85_task_failed'),plan=dict(profile={}),
            cpu_archive_budget=dict(eligible=False),sample_complete=False,children=dict(worker=self.worker),
            pipeline_stop=dict(normal_completed=True),cleanup=cleanup,
            watchdog=types.SimpleNamespace(error=storage_error),guardian=None,
            archive_deadlines=dict(runner=runner_deadline),time=types.SimpleNamespace(monotonic=self.now))
        exec(compile(ast.Module(body=[ABORT],type_ignores=[]),str(HERE/'run.py'),'exec'),scope)
        return scope['archive_abort_reason']


class Process:
    def __init__(self,world,pid,role):
        self.world=world;self.pid=pid;self.role=role;self.returncode=None
        self.saved=dict(pid=pid,pgid=pid,session=pid,start_ticks=pid,boot_id='fixture',ownership_token='fixture')
    def poll(self):
        w=self.world
        if self.role=='gazebo':
            if w.waitable is not None and w.t>=w.waitable:self.returncode=w.native_rc
        elif w.worker_interrupted:self.returncode=-2
        elif w.t>=w.worker_done:
            self.returncode=w.worker_rc
            if w.worker_rc==0:w.fixture_archive_completed=True
        return self.returncode
    def wait(self,timeout):
        w=self.world;w.waits.append(dict(pid=self.pid,started=w.t,timeout=timeout))
        if self.role=='gazebo' and w.reap_error:
            self.returncode=0  # A cache fallback cannot substitute for failed wait evidence.
            raise ChildProcessError('fixture child no longer waitable')
        ready=w.waitable if self.role=='gazebo' else w.worker_done
        if ready is None or ready>w.t+timeout:
            w.advance(timeout);raise subprocess.TimeoutExpired('owned_fixture',timeout)
        w.advance(max(0.,ready-w.t));return self.poll()


def sequence(world,*,deadline=10.,stop_node=STOP,storage_error=None,runner_deadline=100.):
    stop=world.stop(stop_node)
    with patch.object(owned_guardian,'validated_members',side_effect=world.validated):
        kwargs=dict(absolute_deadline=deadline) if stop_node is STOP else {}
        physics=stop(world.native,world.native.saved,**kwargs)
        result=archive.await_archive_close(world.worker,world.worker.saved,physics,world.identity,world.validated,
            abort_reason=world.abort(dict(gazebo=physics),storage_error=storage_error,runner_deadline=runner_deadline),
            clock=world.now,sleep=world.advance,maximum_wait_s=30.,absolute_deadline=deadline)
        worker=stop(world.worker,world.worker.saved,**kwargs)
    return dict(physics=physics,archive_close=result,worker=worker,signals=world.signals,waits=world.waits,
                elapsed_s=world.t,fixture_archive_completed=world.fixture_archive_completed)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    checks={};details={}
    # Use the exact frozen pre-change function, extracted from Git, to show the
    # counterexample through the same archive helper. This command reads Git only.
    old_source=subprocess.check_output(['git','show','95eed066:candidate/run.py'],cwd=HERE,text=True)
    old_node=next(n for n in ast.parse(old_source).body if isinstance(n,ast.FunctionDef) and n.name=='stop_owned')
    d=details['original_counterexample']=sequence(World(),stop_node=old_node)
    checks['original_sequence_reproduces_unconfirmed_native_and_worker_SIGINT']=(
        d['physics']['returncode'] is None and d['physics']['remaining_owned_group_members']==[] and
        d['archive_close']['reason']=='native_physics_stop_not_confirmed' and d['worker']['returncode']==-2 and
        not d['fixture_archive_completed'] and any(x['pgid']==200 and x['signal']==2 for x in d['signals']))
    d=details['late_reap_then_natural_archive']=sequence(World())
    checks['late_native_returncode_confirmed_within_existing_deadline']=(d['physics']['returncode']==0 and
        d['physics']['remaining_owned_group_members']==[] and d['physics']['returncode_confirmation']['wait_attempted'] and
        d['elapsed_s']<=10.)
    checks['confirmed_native_allows_original_30s_task_stop_grace_without_worker_signal']=(
        d['archive_close']['maximum_grace_s']==30. and d['archive_close']['natural_worker_exit'] is True and
        d['archive_close']['worker_returncode']==0 and d['fixture_archive_completed'] and
        not any(x['pgid']==200 for x in d['signals']))
    d=details['unknown_timeout']=sequence(World(waitable=None),deadline=.25)
    checks['unknown_timeout_never_manufactures_returncode_or_archive_success']=(
        d['physics']['returncode'] is None and d['physics']['returncode_confirmation']['timed_out'] and
        d['archive_close']['reason']=='native_physics_stop_not_confirmed' and d['elapsed_s']<=.25)
    d=details['group_still_alive']=sequence(World(gone=None,waitable=None),deadline=.25)
    checks['live_group_preserves_escalation_and_no_reap_wait']=(
        bool(d['physics']['remaining_owned_group_members']) and not d['physics']['returncode_confirmation']['wait_attempted'] and
        [x['signal'] for x in d['signals'] if x['pgid']==100]==[2,15,9] and d['elapsed_s']<=.25)
    w=World(foreign=True)
    with patch.object(owned_guardian,'validated_members',side_effect=w.validated):
        try:w.stop()(w.native,w.native.saved,absolute_deadline=1.)
        except RuntimeError:checks['foreign_PID_rejected_before_any_signal_or_wait']=not w.signals and not w.waits
        else:checks['foreign_PID_rejected_before_any_signal_or_wait']=False
    d=details['immediate_native_exit']=sequence(World(gone=0.,waitable=0.))
    checks['normal_immediate_no_reap_wait_and_natural_worker']=not d['physics']['returncode_confirmation']['wait_attempted'] and d['fixture_archive_completed']
    d=details['expired_deadline']=sequence(World(gone=0.,waitable=.4),deadline=0.)
    checks['expired_deadline_never_waits_or_grants_native_confirmation']=(not d['waits'] and
        d['physics']['returncode'] is None and d['elapsed_s']==0. and d['archive_close']['reason']=='native_physics_stop_not_confirmed')
    d=details['resource_stop_priority']=sequence(World(),storage_error='StorageStop: fixture hard gate')
    checks['resource_stop_still_cancels_grace_and_preserves_worker_escalation']=(
        d['archive_close']['reason']=='grace_cancelled: storage watchdog: StorageStop: fixture hard gate' and
        not d['fixture_archive_completed'] and d['worker']['returncode']==-2)
    d=details['nonzero_native_exit']=sequence(World(native_rc=-2))
    checks['nonzero_native_exit_still_fails_original_cleanup_gate']=d['archive_close']['reason']=='grace_cancelled: original owned role cleanup incomplete: gazebo'
    d=details['worker_failure']=sequence(World(worker_rc=1))
    checks['worker_nonzero_exit_never_counts_as_successful_archive']=(
        d['archive_close']['worker_returncode']==1 and not d['fixture_archive_completed'])
    d=details['lost_wait_evidence']=sequence(World(reap_error=True))
    checks['failed_reap_cannot_upgrade_cached_zero_to_confirmed_exit']=(
        d['physics']['returncode'] is None and 'ChildProcessError' in d['physics']['returncode_confirmation']['error'] and
        d['archive_close']['reason']=='native_physics_stop_not_confirmed')
    source_sha=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    report=dict(schema='r122_original_owned_stop_archive_sequence_checks/v1',passed=all(checks.values()),checks=checks,details=details,
        original_function_commit='95eed066',original_function_AST_sha256=hashlib.sha256(ast.dump(old_node,include_attributes=False).encode()).hexdigest(),
        unchanged_archive_helper_sha256=source_sha(HERE/'worker_archive_shutdown.py'),
        unchanged_original_archive_abort_AST_sha256=hashlib.sha256(ast.dump(ABORT,include_attributes=False).encode()).hexdigest(),
        source_sha256={str(HERE/name):source_sha(HERE/name) for name in ('run.py','r122_native_reap_checks.py')},
        evidence_scope='scheduled fixture processes and original source functions; no real Actor or archive proof',
        actual_OS_signals=False,runtime_process_or_ROS_or_model_launched=False,
        read_only_Git_subprocess=True,arrays_read_or_CRC_invoked=False,
        original_D06_evidence_modified=False,navigation_PASS=False)
    with a.output.open('x')as stream:json.dump(report,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(passed=report['passed'],checks=checks),indent=2))
    if not report['passed']:raise SystemExit(2)


if __name__=='__main__':main()
