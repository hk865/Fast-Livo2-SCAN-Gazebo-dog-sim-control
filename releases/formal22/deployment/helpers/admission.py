"""Host-only, read-only preflight and prior real Actor closure verification."""
from contextlib import ExitStack
from pathlib import Path
import datetime as dt
import fcntl
import hashlib
import json
import os
import subprocess
import sys

A=Path(__file__).resolve().parent
D=A.parent
C=D/'source/candidate'
R=Path('/opt/experiment/projects/go2_teacher_pipeline_v19_20261006_runs')
LOCKS=('/tmp/go2_pid_physics_serial.lock','/tmp/go2_model_pid_simulation.lock',
 '/opt/experiment/history/2026-10-14/task/r83_demo/r73_minimal_baseline_20261017/night_continue_20261017/DETACHED_DISPATCH.lock')
sys.path.insert(0,str(C))
os.environ['GO2_R73_MINIMAL_SIMULATION']='1'
from run import resource,resource_guard,require_private_domain
from r85_session_guard import census,processes
from worker_archive_shutdown import inspect_worker_archive
import owned_guardian


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def inspect(*,domain=220,verify_archives=True,outer_locks_owned=False,maximum_retained_bytes=160000000000):
    if type(maximum_retained_bytes)is not int or maximum_retained_bytes not in (160000000000,170000000000):
        raise ValueError('Unsupported explicit retained budget')
    if Path('/proc/1/comm').read_text().strip()!='systemd':raise RuntimeError('Host namespace required')
    v=dict(schema='r103_structural_fresh_host_admission/v1',UTC=dt.datetime.now(dt.timezone.utc).isoformat(),
        resources=resource(),processes=processes(),retained=census(),locks=[],prior_runs=[],stop_reasons=[],
        source_commit=subprocess.check_output(['git','rev-parse','HEAD'],cwd=D/'source',text=True).strip(),
        source_status=subprocess.check_output(['git','status','--porcelain'],cwd=D/'source',text=True),
        signals_sent=False,data_deleted=False,simulation_started=False)
    reasons=v['stop_reasons']
    try:resource_guard(v['resources'])
    except Exception as error:reasons.append(type(error).__name__+': '+str(error))
    if v['source_status']:reasons.append('Candidate has local changes')
    if v['source_commit']!='45396505e8facd808e38c28d674aa5a5c9c25947':reasons.append('Shared source HEAD differs from frozen baseline')
    if v['processes']['host_process_count']<30 or v['processes']['matching_runtime']:reasons.append('Host visibility/runtime conflict')
    if v['retained']['scan_errors']:reasons.append('Retained census errors')
    retained=v['retained']['current_retained_budget_bytes']
    v['budget']=dict(retained_bytes=retained,round_bytes=10000000000,additional_reserved_bytes=10000000000,
        projected_bytes=retained+20000000000,limit_bytes=maximum_retained_bytes)
    if retained+20000000000>maximum_retained_bytes:reasons.append(str(maximum_retained_bytes//1000000000)+'GB retained budget with round and extra reserve exceeded')
    v['filesystem_available']={p:os.statvfs(p).f_bavail*os.statvfs(p).f_frsize for p in ('/home','/tmp')}
    for p,minimum in (('/home',100*1024**3),('/tmp',80*1024**3)):
        if v['filesystem_available'][p]<minimum:reasons.append('Filesystem reserve: '+p)
    v['blender_pids']=[]
    for path in Path('/proc').glob('[0-9]*/comm'):
        try:
            if 'blender' in path.read_text().lower():v['blender_pids'].append(int(path.parent.name))
        except (FileNotFoundError,ProcessLookupError):pass
    if v['blender_pids']:reasons.append('External renderer remains; no action taken')
    for run in sorted(R.glob('20261023T*_closed_loop_R85_R100*')):
        row=dict(run=str(run),members={},runtime_manifest_sha256=None)
        for path in (run/'owned_roles').glob('*.identity.json'):
            row['members'][path.name]=owned_guardian.validated_members(json.loads(path.read_text()))
        if any(row['members'].values()):reasons.append('Prior owned role remains: '+run.name)
        for name in ('runtime_manifest.json','worker_archive_close.json','worker_archive_integrity.json','guardian_result.json'):
            p=run/name
            if p.exists():row[name]=json.loads(p.read_text());row[name+'_sha256']=sha(p)
        if verify_archives and row.get('worker_archive_integrity.json',{}).get('verified') is True:
            current=inspect_worker_archive(run)
            row['archive_fresh_readback']=current
            original=row['worker_archive_integrity.json']
            row['archive_matches_original']=current==original
            if not row['archive_matches_original']:reasons.append('Prior real Actor archive changed: '+run.name)
        v['prior_runs'].append(row)
    v['created_coordination_lockfiles']=[]
    with ExitStack() as stack:
        for path in LOCKS:
            if outer_locks_owned and path!=LOCKS[0]:
                v['locks'].append(dict(path=path,owned_by_outer_launcher=True));continue
            try:f=stack.enter_context(open(path,'r'))
            except FileNotFoundError:
                try:
                    f=stack.enter_context(open(path,'x+'))
                    v['created_coordination_lockfiles'].append(path)
                except FileExistsError:f=stack.enter_context(open(path,'r'))
            try:fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB);available=True
            except BlockingIOError:available=False;reasons.append('Lock busy: '+path)
            v['locks'].append(dict(path=path,available=available))
        try:require_private_domain(domain);v['domain']=dict(id=domain,empty=True)
        except Exception as error:v['domain']=dict(id=domain,empty=False,error=str(error));reasons.append('DDS domain unavailable')
    v['admitted']=not reasons
    return v


if __name__=='__main__':
    output=A/'INITIAL_HOST_AND_ACTOR_CLOSURE.json'
    if output.exists():raise RuntimeError('Fresh output required')
    value=inspect()
    with output.open('x') as stream:json.dump(value,stream,indent=2);stream.write('\n')
    print(json.dumps(dict(admitted=value['admitted'],stop_reasons=value['stop_reasons'],
        source_commit=value['source_commit'],budget=value['budget'],
        prior_Actor_archives_verified=sum(r.get('archive_matches_original',False) for r in value['prior_runs']))))
    if not value['admitted']:raise SystemExit(2)
