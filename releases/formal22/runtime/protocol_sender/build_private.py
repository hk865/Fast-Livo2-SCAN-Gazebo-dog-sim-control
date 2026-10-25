#!/usr/bin/env python3
"""Authorized single-worker compile only; all outputs stay in this worktree."""
from pathlib import Path
import os,sys,json,hashlib,time,subprocess,fcntl,contextlib,signal
HERE=Path(__file__).resolve().parent; W=HERE.parent; WS=HERE/'scan_ws'; E=HERE/'evidence';E.mkdir(exist_ok=True)
sys.path.insert(0,str(W/'candidate'))
from run import identity,group_members
from r85_session_guard import processes
LOCKS=('/tmp/go2_pid_physics_serial.lock','/tmp/go2_model_pid_simulation.lock','/opt/experiment/history/2026-10-14/task/r83_demo/r73_minimal_baseline_20261017/night_continue_20261017/DETACHED_DISPATCH.lock')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def size(p):
    ss=[x.stat()for x in p.rglob('*')if x.is_file()and not x.is_symlink()]
    return {'logical_bytes':sum(s.st_size for s in ss),'allocated_bytes':sum(s.st_blocks*512 for s in ss)}
def memory():
    v={l.split(':')[0]:int(l.split()[1])*1024 for l in Path('/proc/meminfo').read_text().splitlines()if l.startswith(('MemTotal:','MemAvailable:'))};assert v['MemAvailable']>=4*1024**3;return v
plan=json.loads((HERE/'BUILD_PLAN.json').read_text()); receipt={'compile_only':True,'ROS_started':False,'physics_started':False,'locks':LOCKS,'plan':plan,'samples':[],'success':False};saved=None;child=None;start=time.monotonic()
try:
 with contextlib.ExitStack()as stack:
    for p in LOCKS:
        f=stack.enter_context(open(p,'r'));fcntl.flock(f,fcntl.LOCK_EX|fcntl.LOCK_NB)
    proc=processes();assert proc['host_process_count']>=30,proc;assert not proc['matching_runtime'],proc;receipt['fresh_host']=proc;receipt['memory_before']=memory()
    receipt['source_before']={str(p.relative_to(WS)):sha(p)for p in (WS/'src').rglob('*')if p.is_file()}
    receipt['dependencies_before']={p:sha(p)for p in plan['expected_project_link_inputs_sha256']};assert receipt['dependencies_before']==plan['expected_project_link_inputs_sha256']
    prepared=subprocess.run(['bash','--noprofile','--norc','-c','source "$1" >/dev/null && env -0','protocol-build',plan['shell_environment_setup']],capture_output=True,check=True)
    env=dict(x.decode().split('=',1)for x in prepared.stdout.split(b'\0')if b'='in x);env.update(plan['environment_overrides'])
    out=stack.enter_context((E/'build.stdout.log').open('xb'));err=stack.enter_context((E/'build.stderr.log').open('xb'))
    child=subprocess.Popen(plan['argv'],cwd=WS,env=env,stdout=out,stderr=err,start_new_session=True);saved=identity(child.pid);assert saved and saved['pgid']==child.pid and saved['session']==child.pid;receipt['owner']=saved
    last=0
    while child.poll()is None:
        elapsed=time.monotonic()-start
        if elapsed>=600:raise TimeoutError('600 second private build cap')
        if elapsed-last>=10:
            ss=size(WS);assert max(ss.values())<3_000_000_000,ss;receipt['samples'].append({'elapsed_s':elapsed,'output':ss,'memory':memory()});last=elapsed
        time.sleep(.2)
    receipt['returncode']=child.returncode;assert child.returncode==0,child.returncode
    exe=WS/'install/scan_planner/lib/scan_planner/scan_planner_node';conf=WS/'install/scan_planner/share/scan_planner/config/planner.yaml';receipt['binary']={'path':str(exe),'bytes':exe.stat().st_size,'sha256':sha(exe)};receipt['config']={'path':str(conf),'sha256':sha(conf),'original_byte_identical':conf.read_bytes()==(WS/'src/plan_manage/config/planner.yaml').read_bytes()};assert receipt['config']['original_byte_identical']
    receipt['source_after']={str(p.relative_to(WS)):sha(p)for p in(WS/'src').rglob('*')if p.is_file()};assert receipt['source_before']==receipt['source_after']
    receipt['dependencies_after']={p:sha(p)for p in plan['expected_project_link_inputs_sha256']};assert receipt['dependencies_before']==receipt['dependencies_after']
    receipt['link_command']=(WS/'build/scan_planner/CMakeFiles/scan_planner_node.dir/link.txt').read_text();receipt['success']=True
except BaseException as exc:
 receipt['error']=type(exc).__name__+': '+str(exc)
 if child is not None and child.poll()is None and saved:
    for sig,seconds in [(signal.SIGINT,3),(signal.SIGTERM,2),(signal.SIGKILL,2)]:
        current=identity(saved['pid'])
        if not current:break
        assert all(current[k]==saved[k]for k in['pid','start_ticks','pgid','session'])
        os.killpg(saved['pgid'],sig)
        try:child.wait(timeout=seconds);break
        except subprocess.TimeoutExpired:pass
finally:
 receipt['wall_s']=time.monotonic()-start;receipt['output_size']=size(WS);receipt['remaining_build_group']=group_members(saved['pgid'])if saved else [];assert not [p for p in receipt['remaining_build_group']if p['state']!='Z'];receipt['source_manifest_sha256']=sha(HERE/'SOURCE_COPY_MANIFEST.json');(E/'BUILD_RECEIPT.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps({k:receipt.get(k)for k in['success','error','returncode','wall_s','binary','output_size']}))
if not receipt['success']:sys.exit(1)
