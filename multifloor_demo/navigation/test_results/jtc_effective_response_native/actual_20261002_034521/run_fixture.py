"""Owned ROS77 native quasistatic response; no Gazebo or motion publisher."""
from pathlib import Path
import hashlib,json,os,signal,subprocess,time,shutil
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[2]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
frozen=json.loads((ROOT/'test_results/full18_freeze/source_manifest.json').read_text())['sha256']
before={n:sha(ROOT/n) for n in frozen}
attempt=HERE/('actual_'+time.strftime('%Y%m%d_%H%M%S'));attempt.mkdir()
binary=HERE/'build/jtc_effective_response_native'
for name in ('fixture.cpp','parameters.yaml','CMakeLists.txt','run_fixture.py','analyze.py'):
    shutil.copyfile(HERE/name,attempt/name)
shutil.copyfile(binary,attempt/binary.name)
env=os.environ.copy();env['ROS_DOMAIN_ID']='77';env['ROS_LOCALHOST_ONLY']='1'
runs=[]
for flag in (0,1):
    for batch in range(3):
        label=('true' if flag else 'false')+'_batch'+str(batch)
        out=attempt/(label+'.jsonl')
        with (attempt/(label+'.log')).open('w') as log:
            start=time.monotonic()
            p=subprocess.Popen([str(binary),str(HERE/'parameters.yaml'),str(out),str(flag),str(batch)],
                 env=env,start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
            timeout=False
            try:p.wait(timeout=20)
            except subprocess.TimeoutExpired:
                timeout=True;os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=3)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            owned=subprocess.run(['ps','-g',str(p.pid),'-o','pid=,args='],capture_output=True,text=True)
            runs.append(dict(flag=bool(flag),batch=batch,pid=p.pid,exit_code=p.returncode,timeout=timeout,
                 wall_duration_s=time.monotonic()-start,owned_group_clean=not owned.stdout.strip(),output=out.name))
after={n:sha(ROOT/n) for n in frozen}
result=dict(scope=__doc__,domain=77,runs=runs,source_269_match_before=before==frozen,
      source_269_match_after=after==frozen,source_changes=[n for n in frozen if before[n]!=after[n]],
      source_sha256={n:sha(attempt/n) for n in ('fixture.cpp','parameters.yaml','CMakeLists.txt','run_fixture.py','analyze.py',binary.name)})
(attempt/'execution_receipt.json').write_text(json.dumps(result,indent=2)+'\n')
(HERE/'latest_attempt.txt').write_text(str(attempt)+'\n')
print(json.dumps(dict(attempt=str(attempt),runs=runs,source_269_unchanged=before==after==frozen),indent=2))
raise SystemExit(not all(x['exit_code']==0 and x['owned_group_clean'] and not x['timeout'] for x in runs))
