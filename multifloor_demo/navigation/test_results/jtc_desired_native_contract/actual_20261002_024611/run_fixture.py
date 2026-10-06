"""Owned ROS77 actual JTC contract, no Gazebo or robot topics."""
from pathlib import Path
import hashlib,json,os,signal,subprocess,time
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[2]
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
frozen=json.loads((ROOT/'test_results/full18_freeze/source_manifest.json').read_text())['sha256']
before={n:sha(ROOT/n) for n in frozen}
attempt=HERE/('actual_'+time.strftime('%Y%m%d_%H%M%S'));attempt.mkdir()
env=os.environ.copy();env['ROS_DOMAIN_ID']='77';env['ROS_LOCALHOST_ONLY']='1'
binary=HERE/'build/jtc_desired_native_contract'
results=[]
for flag in [0,1]:
    out=attempt/('true.jsonl' if flag else 'false.jsonl')
    with (attempt/('true.log' if flag else 'false.log')).open('w') as log:
        start=time.monotonic()
        p=subprocess.Popen([str(binary),str(HERE/'parameters.yaml'),str(out),str(flag)],env=env,
            start_new_session=True,stdout=log,stderr=subprocess.STDOUT)
        timeout=False
        try:p.wait(timeout=20)
        except subprocess.TimeoutExpired:
            timeout=True;os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        owned=subprocess.run(['ps','-g',str(p.pid),'-o','pid=,args='],capture_output=True,text=True)
        results.append(dict(flag=bool(flag),pid=p.pid,exit_code=p.returncode,timeout=timeout,
            wall_duration_s=time.monotonic()-start,owned_group_clean=not owned.stdout.strip(),
            output=str(out),output_sha256=sha(out) if out.exists() else None,
            maps=str(out)+'.maps'))
after={n:sha(ROOT/n) for n in frozen}
r=dict(scope=__doc__,domain=77,runs=results,source_269_match_before=before==frozen,
    source_269_match_after=after==frozen,source_changes=[n for n in frozen if before[n]!=after[n]],
    sha256={f:sha(HERE/f) for f in ['fixture.cpp','parameters.yaml','CMakeLists.txt','build/jtc_desired_native_contract','run_fixture.py']})
(attempt/'execution_receipt.json').write_text(json.dumps(r,indent=2)+'\n')
(HERE/'latest_attempt.txt').write_text(str(attempt)+'\n')
print(json.dumps(r,indent=2))
raise SystemExit(not all(x['exit_code']==0 and x['owned_group_clean'] for x in results))
