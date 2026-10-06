#!/usr/bin/env python3
"""Sequential, immutable simulator experiments. Does not control other tasks."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import datetime

HERE=Path(__file__).resolve().parent
def main():
    a=argparse.ArgumentParser();a.add_argument('--plan',type=Path,required=True);args=a.parse_args()
    plan=json.loads(args.plan.read_text());destination=args.plan.parent/(args.plan.stem+'_results.jsonl')
    if destination.exists():raise ValueError('Existing campaign receipts are immutable; use a new plan')
    refs={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest()for name in ('core.py','worker.py','run.py','evaluate.py','protocol.json')}
    (args.plan.parent/(args.plan.stem+'_freeze.json')).write_text(json.dumps({'source_sha256':refs,'plan_sha256':hashlib.sha256(args.plan.read_bytes()).hexdigest(),'started_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()},indent=2)+'\n')
    with destination.open('x',buffering=1)as log:
        for entry in plan['profiles']:
            if any(hashlib.sha256((HERE/name).read_bytes()).hexdigest()!=digest for name,digest in refs.items()):raise ValueError('Controller or evaluator changed during campaign')
            p=Path(entry['path']).resolve();command=[sys.executable,str(HERE/'run.py'),'--profile',str(p)]
            if entry.get('camera'):command+=['--camera']
            completed=subprocess.run(command,capture_output=True,text=True)
            lines=completed.stdout.splitlines();output=json.loads(lines[-1])if lines else{}
            run=Path(output['run'])if output.get('run')else None
            summary=json.loads((run/'summary_truth_pid.json').read_text())if run and (run/'summary_truth_pid.json').exists()else{}
            record={'profile':str(p),'profile_sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'returncode':completed.returncode,'run':str(run)if run else None,'status':summary.get('status','runtime_failed'),'score':summary.get('score'),'runtime_error':output.get('runtime_error'),'failed_checks':{k:v for k,v in summary.get('checks',{}).items()if v.get('status')!='passed'},'error':summary.get('error'),'stderr':completed.stderr[-3000:]}
            log.write(json.dumps(record,allow_nan=False)+'\n');print(json.dumps(record,allow_nan=False),flush=True)

if __name__=='__main__':main()
