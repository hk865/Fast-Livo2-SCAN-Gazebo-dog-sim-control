#!/usr/bin/env python3
"""Sequential prospective CPU simulation tests, using immutable new inputs.

No training/camera-mode mutation. Only run.py owns and stops its new children.
This script's receipt records a failed result without rewriting an ancestor.
"""
import datetime
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[1]

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def main():
    plan=json.loads((HERE/'active_hold_plan_v4r2.json').read_text())
    if plan['schema']!='prospective_tight_S_active_hold_campaign/v4r2':raise ValueError('Wrong plan')
    for name,digest in plan['frozen_sources'].items():
        if sha(name)!=digest:raise ValueError('Frozen source changed: '+name)
    registry=HERE/'active_hold_results_v4r2.jsonl'
    if registry.exists():raise FileExistsError('Preserve prior batch receipt; explicit new plan required')
    spec=importlib.util.spec_from_file_location('go2_new_hold_runner',ROOT/'navigation/curvature_tracking_v4r2/run.py')
    runner=importlib.util.module_from_spec(spec);spec.loader.exec_module(runner)
    for case in plan['cases']:
        p=Path(case['profile_path'])
        if sha(p)!=case['profile_sha256']:raise ValueError('Profile changed: '+str(p))
        print(json.dumps({'event':'starting','case_index':case['case_index'],'role':case['role']},ensure_ascii=False),flush=True)
        directory=runner.run(json.loads(p.read_text()),camera=case['camera'])
        receipt=directory/'summary_active_hold_independent.json'
        subprocess.run([sys.executable,str(directory/'sources/truth/active_hold_receipt.py'),'--run',str(directory)],check=True,
                       stdout=(directory/'active_hold_evaluation.log').open('w'),stderr=subprocess.STDOUT)
        outcome=json.loads(receipt.read_text())
        row={**case,'run':str(directory),'status':outcome['status'],'failed_checks':outcome['failed_checks'],
             'unverified_checks':outcome['unverified_checks'],'receipt_sha256':sha(receipt),
             'runtime_error':json.loads((directory/'runtime_manifest.json').read_text())['error'],
             'recorded_at':datetime.datetime.now(datetime.timezone.utc).isoformat()}
        with registry.open('a') as stream:stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        print(json.dumps({'event':'finished',**row},ensure_ascii=False),flush=True)
        if case['role']=='pilot' and outcome['status']!='passed':
            print(json.dumps({'event':'pilot_did_not_pass','confirmation_not_launched':True,
                              'reason':'Preserve pilot and diagnose before using unchanged candidate as confirmation'}),flush=True)
            return
    print(json.dumps({'event':'batch_complete','runs':len(plan['cases']),'registry':str(registry)}),flush=True)

if __name__=='__main__':main()
