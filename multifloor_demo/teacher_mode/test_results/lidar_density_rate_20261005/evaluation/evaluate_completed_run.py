#!/usr/bin/env python3
"""Append frozen common receipt after completed simulation; preserve originals."""
from pathlib import Path
import argparse,hashlib,json,os,subprocess,sys

def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run',type=Path,required=True);ap.add_argument('--suffix',required=True);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
    run=a.run.resolve();a.output.mkdir(parents=True,exist_ok=True)
    worker=json.loads((run/'worker_result.json').read_text());runtime=json.loads((run/'runtime_manifest.json').read_text());writer=json.loads((run/'fastlivo_diagnostics/writer_stats.json').read_text())
    if worker.get('completed')is not True or writer.get('final')is not True:
        raise ValueError('Actual worker/logger has not completed; no final evaluation')
    if runtime.get('all_owned_and_children_clean')is not True:
        raise ValueError('Owned process cleanup is not confirmed')
    snapshots=json.loads((run/'navigation_source_snapshots.json').read_text())
    matches=[v for k,v in snapshots.items()if Path(k).name=='evaluate_closed_loop.py']
    if len(matches)!=1:raise ValueError('No unique frozen common evaluator')
    frozen=Path(matches[0]['snapshot']).resolve()
    if not frozen.is_relative_to((run/'sources').resolve())or sha(frozen)!=matches[0]['sha256']:
        raise ValueError('Frozen evaluator differs from prospective hash')
    source=run/('summary_closed_loop_cascade_independent.'+a.suffix+'.json')
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1')
    if not source.exists():
        result=subprocess.run([sys.executable,'-B',str(frozen),'--run',str(run),'--receipt-suffix',a.suffix],env=env,text=True,capture_output=True)
        (a.output/('common_'+a.suffix+'_execution.log')).write_text(result.stdout+result.stderr)
        if result.returncode:raise RuntimeError('Frozen common evaluator execution failed')
    payload=source.read_bytes();canonical=run/'summary_closed_loop_cascade_independent.json'
    if canonical.exists():
        if canonical.read_bytes()!=payload:raise ValueError('Existing canonical differs; refused overwrite')
    else:
        with canonical.open('xb')as f:f.write(payload)
    proof={'schema':'append_only_receipt_browser_alias/v1','source':source.name,'source_sha256':sha(source),
        'alias':canonical.name,'alias_sha256':sha(canonical),'identical_bytes':canonical.read_bytes()==payload,
        'frozen_evaluator':str(frozen),'frozen_evaluator_sha256':sha(frozen),
        'meaning':'Exact frozen formal evaluator receipt alias; no thresholds, source bytes or original result changed'}
    alias=run/'lidar_experiment_receipt_display_alias.json'
    if alias.exists():
        old=json.loads(alias.read_text())
        if old['source_sha256']!=proof['source_sha256']or old['alias_sha256']!=proof['alias_sha256']:
            raise ValueError('Existing alias proof differs; refused overwrite')
    else:
        with alias.open('x')as f:json.dump(proof,f,indent=2);f.write('\n')
    receipt=json.loads(payload);print(json.dumps({'run':str(run),'status':receipt['status'],
        'receipt_sha256':sha(source),'checks':{k:v['status']for k,v in receipt['checks'].items()}},indent=2))

if __name__=='__main__':main()
