#!/usr/bin/env python3
"""Launch exactly one new owned experiment and attach the existing read-only sampler.

The candidate runner retains exclusive ownership of physics/control/cleanup.
This wrapper never signals any process. It only reaps its two direct children.
"""
import argparse
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

HERE=Path(__file__).resolve().parent
TEACHER=HERE.parents[1]
STORAGE=Path('/var/tmp/go2_teacher_parallel_20261005')
SAMPLER=TEACHER/'test_results/lidar_density_rate_20261005/evaluation/sample_owned_cpu_v12.py'

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--candidate',choices=['parallel_lio_v15','parallel_vio_v16','ingress_pipeline_v17','combined_compute_v18'],required=True)
    parser.add_argument('--profile',required=True)
    parser.add_argument('--label',required=True)
    parser.add_argument('--domain',type=int,default=86)
    args=parser.parse_args()
    out=HERE/'actual_launches'/args.label
    out.mkdir(parents=True,exist_ok=False)
    before={p.name for p in STORAGE.iterdir()}
    command=[sys.executable,'-B',str(TEACHER/'navigation'/args.candidate/'run.py'),
             '--profile',args.profile,'--label',args.label,'--domain',str(args.domain),
             '--run-storage-root',str(STORAGE)]
    record={'schema':'new_owned_parallel_experiment_launch/v1','UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),
            'argv':command,'wrapper_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'sampler_sha256':hashlib.sha256(SAMPLER.read_bytes()).hexdigest(),'signals_sent':[],
            'training_or_camera_mode_changed':False,'wall_begin':time.monotonic()}
    with (out/'runner.log').open('x') as log:
        runner=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT)
        record['runner_pid']=runner.pid
        deadline=time.monotonic()+15
        run=None
        while time.monotonic()<deadline:
            paths=[p for p in STORAGE.iterdir() if p.name not in before and (p/'runner_request.json').is_file()]
            if len(paths)==1:
                run=paths[0].resolve();break
            if len(paths)>1:raise RuntimeError('Ambiguous new run; will not attach a sampler to another process')
            if runner.poll() is not None:break
            time.sleep(.025)
        sampler=None
        sample_log=(out/'sampler.log').open('x')
        if run is not None and runner.poll() is None:
            record['canonical_run']=str(run)
            sample_cmd=[sys.executable,'-B',str(SAMPLER),'--run',str(run),'--root-pid',str(runner.pid),
                        '--hz','0.5','--max-wall-s','1800']
            sampler=subprocess.Popen(sample_cmd,stdout=sample_log,stderr=subprocess.STDOUT)
            record.update(sampler_pid=sampler.pid,sampler_argv=sample_cmd,sampler_start_wall=time.monotonic())
        (out/'launch.json').write_text(json.dumps(record,indent=2)+'\n')
        print(json.dumps(record),flush=True)
        record['runner_exit']=runner.wait()
        record['sampler_exit']=sampler.wait() if sampler is not None else None
        record['wall_end']=time.monotonic()
        sample_log.close()
    (out/'completion.json').write_text(json.dumps(record,indent=2)+'\n')
    print(json.dumps(record),flush=True)
    return record['runner_exit']

if __name__=='__main__':raise SystemExit(main())
