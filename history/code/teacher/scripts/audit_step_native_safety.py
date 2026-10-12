#!/usr/bin/env python3
"""Append independent 200 Hz safety/provenance receipts; never overwrite verdicts."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def read(p):return json.loads(Path(p).read_text())
def clean(v):
    if isinstance(v,np.ndarray):return clean(v.tolist())
    if isinstance(v,np.generic):return clean(v.item())
    if isinstance(v,dict):return {str(k):clean(x)for k,x in v.items()}
    if isinstance(v,(list,tuple)):return [clean(x)for x in v]
    return v

def audit(run):
    run=Path(run).resolve();errors=[];checks={};metrics={}
    try:
        protocol_path=run/'sources/tests/step_functional_protocol.json';protocol=read(protocol_path)
        rows=[json.loads(s)for s in(run/'actuator.jsonl').read_text().splitlines()if s.strip()];steps=[v for v in rows if v.get('kind')=='physics_step']
        first=json.loads((run/'telemetry.jsonl').read_text().splitlines()[0]);offset=first['world_sim_time']-first['sim_time']
        t=np.asarray([v['t']for v in steps],float)-offset;p=np.asarray([v['position']for v in steps],float);q=np.asarray([v['quaternion_wxyz']for v in steps],float)
        contact=np.asarray([v['contacts']for v in steps],float);start=protocol['timing']['bootstrap_pd_s'];mask=t>=start
        valid=q.shape==(len(steps),4)and p.shape==(len(steps),3)and contact.shape==(len(steps),5)and np.isfinite(q).all()and np.isfinite(p).all()and np.isfinite(contact).all()and np.allclose(np.sum(q*q,axis=1),1,atol=1e-3)
        if not valid:raise ValueError('Native position/quaternion/contact evidence is invalid')
        w,x,y,z=q.T;rp=np.column_stack([np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)),np.arcsin(np.clip(2*(w*y-z*x),-1,1))])
        observation_source=run/'sources/policy/observation.py';module_name='native_safety_observation_'+sha(observation_source)[:10]
        spec=importlib.util.spec_from_file_location(module_name,observation_source);module=importlib.util.module_from_spec(spec);sys.modules[module_name]=module;spec.loader.exec_module(module)
        terrain=module.TerrainHeightMap.from_sdf(run/'world.sdf');ground=terrain.height(p[:,:2],ray_start_z=p[:,2]);clear=p[:,2]-ground
        lim=protocol['safety'];gap=protocol['ascent']['max_native_sample_gap_s']
        checks['native_samples_continuous']=bool(mask.any()and np.all(np.diff(t)>0)and np.max(np.diff(t))<=gap)
        checks['native_attitude_from_bootstrap_end']=bool(abs(rp[mask]).max()<=lim['max_abs_roll_pitch_rad'])
        checks['support_clearance_from_bootstrap_end']=bool(np.isfinite(clear[mask]).all()and clear[mask].min()>=lim['minimum_body_above_support_m'])
        checks['no_native_body_contact_from_bootstrap_end']=bool(not(contact[mask,0]>0).any())
        checks['no_missing_native_contacts_from_bootstrap_end']=bool(not(contact[mask]<0).any())
        checks['no_latched_native_fault']=bool(not any(v.get('kind')=='fault'for v in rows)and not any(v.get('fault')for v in steps))
        early=mask&(t<1.5)
        metrics={'start_s':start,'end_s':float(t[-1]),'native_samples':int(mask.sum()),'max_sample_gap_s':float(np.max(np.diff(t))),
            'max_abs_roll_pitch_rad':float(abs(rp[mask]).max()),'min_support_clearance_m':float(clear[mask].min()),
            'body_contact_samples':int((contact[mask,0]>0).sum()),'missing_contact_samples':int((contact[mask]<0).sum()),
            'previously_excluded_early_window_s':[start,1.5],'early_samples':int(early.sum()),'early_max_abs_roll_pitch_rad':float(abs(rp[early]).max())if early.any()else None,
            'early_min_support_clearance_m':float(clear[early].min())if early.any()else None,'early_body_contact_samples':int((contact[early,0]>0).sum()),
            'attitude_source':'actual native 200 Hz quaternion','clearance_source':'actual base position minus supporting SDF collision directly below base; no body-height-rise ability gate'}
        source_files=[run/'actuator.jsonl',run/'telemetry.jsonl',run/'world.sdf',protocol_path,observation_source,run/'summary_functional.json']
        hashes={str(p.relative_to(run)):sha(p)for p in source_files}
    except Exception as e:errors.append(f'{type(e).__name__}: {e}');hashes={}
    correction=read(ROOT/'test_results/step_functional_evaluator_source_audit_20261003/provenance_correction.json')
    original=read(run/'summary_functional.json')if(run/'summary_functional.json').exists()else {}
    snapshot=run/'sources/scripts/evaluate_step_functional.py'
    receipt={'scope':'additional safety audit of frozen raw evidence; original functional summary is unchanged',
        'status':'passed'if checks and all(checks.values())and not errors else'failed','checks':checks,'metrics':metrics,'errors':errors,
        'original_functional_status':original.get('functional_status'),'original_evaluator_sha256_as_reported':original.get('evaluator_sha256'),
        'evaluator_source_snapshot_sha256':sha(snapshot)if snapshot.exists()else None,
        'actually_loaded_functional_evaluator_sha256':correction['loaded_evaluator_sha256'],
        'loaded_evaluator_source_correction':str(ROOT/'test_results/step_functional_evaluator_source_audit_20261003/provenance_correction.json'),
        'source_hashes':hashes,'safety_auditor_sha256':sha(__file__),'navigation':'unverified','real_robot':'unverified'}
    receipt=clean(receipt);(run/'supplemental_native_safety.json').write_text(json.dumps(receipt,indent=2,allow_nan=False)+'\n');return receipt

def reproduce(run,source):
    """Execute the frozen actually-loaded evaluator and restore original bytes."""
    run=Path(run).resolve();source=Path(source).resolve();path=run/'summary_functional.json';feet=run/'functional_feet.npz'
    original_bytes=path.read_bytes();original=read(path);original_feet=feet.read_bytes()if feet.exists()else None
    original_hash=sha(path);feet_hash=sha(feet)if feet.exists()else None
    spec=importlib.util.spec_from_file_location('frozen_step_functional_'+sha(source)[:10],source);module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
    try:
        reproduced=module.evaluate(run)
        path.replace(run/'summary_functional.reproduced.json')
        if feet.exists():feet.replace(run/'functional_feet.reproduced.npz')
    finally:
        path.write_bytes(original_bytes)
        if original_feet is not None:feet.write_bytes(original_feet)
    actual_hash=sha(source);snapshot=run/'sources/scripts/evaluate_step_functional.py'
    checks_equal=original.get('checks')==reproduced.get('checks');metrics_equal=original.get('metrics')==reproduced.get('metrics')
    original_arrays=np.load(feet);reproduced_arrays=np.load(run/'functional_feet.reproduced.npz')
    arrays_equal=set(original_arrays.files)==set(reproduced_arrays.files)and all(np.array_equal(original_arrays[k],reproduced_arrays[k],equal_nan=True)for k in original_arrays.files)
    receipt={'scope':'Explicit independent replay of all native evidence under the first actually-loaded evaluator; original results restored byte for byte',
        'status':'reproduced_identical'if checks_equal and metrics_equal and arrays_equal else'reproduction_mismatch',
        'actual_executed_evaluator_sha256':actual_hash,'actual_executed_source':str(source),
        'archived_evaluator_file_sha256':sha(snapshot),'original_reported_evaluator_sha256':original.get('evaluator_sha256'),
        'original_report_hash_misattributed':original.get('evaluator_sha256')!=actual_hash,
        'all_checks_identical':checks_equal,'all_metrics_identical':metrics_equal,'all_derived_arrays_identical':arrays_equal,
        'all_protocol_values_identical':original.get('protocol')==reproduced.get('protocol'),
        'original_summary_sha256_before':original_hash,'original_summary_sha256_after':sha(path),'original_summary_bytes_preserved':original_hash==sha(path),
        'original_feet_sha256_before':feet_hash,'original_feet_sha256_after':sha(feet),'original_feet_bytes_preserved':feet_hash==sha(feet),
        'reproduced_summary_path':str(run/'summary_functional.reproduced.json'),'reproduced_summary_sha256':sha(run/'summary_functional.reproduced.json'),
        'reproduced_feet_path':str(run/'functional_feet.reproduced.npz'),'reproduced_feet_sha256':sha(run/'functional_feet.reproduced.npz'),
        'metadata_differences_excluded_from_metric_comparison':['evaluator_sha256','source path metadata'],
        'attribution_reason':'The live runner imported once and used the same cached Python module; a post-ready on-disk edit changed digest(__file__) without changing that cached logic. The separate replay executes the frozen source, and corrections do not replace original receipts.',
        'reproduction_auditor_sha256':sha(__file__)}
    (run/'provenance_correction.json').write_text(json.dumps(receipt,indent=2)+'\n');return receipt

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('runs',type=Path,nargs='+');p.add_argument('--reproduce',action='store_true')
    p.add_argument('--frozen-evaluator',type=Path,default=ROOT/'runs/20261003_232320_step05_continue_functional_retest_r1_5ba8/sources/scripts/evaluate_step_functional.py');args=p.parse_args()
    for run in args.runs:
        if args.reproduce:
            reproduced=reproduce(run,args.frozen_evaluator);print(json.dumps({'run':str(run),'reproduction':reproduced['status'],'checks_identical':reproduced['all_checks_identical'],'metrics_identical':reproduced['all_metrics_identical'],'original_report_hash_misattributed':reproduced['original_report_hash_misattributed']}),flush=True)
        result=audit(run);print(json.dumps({'run':str(run),'status':result['status'],'metrics':result['metrics'],'errors':result['errors']}),flush=True)
if __name__=='__main__':main()
