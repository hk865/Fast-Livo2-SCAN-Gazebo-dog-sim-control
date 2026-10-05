#!/usr/bin/env python3
"""Append original frozen receipts to a completed owned parallel experiment.
This orchestrator defines no numerical gates and never launches physics.
"""
from pathlib import Path
import argparse,json,hashlib,subprocess,sys,os,datetime
E=Path(__file__).resolve().parent;OLD=E.parents[2]/'test_results/lidar_density_rate_20261005/evaluation'
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def read(p):return json.loads(Path(p).read_text())
def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--run',required=True,type=Path);ap.add_argument('--suffix',required=True);ap.add_argument('--full',action='store_true');ap.add_argument('--output-dir',type=Path);a=ap.parse_args();run=a.run.resolve()
 private=Path('/var/tmp/go2_teacher_parallel_20261005').resolve()
 if not run.is_relative_to(private)or run==private:raise ValueError('Only new owned canonical parallel root permitted')
 for name,field in [('worker_result.json','completed'),('runtime_manifest.json','all_owned_and_children_clean'),('fastlivo_diagnostics/writer_stats.json','final')]:
  if read(run/name).get(field)is not True:raise ValueError('Run has not ended with confirmed writer/owned cleanup: '+name)
 dest=(a.output_dir or E/'actual_runs'/run.name).resolve()
 if not dest.is_relative_to(E):raise ValueError('Analysis output must remain in this evaluation directory')
 dest.mkdir(parents=True,exist_ok=True)
 contract=read(E/'PROSPECTIVE_EVALUATION_PLAN.json');refs=contract['immutable_reference_files']
 required=['evaluate_completed_run.py','publication_ledger_v11/audit_publication_ledger.py','performance_proc_v12.py','performance_proc_full.py','analyze_sensor_experiment.py','audit_pipeline_ages.py','audit_terrain_metadata.py','audit_terrain_metadata_v2.py']
 for rel in required:
  p=OLD/rel
  if sha(p)!=refs[str(p)]:raise ValueError('Original evaluator changed: '+rel)
 snapshots=read(run/'navigation_source_snapshots.json');matches=[v for k,v in snapshots.items()if Path(k).name=='evaluate_ramp.py']
 if len(matches)!=1:raise ValueError('No unique frozen original ramp evaluator')
 helper=Path(matches[0]['snapshot']).resolve()
 if not helper.is_relative_to(run/'sources')or sha(helper)!=matches[0]['sha256']:raise ValueError('Original frozen ramp helper differs')
 env=dict(os.environ,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',PYTHONDONTWRITEBYTECODE='1');steps=[]
 def execute(label,argv):
  p=dest/(label+'.log')
  if p.exists():raise ValueError('Existing evaluation execution log; choose new suffix or inspect original: '+str(p))
  r=subprocess.run([sys.executable,'-B',*map(str,argv)],env=env,text=True,capture_output=True);p.write_text(r.stdout+r.stderr);steps.append({'step':label,'argv':[sys.executable,'-B',*map(str,argv)],'exit_code':r.returncode,'execution_log_sha256':sha(p)})
  return r.returncode
 if execute('common_'+a.suffix,[OLD/'evaluate_completed_run.py','--run',run,'--suffix',a.suffix,'--output',dest]):raise RuntimeError('Frozen common receipt unavailable')
 common=run/'summary_closed_loop_cascade_independent.json'
 if not(run/'summary_closed_loop_clock_hold_independent.json').exists():execute('publication_'+a.suffix,[OLD/'publication_ledger_v11/audit_publication_ledger.py','--run',run])
 scoped_ramp=run/('summary_closed_loop_ramp_independent.'+a.suffix+'.json');canonical_ramp=run/'summary_closed_loop_ramp_independent.json'
 if not scoped_ramp.exists():execute('ramp_'+a.suffix,[helper,'--run',run,'--receipt-suffix',a.suffix])
 if scoped_ramp.exists():
  payload=scoped_ramp.read_bytes()
  if canonical_ramp.exists():
   if canonical_ramp.read_bytes()!=payload:raise ValueError('Existing canonical ramp differs; no overwrite')
  else:
   with canonical_ramp.open('xb')as f:f.write(payload)
 timing=read(run/'navigation_profile.json').get('sensor_sampling',{})
 request=read(run/'request.json')if(run/'request.json').exists()else{}
 if (run/'external_owned_cpu_profile.jsonl').exists():
  # Caller chooses actual kind300 scope via frozen profile boolean, never infers disabled timing as zero.
  profile=read(run/'navigation_profile.json');enabled=bool(profile.get('boundary_timing_diagnostics',profile.get('boundary_timing',False)))
  reader=OLD/('performance_proc_v12.py'if enabled else'performance_proc_full.py')
  execute('performance_'+a.suffix,[reader,'--run',run,'--output',dest/'owned_cpu_and_boundary.json'])
 execute('pipeline_ages_'+a.suffix,[OLD/'audit_pipeline_ages.py','--run',run,'--output',dest/'pipeline_ages.json'])
 execute('source_height_'+a.suffix,[OLD/'analyze_sensor_experiment.py','--run',run,'--output',dest/'source_height'])
 if a.full:
  if not(run/'summary_closed_loop_terrain_metadata_join_independent.json').exists():execute('metadata_v1_'+a.suffix,[OLD/'audit_terrain_metadata.py','--run',run])
  if not(run/'summary_closed_loop_terrain_metadata_join_independent_v2.json').exists():execute('metadata_v2_'+a.suffix,[OLD/'audit_terrain_metadata_v2.py','--run',run])
 receipts={p.name:{'sha256':sha(p),'status':read(p).get('status')}for p in run.glob('summary_closed_loop*independent*.json')}
 out=dest/('orchestration_'+a.suffix+'.json');assert not out.exists();d={'schema':'append_original_parallel_candidate_receipts/v1','created_UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),'run':str(run),'full_requested':a.full,'pre_test_plan_sha256':sha(E/'PROSPECTIVE_EVALUATION_PLAN.json'),'orchestrator_sha256':sha(__file__),'no_new_numeric_thresholds':True,'steps':steps,'receipts':receipts,'all_execution_returncodes_zero':all(x['exit_code']==0 for x in steps),'actual_status_must_be_read_from_original_receipts':True};out.write_text(json.dumps(d,indent=2)+'\n');print(json.dumps({'run':str(run),'steps':[{k:x[k]for k in['step','exit_code']}for x in steps],'receipt_statuses':{k:v['status']for k,v in receipts.items()},'orchestration_sha256':sha(out)}))
if __name__=='__main__':main()
