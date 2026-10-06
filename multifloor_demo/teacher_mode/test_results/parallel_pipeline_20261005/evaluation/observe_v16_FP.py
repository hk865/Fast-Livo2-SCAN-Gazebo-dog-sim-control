#!/usr/bin/env python3
from pathlib import Path
import json,hashlib,os,subprocess,datetime
E=Path(__file__).resolve().parent;A=E.parent/'vio_v16';L=E.parent/'lio_v15';D=E/'v16_FP_observation';D.mkdir(exist_ok=False)
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
num=json.loads((A/'numeric_comparison.json').read_text());build=json.loads((A/'fixture_build_receipt.json').read_text());witness=json.loads((L/'all_variants_FP_observation/receipt.json').read_text());observer=L/'observe_gomp.so';assert sha(observer)==witness['observer_so_sha256'];assert sha(L/'observe_gomp.cpp')==witness['observer_cpp_sha256']
rows=[]
for variant in num['variants']:
 for case in ['forward_large512','inverse_large512']:
  original=num['results'][variant][case];dest=D/variant/case;dest.mkdir(parents=True);argv=original['argv'].copy();exe=Path(argv[3]);assert sha(exe)==build['variants'][variant]['executable_sha256'];assert sha(build['variants'][variant]['lib_path'])==build['variants'][variant]['lib_sha256'];argv[-1]=str(dest/'output.bin')
  env=os.environ.copy();env.update(build['variants'][variant]['env']);env.update(LD_PRELOAD=str(observer),V15_MAIN_FP_RECORDS=str(dest/'main_fp.jsonl'),V15_GOMP_RECORDS=str(dest/'teams.jsonl'));env.pop('FASTLIVO_DIAGNOSTIC_DIR',None)
  result=subprocess.run(argv,env=env,text=True,capture_output=True);(dest/'execution.log').write_text(result.stdout+result.stderr);assert result.returncode==0,(case,result.stderr)
  outputsha=sha(dest/'output.bin');assert outputsha==original['output_sha256']
  mains=[json.loads(x)for x in(dest/'main_fp.jsonl').read_text().splitlines()];valid=[x for x in mains if Path(x['executable']).resolve()==exe.resolve()];ignored=[x for x in mains if x not in valid];assert len(valid)==2 and {x['phase']for x in valid}=={'main_enter','main_exit'}
  assert all(x['cpu']in[0,2,4,6]and x['fp_rounding_mode']==0 and x['mxcsr_control_mask']==8064 for x in valid)
  teams=[json.loads(x)for x in(dest/'teams.jsonl').read_text().splitlines()];
  for team in teams:assert all(x in[0,2,4,6]for x in team['cpus'])and set(team['fp_rounding_modes'])=={0}and set(team['mxcsr_control_masks'])=={8064}
  if variant=='candidate_V16_t4':assert teams and all(x['actual_team']==4 for x in teams)
  if variant=='candidate_V16_t1':assert teams and all(x['actual_team']==1 for x in teams)
  rows.append({'variant':variant,'case':case,'argv':argv,'returncode':result.returncode,'library_sha256':build['variants'][variant]['lib_sha256'],'executable_sha256':sha(exe),'output_sha256':outputsha,'original_numeric_output_sha256':original['output_sha256'],'main_FP':valid,'taskset_ignored':ignored,'teams':teams,'metadata_files_sha256':{n:sha(dest/n)for n in ['main_fp.jsonl','teams.jsonl','execution.log','output.bin']}})
r={'schema':'independent_V16_runtime_FP_observation/v1','status':'PASS_FINITE_FP_METADATA_ONLY','created_UTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),'test_only':True,'performance_eligible':False,'timing_reason':'LD_PRELOAD adds observation overhead; no statistical benchmark rerun','numeric_originals_unchanged':True,'main_enter_exit_all_P_cores_FE_TONEAREST_MXCSR8064':True,'all_observed_workers_same_FP':True,'numeric_receipt_sha256':sha(A/'numeric_comparison.json'),'fixture_build_receipt_sha256':sha(A/'fixture_build_receipt.json'),'observer_source_sha256':sha(L/'observe_gomp.cpp'),'observer_library_sha256':sha(observer),'script_sha256':sha(__file__),'runs':rows}
p=D/'receipt.json';p.write_text(json.dumps(r,indent=2)+'\n');print(json.dumps({'status':r['status'],'processes':len(rows),'receipt':str(p),'sha256':sha(p)}))
# Append final audit closes explicit missing FP witness, never modifies predecessor.
base=E/'V16_CONFIRMATORY_INDEPENDENT.json';orig=json.loads(base.read_text());final=json.loads(json.dumps(orig));final.update(schema='independent_V16_confirmatory_component_audit/v2',status='PASS_LIMITED_KERNEL_T1_ONLY',parent_v1_sha256=sha(base),actual_FP_mode_receipt={'path':str(p),'sha256':sha(p),'status':r['status']},FP_witness_performance_separate=True,created_UTC=datetime.datetime.now(datetime.timezone.utc).isoformat());final['verified_input_sha256'][str(p)]=sha(p);out=E/'V16_CONFIRMATORY_INDEPENDENT_V2.json';assert not out.exists();out.write_text(json.dumps(final,indent=2)+'\n');print(json.dumps({'status':final['status'],'receipt':str(out),'sha256':sha(out)}))
