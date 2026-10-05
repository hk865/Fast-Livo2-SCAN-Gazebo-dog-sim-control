#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,os,subprocess,struct
OUT=Path(__file__).resolve().parent;build=json.loads((OUT/'fixture_build_receipt.json').read_text())
cases=[]
for mode in ['forward','inverse']:
 for n,level,null,mixed,expo,fresh,multi,wrong,allnull,name in [(0,0,0,0,0,0,0,0,0,'zero'),(9,0,0,0,0,0,0,0,0,'small9'),(16,2,0,1,1,0,0,0,0,'small_exposure'),(128,1,1,1,0,0,0,0,0,'nullable'),(128,2,1,1,1,0,0,0,0,'nullable_exposure'),(128,2,0,0,0,0,0,0,1,'all_null'),(128,1,0,0,0,1,0,0,0,'fresh_cache'),(128,1,0,0,1,1,0,0,0,'fresh_exposure'),(128,0,0,0,0,0,1,0,0,'multi_pyramid'),(128,0,0,1,1,0,1,0,0,'multi_exposure'),(512,1,0,1,0,0,0,0,0,'large512')]:
  cases.append((mode+'_'+name,[mode,str(n),str(level),str(null),str(mixed),str(expo),str(fresh),str(multi),str(wrong),str(allnull),'1']))
 for level in [0,1,2]:
  for expo in [0,1]:cases.append((mode+'_level'+str(level)+'_exposure'+str(expo),[mode,'128',str(level),'0','1',str(expo),'0','0','0','0','1']))
 for n in [63,64,65]:cases.append((mode+'_threshold_'+str(n),[mode,str(n),'0','0','0','0','0','0','0','0','1']))
cases.append(('inverse_rollback_probe',['inverse','128','0','0','0','0','0','0','1','0','1']))
variants=list(build['variants']);results={};comparisons=[];kind_accepts={};total=0
for variant in variants:
 meta=build['variants'][variant];d=OUT/variant;results[variant]={}
 for name,args in cases:
  folder=d/'numeric'/name;folder.mkdir(parents=True,exist_ok=False)
  env={**os.environ,**meta['env']};env.update(FASTLIVO_DIAGNOSTIC_DIR=str(folder/'diagnostics'),FASTLIVO_DIAGNOSTIC_BEGIN='129',FASTLIVO_DIAGNOSTIC_END='131',FASTLIVO_BOUNDARY_TIMING='0')
  if variant!='baseline_V12':
   env['LD_LIBRARY_PATH']=str(d)+':'+env['LD_LIBRARY_PATH'];env['LD_PRELOAD']=str(OUT/'gomp_vio_observe.so');env['V16_TEST_TEAM_RECORDS']=str(folder/'actual_teams.jsonl')
  cmd=['taskset','-c','0,2,4,6',str(d/'fixture'),*args,str(folder/'output.bin')]
  r=subprocess.run(cmd,env=env,text=True,capture_output=True);(folder/'stdout.log').write_text(r.stdout);(folder/'stderr.log').write_text(r.stderr);total+=1
  if r.returncode:raise RuntimeError((variant,name,r.returncode,r.stdout,r.stderr))
  teams=[json.loads(x)for x in (folder/'actual_teams.jsonl').read_text().splitlines()]if(folder/'actual_teams.jsonl').exists()else[]
  results[variant][name]={'argv':cmd,'returncode':r.returncode,'output_sha256':hashlib.sha256((folder/'output.bin').read_bytes()).hexdigest(),'actual_teams':teams,'stdout':r.stdout}

def records(p):
 with p.open('rb')as f:
  assert f.read(16)==b'FLIVODIAG0001LE\0'
  while b:=f.read(56):
   h=struct.unpack('<7Q',b);payload=f.read(h[-1]*8);assert len(payload)==h[-1]*8;yield h,payload
for name,args in cases:
 a=OUT/variants[0]/'numeric'/name;av=(a/'output.bin').read_bytes();aa=list(records(a/'diagnostics/records.bin'))
 accepts=[struct.unpack('<'+str(len(p)//8)+'d',p)[9]for h,p in aa if h[0]==201 and len(p)>12*8];kind_accepts[name]=accepts
 for variant in variants[1:]:
  b=OUT/variant/'numeric'/name;bv=(b/'output.bin').read_bytes();bb=list(records(b/'diagnostics/records.bin'));equal=av==bv;diag=aa==bb
  comparisons.append({'case':name,'candidate':variant,'state_cov_G_H_errors_reference_H_byte_identical':equal,'original_nonwall_diagnostic_records_byte_identical':diag,'diagnostic_records':len(aa),'iteration_accepted_flags':accepts})
  if not equal or not diag:
   (OUT/'numeric_first_failure.json').write_text(json.dumps({'case':name,'candidate':variant,'state_equal':equal,'diag_equal':diag,'comparisons_before':comparisons,'process_runs':total},indent=2)+'\n');raise AssertionError(('Numeric difference',name,variant,equal,diag))
report={'schema':'independent_V16_VIO_patch_numeric/v1','status':'PASS_LIMITED_SYNTHETIC','process_runs':total,'fixtures':len(cases),'variants':variants,'comparisons':comparisons,'observed_accepted_iterations':sum(v.count(1.)for v in kind_accepts.values()),'observed_rollback_iterations':sum(v.count(0.)for v in kind_accepts.values()),'results':results,'scope':'Synthetic updateState/updateStateInverse/computeJacobianAndUpdateEKF only; not full processFrame or actual run replay','nullptr_precompute_limitation':'Original inverse precomputeReferencePatches dereferences pt before null guard. Null tests use existing prepared H cache; multi-level inverse tests use nonnull inputs.'}
(OUT/'numeric_comparison.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:report[k]for k in ['status','process_runs','fixtures','observed_accepted_iterations','observed_rollback_iterations']}))
