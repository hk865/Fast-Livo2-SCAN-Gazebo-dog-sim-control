#!/usr/bin/env python3
"""Fresh local finite checks on rebuilt source; original PASS is never consumed."""
from pathlib import Path
if not __debug__:
 raise RuntimeError("Optimized Python (-O/PYTHONOPTIMIZE) is forbidden for portable validation")
import argparse,hashlib,json,os,shlex,struct,subprocess,sys,time
import numpy as np
from portable_common import REPO,LOCAL,NAMES,sha,required_repository_files
TEACHER=REPO/'multifloor_demo/teacher_mode';PAR=TEACHER/'test_results/parallel_pipeline_20261005'
def compile_fixture(ws,source,folder):
 folder.mkdir(parents=True,exist_ok=False);lib=ws/'install/fast_livo2_core/lib/libfast_livo2_core.so';flags={}
 for line in(ws/'build/fast_livo2_core/CMakeFiles/fast_livo2_core.dir/flags.make').read_text().splitlines():
  if line.startswith('CXX_'):k,v=line.split('=',1);flags[k.strip()]=shlex.split(v)
 under=LOCAL/'underlay_ws/install';vk=under/'vikit_common/lib/libvikit_common.so'
 cmd=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],str(source),str(lib),str(vk),'/usr/lib/x86_64-linux-gnu/libpcl_common.so','/usr/lib/x86_64-linux-gnu/libopencv_core.so','/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so','/opt/ros/jazzy/lib/librclcpp.so','-Wl,-rpath,'+str(lib.parent),'-Wl,-rpath-link,/opt/ros/jazzy/lib','-o',str(folder/'fixture')]
 env={**os.environ,'LD_LIBRARY_PATH':':'.join([str(lib.parent),str(under/'livox_ros_driver2/lib'),str(under/'vikit_common/lib'),str(under/'vikit_ros/lib'),str(LOCAL/'livox_sdk/lib'),'/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu']),'OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1','OMP_DYNAMIC':'FALSE','FASTLIVO_BOUNDARY_TIMING':'0','FASTLIVO_LIO_JACOBIAN_THREADS':'4'}
 rr=subprocess.run(cmd,env=env,capture_output=True,text=True);(folder/'compile.log').write_text(rr.stdout+rr.stderr)
 if rr.returncode:raise RuntimeError('Fixture compile failed '+str(folder))
 return env,{'library':str(lib),'library_sha256':sha(lib),'compile_argv':cmd,'fixture_sha256':sha(folder/'fixture'),'source':str(source),'source_sha256':sha(source)}
def records(p):
 out=[]
 with p.open('rb')as f:
  assert f.read(16)==b'FLIVODIAG0001LE\0'
  while h:=f.read(56):
   head=struct.unpack('<7Q',h);payload=f.read(head[-1]*8);assert len(payload)==head[-1]*8
   if head[0]==100:
    v=np.frombuffer(payload,dtype='<f8').copy();v[9:13]=0;payload=v.tobytes()
   out.append((head,payload))
 return out

def execute(exe,args,env,log,cores):
 r=subprocess.run(['taskset','-c',cores,str(exe),*args],env=env,capture_output=True,text=True,timeout=60);log.write_text(r.stdout+r.stderr)
 if r.returncode:raise RuntimeError('Finite fixture failed '+str(log))
 return r.stdout

def main():
 p=argparse.ArgumentParser();p.add_argument('--variants',nargs='+',choices=['v18','v17'],default=['v18','v17']);a=p.parse_args()
 buildpath=LOCAL/'SOURCE_BUILD_RECEIPT.json';build=json.loads(buildpath.read_text());assert build['status']=='PASS_SOURCE_BUILD_ONLY'
 for v in ['v12',*a.variants]:
  for file,digest in build['variants'][v]['artifacts_sha256'].items():
   if sha(file)!=digest:raise RuntimeError('Freshly built binary/configuration differs '+file)
 root=LOCAL/('finite_'+time.strftime('%Y%m%d_%H%M%S'));root.mkdir(exist_ok=False);available=sorted(os.sched_getaffinity(0));selected=available[:4];cores=','.join(map(str,selected))
 report={'schema':'portable_fresh_finite_checks/v1','output':str(root),'CPU_affinity':selected,'scope':'Fresh same-host finite StateEstimation and patch components, not full trajectory, benchmark or navigation','variants':{},'comparisons':[],'historical_results_consumed':False}
 variants=['v12',*a.variants]
 source=PAR/'lio_v15/finite_lio_vio_fixture.cpp'
 for v in variants:
  ws=TEACHER/'navigation'/NAMES[v]/'slam_ws';folder=root/(v+'_lio');env,meta=compile_fixture(ws,source,folder);meta['runs']={}
  for scene in ['lio','vio','inverse']:
   for mode in ['off','inside','outside']:
    tag=scene+'_'+mode;e=env.copy();e.pop('FASTLIVO_DIAGNOSTIC_DIR',None)
    if mode!='off':e.update(FASTLIVO_DIAGNOSTIC_DIR=str(folder/(tag+'_records')),FASTLIVO_DIAGNOSTIC_BEGIN='129'if mode=='inside'else'115',FASTLIVO_DIAGNOSTIC_END='131'if mode=='inside'else'118')
    execute(folder/'fixture',[scene,str(folder/(tag+'.bin'))],e,folder/(tag+'.log'),cores);meta['runs'][tag]=sha(folder/(tag+'.bin'))
    if mode!='off':
     st=json.loads((folder/(tag+'_records/writer_stats.json')).read_text());assert not st['dropped']and not st['writer_io_failed']and st['attempted']==st['written']
  report['variants'][v]={'LIO':meta}
 for v in a.variants:
  checks=[]
  for scene in ['lio','vio','inverse']:
   byteeq=all((root/(v+'_lio')/(scene+'_'+mode+'.bin')).read_bytes()==(root/'v12_lio'/(scene+'_inside.bin')).read_bytes()for mode in ['off','inside','outside'])
   diag=records(root/(v+'_lio')/(scene+'_inside_records/records.bin'))==records(root/'v12_lio'/(scene+'_inside_records/records.bin'))
   checks.append({'scene':scene,'state_cov_and_outputs_byte_identical':byteeq,'all_nonwall_original_diagnostics_byte_identical':diag});assert byteeq and diag
  report['comparisons'].append({'variant':v,'full_729_StateEstimation_and_VIO9_cases':checks})
 # Original41 patch cases freshly exercise null/cache/threshold/exposure/pyramid/rollback.
 import ast
 old=(PAR/'vio_v16/run_numeric_fixtures.py').read_text();caseblock=old[old.index('cases=[]'):old.index('variants=list(build')];ns={};exec(caseblock,ns);cases=ns['cases'];assert len(cases)==41
 source=PAR/'vio_v16/vio_patch_fixture.cpp'
 for v in variants:
  ws=TEACHER/'navigation'/NAMES[v]/'slam_ws';folder=root/(v+'_patch');env,meta=compile_fixture(ws,source,folder);meta['runs']={}
  for name,args in cases:
   case=folder/name;case.mkdir();e={**env,'FASTLIVO_DIAGNOSTIC_DIR':str(case/'diagnostics'),'FASTLIVO_DIAGNOSTIC_BEGIN':'129','FASTLIVO_DIAGNOSTIC_END':'131'}
   execute(folder/'fixture',[*args,str(case/'output.bin')],e,case/'execution.log',cores);meta['runs'][name]=sha(case/'output.bin')
  report['variants'][v]['VIO41']=meta
 for v in a.variants:
  checks=[]
  for name,args in cases:
   x=root/'v12_patch'/name;y=root/(v+'_patch')/name;equal=(x/'output.bin').read_bytes()==(y/'output.bin').read_bytes();diag=records(x/'diagnostics/records.bin')==records(y/'diagnostics/records.bin');assert equal and diag
   checks.append({'case':name,'state_cov_G_H_errors_cache_byte_identical':equal,'original_nonwall_diagnostics_byte_identical':diag})
  report['comparisons'].append({'variant':v,'VIO41_boundary_cases':checks})
 report['all_byte_identical']=True;report['process_runs']=len(variants)*(9+41);(root/'FINITE_CHECKS.json').write_text(json.dumps(report,indent=2)+'\n')
 # Test-only FP/team observation: no override, no production preload, no timing claim.
 observer=root/'observe_gomp.so';observer_source=PAR/'lio_v15/observe_gomp.cpp'
 subprocess.run(['/usr/bin/c++','-shared','-fPIC','-O2','-std=c++17','-fopenmp',str(observer_source),'-ldl','-pthread','-o',str(observer)],check=True)
 observation={'schema':'portable_test_only_FP_team_loader/v1','test_only_preload':True,'production_preload':False,'variants':{},'all_observed_FP_valid':True,'all_observed_outputs_byte_identical':True,'observer_source':str(observer_source),'observer_source_sha256':sha(observer_source),'observer_sha256':sha(observer)}
 for v in variants:
  folder=root/(v+'_lio');ws=TEACHER/'navigation'/NAMES[v]/'slam_ws';lib=ws/'install/fast_livo2_core/lib/libfast_livo2_core.so';under=LOCAL/'underlay_ws/install'
  baseenv={**os.environ,'LD_LIBRARY_PATH':':'.join([str(lib.parent),str(under/'livox_ros_driver2/lib'),str(under/'vikit_common/lib'),str(under/'vikit_ros/lib'),str(LOCAL/'livox_sdk/lib'),'/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu']),'OMP_NUM_THREADS':'4','OPENBLAS_NUM_THREADS':'1','OMP_DYNAMIC':'FALSE','FASTLIVO_BOUNDARY_TIMING':'0','FASTLIVO_LIO_JACOBIAN_THREADS':'4'}
  baseenv.pop('FASTLIVO_DIAGNOSTIC_DIR',None);witness={'library_sha256':sha(lib),'main_FP_processes':0,'source_file_sha256':sha(observer_source),'observations':{}}
  for scene in ['lio','vio']:
   exe=folder/'fixture';args=[scene,str(folder/(scene+'_observed.bin'))]
   if scene=='vio':
    folder2=root/(v+'_patch');exe=folder2/'fixture';name,args0=next((name,args0)for name,args0 in cases if int(args0[1])>=64 and args0[0]=='forward');args=[*args0,str(folder/(scene+'_observed.bin'))];prior=root/(v+'_patch')/name/'output.bin'
   else:prior=folder/'lio_off.bin'
   mainfile=folder/(scene+'_main_FP.jsonl');teamfile=folder/(scene+'_teams.jsonl');e={**baseenv,'LD_PRELOAD':str(observer),'V15_MAIN_FP_RECORDS':str(mainfile),'V15_GOMP_RECORDS':str(teamfile)}
   execute(exe,args,e,folder/(scene+'_FP.log'),cores)
   rows=[json.loads(x)for x in mainfile.read_text().splitlines()if x];mine=[x for x in rows if x['executable']==str(exe)]
   assert len(mine)==2 and all(x['fp_rounding_mode']==0 and x['mxcsr_control_mask']==8064 for x in mine)
   teams=[json.loads(x)for x in teamfile.read_text().splitlines()if x]
   assert all(all(x==0 for x in row['fp_rounding_modes'])and all(x==8064 for x in row['mxcsr_control_masks'])for row in teams)
   assert (folder/(scene+'_observed.bin')).read_bytes()==prior.read_bytes()
   wanted=[x for x in teams if ('BuildJacobianRows'in x['caller']if scene=='lio'and v=='v18'else 'updateState'in x['caller']if scene=='vio'and v=='v18'else 'BuildResidualListOMP'in x['caller'])]
   if v=='v18':assert wanted and all(x['actual_team']==(4 if scene=='lio'else 1)for x in wanted)
   witness['main_FP_processes']+=1;witness['observations'][scene]={'main_records':str(mainfile),'main_sha256':sha(mainfile),'team_records':str(teamfile),'team_sha256':sha(teamfile),'all_FP_valid':True,'output_byte_identical':True}
   if v=='v18':witness['LIO_actual_team'if scene=='lio'else'VIO_actual_team']=4 if scene=='lio'else 1
  # Fresh loader trace without the observer proves the private core actually loaded.
  loader=folder/'loader.log';e={**baseenv,'LD_DEBUG':'libs'};args=['lio',str(folder/'loader_output.bin')]
  execute(folder/'fixture',args,e,loader,cores);trace=loader.read_text();assert str(lib)in trace and (folder/'loader_output.bin').read_bytes()==(folder/'lio_off.bin').read_bytes()
  witness['loaded_private_library_verified']=True;witness['loader_log']=str(loader);witness['loader_log_sha256']=sha(loader);observation['variants'][v]=witness
 (root/'FP_TEAM_LOADER.json').write_text(json.dumps(observation,indent=2)+'\n')
 # Gate producer bindings are the new clone's current sources and freshly built libraries.
 for v in a.variants:
  here=TEACHER/'navigation'/NAMES[v];repository=required_repository_files(here)
  artifacts=dict(build['variants'][v]['artifacts_sha256']);artifacts.update({str(x):sha(x)for x in root.rglob('*')if x.is_file()});artifacts[str(buildpath)]=sha(buildpath);artifacts[str(root/'FINITE_CHECKS.json')]=sha(root/'FINITE_CHECKS.json');artifacts[str(root/'FP_TEAM_LOADER.json')]=sha(root/'FP_TEAM_LOADER.json');
  for observed in observation['variants'].values():
   artifacts[observed['loader_log']]=observed['loader_log_sha256']
   for row in observed['observations'].values():
    artifacts[row['main_records']]=row['main_sha256'];artifacts[row['team_records']]=row['team_sha256']
  artifacts[str(TEACHER/'simulation/build/libteacher_actuator.so')]=sha(TEACHER/'simulation/build/libteacher_actuator.so')
  gate={'schema':'portable_local_finite_preflight/v1','status':'PASS_LOCAL_BUILD_FINITE_CHECKS_EXPERIMENTAL','repo':str(REPO),'variant':v,'candidate_root':str(here),'actual_simulation_verified':False,'historic_pass_inherited':False,'finite_receipt':str(root/'FINITE_CHECKS.json'),'FP_team_loader_receipt':str(root/'FP_TEAM_LOADER.json'),'finite_numeric_all_byte_identical':True,'process_runs':report['process_runs'],'queue_semantics_verified':False,'repository_bindings':{str(x.relative_to(REPO)):sha(x)for x in sorted(set(repository))},'local_artifact_bindings':artifacts,'limits':['Finite synthetic new same-host components only; no original full historical estimator replay or timing benchmark','No navigation/parking/ramp pass is inherited','V17 queue receive/decode/commit requires independent local semantic receipt before runtime']}
  if v=='v17':gate['status']='BLOCKED_QUEUE_SEMANTICS_REQUIRED'
  (LOCAL/('LOCAL_PREFLIGHT_'+v.upper()+'.json')).write_text(json.dumps(gate,indent=2)+'\n')
 print('Fresh finite checks passed',report['process_runs'],'processes; V18 experimental gate, V17 queue semantic gate pending')
if __name__=='__main__':main()
