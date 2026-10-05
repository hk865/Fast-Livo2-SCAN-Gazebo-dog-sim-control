#!/usr/bin/env python3
"""Test-only existing residual team override; never a production invocation."""
from pathlib import Path
import hashlib,json,os,subprocess,time,statistics,math
HERE=Path(__file__).resolve().parent
PARENT=HERE.parent
PROJECT=HERE.parents[5]
build=json.loads((HERE/'build_receipt.json').read_text())
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
assert sha(build['library'])==build['library_sha256']
results={};begin=time.monotonic()
for cls,cpus in [('P',list(range(8))),('E',list(range(8,16)))]:
 for team,override in [(4,False)]+[(n,True) for n in (1,2,4,8)]:
  name=cls+('_team'+str(team) if override else '_nopreload4');out=HERE/name;out.mkdir(exist_ok=False)
  env=os.environ.copy()
  for k in ('LD_PRELOAD','FASTLIVO_DIAGNOSTIC_DIR','GO2_TEST_LIO_TEAM','GO2_TEST_TEAM_RECORDS'):env.pop(k,None)
  env.update(FASTLIVO_BOUNDARY_TIMING='0',OMP_DYNAMIC='FALSE',OMP_NUM_THREADS='4',OMP_THREAD_LIMIT='8',OMP_MAX_ACTIVE_LEVELS='1',OPENBLAS_NUM_THREADS='1',OMP_PROC_BIND='close',OMP_PLACES=','.join('{'+str(x)+'}' for x in cpus))
  env['LD_LIBRARY_PATH']=':'.join([str(Path(build['library']).parent),'/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu','/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib',str(PROJECT/'slam5_navigation/ros2_ws/install/vikit_common/lib'),str(PROJECT/'slam5_navigation/ros2_ws/install/vikit_ros/lib'),env.get('LD_LIBRARY_PATH','')])
  if override:env.update(LD_PRELOAD=str(PARENT/'gomp_team_override.so'),GO2_TEST_LIO_TEAM=str(team),GO2_TEST_TEAM_RECORDS=str(out/'actual_teams.jsonl'))
  t=time.monotonic();r=subprocess.run([str(HERE/'fixture'),str(out/'canonical.bin'),str(out/'timings.json'),'30'],env=env,preexec_fn=lambda:os.sched_setaffinity(0,set(cpus)),capture_output=True,text=True,timeout=20)
  (out/'stdout.log').write_text(r.stdout);(out/'stderr.log').write_text(r.stderr);assert r.returncode==0,r.stderr
  data=json.loads((out/'timings.json').read_text());teams=[json.loads(x) for x in (out/'actual_teams.jsonl').read_text().splitlines()] if override else []
  if override:assert teams and all(x['actual_team']==team and set(x['cpus'])<=set(cpus) for x in teams)
  results[name]={'affinity_cpus':cpus,'test_override':override,'team':team,'actual_teams':sorted({x['actual_team'] for x in teams}),'observed_regions':len(teams),'canonical_sha256':sha(out/'canonical.bin'),'map_root_voxels':data['map_root_voxels'],'accepted_residuals':data['accepted_residuals'],'StateEstimation_median_ms':statistics.median(data['StateEstimation_ms']),'BuildResidualListOMP_median_ms':statistics.median(data['BuildResidualListOMP_ms']),'StateEstimation_samples_ms':data['StateEstimation_ms'],'BuildResidualListOMP_samples_ms':data['BuildResidualListOMP_ms'],'subprocess_wall_ms':(time.monotonic()-t)*1000}
  print(json.dumps({'case':name,'state_ms':results[name]['StateEstimation_median_ms'],'residual_ms':results[name]['BuildResidualListOMP_median_ms'],'hash':results[name]['canonical_sha256']}),flush=True)
expected={'P':'cafaa07675c760480269e8faa5af7945080be0cb74b2d3210f4766261ff6dae0','E':'3f17e1bf1f11cec6eb147af9042c32dd63e0a98da1c21eaf37aae8d79a29842d'}
exact={name:row['canonical_sha256']==expected[name[0]] for name,row in results.items()}
report={'schema':'teacher_test_only_V12_no_lock_LIO_scaling/v1','library_sha256':build['library_sha256'],'timing_observer_enabled':False,'test_preload_not_for_production':True,'repetitions':30,'synthetic_not_actual_complete_map':True,'cases':results,'same_CPU_class_vs_V11_whole_state_cov_ordered_residual_byte_exact':exact,'all_cases_exact_vs_same_CPU_class_V11':all(exact.values()),'whole_batch_wall_s':time.monotonic()-begin}
(HERE/'results.json').write_text(json.dumps(report,indent=2)+'\n')
assert all(exact.values()),'numerical difference retained in report'
print(json.dumps({'all_same_class_bitexact':True,'whole_wall_s':report['whole_batch_wall_s']}))
