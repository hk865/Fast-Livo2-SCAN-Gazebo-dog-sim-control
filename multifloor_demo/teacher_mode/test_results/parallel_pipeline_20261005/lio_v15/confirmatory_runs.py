#!/usr/bin/env python3
from pathlib import Path
import json,os,subprocess,hashlib,sys,time,statistics
import numpy as np
art=Path(__file__).resolve().parent;base=art.parents[2];real=base/'test_results/parallel_pipeline_20261005/evaluation/real_lio_rows_v12_format16'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def env(variant):
 d=os.environ.copy();d.update(OMP_NUM_THREADS='1',OMP_DYNAMIC='FALSE',OMP_PROC_BIND='TRUE',OMP_PLACES=','.join('{'+str(x)+'}'for x in range(8)),OPENBLAS_NUM_THREADS='1',FASTLIVO_BOUNDARY_TIMING='0');d.pop('FASTLIVO_DIAGNOSTIC_DIR',None);d.pop('LD_PRELOAD',None)
 ws=base/('navigation/lidar_sampling_v12/slam_ws'if variant=='baseline_V12'else'navigation/parallel_lio_v15/slam_ws');vikit=base.parents[1]/'slam5_navigation/ros2_ws/install'
 d['LD_LIBRARY_PATH']=':'.join([str(ws/'install/fast_livo2_core/lib'),'/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu','/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib',str(vikit/'vikit_common/lib'),str(vikit/'vikit_ros/lib'),d.get('LD_LIBRARY_PATH','')]);d['FASTLIVO_LIO_JACOBIAN_THREADS']='1'if variant.endswith('T1')else'4';return d
variants=['baseline_V12','candidate_V15_T1','candidate_V15_T4']
def snapshot():
 freq={}
 for cpu in range(8):
  p=Path(f'/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_cur_freq');freq[str(cpu)]=int(p.read_text())if p.exists()else None
 return {'monotonic_ns':time.monotonic_ns(),'cpu_cur_freq_khz':freq,'loadavg':Path('/proc/loadavg').read_text().strip(),'procstat':Path('/proc/stat').read_text(),'thermal':['not_locked'],'P_core_affinity':'0-7','CPU_times_source':'CLOCK_PROCESS_CPUTIME_ID/CLOCK_THREAD_CPUTIME_ID'}
def execute(group,variant,input,reps,dest,observer=False):
 dest.mkdir(parents=True,exist_ok=False);exe=art/group/variant/'fixture';output=dest/'output.bin';report=dest/'samples.json';before=snapshot();args=[str(exe),str(input),str(output),str(report),str(reps)]if group=='real_rows'else[str(exe),str(output),str(report),str(reps),str(input)]
 e=env(variant)
 if observer:e.update(LD_PRELOAD=str(art/'observe_gomp.so'),V15_GOMP_RECORDS=str(dest/'teams.jsonl'))
 start=time.monotonic_ns();r=subprocess.run(['taskset','-c','0-7',*args],env=e,text=True,capture_output=True);end=time.monotonic_ns();(dest/'execution.log').write_text(r.stdout+r.stderr)
 if r.returncode:raise RuntimeError((group,variant,input,r.returncode,r.stderr))
 row={'group':group,'variant':variant,'input':str(input),'input_sha256':sha(input)if group=='real_rows'else sha(art/'lio_rows_scaling_fixture.cpp')+':half_extent='+str(input),'executable_sha256':sha(exe),'output_sha256':sha(output),'samples':json.loads(report.read_text()),'process_wall_ns':end-start,'before':before,'after':snapshot(),'observer_enabled':observer,'partition':'schedule(static); unique output row i; original matrix products after barrier'}
 (dest/'run_receipt.json').write_text(json.dumps(row,indent=2)+'\n');return row,output

if '--execute' not in sys.argv:raise SystemExit('Only --execute in the scheduled exclusive CPU window')
result={'scope':'Same-input production row kernel benefit only; complete synthetic StateEstimation times retained separately','scheme':'32 independent process batches per variant per scene; alternating ABBA/BAAB;10 warmup40 valid repeats within each batch; repeats are not independent batches','pilot48_retained':str(art/'benchmark_results.json'),'batches':[],'same_input_outputs':[],'criterion_plan':str(base/'test_results/parallel_pipeline_20261005/evaluation/PROSPECTIVE_EVALUATION_PLAN.json')}
scenes=[('scaling',3,'small147'),('scaling',15,'medium2883'),('scaling',45,'large24843')]+[('real_rows',x,'actual_'+x.stem)for x in sorted(real.glob('*.bin'))]
pattern=['baseline_V12','candidate_V15_T4','candidate_V15_T4','baseline_V12','candidate_V15_T4','baseline_V12','baseline_V12','candidate_V15_T4']
for group,input,label in scenes:
 hashes=[]
 for index,variant in enumerate(pattern*8):
  row,output=execute(group,variant,input,40,art/'confirmatory'/label/f'{index:03}_{variant}');row['sequence']=len(result['batches'])+1;row['scene']=label;row['balanced_pattern_position']=index%8;row['balanced_cycle']=index//8;result['batches'].append(row);hashes.append(row['output_sha256'])
 result['same_input_outputs'].append({'scene':label,'all64_process_outputs_bitexact':len(set(hashes))==1});print(json.dumps({'scene_finished':label,'independent_batches':64,'all_output_bitexact':len(set(hashes))==1}),flush=True)
result['all_process_outputs_bitexact']=all(x['all64_process_outputs_bitexact']for x in result['same_input_outputs']);(art/'confirmatory_results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'confirmatory_finished':True,'independent_process_batches':len(result['batches']),'all_output_bitexact':result['all_process_outputs_bitexact']}),flush=True)
