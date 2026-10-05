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
if '--numeric' in sys.argv:
 result={'scope':'Selected actual V12 ptpl rows only, no map/matching/full trajectory replay','real_export_manifest_sha256':sha(real/'manifest.json'),'cases':[]}
 for input in sorted(real.glob('*.bin')):
  outputs=[];records=[]
  for variant in variants:
   receipt,output=execute('real_rows',variant,input,1,art/'real_numeric'/input.stem/variant);outputs.append(output.read_bytes());records.append(receipt)
  npz=np.load(input.with_suffix('.npz'));n=len(npz['rows']);values=np.frombuffer(outputs[0],dtype='<f8');components={'expected_H':values[:6*n].reshape(n,6),'expected_R_inv':values[12*n:13*n],'expected_meas':values[13*n:14*n],'expected_sigma':values[14*n:15*n],'expected_body_world':values[15*n:24*n].reshape(n,3,3),'expected_HTH':values[24*n:24*n+36].reshape(6,6),'expected_HTz':values[-6:]}
  checks={}
  for name,value in components.items():
   expected=np.asarray(npz[name],dtype='<f8').reshape(value.shape);checks[name]={'byte_equal':value.tobytes()==expected.tobytes(),'max_abs_error':float(np.max(np.abs(value-expected)))}
  result['cases'].append({'input':str(input),'rows':n,'input_sha256':sha(input),'V12_V15_T1_T4_output_byte_identical':len(set(outputs))==1,'recorded_expected_checks':checks,'runs':records})
 result['all_pass']=all(c['V12_V15_T1_T4_output_byte_identical']and all(v['byte_equal']for v in c['recorded_expected_checks'].values())for c in result['cases']);(art/'real_numeric_comparison.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'real_numeric_pass':result['all_pass'],'cases':[(x['rows'],x['recorded_expected_checks'])for x in result['cases']]}));
 # Observe teams separately; timing never uses preload.
 for variant in variants[1:]:execute('real_rows',variant,sorted(real.glob('*.bin'))[0],1,art/'team_observation'/variant,True)
 if not result['all_pass']:raise SystemExit(5)
elif '--benchmark' in sys.argv:
 result={'scope':'Same-input production row kernel benefit only; complete synthetic StateEstimation times also retained','scheme':'ABBA then BAAB per scene, A=V12/B=V15_T4; 48 batches,40 valid per batch,10 warmup','batches':[],'synthetic_same_input_outputs':[]}
 scenes=[('scaling',3,'small147'),('scaling',15,'medium2883'),('scaling',45,'large24843')]+[('real_rows',x,'actual_'+x.stem)for x in sorted(real.glob('*.bin'))]
 for group,input,label in scenes:
  outputs=[]
  for index,variant in enumerate(['baseline_V12','candidate_V15_T4','candidate_V15_T4','baseline_V12','candidate_V15_T4','baseline_V12','baseline_V12','candidate_V15_T4']):
   row,output=execute(group,variant,input,40,art/'benchmark'/label/f'{index}_{variant}');row['sequence']=len(result['batches'])+1;row['scene']=label;row['ABBA_position']=index%4;row['balanced_cycle']=index//4;result['batches'].append(row);outputs.append(output.read_bytes())
  result['synthetic_same_input_outputs'].append({'scene':label,'all_ABBA_output_bytes_equal':len(set(outputs))==1})
 result['all_ABBA_output_bytes_equal']=all(x['all_ABBA_output_bytes_equal']for x in result['synthetic_same_input_outputs']);(art/'benchmark_results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'benchmark_finished':True,'batches':len(result['batches']),'all_output_bytes_equal':result['all_ABBA_output_bytes_equal']}))
else:raise SystemExit('Choose --numeric or --benchmark; do not run statistical benchmarks without root CPU window')
