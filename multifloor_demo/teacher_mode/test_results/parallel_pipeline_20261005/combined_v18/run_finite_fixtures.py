#!/usr/bin/env python3
from pathlib import Path
import shlex,subprocess,os,json,hashlib,struct
import numpy as np
art=Path(__file__).resolve().parent;base=art.parents[2]
variants=[('baseline_V12',base/'navigation/lidar_sampling_v12/slam_ws',None),('candidate_V18_T1',base/'navigation/combined_compute_v18/slam_ws','1'),('candidate_V18_T4',base/'navigation/combined_compute_v18/slam_ws','4')]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def compile_fixture(ws,dest,source):
 lib=ws/'install/fast_livo2_core/lib/libfast_livo2_core.so';flags={}
 for line in (ws/'build/fast_livo2_core/CMakeFiles/fast_livo2_core.dir/flags.make').read_text().splitlines():
  if line.startswith('CXX_'):k,v=line.split('=',1);flags[k.strip()]=shlex.split(v)
 vikit=base.parents[1]/'slam5_navigation/ros2_ws/install/vikit_common/lib/libvikit_common.so'
 cmd=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],str(source),str(lib),str(vikit),'/usr/lib/x86_64-linux-gnu/libpcl_common.so','/usr/lib/x86_64-linux-gnu/libopencv_core.so','/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so','/opt/ros/jazzy/lib/librclcpp.so','-Wl,-rpath,'+str(lib.parent),'-Wl,-rpath-link,/opt/ros/jazzy/lib','-o',str(dest/'fixture')]
 env=os.environ.copy();env['LD_LIBRARY_PATH']=':'.join([str(lib.parent),'/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu','/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib',str(vikit.parent),str(base.parents[1]/'slam5_navigation/ros2_ws/install/vikit_ros/lib'),env.get('LD_LIBRARY_PATH','')]);env.update({'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','OMP_DYNAMIC':'FALSE','FASTLIVO_BOUNDARY_TIMING':'0'})
 result=subprocess.run(cmd,env=env,text=True,capture_output=True);(dest/'compile.log').write_text(result.stdout+result.stderr)
 if result.returncode:raise RuntimeError((dest.name,result.stderr[-3000:]))
 return env,{'lib_path':str(lib),'lib_sha256':sha(lib),'compile_argv':cmd,'executable_sha256':sha(dest/'fixture')}
def records(p):
 with p.open('rb')as f:
  assert f.read(16)==b'FLIVODIAG0001LE\0'
  while b:=f.read(56):
   h=struct.unpack('<7Q',b);data=f.read(h[-1]*8);assert len(data)==h[-1]*8
   if h[0]==100:
    a=np.frombuffer(data,dtype='<f8').copy();a[9:13]=0;data=a.tobytes()
   yield h,data
meta={'scope':'Synthetic LIO729point and VIO9patch forward/inverse regression; no actual full SLAM trajectory','variants':{},'comparisons':[],'source_sha256':sha(art/'finite_lio_vio_fixture.cpp')}
for name,ws,threads in variants:
 dest=art/'finite'/name;dest.mkdir(parents=True,exist_ok=False);env,v=compile_fixture(ws,dest,art/'finite_lio_vio_fixture.cpp');v['runs']={};v['jacobian_threads']=threads
 if threads:env['FASTLIVO_LIO_JACOBIAN_THREADS']=threads
 else:env.pop('FASTLIVO_LIO_JACOBIAN_THREADS',None)
 for scene in ['lio','vio','inverse']:
  for mode in ['off','inside','outside']:
   tag=scene+'_'+mode;runenv=env.copy();runenv.pop('FASTLIVO_DIAGNOSTIC_DIR',None)
   if mode!='off':runenv.update(FASTLIVO_DIAGNOSTIC_DIR=str(dest/(tag+'_records')),FASTLIVO_DIAGNOSTIC_BEGIN='129'if mode=='inside'else'115',FASTLIVO_DIAGNOSTIC_END='131'if mode=='inside'else'118')
   rr=subprocess.run(['taskset','-c','0-7',str(dest/'fixture'),scene,str(dest/(tag+'.bin'))],env=runenv,text=True,capture_output=True);(dest/(tag+'.log')).write_text(rr.stdout+rr.stderr)
   v['runs'][tag]={'returncode':rr.returncode,'stdout':rr.stdout,'stderr':rr.stderr,'output_sha256':sha(dest/(tag+'.bin'))if(dest/(tag+'.bin')).exists()else None}
   if rr.returncode:raise RuntimeError((name,tag,rr.stderr))
   if mode!='off':
    stats=json.loads((dest/(tag+'_records/writer_stats.json')).read_text());v['runs'][tag]['writer_stats']=stats
    if stats['dropped']or stats['writer_io_failed']or stats['attempted']!=stats['written']:raise RuntimeError(('incomplete diagnostics',name,tag,stats))
 meta['variants'][name]=v
for scene in ['lio','vio','inverse']:
 values=[(art/'finite'/name/(scene+'_'+mode+'.bin')).read_bytes()for name,ws,t in variants for mode in ['off','inside','outside']]
 a=list(records(art/'finite/baseline_V12'/(scene+'_inside_records/records.bin')))
 diags=[list(records(art/'finite'/name/(scene+'_inside_records/records.bin')))for name,ws,t in variants]
 meta['comparisons'].append({'scene':scene,'nine_outputs_byte_identical':len(set(values))==1,'inside_nonwall_all_records_byte_identical':all(x==a for x in diags),'inside_diagnostic_records':len(a),'inside_kind_counts':{str(k):sum(h[0]==k for h,d in a)for k in set(h[0]for h,d in a)},'outside_diagnostic_count':[len(list(records(art/'finite'/name/(scene+'_outside_records/records.bin'))))for name,ws,t in variants]})
meta['all_numeric_comparisons_pass']=all(x['nine_outputs_byte_identical']and x['inside_nonwall_all_records_byte_identical']for x in meta['comparisons']);meta['actual_full_trajectory_equivalence']='UNVERIFIED'
(art/'finite_fixture_comparison.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps({'pass':meta['all_numeric_comparisons_pass'],'comparisons':meta['comparisons']}))
if not meta['all_numeric_comparisons_pass']:raise SystemExit(5)
