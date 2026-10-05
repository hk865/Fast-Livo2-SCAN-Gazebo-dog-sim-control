#!/usr/bin/env python3
from pathlib import Path
import hashlib,json,os,shlex,subprocess
OUT=Path(__file__).resolve().parent;ROOT=OUT.parents[2];BASE=ROOT/'navigation/lidar_sampling_v12/slam_ws';NEW=ROOT/'navigation/parallel_vio_v16/slam_ws'
meta={'schema':'independent_vio_patch_fixture_build/v1','scope':'Synthetic updateState/updateStateInverse boundary only; no processFrame/ROS/physics replay','fixture_source_sha256':hashlib.sha256((OUT/'vio_patch_fixture.cpp').read_bytes()).hexdigest(),'variants':{}}
for name,ws in [('baseline_V12',BASE),('candidate_V16_t4',NEW),('candidate_V16_t1',NEW)]:
 d=OUT/name;d.mkdir(exist_ok=True);lib=ws/'install/fast_livo2_core/lib/libfast_livo2_core.so';flags={}
 if name=='candidate_V16_t4':lib=d/'libfast_livo2_core.so'
 if name=='candidate_V16_t1':
  (d/'libfast_livo2_core.so').write_bytes(lib.read_bytes());lib=d/'libfast_livo2_core.so'
 for line in (ws/'build/fast_livo2_core/CMakeFiles/fast_livo2_core.dir/flags.make').read_text().splitlines():
  if line.startswith('CXX_'):k,val=line.split('=',1);flags[k.strip()]=shlex.split(val)
 vikit=ROOT.parents[1]/'slam5_navigation/ros2_ws/install/vikit_common/lib/libvikit_common.so'
 cmd=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],str(OUT/'vio_patch_fixture.cpp'),str(lib),str(vikit),'/usr/lib/x86_64-linux-gnu/libpcl_common.so','/usr/lib/x86_64-linux-gnu/libopencv_core.so','/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so','/opt/ros/jazzy/lib/librclcpp.so','-Wl,-rpath,'+str(lib.parent),'-Wl,-rpath-link,/opt/ros/jazzy/lib','-o',str(d/'fixture')]
 env=os.environ.copy();env['LD_LIBRARY_PATH']=':'.join([str(lib.parent),'/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu','/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib',str(vikit.parent),str(ROOT.parents[1]/'slam5_navigation/ros2_ws/install/vikit_ros/lib'),env.get('LD_LIBRARY_PATH','')]);env.update(OMP_NUM_THREADS='4',OMP_DYNAMIC='FALSE',OPENBLAS_NUM_THREADS='1')
 result=subprocess.run(cmd,env=env,text=True,capture_output=True);(d/'compile.log').write_text(result.stdout+result.stderr)
 meta['variants'][name]={'lib_path':str(lib),'lib_sha256':hashlib.sha256(lib.read_bytes()).hexdigest(),'compile_argv':cmd,'returncode':result.returncode,'env':{k:env[k]for k in ['LD_LIBRARY_PATH','OMP_NUM_THREADS','OMP_DYNAMIC','OPENBLAS_NUM_THREADS']}}
 if result.returncode:raise RuntimeError(name+result.stderr[-5000:])
 meta['variants'][name]['executable_sha256']=hashlib.sha256((d/'fixture').read_bytes()).hexdigest()
(OUT/'fixture_build_receipt.json').write_text(json.dumps(meta,indent=2)+'\n');print(json.dumps({k:v['lib_sha256']for k,v in meta['variants'].items()}))
