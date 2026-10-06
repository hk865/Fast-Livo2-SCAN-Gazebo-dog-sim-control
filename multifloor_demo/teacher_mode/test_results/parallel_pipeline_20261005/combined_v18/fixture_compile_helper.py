#!/usr/bin/env python3
from pathlib import Path
import shlex,subprocess,os,json,hashlib,struct
import numpy as np
art=Path(__file__).resolve().parent;base=art.parents[2]
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
