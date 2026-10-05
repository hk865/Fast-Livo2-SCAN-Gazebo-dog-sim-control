#!/usr/bin/env python3
from pathlib import Path
import shlex,subprocess,os,json,hashlib,struct
import numpy as np
base=Path(__file__).resolve().parents[3]
old=base/'navigation/closed_loop_multifloor_v7/slam_ws'
new=base/'navigation/lidar_sampling_v10/slam_ws'
out=Path(__file__).resolve().parent
fixture=out/'existing_lio_fixture.cpp'
fixture.write_bytes((old.parent/'analysis/lio_fixture/go2_v7_lio_equivalence_fixture.cpp').read_bytes())
metadata={'fixture_source_sha256':hashlib.sha256(fixture.read_bytes()).hexdigest(),'scope':'original synthetic logging and LIO estimator fixture; not full actual replay or navigation acceptance','variants':{}}
for name,ws in [('serial',old),('lio_two_threads',new)]:
 build=ws/'build/fast_livo2_core';lib=ws/'install/fast_livo2_core/lib/libfast_livo2_core.so';dest=out/name;dest.mkdir(exist_ok=True)
 flags={}
 for line in (build/'CMakeFiles/fast_livo2_core.dir/flags.make').read_text().splitlines():
  if line.startswith('CXX_'):
   k,value=line.split('=',1);flags[k.strip()]=shlex.split(value)
 cmd=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],str(fixture),str(lib),'/usr/lib/x86_64-linux-gnu/libpcl_common.so','/opt/ros/jazzy/lib/librclcpp.so','-Wl,-rpath,'+str(lib.parent),'-Wl,-rpath-link,/opt/ros/jazzy/lib','-o',str(dest/'fixture')]
 env=os.environ.copy();env['LD_LIBRARY_PATH']=':'.join([str(lib.parent),'/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu','/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib',str(base.parents[1]/'slam5_navigation/ros2_ws/install/vikit_common/lib'),str(base.parents[1]/'slam5_navigation/ros2_ws/install/vikit_ros/lib'),env.get('LD_LIBRARY_PATH','')]);env.update({'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','OMP_DYNAMIC':'FALSE'})
 r=subprocess.run(cmd,env=env,text=True,capture_output=True);(dest/'compile.log').write_text(r.stdout+r.stderr)
 assert r.returncode==0,(name,r.stderr[-3000:])
 v={'lib_path':str(lib),'lib_sha256':hashlib.sha256(lib.read_bytes()).hexdigest(),'compile_argv':cmd,'executable_sha256':hashlib.sha256((dest/'fixture').read_bytes()).hexdigest(),'runs':{}}
 for mode in ['off','on']:
  runenv=env.copy();runenv.pop('FASTLIVO_DIAGNOSTIC_DIR',None)
  if mode=='on':runenv['FASTLIVO_DIAGNOSTIC_DIR']=str(dest/'on_records')
  rr=subprocess.run([str(dest/'fixture'),str(dest/(mode+'.bin'))],env=runenv,text=True,capture_output=True)
  (dest/(mode+'.log')).write_text(rr.stdout+rr.stderr);assert rr.returncode==0,(name,mode,rr.stderr)
  v['runs'][mode]={'returncode':rr.returncode,'stdout':rr.stdout,'stderr':rr.stderr,'output_sha256':hashlib.sha256((dest/(mode+'.bin')).read_bytes()).hexdigest()}
 metadata['variants'][name]=v
# Compare every payload, in order, excluding only four wall-clock timing doubles.
def records(p):
 with p.open('rb') as f:
  assert f.read(16)==b'FLIVODIAG0001LE\0'
  while b:=f.read(56):
   h=struct.unpack('<7Q',b);data=f.read(h[-1]*8);assert len(data)==h[-1]*8
   if h[0]==100:
    a=np.frombuffer(data,dtype='<f8').copy();a[9:13]=0;data=a.tobytes()
   yield h,data
s=list(records(out/'serial/on_records/records.bin'));m=list(records(out/'lio_two_threads/on_records/records.bin'))
assert len(s)==len(m)
comparison=[]
for (hs,ds),(hm,dm) in zip(s,m):
 assert hs==hm,('header changed',hs,hm)
 same=ds==dm;comparison.append({'kind':hs[0],'sequence':hs[1],'iteration':hs[4],'values':hs[-1],'byte_identical_excluding_timing':same})
 assert same,('non-wall diagnostic differs',hs)
output_set=[(out/name/(mode+'.bin')).read_bytes() for name in ['serial','lio_two_threads'] for mode in ['off','on']]
assert len(set(output_set))==1
metadata.update({'all_four_state25_cov361_outputs_byte_identical':True,'all_diagnostic_payloads_byte_identical_except_wall_fields_9_12':True,'diagnostic_comparisons':comparison,'clock_fields_excluded_only':{'100':[9,10,11,12]},'OMP_NUM_THREADS':'1','OMP_DYNAMIC':'FALSE','note':'Candidate per-point loop overrides inherited OMP_NUM_THREADS=1 to MP_PROC_NUM=2; actual team size checked separately.'})
(out/'fixture_comparison.json').write_text(json.dumps(metadata,indent=2)+'\n')
print(json.dumps({'state_outputs_byte_identical':True,'diagnostic_records':len(s),'all_nonwall_payloads_byte_identical':True}))
