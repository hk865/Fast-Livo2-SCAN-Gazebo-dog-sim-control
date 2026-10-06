#!/usr/bin/env python3
from pathlib import Path
import shlex,subprocess,os,json,hashlib,struct
import numpy as np
art=Path(__file__).resolve().parent;base=art.parents[2]
old=base/'navigation/lidar_sampling_v10/slam_ws';new=base/'navigation/lidar_sampling_v11/slam_ws'
original=(base/'navigation/closed_loop_multifloor_v7/analysis/lio_fixture/go2_v7_lio_equivalence_fixture.cpp').read_text()
original=original.replace('int main(int argc,char **argv)','int run_lio(int argc,char **argv)')
original=original.replace('<<" state_values="<<values.size()', '<<" query_rows="<<manager.diagnostic_query_rows_.size()<<" state_values="<<values.size()')
fixture=art/'finite_lio_vio_fixture.cpp'
fixture.write_text(original+r'''
#include "vio.h"
#include <memory>
int run_vio(const char* output,bool inverse){
  StatesGroup current,predicted;
  current.gravity=predicted.gravity=Eigen::Vector3d(0,0,-9.81);
  current.inv_expo_time=predicted.inv_expo_time=1;
  current.pos_end=predicted.pos_end=Eigen::Vector3d::Zero();
  vk::PinholeCamera camera(128,128,1,100,100,64,64,0,0,0,0,0);
  cv::Mat image(128,128,CV_8UC1);
  for(int y=0;y<128;++y) for(int x=0;x<128;++x) image.at<uint8_t>(y,x)=40+x/2+y/3;
  VIOManager manager;
  manager.cam=&camera;manager.state=&current;manager.state_propagat=&predicted;
  manager.visual_submap=new SubSparseMap;
  manager.Rci=Eigen::Matrix3d::Identity();manager.Pci=Eigen::Vector3d::Zero();
  manager.Jdphi_dR=Eigen::Matrix3d::Identity();manager.Jdp_dR=Eigen::Matrix3d::Zero();
  manager.width=128;manager.height=128;manager.fx=100;manager.fy=100;manager.cx=64;manager.cy=64;
  manager.patch_size=4;manager.patch_size_half=2;manager.patch_size_total=16;
  manager.total_points=9;manager.max_iterations=3;manager.img_point_cov=1000;
  manager.exposure_estimate_en=false;manager.has_ref_patch_cache=true;
  manager.G.setZero();manager.H_T_H.setZero();
  manager.new_frame_.reset(new Frame(&camera,image));
  std::vector<std::unique_ptr<VisualPoint>> owners;
  for(int j=0;j<3;++j) for(int i=0;i<3;++i){
    Eigen::Vector3d location((i-1)*.24,(j-1)*.24,2);
    owners.emplace_back(new VisualPoint(location));
    manager.visual_submap->voxel_points.push_back(owners.back().get());
    manager.visual_submap->search_levels.push_back(0);manager.visual_submap->errors.push_back(0);
    manager.visual_submap->inv_expo_list.push_back(1);
    auto pixel=camera.world2cam(location);std::vector<float> patch;
    for(int x=0;x<4;++x) for(int y=0;y<4;++y)
      patch.push_back(image.at<uint8_t>(int(pixel[1])+x-2,int(pixel[0])+y-2)+.125f);
    manager.visual_submap->warp_patch.push_back(patch);
  }
  manager.H_sub_inv.resize(9*16,6);
  for(int r=0;r<9*16;++r) for(int c=0;c<6;++c) manager.H_sub_inv(r,c)=.01*((r+3*c)%11-5);
  fastlivo_diag::Logger::instance().set_context(2,130100000000ULL);
  // Match the OpenMP main-thread default after the existing LIO two-thread loop.
  omp_set_num_threads(2);
  if(inverse)manager.updateStateInverse(image,0);else manager.updateState(image,0);
  std::vector<double> values;
  fastlivo_diag::state(values,current);fastlivo_diag::append(values,current.cov);
  fastlivo_diag::append(values,manager.G);fastlivo_diag::append(values,manager.H_T_H);
  for(double v:values)if(!std::isfinite(v))return 8;
  std::ofstream out(output,std::ios::binary);out.write(reinterpret_cast<const char*>(values.data()),values.size()*sizeof(double));
  std::cout<<"mode="<<(inverse?"inverse":"forward")<<" points="<<manager.total_points<<" values="<<values.size()<<" pos="<<current.pos_end.transpose()<<'\n';
  return out?0:9;
}
int main(int argc,char** argv){
  if(argc!=3)return 2;
  std::string mode=argv[1];
  if(mode=="lio"){char* local[]={argv[0],argv[2]};return run_lio(2,local);}
  if(mode=="vio")return run_vio(argv[2],false);
  if(mode=="inverse")return run_vio(argv[2],true);
  return 3;
}
''')
meta={'scope':'Limited synthetic LIO729 points and synthetic VIO9patches forward/inverse; no full actual trajectory replay','fixture_source_sha256':hashlib.sha256(fixture.read_bytes()).hexdigest(),'variants':{},'comparisons':[]}
envs={}
for name,ws in [('baseline_V10',old),('candidate_V11',new)]:
 dest=art/name;dest.mkdir(exist_ok=True);lib=ws/'install/fast_livo2_core/lib/libfast_livo2_core.so';flags={}
 for line in (ws/'build/fast_livo2_core/CMakeFiles/fast_livo2_core.dir/flags.make').read_text().splitlines():
  if line.startswith('CXX_'):k,value=line.split('=',1);flags[k.strip()]=shlex.split(value)
 vikit=base.parents[1]/'slam5_navigation/ros2_ws/install/vikit_common/lib/libvikit_common.so'
 cmd=['/usr/bin/c++',*flags['CXX_DEFINES'],*flags['CXX_INCLUDES'],*flags['CXX_FLAGS'],str(fixture),str(lib),str(vikit),'/usr/lib/x86_64-linux-gnu/libpcl_common.so','/usr/lib/x86_64-linux-gnu/libopencv_core.so','/usr/lib/x86_64-linux-gnu/libopencv_imgproc.so','/opt/ros/jazzy/lib/librclcpp.so','-Wl,-rpath,'+str(lib.parent),'-Wl,-rpath-link,/opt/ros/jazzy/lib','-o',str(dest/'fixture')]
 env=os.environ.copy();env['LD_LIBRARY_PATH']=':'.join([str(lib.parent),'/opt/ros/jazzy/lib','/opt/ros/jazzy/lib/x86_64-linux-gnu','/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib',str(vikit.parent),str(base.parents[1]/'slam5_navigation/ros2_ws/install/vikit_ros/lib'),env.get('LD_LIBRARY_PATH','')]);env.update({'OMP_NUM_THREADS':'1','OPENBLAS_NUM_THREADS':'1','OMP_DYNAMIC':'FALSE'})
 r=subprocess.run(cmd,env=env,text=True,capture_output=True);(dest/'compile.log').write_text(r.stdout+r.stderr)
 assert r.returncode==0,(name,r.stderr[-3000:])
 envs[name]=env;v={'lib_path':str(lib),'lib_sha256':hashlib.sha256(lib.read_bytes()).hexdigest(),'compile_argv':cmd,'executable_sha256':hashlib.sha256((dest/'fixture').read_bytes()).hexdigest(),'runs':{}}
 for scene in ['lio','vio','inverse']:
  for mode in ['off','inside','outside']:
   tag=scene+'_'+mode;runenv=env.copy();runenv.pop('FASTLIVO_DIAGNOSTIC_DIR',None)
   if mode!='off':runenv.update(FASTLIVO_DIAGNOSTIC_DIR=str(dest/(tag+'_records')),FASTLIVO_DIAGNOSTIC_BEGIN='129'if mode=='inside'else'115',FASTLIVO_DIAGNOSTIC_END='131'if mode=='inside'else'118')
   rr=subprocess.run([str(dest/'fixture'),scene,str(dest/(tag+'.bin'))],env=runenv,text=True,capture_output=True)
   (dest/(tag+'.log')).write_text(rr.stdout+rr.stderr);assert rr.returncode==0,(name,tag,rr.stdout,rr.stderr)
   v['runs'][tag]={'returncode':rr.returncode,'stdout':rr.stdout,'stderr':rr.stderr,'output_sha256':hashlib.sha256((dest/(tag+'.bin')).read_bytes()).hexdigest()}
 meta['variants'][name]=v
# Inside-window recorded solver/query/VIO values must be identical, in order.
def records(p):
 with p.open('rb')as f:
  assert f.read(16)==b'FLIVODIAG0001LE\0'
  while b:=f.read(56):
   h=struct.unpack('<7Q',b);data=f.read(h[-1]*8);assert len(data)==h[-1]*8
   if h[0]==100:
    a=np.frombuffer(data,dtype='<f8').copy();a[9:13]=0;data=a.tobytes()
   yield h,data
for scene in ['lio','vio','inverse']:
 values=[(art/name/(scene+'_'+mode+'.bin')).read_bytes()for name in meta['variants']for mode in ['off','inside','outside']]
 assert len(set(values))==1,scene
 a=list(records(art/'baseline_V10'/(scene+'_inside_records/records.bin')));b=list(records(art/'candidate_V11'/(scene+'_inside_records/records.bin')))
 assert a==b,('inside diagnostics differ',scene)
 outside10=list(records(art/'baseline_V10'/(scene+'_outside_records/records.bin')));outside11=list(records(art/'candidate_V11'/(scene+'_outside_records/records.bin')))
 assert not outside11,('unexpected outside copied diagnostics',scene)
 assert outside10,('baseline fixture did not exercise old outside allocation',scene)
 meta['comparisons'].append({'scene':scene,'all_six_state_cov_and_solver_outputs_byte_identical':True,'inside_diagnostic_records':len(a),'inside_nonwall_payloads_byte_identical':True,'baseline_outside_record_kinds':[h[0]for h,d in outside10],'candidate_outside_record_count':len(outside11)})
assert 'query_rows=729' in meta['variants']['baseline_V10']['runs']['lio_outside']['stdout']
assert 'query_rows=0' in meta['variants']['candidate_V11']['runs']['lio_outside']['stdout']
meta['outside_query_allocation_729_to_zero']=True;meta['total_process_runs']=18;meta['full_actual_replay_equivalence']='UNVERIFIED'
(art/'finite_fixture_comparison.json').write_text(json.dumps(meta,indent=2)+'\n')
print(json.dumps({'process_runs':18,'comparisons':meta['comparisons'],'outside_query_allocation_729_to_zero':True}))
