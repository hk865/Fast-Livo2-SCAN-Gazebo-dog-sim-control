#!/usr/bin/env python3
"""Rebuild private simulation dependencies from exported source, never start ROS."""
from pathlib import Path
import argparse,hashlib,json,os,shutil,subprocess,time
REPO=Path(__file__).resolve().parents[1];LOCAL=REPO/'.local';TEACHER=REPO/'multifloor_demo/teacher_mode'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def run(cmd,log,env=None,cwd=None):
 started=time.monotonic()
 with log.open('w')as f:r=subprocess.run(cmd,stdout=f,stderr=subprocess.STDOUT,env=env,cwd=cwd)
 if r.returncode:raise RuntimeError('Build failed: '+str(log))
 return {'argv':cmd,'returncode':r.returncode,'wall_s':time.monotonic()-started,'log':str(log),'log_sha256':sha(log)}
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--variants',nargs='+',choices=['v12','v18','v17'],default=['v12','v18','v17']);ap.add_argument('--ros-setup',type=Path,default=Path('/opt/ros/jazzy/setup.bash'));a=ap.parse_args()
 if not a.ros_setup.is_file():raise RuntimeError('ROS Jazzy setup missing; install declared dependencies first')
 LOCAL.mkdir(exist_ok=True);logs=LOCAL/'build_logs';logs.mkdir(exist_ok=True)
 env={**os.environ,'MAKEFLAGS':'-j2 -l2','CMAKE_BUILD_PARALLEL_LEVEL':'2','PYTHONDONTWRITEBYTECODE':'1'};receipt={'schema':'portable_source_build/v1','status':'BUILDING','original_installs_used_as_project_underlay':False,'jobs':2,'load_limit':2,'steps':[],'variants':{},'source_inputs':{}}
 # SDK built privately from source. No /usr/local mutation or hardware operation.
 sdk=REPO/'tools/vendor/livox_sdk2';prefix=LOCAL/'livox_sdk';build=LOCAL/'livox_sdk_build'
 for cmd,tag in [(['cmake','-S',str(sdk),'-B',str(build),'-DCMAKE_BUILD_TYPE=Release','-DCMAKE_INSTALL_PREFIX='+str(prefix)],'sdk_configure'),(['cmake','--build',str(build),'--parallel','2','--','-l2'],'sdk_build'),(['cmake','--install',str(build)],'sdk_install')]:receipt['steps'].append(run(cmd,logs/(tag+'.log'),env))
 under=LOCAL/'underlay_ws';src=under/'src';src.mkdir(parents=True,exist_ok=True)
 sources={'vikit_common':REPO/'slam5_navigation/ros2_ws/src/vikit_common','vikit_ros':REPO/'slam5_navigation/ros2_ws/src/vikit_ros','scan_planner_msgs':REPO/'scan_multifloor/ros2_ws/src/planner/scan_planner_msgs','livox_ros_driver2':REPO/'tools/vendor/livox_ros_driver2'}
 for name,p in sources.items():
  q=src/name
  if not q.exists():shutil.copytree(p,q)
  receipt['source_inputs'][name]={str(x.relative_to(p)):sha(x)for x in p.rglob('*')if x.is_file()}
 def colcon(ws,tag,setup,extra=[]):
  ws.mkdir(parents=True,exist_ok=True)
  # Passing paths positionally avoids shell expansion of checkpoint or source text.
  script='set -eo pipefail; source "$1"; shift; if [ "$1" != "-" ]; then source "$1"; fi; shift; cd "$1"; shift; exec colcon build --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON "$@"'
  cmd=['bash','-c',script,'portable-source-build',str(a.ros_setup),str(setup)if setup else'-',str(ws),*extra]
  receipt['steps'].append(run(cmd,logs/(tag+'.log'),env))
 colcon(under,'underlay',None,['-DROS_EDITION=ROS2','-DDISTRO_ROS=jazzy','-DLIVOX_LIDAR_SDK_LIBRARY='+str(prefix/'lib/liblivox_lidar_sdk_shared.so'),'-DLIVOX_LIDAR_SDK_INCLUDE_DIR='+str(prefix/'include')])
 scan=LOCAL/'scan_ws';scan_src=scan/'src';scan_src.mkdir(parents=True,exist_ok=True)
 for source in (REPO/'multifloor_demo/navigation/ros2_ws/src').iterdir():
  if source.is_dir()and not(scan_src/source.name).exists():shutil.copytree(source,scan_src/source.name)
 if not(scan_src/'traj_utils').exists():shutil.copytree(REPO/'scan_multifloor/ros2_ws/src/planner/traj_utils',scan_src/'traj_utils')
 colcon(scan,'SCAN',under/'install/setup.bash')
 # Native actuator rebuilt; it is the sole joint writer.
 plugin=TEACHER/'simulation/build'
 receipt['steps'].append(run(['cmake','-S',str(plugin.parent),'-B',str(plugin),'-DCMAKE_BUILD_TYPE=Release'],logs/'actuator_configure.log',env))
 receipt['steps'].append(run(['cmake','--build',str(plugin),'--parallel','2','--','-l2'],logs/'actuator_build.log',env))
 names={'v12':'lidar_sampling_v12','v18':'combined_compute_v18','v17':'ingress_pipeline_v17'}
 for v in a.variants:
  ws=TEACHER/'navigation'/names[v]/'slam_ws';extra=['-DV16_VIO_PATCH_THREADS=1','-DV16_VIO_PATCH_MIN_POINTS=64']if v=='v18'else[]
  colcon(ws,v,under/'install/setup.bash',extra)
  files=[ws/'install/fast_livo2_core/lib/libfast_livo2_core.so',ws/'install/fast_livo2_ros/lib/fast_livo2_ros/fastlivo_mapping',ws/'build/fast_livo2_core/CMakeCache.txt',ws/'build/fast_livo2_core/compile_commands.json',ws/'build/fast_livo2_ros/CMakeCache.txt',ws/'build/fast_livo2_ros/compile_commands.json']
  receipt['variants'][v]={'workspace':str(ws),'artifacts_sha256':{str(p):sha(p)for p in files},'compile_commands':json.loads((ws/'build/fast_livo2_core/compile_commands.json').read_text())}
 receipt['native_plugin_sha256']=sha(plugin/'libteacher_actuator.so');receipt['status']='PASS_SOURCE_BUILD_ONLY';receipt['actual_simulation_verified']=False
 (LOCAL/'SOURCE_BUILD_RECEIPT.json').write_text(json.dumps(receipt,indent=2)+'\n');print('PASS_SOURCE_BUILD_ONLY; finite local preflight still required')
if __name__=='__main__':main()
