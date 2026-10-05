#!/usr/bin/env python3
"""Check local Ubuntu/ROS/Gazebo/CPU interpreter and external Frozen Teacher."""
from pathlib import Path
if not __debug__:
 raise RuntimeError("Optimized Python (-O/PYTHONOPTIMIZE) is forbidden for portable validation")
import argparse,importlib.util,json,os,platform,subprocess,sys
from portable_common import REPO,LOCAL,MODEL_SHA,sha
REQUIRED_APT=['ros-jazzy-desktop','ros-jazzy-ros-gz','ros-jazzy-pcl-ros','ros-jazzy-cv-bridge','ros-jazzy-sophus','ros-jazzy-rmw-fastrtps-cpp','python3-colcon-common-extensions','python3-scipy','python3-yaml','python3-pil','libeigen3-dev','libopencv-dev','libpcl-dev','libfmt-dev','libboost-thread-dev','libssl-dev','libapr1-dev']
def main():
 p=argparse.ArgumentParser();p.add_argument('--model',type=Path,required=True);p.add_argument('--cpu-python',type=Path,required=True);p.add_argument('--ros-setup',type=Path,default=Path('/opt/ros/jazzy/setup.bash'));a=p.parse_args();errors=[]
 os_release=Path('/etc/os-release').read_text();model=a.model.expanduser().resolve();cpu=a.cpu_python.expanduser().resolve()
 if 'VERSION_ID="24.04"'not in os_release:errors.append('This supported recipe is Ubuntu24.04 only')
 for x in [model,cpu,a.ros_setup,Path('/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge')]:
  if not x.is_file():errors.append('Missing '+str(x))
 mh=sha(model)if model.is_file()else None
 if mh!=MODEL_SHA:errors.append('Frozen Teacher SHA256 differs')
 probe={'status':'NOT_RUN'}
 if cpu.is_file():
  code='import importlib.util,json,sys;print(json.dumps({"python":sys.executable,"version":sys.version,"torch_spec":None if importlib.util.find_spec("torch")is None else importlib.util.find_spec("torch").origin,"numpy_spec":None if importlib.util.find_spec("numpy")is None else importlib.util.find_spec("numpy").origin}))'
  r=subprocess.run([str(cpu),'-B','-c',code],capture_output=True,text=True);probe=json.loads(r.stdout)if r.returncode==0 else{'error':r.stderr}
  if not probe.get('torch_spec')or not probe.get('numpy_spec'):errors.append('CPU interpreter needs torch and numpy')
 versions=subprocess.run(['dpkg-query','-W','-f=${Package}\t${Version}\n',*REQUIRED_APT],capture_output=True,text=True)
 d={'schema':'portable_environment/v1','status':'PASS_ENV_ONLY'if not errors else'BLOCKED_ENV','repo':str(REPO),'os_release':os_release,'machine':platform.machine(),'ros_setup':str(a.ros_setup),'model_path':str(model),'model_sha256':mh,'CPU_python':probe,'apt_version_observations':versions.stdout,'apt_query_missing_messages':versions.stderr,'install_recipe_packages':REQUIRED_APT,'errors':errors,'source_build_verified':False,'actual_runtime_verified':False,'scope':'No ROS nodes, Gazebo or actor forward created; model digest/spec only'}
 LOCAL.mkdir(exist_ok=True);(LOCAL/'ENV_CHECK.json').write_text(json.dumps(d,indent=2)+'\n');print(d['status']);
 if errors:raise SystemExit(2)
if __name__=='__main__':main()
