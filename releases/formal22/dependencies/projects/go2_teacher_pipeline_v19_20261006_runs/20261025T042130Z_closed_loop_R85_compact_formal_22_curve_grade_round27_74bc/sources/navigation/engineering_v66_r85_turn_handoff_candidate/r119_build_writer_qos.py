"""Compile only the local readonly helper; creates no ROS/DDS entity."""
import hashlib,json,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parent
PREFIX=Path('/opt/ros/jazzy')
source=ROOT/'r119_writer_qos.cpp';output=ROOT/'r119_writer_qos.so'
args=['c++','-std=c++17','-O2','-fPIC','-shared',str(source),'-o',str(output)]
for include in sorted((PREFIX/'include').iterdir()):
 if include.is_dir():args+=['-I'+str(include)]
args+=['-L'+str(PREFIX/'lib'),'-Wl,-rpath,'+str(PREFIX/'lib'),'-lrcl','-lrmw_fastrtps_shared_cpp','-lfastrtps','-lfastcdr']
subprocess.run(args,check=True)
paths=[source,output,PREFIX/'lib/librcl.so',PREFIX/'lib/librmw_fastrtps_cpp.so',
 PREFIX/'lib/librmw_fastrtps_shared_cpp.so',PREFIX/'lib/libfastrtps.so',
 PREFIX/'include/rmw_fastrtps_shared_cpp/rmw_fastrtps_shared_cpp/custom_publisher_info.hpp',
 PREFIX/'lib/python3.12/site-packages/rclpy/_rclpy_pybind11.cpython-312-x86_64-linux-gnu.so']
manifest=dict(schema='r119_readonly_writer_qos_build/v1',command=args,
 compiler=subprocess.check_output(['c++','--version'],text=True).splitlines()[0],
 package_versions=subprocess.check_output(['dpkg-query','-W','ros-jazzy-fastrtps',
 'ros-jazzy-rmw-fastrtps-cpp','ros-jazzy-rmw-fastrtps-shared-cpp'],text=True),
 hashes={str(p.resolve()):hashlib.sha256(p.read_bytes()).hexdigest()for p in paths},
 no_ROS_DDS_entities_created=True)
(ROOT/'r119_writer_qos_build.json').write_text(json.dumps(manifest,indent=2)+'\n')
print(json.dumps({'output':str(output),'bytes':output.stat().st_size,'sha256':hashlib.sha256(output.read_bytes()).hexdigest()}))
