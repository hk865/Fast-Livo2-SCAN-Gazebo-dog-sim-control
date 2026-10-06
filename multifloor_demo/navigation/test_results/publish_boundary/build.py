#!/usr/bin/env python3
"""Build only excluded local observer/fixture; never an installed ROS package."""
import hashlib,json,subprocess
from pathlib import Path
HERE=Path(__file__).resolve().parent
include=[arg for p in Path('/opt/ros/jazzy/include').iterdir() if p.is_dir() for arg in ('-I',str(p))]
common=['g++','-O2','-std=c++17','-pthread',*include,'-L/opt/ros/jazzy/lib','-Wl,-rpath,/opt/ros/jazzy/lib']
commands=[[*common,'-fPIC','-shared',str(HERE/'rcl_publish_audit.cpp'),'-o',str(HERE/'librcl_publish_audit.so'),'-lrcl','-lsensor_msgs__rosidl_typesupport_cpp','-lrosidl_typesupport_cpp','-ldl'],
          [*common,str(HERE/'publisher.cpp'),'-o',str(HERE/'publisher'),'-lrclcpp','-lrcl','-lrmw','-lrcutils','-ltracetools','-lsensor_msgs__rosidl_typesupport_cpp','-lsensor_msgs__rosidl_typesupport_c','-lsensor_msgs__rosidl_generator_c']]
for command in commands:subprocess.run(command,check=True)
receipt=dict(commands=commands,sha256={name:hashlib.sha256((HERE/name).read_bytes()).hexdigest() for name in ['rcl_publish_audit.cpp','librcl_publish_audit.so','publisher.cpp','publisher','build.py']})
(HERE/'build_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps(receipt['sha256'],indent=2))
