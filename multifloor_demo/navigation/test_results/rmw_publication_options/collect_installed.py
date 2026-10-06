#!/usr/bin/env python3
"""Read-only package/file receipts. No ROS context, node, library load or policy change."""
import datetime
import hashlib
import json
import os
import pathlib
import subprocess

ROOT = pathlib.Path(__file__).resolve().parents[3]
OUT = pathlib.Path(__file__).resolve().parent
packages = ['ros-jazzy-rmw-fastrtps-cpp', 'ros-jazzy-rmw-fastrtps-shared-cpp',
            'ros-jazzy-fastrtps', 'ros-jazzy-fastcdr', 'ros-jazzy-rmw',
            'ros-jazzy-rmw-implementation', 'ros-jazzy-rmw-cyclonedds-cpp',
            'ros-jazzy-rmw-connextdds']
query = subprocess.run(['dpkg-query', '-W', '-f=${Package}\t${Version}\t${Status}\n',
                        *packages], capture_output=True, text=True, check=False)
files = [
    '/opt/ros/jazzy/lib/librmw_fastrtps_cpp.so',
    '/opt/ros/jazzy/lib/librmw_fastrtps_shared_cpp.so',
    '/opt/ros/jazzy/lib/libfastrtps.so.2.14.6',
    '/opt/ros/jazzy/lib/libfastcdr.so.2.2.7',
    '/opt/ros/jazzy/include/fastrtps/fastrtps/config.h',
    '/opt/ros/jazzy/include/fastrtps/fastdds/dds/core/policy/QosPolicies.hpp',
    '/opt/ros/jazzy/include/rmw/rmw/qos_profiles.h',
    '/opt/ros/jazzy/share/rmw_implementation/cmake/export_rmw_implementationExport.cmake',
]
local_inputs = [ROOT/'simulation/joint_reference_adapter.py',
                ROOT/'simulation/control_bridge.py', ROOT/'simulation/execution_safety.py',
                ROOT/'navigation/test_results/full17_startup_status_audit/result.json',
                ROOT/'simulation/test_results/20261001_final_cm_dynamic/actual_gz_process.json',
                ROOT/'simulation/test_results/20261001_final_cm_dynamic/actuator/observer_result.json']

def receipt(path):
    path = pathlib.Path(path)
    if not path.is_file():
        return {'path': str(path), 'exists': False}
    raw = path.read_bytes()
    return {'path': str(path), 'exists': True, 'bytes': len(raw),
            'sha256': hashlib.sha256(raw).hexdigest()}

env_keys = ['RMW_IMPLEMENTATION', 'RMW_FASTRTPS_PUBLICATION_MODE',
            'RMW_FASTRTPS_USE_QOS_FROM_XML', 'FASTRTPS_DEFAULT_PROFILES_FILE',
            'FASTDDS_DEFAULT_PROFILES_FILE', 'ROS_LOCALHOST_ONLY']
result = {
    'captured_utc': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'scope': 'current installed packages/headers and analysis shell only; not Full17 process environment',
    'command': query.args, 'exit_code': query.returncode,
    'package_stdout': query.stdout, 'package_stderr': query.stderr,
    'analysis_shell_environment': {key: os.environ.get(key) for key in env_keys},
    'installed_files': [receipt(path) for path in files],
    'local_analysis_inputs': [receipt(path) for path in local_inputs],
    'primary_sources_receipt': receipt(OUT/'primary_sources/receipt.json'),
    'executed_ros_context_or_nodes': False, 'modified_runtime_or_qos': False,
}
(OUT/'installed_receipt.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps({'output':str(OUT/'installed_receipt.json'),
                  'exists_all_local_inputs':all(x['exists'] for x in result['local_analysis_inputs'])}))
