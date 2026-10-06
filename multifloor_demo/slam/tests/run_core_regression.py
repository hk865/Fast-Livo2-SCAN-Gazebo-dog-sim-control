#!/usr/bin/env python3
"""Compile a small test against the current installed production core library."""
import argparse
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

parser = argparse.ArgumentParser()
parser.add_argument('test', type=Path)
parser.add_argument('arguments', nargs='*')
parser.add_argument('--legacy-evidence', action='store_true')
parser.add_argument('--output', type=Path)
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
workspace = root / 'ros2_ws'
flags = (workspace / 'build/fast_livo2_core/CMakeFiles/fast_livo2_core.dir/flags.make').read_text()
values = {}
for line in flags.splitlines():
    if ' = ' in line:
        key, value = line.split(' = ', 1)
        values[key] = shlex.split(value)
with tempfile.TemporaryDirectory(prefix='slam-core-fixture-') as directory:
    binary = Path(directory) / 'test'
    libraries = [workspace / 'install/fast_livo2_core/lib', Path('/opt/ros/jazzy/lib'),
                 Path('/home/hyh001/projects/third_party/livox_ws/install/livox_ros_driver2/lib')]
    libraries += sorted((root.parents[1] / 'slam5_navigation/ros2_ws/install').glob('*/lib'))
    environment = os.environ.copy()
    environment['LD_LIBRARY_PATH'] = ':'.join(map(str, libraries)) + ':' + environment.get('LD_LIBRARY_PATH', '')
    environment['ROS_DOMAIN_ID'] = '79'
    environment['DEMO_RUN_DIR'] = str(Path(directory) / 'fixture_run')
    command = ['g++', '-std=c++17', '-O0', '-UNDEBUG', *values['CXX_DEFINES'],
               *values['CXX_INCLUDES'], str(args.test.resolve()), '-o', str(binary),
               '-L' + str(workspace / 'install/fast_livo2_core/lib'), '-L/opt/ros/jazzy/lib',
               '-Wl,-rpath,' + str(workspace / 'install/fast_livo2_core/lib'),
               '-Wl,-rpath,/opt/ros/jazzy/lib',
               *['-Wl,-rpath-link,' + str(path) for path in libraries],
               '-lfast_livo2_core', '-lrclcpp',
               '-lopencv_core', '-lrcutils', '-lpthread']
    subprocess.run(command, check=True, env=environment)
    invocation = [str(binary)]
    if args.legacy_evidence:
        invocation.append('--legacy-evidence')
    invocation += ['--ros-args', '--params-file', str(root / 'fastlivo.yaml'),
                   '--params-file', str(root / 'camera.yaml')]
    result = subprocess.run(invocation, env=environment, text=True, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT)
    if args.output:
        args.output.write_text(result.stdout)
    print(result.stdout)
    raise SystemExit(result.returncode)
